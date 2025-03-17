#include <alexsto-project-1_inferencing.h>
#include"TFT_eSPI.h"
TFT_eSPI tft;


// If your target is limited in memory remove this macro to save 10K RAM
#define EIDSP_QUANTIZE_FILTERBANK   0

/**
 * Define the number of slices per model window. E.g. a model window of 1000 ms
 * with slices per model window set to 4. Results in a slice size of 250 ms.
 * For more info: https://docs.edgeimpulse.com/docs/continuous-audio-sampling
 */
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 2

/*
 ** NOTE: If you run into TFLite arena allocation issue.
 **
 ** This may be due to may dynamic memory fragmentation.
 ** Try defining "-DEI_CLASSIFIER_ALLOCATION_STATIC" in boards.local.txt (create
 ** if it doesn't exist) and copy this file to
 ** <ARDUINO_CORE_INSTALL_PATH>/arduino/hardware/<mbed_core>/<core_version>/.
 **
 ** See
 ** (https://support.arduino.cc/hc/en-us/articles/360012076960-Where-are-the-installed-cores-located-)
 ** to find where Arduino installs cores on your machine.
 **
 ** If the problem persists then there's not enough memory for this model and application.
 */

/* Includes ---------------------------------------------------------------- */
#include <alexsto-project-1_inferencing.h>
#include "Ultrasonic.h"
#include <Servo.h>
#include <Arduino.h>
Servo gate;
int current;

Ultrasonic ultrasonic(0);
// Settings
#define DEBUG 1                 // Enable pin pulse during ISR  
enum {ADC_BUF_LEN = 4096};    // Size of one of the DMA double buffers
static const int debug_pin = 1; // Toggles each DAC ISR (if DEBUG is set to 1)

// DMAC descriptor structure
typedef struct {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor;

/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static bool record_ready = false;
static signed short *sampleBuffer;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);


// Globals - DMA and ADC
volatile uint8_t recording = 0;
volatile boolean results0Ready = false;
volatile boolean results1Ready = false;
uint16_t adc_buf_0[ADC_BUF_LEN];    // ADC results array 0
uint16_t adc_buf_1[ADC_BUF_LEN];    // ADC results array 1
volatile dmacdescriptor wrb[DMAC_CH_NUM] __attribute__ ((aligned (16)));          // Write-back DMAC descriptors
dmacdescriptor descriptor_section[DMAC_CH_NUM] __attribute__ ((aligned (16)));    // DMAC channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));                         // Place holder descriptor

//High pass butterworth filter order=1 alpha1=0.0125 
class  FilterBuHp1
{
  public:
    FilterBuHp1()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (9.621952458291035404e-1f * x)
         + (0.92439049165820696974f * v[0]);
      return 
         (v[1] - v[0]);
    }
};

FilterBuHp1 filter;


/***************************
 * Interrupt Service Routines (ISRs)
 */

/**
 * @brief      Copy sample data in selected buf and signal ready when buffer is full
 *
 * @param[in]  *buf  Pointer to source buffer
 * @param[in]  buf_len  Number of samples to copy from buffer
 */
static void audio_rec_callback(uint16_t *buf, uint32_t buf_len) {

  static uint32_t idx = 0;

  // Copy samples from DMA buffer to inference buffer
  if (recording) {
    for (uint32_t i = 0; i < buf_len; i++) {
  
      // Convert 12-bit unsigned ADC value to 16-bit PCM (signed) audio value
      inference.buffers[inference.buf_select][inference.buf_count++] = filter.step(((int16_t)buf[i] - 1024) * 16);
      // Swap double buffer if necessary
      if (inference.buf_count >= inference.n_samples) {
        inference.buf_select ^= 1;
        inference.buf_count = 0;
        inference.buf_ready = 1;
      }
    }
  }
}


/**
 * Interrupt Service Routine (ISR) for DMAC 1
 */
void DMAC_1_Handler() {

  static uint8_t count = 0;

  // Check if DMAC channel 1 has been suspended (SUSP)
  if (DMAC->Channel[1].CHINTFLAG.bit.SUSP) {

     // Debug: make pin high before copying buffer
#if DEBUG
    digitalWrite(debug_pin, HIGH);
#endif

    // Restart DMAC on channel 1 and clear SUSP interrupt flag
    DMAC->Channel[1].CHCTRLB.reg = DMAC_CHCTRLB_CMD_RESUME;
    DMAC->Channel[1].CHINTFLAG.bit.SUSP = 1;

    // See which buffer has filled up, and dump results into large buffer
    if (count) {
      audio_rec_callback(adc_buf_0, ADC_BUF_LEN);
    } else {
      audio_rec_callback(adc_buf_1, ADC_BUF_LEN);
    }

    // Flip to next buffer
    count = (count + 1) % 2;

    // Debug: make pin low after copying buffer
#if DEBUG
    digitalWrite(debug_pin, LOW);
#endif
  }
}


// Configure DMA to sample from ADC at regular interval
void config_dma_adc() {
  
  // Configure DMA to sample from ADC at a regular interval (triggered by timer/counter)
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;                          // Specify the location of the descriptors
  DMAC->WRBADDR.reg = (uint32_t)wrb;                                          // Specify the location of the write back descriptors
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);                // Enable the DMAC peripheral
  DMAC->Channel[1].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(TC5_DMAC_ID_OVF) |      // Set DMAC to trigger on TC5 timer overflow
                                 DMAC_CHCTRLA_TRIGACT_BURST;                  // DMAC burst transfer
                                 
  descriptor.descaddr = (uint32_t)&descriptor_section[1];                     // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t)&ADC1->RESULT.reg;                           // Take the result from the ADC0 RESULT register
  descriptor.dstaddr = (uint32_t)adc_buf_0 + sizeof(uint16_t) * ADC_BUF_LEN;  // Place it in the adc_buf_0 array
  descriptor.btcnt = ADC_BUF_LEN;                                             // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                            // Beat size is HWORD (16-bits)
                      DMAC_BTCTRL_DSTINC |                                    // Increment the destination address
                      DMAC_BTCTRL_VALID |                                     // Descriptor is valid
                      DMAC_BTCTRL_BLOCKACT_SUSPEND;                           // Suspend DMAC channel 0 after block transfer
  memcpy(&descriptor_section[0], &descriptor, sizeof(descriptor));            // Copy the descriptor to the descriptor section
  
  descriptor.descaddr = (uint32_t)&descriptor_section[0];                     // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t)&ADC1->RESULT.reg;                           // Take the result from the ADC0 RESULT register
  descriptor.dstaddr = (uint32_t)adc_buf_1 + sizeof(uint16_t) * ADC_BUF_LEN;  // Place it in the adc_buf_1 array
  descriptor.btcnt = ADC_BUF_LEN;                                             // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                            // Beat size is HWORD (16-bits)
                      DMAC_BTCTRL_DSTINC |                                    // Increment the destination address
                      DMAC_BTCTRL_VALID |                                     // Descriptor is valid
                      DMAC_BTCTRL_BLOCKACT_SUSPEND;                           // Suspend DMAC channel 0 after block transfer
  memcpy(&descriptor_section[1], &descriptor, sizeof(descriptor));            // Copy the descriptor to the descriptor section

  // Configure NVIC
  NVIC_SetPriority(DMAC_1_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for DMAC1 to 0 (highest)

  NVIC_EnableIRQ(DMAC_1_IRQn);         // Connect DMAC1 to Nested Vector Interrupt Controller (NVIC)

  // Activate the suspend (SUSP) interrupt on DMAC channel 1
  DMAC->Channel[1].CHINTENSET.reg = DMAC_CHINTENSET_SUSP;

  // Configure ADC
  ADC1->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN12_Val; // Set the analog input to ADC0/AIN2 (PB08 - A4 on Metro M4)
  while(ADC1->SYNCBUSY.bit.INPUTCTRL);                // Wait for synchronization
  ADC1->SAMPCTRL.bit.SAMPLEN = 0x00;                  // Set max Sampling Time Length to half divided ADC clock pulse (2.66us)
  while(ADC1->SYNCBUSY.bit.SAMPCTRL);                 // Wait for synchronization 
  ADC1->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV128;       // Divide Clock ADC GCLK by 128 (48MHz/128 = 375kHz)
  ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |          // Set ADC resolution to 12 bits
                    ADC_CTRLB_FREERUN;                // Set ADC to free run mode       
  while(ADC1->SYNCBUSY.bit.CTRLB);                    // Wait for synchronization
  ADC1->CTRLA.bit.ENABLE = 1;                         // Enable the ADC
  while(ADC1->SYNCBUSY.bit.ENABLE);                   // Wait for synchronization
  ADC1->SWTRIG.bit.START = 1;                         // Initiate a software trigger to start an ADC conversion
  while(ADC1->SYNCBUSY.bit.SWTRIG);                   // Wait for synchronization

  // Enable DMA channel 1
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;

  // Configure Timer/Counter 5
  GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |        // Enable perhipheral channel for TC5
                                   GCLK_PCHCTRL_GEN_GCLK1;    // Connect generic clock 0 at 48MHz
   
  TC5->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;               // Set TC5 to Match Frequency (MFRQ) mode
  TC5->COUNT16.CC[0].reg =  299;                          // Set the trigger to 16 kHz: (4Mhz / 16000) - 1
  while (TC5->COUNT16.SYNCBUSY.bit.CC0);                      // Wait for synchronization

  // Start Timer/Counter 5
  TC5->COUNT16.CTRLA.bit.ENABLE = 1;                          // Enable the TC5 timer
  while (TC5->COUNT16.SYNCBUSY.bit.ENABLE);                   // Wait for synchronization
}








/**
 * @brief      Arduino setup function
 */
void setup()
{
    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_WHITE);
     // Configure pin to toggle on DMA interrupt
    #if DEBUG
      pinMode(debug_pin, OUTPUT);
    #endif
    // put your setup code here, to run once:
    Serial.begin(9600);
    Serial1.begin(9600); 

    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));

    run_classifier_init();
   // Create double buffer for inference
  inference.buffers[0] = (int16_t *)malloc(EI_CLASSIFIER_SLICE_SIZE * sizeof(int16_t));
  
  if (inference.buffers[0] == NULL) {
    ei_printf("ERROR: Failed to create inference buffer 0");
    return;
  }
  inference.buffers[1] = (int16_t *)malloc(EI_CLASSIFIER_SLICE_SIZE * 
      sizeof(int16_t));
  if (inference.buffers[1] == NULL) {
    ei_printf("ERROR: Failed to create inference buffer 1");
    free(inference.buffers[0]);
    return;
  }

  // Set inference parameters
  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = EI_CLASSIFIER_SLICE_SIZE;
  inference.buf_ready = 0;

  // Configure DMA to sample from ADC at 16kHz (start sampling immediately)
  config_dma_adc();

  // Start recording to inference buffers
  recording = 1;
}



/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop() {
  long RangeInCentimeters;
  static int stable_count = 0;    // Counter for consecutive stable predictions
  static int stable_class = -1; 
    RangeInCentimeters = ultrasonic.MeasureInCentimeters();
    ei_printf("%d",RangeInCentimeters);
    if (RangeInCentimeters < 3 && RangeInCentimeters!=0) {
        bool trash_detected = false;
        for (int ss = 0; ss < 50 && !trash_detected; ss++) {
            signal_t signal;
            signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
            signal.get_data = &microphone_audio_signal_get_data;
            ei_impulse_result_t result = {0};

            EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
            if (r != EI_IMPULSE_OK) {
                ei_printf("ERR: Failed to run classifier (%d)\n", r);
                continue;
            }

            if (++print_results >= EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW) {
                ei_printf("Predictions ");
                ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms. SS= %d)",
                    result.timing.dsp, result.timing.classification, result.timing.anomaly, ss);
                ei_printf(": \n");
                for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                    ei_printf("    %s: %.5f\n", result.classification[ix].label,
      
                    result.classification[ix].value);
                    if (result.classification[1].value >= 0.70 && ix != 3) { 
                          stable_count = 1; // Reset the counter if a new class is detected
                          stable_class = ix;
                          ei_printf("CAN DETECTED\n");
                          tft.setTextSize(10);
                          tft.fillScreen(TFT_PURPLE);
                          tft.drawString("Plastic", 20, 80);
                          trash_detected = true;                            
                          Serial1.println(1);
                          delay(1000);
                          //open gate
                          break;
                    }
                    else if (result.classification[2].value >= 0.60 && ix != 3) { 
                          stable_count = 1; // Reset the counter if a new class is detected
                          stable_class = ix;
                          ei_printf("PLASTIC DETECTED\n");
                          tft.setTextSize(10);
                          tft.fillScreen(TFT_PURPLE);
                          tft.drawString("Can", 20, 80);
                          trash_detected = true;
                          //clasify
                          delay(1000);
                           Serial1.println(2);
                          break;
                    }
                    else if (ss > 40){
                          ei_printf("OTHER TRASH DETECTED\n");
                          tft.fillScreen(TFT_PURPLE);
                          tft.setTextSize(10); // Increase the text size; adjust the value as needed (default is 1)
                          tft.drawString("OTHER TRASH", 20, 80);
                          trash_detected = true;
                          Serial1.println(3);
                          delay(1000);
                          break;
                    }
                }
                print_results = 0;
            }
        }
    }
}


/**
 * @brief      PDM buffer full callback
 *             Get data and call audio thread callback
 */


/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[1] == NULL) {
        free(inference.buffers[0]);
        return false;
    }

    sampleBuffer = (signed short *)malloc((n_samples >> 1) * sizeof(signed short));

    if (sampleBuffer == NULL) {
        free(inference.buffers[0]);
      free(inference.buffers[1]);
      return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;


    record_ready = true;

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */



/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers

 */
static void microphone_inference_end(void)
{
    free(inference.buffers[0]);
    free(inference.buffers[1]);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif


