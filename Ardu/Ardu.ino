#include <Servo.h>

Servo servo2;
Servo servo1;

void setup() {
  Serial.begin(9600);
  servo2.attach(9);
  servo1.attach(5);  // Initialize USB Serial for debugging
}

void loop() {
  static String receivedData = "";  // Buffer to store received data

  // Check if data is received from Wio Terminal
  if (Serial.available()) {
    receivedData = Serial.readString();  // Read data from Wio Terminal
    Serial.println("Received from Wio Terminal: " + receivedData);
    receivedData.trim(); // Remove any extra spaces or newline characters
  }

  // Validate and process the received data
  int command = receivedData.toInt();
  if (command >= 1 && command <= 3) {
    processCommand(command);  // Call a function to process the command
    receivedData = "";        // Reset the receivedData after processing
  }
}



// Function to process received commands
void processCommand(int command) {
  if (command == 1) {
    servo2.write(90);     // Move servo2 to 0 degrees
    delay(2000);
    servo1.write(0);
    delay(1700);
    servo1.write(90);
    delay(2000); 
    servo1.write(180);
    delay(1700);
    servo1.write(90);
    delay(2000);        // Wait for servo2 to finish
  } 
  else if (command == 2) {
    servo2.write(0);     // Move servo2 to 0 degrees
    delay(2000);
    servo1.write(0);
    delay(1700);
    servo1.write(90);
    delay(2000); 
    servo1.write(180);
    delay(1700);
    servo1.write(90);
    delay(2000);        // Wait for servo2 to finish
  } 
  else if (command == 3) {
    servo2.write(180);     // Move servo2 to 0 degrees
    delay(2000);
    servo1.write(0);
    delay(1700);
    servo1.write(90);
    delay(2000); 
    servo1.write(180);
    delay(1700);
    servo1.write(90);
    delay(2000);         // Wait for servo2 to finish
  }      // Detach servo2 after the movement
}
