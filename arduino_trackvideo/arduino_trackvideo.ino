#include <Servo.h>

// Create servo objects for horizontal (X) and vertical (Y) servos
Servo servoX;
Servo servoY;

// Define pins for each servo
const int servoXPin = 9;  // Pin connected to the X servo
const int servoYPin = 10; // Pin connected to the Y servo

// Define angle limits for servos
const int minAngle = 10;   // Minimum angle for servos
const int maxAngle = 170;  // Maximum angle for servos

// Variables to hold target angles
int targetAngleX = 90; // Initial center position for X servo
int targetAngleY = 90; // Initial center position for Y servo

void setup() {
  // Attach servos to pins
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  // Move servos to initial positions
  servoX.write(targetAngleX);
  servoY.write(targetAngleY);

  // Begin serial communication
  Serial.begin(9600);
  Serial.println("Servo control initialized.");
}

void loop() {
  // Check if data is available on serial port
  if (Serial.available() > 0) {
    // Read the incoming string until newline
    String data = Serial.readStringUntil('\n');
    data.trim();

    // Parse the data to extract angles for X and Y
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      String xAngleStr = data.substring(0, commaIndex);
      String yAngleStr = data.substring(commaIndex + 1);

      // Convert strings to integers and constrain within limits
      targetAngleX = constrain(xAngleStr.toInt(), minAngle, maxAngle);
      targetAngleY = constrain(yAngleStr.toInt(), minAngle, maxAngle);

      // Move servos to target angles
      servoX.write(targetAngleX);
      servoY.write(targetAngleY);

      // Output the current angles to the Serial Monitor for debugging
      Serial.print("Servo X Angle: ");
      Serial.print(targetAngleX);
      Serial.print(" | Servo Y Angle: ");
      Serial.println(targetAngleY);
    }
  }
}
