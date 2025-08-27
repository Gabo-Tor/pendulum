#include <Wire.h>
#include "AS5600.h"

// === Pin Definitions ===
#define A_IN1 19   // L9110H Motor Driver INA
#define A_IN2 18   // L9110H Motor Driver INB
#define SDA 21  // I2C SDA pin for AS5600
#define SCL 22  // I2C SCL pin for AS5600

// === Constants ===
#define ZERO_ANGLE_OFFSET 869           // Calibration offset (raw units from AS5600) 0 is vertical
#define MAX_MOTOR_SPEED   255           // Max PWM value
#define MIN_MOTOR_SPEED   0

// === Globals ===
AS5600 as5600;
int currentMotorCmd = 0;  // Last received command (range -255 to 255)

// === Helper Functions ===

// Set motor output based on command value
void setMotor(int cmd) {
  cmd = constrain(cmd, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

  if (cmd > 0) {
    analogWrite(A_IN1, cmd);
    analogWrite(A_IN2, 0);
  } else if (cmd < 0) {
    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, -cmd);  // use magnitude
  } else {
    analogWrite(A_IN1, 0);
    analogWrite(A_IN2, 0);
  }
}

// Read pendulum angle in degrees
float readAngle() {
  long rawAngle = as5600.readAngle();
  float angleDeg = (rawAngle - ZERO_ANGLE_OFFSET) * AS5600_RAW_TO_DEGREES;
  return angleDeg;
}

// Handle incoming serial command
void handleSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Expect one line per action
    input.trim();

    if (input.length() > 0) {
      int cmd = input.toInt();   // Action = integer motor command
      currentMotorCmd = cmd;
      setMotor(currentMotorCmd);

      // Immediately respond with observation
      float angleDeg = readAngle();
      Serial.println(angleDeg, 3);  // report angle with 3 decimals
    }
  }
}

// === Arduino Setup ===
void setup() {
  // Motor pins
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);

  // Serial + I2C
  Serial.begin(19200);
  Wire.begin(SDA, SCL);

  // Sensor setup
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  
  // Stop motor initially
  setMotor(0);
}

// === Arduino Loop ===
void loop() {
  handleSerial();  // Only act when command received
}
