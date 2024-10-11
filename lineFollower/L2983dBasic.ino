#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create Adafruit Motor Shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Connect motors to ports on the motor shield
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);  // Motor 1 connected to M1 on shield
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Motor 2 connected to M2 on shield

// Define speed
int motorSpeed = 200;

// Create QTR sensor object
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(9600); // Initialize serial communication
  
  // Initialize motor shield
  AFMS.begin();
  
  // Initialize QTR sensor array
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);
  
  // Calibrate the sensors
  calibrateSensors();
}

void loop() {
  // Read calibrated sensor values (0 - 1000)
  qtr.readCalibrated(sensorValues);
  
  // Debugging: Print calibrated sensor values
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  // Right two sensors detect black (turn left)
  if (sensorValues[3] > 600 && sensorValues[4] > 600) {
    turnLeft();
  }
  // Left two sensors detect black (turn right)
  else if (sensorValues[0] > 600 && sensorValues[1] > 600) {
    turnRight();
  }
  // Middle sensor detects black (move forward)
  else if (sensorValues[2] > 600) {
    moveForward();
  }
  // Otherwise, stop
  else {
    stopMotors();
  }

  delay(100); // Small delay for stability
}

// Function to calibrate the sensors
void calibrateSensors() {
  Serial.println("Calibrating...");
  
  for (uint16_t i = 0; i < 400; i++) {  // Run for 5 seconds
    qtr.calibrate();
    delay(12);
  }
  
  Serial.println("Calibration Complete");
  
  // Display min/max values (for debugging)
  Serial.println("Min/Max values:");
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(qtr.calibratedMinimumOn[i]);
    Serial.print('/');
    Serial.print(qtr.calibratedMaximumOn[i]);
    Serial.print('\t');
  }
  Serial.println();
}

// Function to move forward
void moveForward() {
  leftMotor->setSpeed(motorSpeed);    // Set motor speed (0-255)
  rightMotor->setSpeed(motorSpeed);
  leftMotor->run(FORWARD);            // Move both motors forward
  rightMotor->run(FORWARD);
}

// Function to turn left
void turnLeft() {
  leftMotor->setSpeed(0);             // Stop left motor
  rightMotor->setSpeed(motorSpeed);   // Run right motor to turn left
  leftMotor->run(RELEASE);            // Release the left motor
  rightMotor->run(FORWARD);           // Move right motor forward
}

// Function to turn right
void turnRight() {
  rightMotor->setSpeed(0);            // Stop right motor
  leftMotor->setSpeed(motorSpeed);    // Run left motor to turn right
  rightMotor->run(RELEASE);           // Release the right motor
  leftMotor->run(FORWARD);            // Move left motor forward
}

// Function to stop both motors
void stopMotors() {
  leftMotor->setSpeed(0);             // Stop left motor
  rightMotor->setSpeed(0);            // Stop right motor
  leftMotor->run(RELEASE);            // Release the motors
  rightMotor->run(RELEASE);
}
