#include <AFMotor.h>

// Define the number of sensors you're using
#define NUM_SENSORS 5

// Define the pins for each sensor
const uint8_t sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

// Arrays to store the calibration values
uint16_t sensorMin[NUM_SENSORS];
uint16_t sensorMax[NUM_SENSORS];

// Array to hold the current sensor readings
uint16_t sensorValues[NUM_SENSORS];

// Connect motors to ports on the motor shield
AF_DCMotor leftMotor(1);  // Motor 1 connected to M1 on shield
AF_DCMotor rightMotor(2); // Motor 2 connected to M2 on shield

// PID control variables
int baseSpeed = 140;  // Base motor speed for fast turns
int maxSpeed = 240;   // Maximum motor speed (0-255 for the motor driver)
float Kp = 0.6;       // Proportional constant
float Ki = 0.0;       // Integral constant (start with 0)
float Kd = 1.2;       // Derivative constant

int lastError = 0;    // Previous error for calculating derivative
int integral = 0;     // Integral of the error
int lastKnownDirection = 0; // Tracks last known sensor position for overshoot recovery

void setup() {
  // Initialize serial communication for output
  Serial.begin(9600);

  // Set initial motor speeds to 0
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  leftMotor.run(FORWARD);  // Set initial direction to FORWARD
  rightMotor.run(FORWARD);

  // Initialize the sensor calibration
  calibrateSensors();
}

void loop() {
  if (allSensorsWhite()) {
    // Stop motors if all sensors are detecting white (no line)
    recoverFromOvershoot();
  } else {
    readSensors();
    int error = calculateError();
    adjustForSharpTurn(error);
    PID_control(error);
  }
  delay(20);  // Short delay for smoother control
}

// Function to calibrate the sensors
void calibrateSensors() {
  Serial.println("Calibrating... Move the sensors over white and black surfaces.");
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023; // Max possible value
    sensorMax[i] = 0;    // Min possible value
  }
  for (uint16_t t = 0; t < 1000; t++) { // Calibrate for a duration
    readSensors();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      sensorMin[i] = min(sensorMin[i], sensorValues[i]);
      sensorMax[i] = max(sensorMax[i], sensorValues[i]);
    }
    delay(5);
  }
  Serial.println("Calibration complete.");
}

// Function to read sensor values
void readSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    sensorValues[i] = constrain(map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000), 0, 1000);
  }
}

// Calculate the error for PID control based on sensor readings
int calculateError() {
  long weightedSum = 0;
  long sum = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (long)sensorValues[i] * (i * 1000);
    sum += sensorValues[i];
  }

  if (sum == 0) return 0;

  int position = weightedSum / sum;
  int error = position - 2000;

  // Set a memory of last known direction for recovery from overshoot
  if (error < 0) lastKnownDirection = -1;
  else if (error > 0) lastKnownDirection = 1;

  return constrain(error, -800, 800); // Limit error for control
}

// Function to detect and handle sharp turns by reversing the inside wheel
void adjustForSharpTurn(int error) {
  // Detect left or right sharp turn
  if (error < -700) {
    leftMotor.run(BACKWARD); // Reverse left motor for sharper left turn
    delay(50);
  } else if (error > 700) {
    rightMotor.run(BACKWARD); // Reverse right motor for sharper right turn
    delay(50);
  }
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}

// PID control for adjusting motor speeds based on error
void PID_control(int error) {
  integral += error;
  int derivative = error - lastError;
  lastError = error;

  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Adjust motor speeds and constrain them
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  setMotorSpeeds(leftSpeed, rightSpeed);
}

// Function to set motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}

// Function to check if all sensors are detecting white
bool allSensorsWhite() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > 800) return false;
  }
  return true;
}

// Function to recover from an overshoot by moving based on last known position
void recoverFromOvershoot() {
  if (lastKnownDirection == -1) {
    leftMotor.setSpeed(baseSpeed);
    rightMotor.setSpeed(0);
  } else if (lastKnownDirection == 1) {
    leftMotor.setSpeed(0);
    rightMotor.setSpeed(baseSpeed);
  }
  delay(50); // Small delay for recovery
}
