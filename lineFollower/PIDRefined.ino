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
int baseSpeed = 80;   // Base motor speed
int maxSpeed = 150;    // Maximum motor speed (0-255 for the motor driver)
float Kp = 0.5;        // Proportional constant
float Ki = 0.0;        // Integral constant (start with 0)
float Kd = 1.0;        // Derivative constant

int lastError = 0;     // Previous error for calculating derivative
int integral = 0;      // Integral of the error

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
        // Stop motors if all sensors are not detecting black
        leftMotor.setSpeed(0);
        rightMotor.setSpeed(0);
        leftMotor.run(RELEASE);
        rightMotor.run(RELEASE);
        Serial.println("All sensors white. Motors stopped.");
    } else {

  readSensors();

  // Use PID control to adjust motor speeds based on the error
  PID_control();
        // Continue driving or implement PID control logic
        // PID_control(); // Uncomment this if you have PID control logic
    }
  // Get the sen

  delay(50);  // Small delay between loops to allow time for motors to adjust
}

// Function to calibrate the sensors (stores min and max values)
void calibrateSensors() {
  Serial.println("Calibrating... Move the sensors over white and black surfaces.");

  // Initialize calibration values
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;  // Set to the maximum possible value initially
    sensorMax[i] = 0;     // Set to the minimum possible value initially
  }

  // Perform calibration
  for (uint16_t t = 0; t < 1000; t++) {  // Calibrate for some duration
    readSensors();

    // Update min and max values for each sensor
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] < sensorMin[i]) {
        sensorMin[i] = sensorValues[i];
      }
      if (sensorValues[i] > sensorMax[i]) {
        sensorMax[i] = sensorValues[i];
      }
    }

    delay(5);
  }

  Serial.println("Calibration complete.");
}

// Function to read sensor values using analogRead
void readSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);

    // Normalize the readings between 0 and 1000 based on calibration
    if (sensorValues[i] < sensorMin[i]) {
      sensorValues[i] = 0;
    } else if (sensorValues[i] > sensorMax[i]) {
      sensorValues[i] = 1000;
    } else {
      sensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    }
  }
}

// Function to calculate the error for PID control
int calculateError() {
  long weightedSum = 0;
  long sum = 0;

  // Calculate a weighted average based on sensor positions and readings
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (long)sensorValues[i] * (i * 1000);  // Weights: 0, 1000, 2000, 3000, 4000
    sum += sensorValues[i];
  }

  // If no line detected (all sensors are reading white), return error = 0
  if (sum == 0) {
    return 0;  // Could be adjusted to another default value if needed
  }

  // Calculate the weighted position of the line
  int position = weightedSum / sum;

  // Special handling for edge cases:
  // Case 1: If the last two sensors on the left (sensors 0 and 1) detect the line
  if (sensorValues[0] > 800 && sensorValues[1] > 800 && sensorValues[2] < 500 && sensorValues[3] < 500 && sensorValues[4] < 500) {
    return -700;  // Error for the line being far left
  }

  // Case 2: If the last two sensors on the right (sensors 3 and 4) detect the line
  if (sensorValues[3] > 800 && sensorValues[4] > 800 && sensorValues[0] < 500 && sensorValues[1] < 500 && sensorValues[2] < 500) {
    return 700;  // Error for the line being far right
  }

  // The target position is the center (2000), so error is the deviation from this center
  int error = position - 2000;  // Error will be negative if too far left, positive if too far right

  // Limit the error to be within the desired range
  if (error > 600) {
    error = 300;
  } else if (error < -600) {
    error = -300;
  }

  return error;
}

// Function to implement PID control and adjust motor speeds
void PID_control() {
  // Get the current error from sensor readings
  int error = calculateError();

  // Calculate the PID control values
  integral += error;  // Integral term
  int derivative = error - lastError;  // Derivative term
  lastError = error;  // Save the error for the next loop

  // Calculate the correction using the PID formula
  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Calculate the motor speeds
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Constrain motor speeds to max and min values
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Set motor speeds
  setMotorSpeeds(leftSpeed, rightSpeed);
}

// Function to set motor speeds using AFMotor library
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftMotor.setSpeed(leftSpeed);
  rightMotor.setSpeed(rightSpeed);
  
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
}
bool allSensorsWhite() {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (sensorValues[i] > 800) {  // Assuming 800 is the threshold for detecting black
            return false; // At least one sensor is detecting black
        }
    }
    return true; // All sensors are detecting white
}