#include <QTRSensors.h>                 // For QTR sensor array
#include <Wire.h>                       // For I2C communication
#include <Adafruit_MotorShield.h>      // Adafruit Motor Shield Library

/*************************************************************************
*  Sensor Array object initialization 
*************************************************************************/
QTRSensors qtr;
const uint8_t SensorCount = 5;        // Using a 5-channel IR sensor array
uint16_t sensorValues[SensorCount];

/*************************************************************************
*  PID control system variables 
*************************************************************************/
const float Kp = 0.1;                 // Proportional control
const float Ki = 0.0002;              // Integral control
const float Kd = 0.7;                 // Derivative control
int lastError = 0;
float integral = 0;                    // Integral term to prevent windup

/*************************************************************************
*  Motor speed variables
*************************************************************************/
const uint8_t maxSpeed = 200;         // Max motor speed
const uint8_t baseSpeed = 150;        // Base motor speed

/*************************************************************************
*  Motor Shield Setup
*************************************************************************/
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // Create the motor shield object
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);     // Motor A
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);    // Motor B

/*************************************************************************
*  Buttons pins declaration
*************************************************************************/
const int buttonCalibrate = 8;
const int buttonStart = 2;
bool onoff = false;

/*************************************************************************
*  Setup function: Initializes motors, sensors, and calibration
*************************************************************************/
void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);
  qtr.setEmitterPin(7); // LED control pin
  
  pinMode(buttonCalibrate, INPUT_PULLUP);
  pinMode(buttonStart, INPUT_PULLUP);
  
  AFMS.begin();  // Initialize the motor shield
  calibrateSensors();  // Calibrate IR sensors
}

/*************************************************************************
*  Calibration Function: Calibrates the IR sensor array
*************************************************************************/
void calibrateSensors() {
  Serial.begin(9600);
  Serial.println("Calibrating sensors...");
  
  for (uint16_t i = 0; i < 200; i++) { // Reduced calibration iterations
    qtr.calibrate();
    delay(10); // Reduced delay for faster calibration
  }
  Serial.println("Calibration complete.");
}

void handleTurn(int error) {
  if (error < -1000) {  // Sharp right turn
    // Turn right: stop left motor, move right motor forward
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(maxSpeed);
    rightMotor->run(FORWARD);
    delay(300);  // Adjust delay for turn duration
  } else if (error > 1000) {  // Sharp left turn
    // Turn left: stop right motor, move left motor forward
    leftMotor->setSpeed(maxSpeed);
    rightMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    delay(300);  // Adjust delay for turn duration
  }
  
  // Stop motors after turn
  stopMotors();
}


/*************************************************************************
*  Main Loop: Handles robot's line following with PID control
*************************************************************************/
void loop() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;  // Debounce delay
  
  int reading = digitalRead(buttonStart);
  
  if (reading == LOW && (millis() - lastDebounceTime > debounceDelay)) {
    onoff = !onoff;
    lastDebounceTime = millis();  // Record the time of the button press
  }
  
  if (onoff) {
    PID_control();  // Apply PID control for line following
  } else {
    stopMotors();  // Stop motors if off
  }
}

/*************************************************************************
*  PID Control Function: Implements the PID logic for smooth line following
*************************************************************************/
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);  // Read sensor data
  int error = 2000 - position;  // Center position is 2000 (ideal)

  // Detect a sharp turn
  if (error < -1000 || error > 1000) { // Adjust threshold based on your setup
    handleTurn(error);
    return;  // Exit PID control to avoid conflicting commands
  }

  // Regular PID calculations
  integral += error;  
  integral = constrain(integral, -maxSpeed, maxSpeed);
  int derivative = error - lastError;
  lastError = error;
  
  int correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;
  
  // Clamp speeds to max limits
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);
  
  setMotorSpeeds(leftSpeed, rightSpeed);
}


/*************************************************************************
*  Motor Control Functions: Set motor speeds
*************************************************************************/
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  leftMotor->setSpeed(leftSpeed);
  rightMotor->setSpeed(rightSpeed);
  
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void stopMotors() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}