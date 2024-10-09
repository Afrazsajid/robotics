#include <QTRSensors.h>

// Sensor and Motor Setup
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Motor speed settings
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

// Pin definitions for motors
int mode = 8;
int aphase = 9;
int aenbl = 6;
int bphase = 5;
int benbl = 3;

// Button to start the robot
int buttonstart = 2;

bool onoff = false; // State to track if the robot is running

void setup() {
  // Set up the QTR sensor
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
  qtr.setEmitterPin(7); // LED control pin

  // Set up the motor driver pins
  pinMode(mode, OUTPUT);
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  digitalWrite(mode, HIGH); // Set mode for motor control

  // Set up the start button
  pinMode(buttonstart, INPUT_PULLUP); // Start button with pull-up

  // Initial motor stop
  forward_brake(0, 0);
}

void loop() {
  // Check if the start button is pressed
  if (digitalRead(buttonstart) == LOW) {
    onoff = !onoff;
    delay(500); // Debounce delay
  }

  // If the robot is on, follow the line
  if (onoff) {
    follow_line();
  } else {
    forward_brake(0, 0); // Stop the motors
  }
}

void follow_line() {
  // Read the sensor array and get the position of the line
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calculate error (difference from the center position)
  int error = 3500 - position; // 3500 is the center position of the array

  // Basic proportional control: adjust speed based on error
  int motorSpeedAdjustment = error / 10; // Change the 10 value to adjust responsiveness

  // Calculate motor speeds
  int motorspeeda = basespeeda + motorSpeedAdjustment;
  int motorspeedb = basespeedb - motorSpeedAdjustment;

  // Constrain motor speeds to max speed
  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);

  // Set motor speeds
  forward_brake(motorspeeda, motorspeedb);
}

void forward_brake(int posa, int posb) {
  // Set motor direction
  digitalWrite(aphase, LOW); // Forward for motor A
  digitalWrite(bphase, LOW); // Forward for motor B

  // Set motor speed
  analogWrite(aenbl, posa); // Speed for motor A
  analogWrite(benbl, posb); // Speed for motor B
}
