#include <QTRSensors.h> //Make sure to install the library

/*************************************************************************
*  Sensor Array object initialization 
*************************************************************************/
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*************************************************************************
*  PID control system variables 
*************************************************************************/
float Kp = 0;
float Ki = 0;
float Kd = 0;
int P;
int I;
int D;

/*************************************************************************
*  Global variables
*************************************************************************/
int lastError = 0;
boolean onoff = false;

/*************************************************************************
*  Motor speed variables (choose between 0 - no speed, and 255 - max speed)
*************************************************************************/
const uint8_t maxspeeda = 150;
const uint8_t maxspeedb = 150;
const uint8_t basespeeda = 100;
const uint8_t basespeedb = 100;

/*************************************************************************
*  TB6612FNG GPIO pins declaration
*************************************************************************/
int PWMA = 9;  // PWM for Motor A
int AIN1 = 8;  // Motor A direction
int AIN2 = 7;  // Motor A direction
int PWMB = 6;  // PWM for Motor B
int BIN1 = 5;  // Motor B direction
int BIN2 = 4;  // Motor B direction

/*************************************************************************
*  Buttons pins declaration
*************************************************************************/
int buttoncalibrate = 17; // or pin A3
int buttonstart = 2;

/*************************************************************************
*  Setup Function
*************************************************************************/
void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
  qtr.setEmitterPin(7); // LEDON PIN

  // Motor driver pin configuration
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // Setup LED and buttons
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttoncalibrate, INPUT);
  pinMode(buttonstart, INPUT);

  boolean Ok = false;
  while (Ok == false) {
    if (digitalRead(buttoncalibrate) == HIGH) {
      calibration();  // Calibrate the robot
      Ok = true;
    }
  }
  forward_brake(0, 0);  // Stop the motors
}

/*************************************************************************
*  Calibration Function
*************************************************************************/
void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

/*************************************************************************
*  Main Loop Function
*************************************************************************/
void loop() {
  if (digitalRead(buttonstart) == HIGH) {
    onoff = !onoff;
    if (onoff == true) {
      delay(1000);  // Delay when the robot starts
    } else {
      delay(50);
    }
  }
  
  if (onoff == true) {
    PID_control();
  } else {
    forward_brake(0, 0);  // Stop the motors
  }
}

/*************************************************************************
*  Motor Control Function for TB6612FNG
*************************************************************************/
void forward_brake(int speedA, int speedB) {
  digitalWrite(AIN1, HIGH);  // Set Motor A forward
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speedA);
  
  digitalWrite(BIN1, HIGH);  // Set Motor B forward
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, speedB);
}

/*************************************************************************
*  PID Control Function
*************************************************************************/
void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);  // Read sensor position
  int error = 3500 - position;  // Ideal position is the center

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd;  // Calculate motor correction
  
  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) motorspeeda = maxspeeda;
  if (motorspeedb > maxspeedb) motorspeedb = maxspeedb;
  if (motorspeeda < 0) motorspeeda = 0;
  if (motorspeedb < 0) motorspeedb = 0;
  
  forward_brake(motorspeeda, motorspeedb);
}
