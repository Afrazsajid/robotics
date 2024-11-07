#include <AFMotor.h>


// Define motor objects
AF_DCMotor leftMotor(1);  // Left motor connected to port 1
AF_DCMotor rightMotor(2); // Right motor connected to port 2

void motor(int a, int b) {
  // Control left motor
  if (a > 0) {
    leftMotor.setSpeed(a > 250 ? 250 : a); // Limit speed to 250
    leftMotor.run(FORWARD); // Move forward
  } else {
    leftMotor.setSpeed(-a > 250 ? 250 : -a); // Limit speed to 250
    leftMotor.run(BACKWARD); // Move backward
  }

  // Control right motor
  if (b > 0) {
    rightMotor.setSpeed(b > 250 ? 250 : b); // Limit speed to 250
    rightMotor.run(FORWARD); // Move forward
  } else {
    rightMotor.setSpeed(-b > 250 ? 250 : -b); // Limit speed to 250
    rightMotor.run(BACKWARD); // Move backward
  }
}


int sensor[5];  //to store the sensor value
int threshold = 512; //threshold = (minimum analog value + Maximum Analog Value) / 2
float c;
int left_motor_speed = 200, right_motor_speed = 200;
int left_motor, right_motor;
int kp = 40, kd = 1000;
int PID_value;
float current_error, previous_error;
int turn_speed = 120;
char t;


#define SENSOR_A0 A0
#define SENSOR_A1 A1
#define SENSOR_A2 A2
#define SENSOR_A3 A3
#define SENSOR_A4 A4
#define SENSOR_A5 A5




void setup() {
    // other setup code...

    // Initialize sensor pins
    pinMode(SENSOR_A0, INPUT);
    pinMode(SENSOR_A1, INPUT);
    pinMode(SENSOR_A2, INPUT);
    pinMode(SENSOR_A3, INPUT);
    pinMode(SENSOR_A4, INPUT);
    pinMode(SENSOR_A5, INPUT);

    calibrateSensors(); // Run calibration once at startup
}


void calibrateSensors() {
    Serial.println("Starting Calibration...");

    // Variables to hold sensor readings
    int sensorReadings[6];   // Assuming you have 6 sensors
    int minValue = 1023;     // Maximum possible sensor value
    int maxValue = 0;        // Minimum possible sensor value

    // Number of calibration samples
    const int numSamples = 100; // You can adjust this value

    for (int i = 0; i < numSamples; i++) {
        delay(100); // Wait time between readings for stability

        // Read sensor values
        sensor_reading(); // Function to get the latest sensor readings

        // Update min and max values across all sensors
        for (int j = 0; j < 6; j++) {
            sensorReadings[j] = sensor[j]; // Assume sensor[j] holds the latest reading
            if (sensorReadings[j] < minValue) {
                minValue = sensorReadings[j];
            }
            if (sensorReadings[j] > maxValue) {
                maxValue = sensorReadings[j];
            }
        }
    }

    Serial.println("Calibration Complete!");

    // Output the minimum and maximum values for each sensor
    Serial.print("Min Value = ");
    Serial.println(minValue);
    Serial.print("Max Value = ");
    Serial.println(maxValue);
    
    // Calculate the single threshold value as the midpoint
     threshold = (minValue + maxValue) / 2;
    Serial.print("Single Threshold Value: ");
    Serial.println(singleThreshold);

    // You may want to store the calibration values and threshold in variables or in EEPROM for future use
}






void loop() {
 

    Line_Follow();  //line follow using pid
  
}

void sensor_reading() {
    float a = 0;
    float b = 0;
    for (int i = 0; i < 5; i++) {
        sensor[i] = analogRead(i + 1);  // Read A1 to A5 for sensors
        if (sensor[i] > threshold) {
            sensor[i] = 1;
        } else {
            sensor[i] = 0;
        }
    }
    a = (sensor[0] * 1 + sensor[1] * 2 + sensor[2] * 3 + sensor[3] * 4 + sensor[4] * 5);
    b = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4]);
    if (b > 0) c = a / b; // Calculate the center of the line
}

void Line_Follow() {
  while (1) {
    sensor_reading();
    //Straight Line Follow
    if (sensor[2] == 1 || sensor[3] == 1) {
      current_error = 3.5 - c;
      PID_value = current_error * kp + kd * (current_error - previous_error);
      previous_error = current_error;

      right_motor = right_motor_speed - PID_value;
      left_motor = left_motor_speed + PID_value;
      motor(left_motor, right_motor);
    }

    //all sensor in white surface
    if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0 && sensor[5] == 0) {
      if (t == 'r') right();
      else if (t == 'l') left();
      else U_turn();
    }

    //Right Turn
    if (sensor[5] == 0 && sensor[0] == 1) t = 'r';
    //Left Turn
    if (sensor[5] == 1 && sensor[0] == 0) t = 'l';

    //all sensor in black surface
    if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1) {
      delay(30);
      sensor_reading();
      if ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]) == 6) {
        motor(0, 0);  //stop
        while ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]) == 6) sensor_reading();
      } else if ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5]) == 0) t = 'r';
    }
  }
}

void right() {
  while (1) {
    motor(turn_speed, -turn_speed);
    while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
    motor(-turn_speed, turn_speed);
    delay(20);
    break;
  }
}

void left() {
  while (1) {
    motor(-turn_speed, turn_speed);
    while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
    motor(turn_speed, -turn_speed);
    delay(20);
    break;
  }
}
void U_turn() {
  while (1) {
    delay(120);
    digitalWrite(led, HIGH);
    motor(-turn_speed, turn_speed);
    while (sensor[2] == 0 && sensor[3] == 0) sensor_reading();
    motor(turn_speed, -turn_speed);
    delay(20);
    digitalWrite(led, LOW);
    break;
  }
}



//motor run function
void motor(int a, int b) {
  // Control left motor
  if (a > 0) {
    leftMotor.setSpeed(a > 250 ? 250 : a); // Limit speed to 250
    leftMotor.run(FORWARD); // Move forward
  } else {
    leftMotor.setSpeed(-a > 250 ? 250 : -a); // Limit speed to 250
    leftMotor.run(BACKWARD); // Move backward
  }

  // Control right motor
  if (b > 0) {
    rightMotor.setSpeed(b > 250 ? 250 : b); // Limit speed to 250
    rightMotor.run(FORWARD); // Move forward
  } else {
    rightMotor.setSpeed(-b > 250 ? 250 : -b); // Limit speed to 250
    rightMotor.run(BACKWARD); // Move backward
  }
}


void show_analog_value() {
  for (short int i = 5; i >= 0; i--) {
    if (i > 3) Serial.print(String(analogRead(i + 2)) + " ");
    else Serial.print(String(analogRead(i)) + " ");
  }
  delay(100);
  Serial.println();
}

// https://github.com/aslam-Hossain-YT/Fast-Line-Follower-Robot/blob/main/PID_LINE_FOLLOWER_CODE/PID_LINE_FOLLOWER_CODE.ino