### Line Follower Robot using PID Control
-----
###  Description:
  This program implements a PID-based control system for a line follower robot
  using QTR sensors and L298N motor driver. It includes speed control and
  turning logic for sharp edges.
*/

#include <QTRSensors.h>

## ====================== SENSOR SETUP ======================
QTRSensors qtr;

const uint8_t SensorCount = 9;
uint16_t sensorValues[SensorCount];

## ====================== PID CONSTANTS ======================

#define Kp 0.5      // Proportional Gain

#define Kd 0.3      // Derivative Gain

#define Ksr 0       // Speed Reduction Factor

#define MaxSpeed 255

#define BaseSpeed 255

#define speedturn 255

##====================== USER INPUT ======================
#define userButton 0

## ====================== MOTOR PINS ======================
// Left Motor
int PWMA = 6;
int AIN1 = 5;
int AIN2 = 4;

// Right Motor
int PWMB = 3;
int BIN1 = 2;
int BIN2 = 1;

## ====================== VARIABLES ======================
int lastError = 0;

## ====================== SETUP ======================
void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){7, 8, A0, A1, A2, A3, A4, A5}, SensorCount);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(userButton, INPUT_PULLUP);

  delay(200);

  // 🔧 Sensor Calibration
  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(10);
  }

  delay(1000);
}

## ====================== MAIN LOOP ======================
void loop() {

  uint16_t position = qtr.readLineBlack(sensorValues);

  // Extreme Right Turn
  if (position > 6700) {
    move(1, speedturn, 0);
    move(0, speedturn, 1);
    return;
  }

  // Extreme Left Turn
  if (position < 300) {
    move(1, speedturn, 1);
    move(0, speedturn, 0);
    return;
  }

  ## ====================== PID CONTROL ======================
  int error = position - 3500;

  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;

  // Optional speed reduction logic
  double speedError = Ksr * (exp(1) * ((8 * (8 + 1)) / 2));

  int rightMotorSpeed = BaseSpeed + motorSpeed - speedError;
  int leftMotorSpeed  = BaseSpeed - motorSpeed - speedError;

## ====================== SPEED LIMITING ======================
  rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
  leftMotorSpeed  = constrain(leftMotorSpeed, 0, MaxSpeed);

## ====================== MOTOR OUTPUT ======================
  move(1, rightMotorSpeed, 1);
  move(0, leftMotorSpeed, 1);
}

## ====================== MOTOR FUNCTION ======================
void move(int motor, int speed, int direction) {

  boolean inPin1 = HIGH;
  boolean inPin2 = LOW;

  if (direction == 0) {
    inPin1 = LOW;
    inPin2 = HIGH;
  }

  if (motor == 0) {
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }

  if (motor == 1) {
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}

