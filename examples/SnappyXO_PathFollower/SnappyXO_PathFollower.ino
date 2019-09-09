#include "L293.h"
#include "ArduinoBlue.h"
#include "PreMo.h"
#include <SoftwareSerial.h>
#include <Arduino.h>


// MOTOR PINS
const int LEFT_EN = 11;
const int LEFT1 = 12;
const int LEFT2 = 10;
const int RIGHT_EN = 5;
const int RIGHT1 = 4;
const int RIGHT2 = 6;


// BLUETOOTH PINS
const int BLUETOOTH_TX = 8;
const int BLUETOOTH_RX = 7;


// ENCODER PINS
const int ENCODER_LEFT_PIN = 3;
const int ENCODER_RIGHT_PIN = 2;


// ROBOT MEASUREMENTS]
const float RADIUS = 31; // wheel radius in mm
const float LENGTH = 167; // wheel base length in mm


// PULSES PER REV
const int PULSES_PER_REV = 144; // number of pulses per revolution of the wheel.


// MAXIMUM MOTOR SPEED VALUE
const int MOTOR_SPEED_MAX = 255;


// SERIAL COMMUNICATION BAUD RATE
const unsigned long BAUD_RATE = 115200;


// PID TUNING
// PID for path following (for turning when followiung path)
const double KP = 50;
const double KD = 1;
// PID for motors (for twist method)
const double KP_motor = 1.5;
const double KI_motor = 0;


// INTERVALS
// MIN_PULSE_INTERVAL is the minimum time in microseconds before registering each encoder pulse.
// This is to compensate for "bouncing" on certain encoders.
const unsigned long MIN_PULSE_INTERVAL = 175;
// When following path, send the location to app at this interval in millisecond.
const unsigned long SEND_INTERVAL = 200;


// ENCODER PULSES
unsigned long leftPulses;
unsigned long rightPulses;


// TIMING VARIABLES
unsigned long prevLeftTime;
unsigned long prevRightTime;
unsigned long prevSendTime;


// PATH FOLLOWING SPEED
// Target speed of path following in percentage.
const int PATH_FOLLOW_SPEED = 100;

// MOTORS
L293 leftMotor(LEFT_EN, LEFT1, LEFT2);
L293 rightMotor(RIGHT_EN, RIGHT1, RIGHT2);

// MOTOR SPEED FUNCTIONS
void setLeftForward(int speed) {
  leftMotor.forward(speed);
}

void setLeftReverse(int speed) {
  leftMotor.back(speed);
}

void setRightForward(int speed) {
  rightMotor.forward(speed);
}

void setRightReverse(int speed) {
  rightMotor.back(speed);
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}


// ARDUINO BLUE
SoftwareSerial bluetoothSerial(BLUETOOTH_TX, BLUETOOTH_RX);
ArduinoBlue bluetooth(bluetoothSerial);
int button;

// PATH FOLLOWER
MotorManager motorManager(setLeftForward, setLeftReverse, setRightForward, setRightReverse, stopMotors);
EncoderManager encoderManager(&leftPulses, &rightPulses, PULSES_PER_REV);
PreMo premo(RADIUS, LENGTH, KP, KD, KP_motor, KI_motor, &motorManager, &encoderManager);


void pulseLeft() {
  if (digitalRead(ENCODER_LEFT_PIN) && micros() - prevRightTime > MIN_PULSE_INTERVAL) {
      leftPulses++;
      prevRightTime = micros();
  }
}

void pulseRight() {
  if (digitalRead(ENCODER_RIGHT_PIN) && micros() - prevLeftTime > MIN_PULSE_INTERVAL) {
    rightPulses++;
    prevLeftTime = micros();
  }
}

void attachInterrupts() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, HIGH);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, HIGH);
}

void detachInterrupts() {
  detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN));
}

void testMotors() {
  Serial.println("MOTORS TEST");

  while (true)
  {
    Serial.println("left motor forward max speed");
    stopMotors();
    setLeftForward(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("right motor forward max speed");
    stopMotors();
    setRightForward(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("both motors forward max speed");
    setLeftForward(MOTOR_SPEED_MAX);
    setRightForward(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("both motors forward 75% speed");
    setLeftForward(MOTOR_SPEED_MAX*0.75);
    setRightForward(MOTOR_SPEED_MAX*0.75);
    delay(5000);
    
    Serial.println("both motors reverse max speed");
    setLeftReverse(MOTOR_SPEED_MAX);
    setRightReverse(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("both motors reverse 75% speed");
    setLeftReverse(MOTOR_SPEED_MAX*0.75);
    setRightReverse(MOTOR_SPEED_MAX*0.75);
    delay(5000);

    Serial.println("Observe motors stop");
    stopMotors();
    delay(5000);
  }
}

void testEncoders() {
    Serial.println("ENCODERS TEST");
    while (true) {
        Serial.print("left: ");
        Serial.print(leftPulses);
        Serial.print("\tright: ");
        Serial.println(rightPulses);
    }
}

void setupStuff() {
  Serial.begin(BAUD_RATE);
  bluetoothSerial.begin(BAUD_RATE);
  bluetooth.setInterruptToggle(attachInterrupts, detachInterrupts);
  motorManager.setSpeedLimits(0, MOTOR_SPEED_MAX);
  premo.twistBothMotors(false);
  premo.setPathFollowSpeed(PATH_FOLLOW_SPEED);

  attachInterrupts();
}

void sendLocation() {
  if (millis() - prevSendTime > SEND_INTERVAL) {
    // Todo sending location should be less code for user.
    float xPos = premo.getX();
    float yPos = premo.getY();
    float heading = premo.getHeading();
    float xGoal = premo.getGoalX();
    float yGoal = premo.getGoalY();
    bluetooth.sendLocation(xPos, yPos, heading, xGoal, yGoal);
    prevSendTime = millis();
  }
}

void pathFollowing() {
  if (bluetooth.isPathAvailable()) {
      delay(1000);
      float* pathX = bluetooth.getPathArrayX();
      float* pathY = bluetooth.getPathArrayY();
      int pathLength = bluetooth.getPathLength();
      premo.startPathFollowing(pathX, pathY, pathLength);
  }

  if (premo.isFollowingPath()) {
    sendLocation();
  }
}

void handleSteering() {
  
}

void loopStuff() {
  // Path follower loop
  premo.loop();

  // Bluetooth
  bluetooth.checkBluetooth();
  button = bluetooth.getButton();
}
