#include "drv8833.h"
#include "RobotConfig.h"

DRV8833 motor;

void DRV8833::begin() {
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
#ifdef MOTOR_SLEEP_PIN
  pinMode(MOTOR_SLEEP_PIN, OUTPUT);
  digitalWrite(MOTOR_SLEEP_PIN, HIGH);   // thức (không ngủ)
#endif
  stop();
}

void DRV8833::sleep(bool on) {
#ifdef MOTOR_SLEEP_PIN
  digitalWrite(MOTOR_SLEEP_PIN, on ? LOW : HIGH);
#endif
}

void DRV8833::driveOne(int in1Pin, int in2Pin, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(in1Pin, speed);
    analogWrite(in2Pin, 0);
  } else if (speed < 0) {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, -speed);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, 0);
  }
}

void DRV8833::run(int speedLeft, int speedRight) {
  driveOne(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, speedLeft);
  driveOne(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, speedRight);
}

void DRV8833::stop() {
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
}

void DRV8833::brake() {
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
}