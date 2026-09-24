#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
  : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax) {}

void PIDController::setGains(float kp, float ki, float kd) {
  _kp = kp; _ki = ki; _kd = kd;
}

void PIDController::setOutputLimits(float outMin, float outMax) {
  _outMin = outMin; _outMax = outMax;
}

void PIDController::reset() {
  _integral = 0;
  _prevError = 0;
}

float PIDController::compute(float error, float dt) {
  if (dt <= 0) dt = 0.001f;

  _integral += error * dt;
  float derivative = (error - _prevError) / dt;
  _prevError = error;

  float out = _kp * error + _ki * _integral + _kd * derivative;
  return constrain(out, _outMin, _outMax);
}