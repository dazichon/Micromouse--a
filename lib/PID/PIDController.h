#pragma once

class PIDController {
public:
  PIDController(float kp = 0, float ki = 0, float kd = 0, float outMin = -255, float outMax = 255);

  void setGains(float kp, float ki, float kd);
  void setOutputLimits(float outMin, float outMax);
  void reset();

  float compute(float error, float dt);   // dt tính bằng giây

private:
  float _kp, _ki, _kd;
  float _outMin, _outMax;
  float _integral = 0;
  float _prevError = 0;
};