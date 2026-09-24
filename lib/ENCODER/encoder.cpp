#include "encoder.h"
#include "RobotConfig.h"

Encoder encoder;
Encoder* Encoder::instance = nullptr;

static void IRAM_ATTR isrLeftTrampoline() { if (Encoder::instance) Encoder::instance->isrLeft(); }
static void IRAM_ATTR isrRightTrampoline() { if (Encoder::instance) Encoder::instance->isrRight(); }

void Encoder::begin() {
  instance = this;

  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftTrampoline, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightTrampoline, CHANGE);
}

void IRAM_ATTR Encoder::isrLeft() {
  int a = digitalRead(ENC_LEFT_A);
  int b = digitalRead(ENC_LEFT_B);
  if (a != b) _left--; else _left++;
}

void IRAM_ATTR Encoder::isrRight() {
  int a = digitalRead(ENC_RIGHT_A);
  int b = digitalRead(ENC_RIGHT_B);
  if (a != b) _right++; else _right--;
}

void Encoder::reset() {
  noInterrupts();
  _left = 0;
  _right = 0;
  interrupts();
}

long Encoder::left() const  { return _left; }
long Encoder::right() const { return _right; }

int Encoder::spinCount() const {
  noInterrupts();
  long l = _left, r = _right;
  interrupts();
  return (abs(l) + abs(r)) / 2;
}

float Encoder::distanceLeftCm() const {
  float circumference = 2.0f * WHEEL_RADIUS_CM * PI;
  return (left() / ENC_PULSES_PER_REV) * circumference;
}

float Encoder::distanceRightCm() const {
  float circumference = 2.0f * WHEEL_RADIUS_CM * PI;
  return (right() / ENC_PULSES_PER_REV) * circumference;
}

float Encoder::distanceAvgCm() const {
  return (fabs(distanceLeftCm()) + fabs(distanceRightCm())) / 2.0f;
}