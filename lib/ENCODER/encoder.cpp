#include "encoder.h"
#include "RobotConfig.h"

Encoder encoder;
Encoder* Encoder::instance = nullptr;

static void IRAM_ATTR isrLeftTrampoline() { if (Encoder::instance) Encoder::instance->isrLeft(); }
static void IRAM_ATTR isrRightTrampoline() { if (Encoder::instance) Encoder::instance->isrRight(); }

void Encoder::begin() {
  instance = this;

  // pull-up: encoder Hall/quang thường ra open-collector, để INPUT thả nổi sẽ không đếm được
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

#if ENC_SINGLE_CHANNEL
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftTrampoline, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightTrampoline, RISING);
#else
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), isrLeftTrampoline, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), isrRightTrampoline, CHANGE);
#endif
}

#if ENC_SINGLE_CHANNEL
  // 1 kênh: mỗi cạnh lên = 1 xung, không biết chiều (quãng đường / góc quay đều lấy trị tuyệt đối)
  static volatile uint32_t lastLeftUs = 0, lastRightUs = 0;

  void IRAM_ATTR Encoder::isrLeft() {
    uint32_t now = micros();
    if (now - lastLeftUs < ENC_DEBOUNCE_US) return;
    lastLeftUs = now;
    _left++;
  }

  void IRAM_ATTR Encoder::isrRight() {
    uint32_t now = micros();
    if (now - lastRightUs < ENC_DEBOUNCE_US) return;
    lastRightUs = now;
    _right++;
  }
#else
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
#endif

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