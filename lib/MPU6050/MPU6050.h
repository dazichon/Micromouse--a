#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// Driver MPU6050 lấy từ project mm: đọc thanh ghi trực tiếp qua I2C, gyro ±2000°/s,
// chỉ tích phân trục Z (yaw). Không cần thư viện Adafruit.
class MPU6050 {
public:
    MPU6050(uint8_t addr = 0x68);
    bool begin();
    void calibrate(int samples = 300);
    void update();

    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    float getAngleZ() const { return getYaw(); }
    float getRelativeYaw() const;
    void setYawSetpoint();
    void resetYaw();

    // góc hướng liên tục (độ) cho đồng hồ trên dashboard — KHÔNG bị resetYaw() đưa về 0
    float headingDeg() const { return _heading; }
    void resetHeading() { _heading = 0.0f; }
    bool ok() const { return _ok; }

    // Robot đang chạy/quay thì báo true: tắt tự bù trôi (chỉ bù khi xe đứng yên)
    void setMoving(bool moving) { _moving = moving; }
    float biasDps() const { return _gyroZOffset; }
    float rateDps() const { return _rateDps; }   // tốc độ quay hiện tại (đã trừ offset) — để chẩn đoán trôi

private:
    uint8_t _addr;
    float _gyroZOffset;
    float _roll;
    float _pitch;
    float _yaw;
    float _yawSetpoint;
    float _heading;
    bool _ok;
    unsigned long _lastTime;
    bool _moving = false;
    float _rateDps = 0.0f;   // tốc độ quay Z đã trừ offset và lọc (°/s)

    bool readRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
};

extern MPU6050 gyro;

#endif
