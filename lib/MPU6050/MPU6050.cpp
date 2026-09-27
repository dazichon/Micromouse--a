#include "MPU6050.h"
#include "RobotConfig.h"
#include <math.h>

// Các địa chỉ thanh ghi phần cứng MPU6050
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

MPU6050 gyro(MPU6050_ADDR);

MPU6050::MPU6050(uint8_t addr) {
    _addr = addr;
    _gyroZOffset = 0.0;
    _roll = 0.0;
    _pitch = 0.0;
    _yaw = 0.0;
    _yawSetpoint = 0.0;
    _heading = 0.0;
    _ok = false;
    _lastTime = 0;
}

bool MPU6050::begin() {
    _ok = false;
    // Wire.begin() đã được gọi trong VL53Sensors::begin() (dùng chung bus I2C)
    // 1. Kiểm tra thiết bị phản hồi
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) {
        return false; // Không tìm thấy MPU6050 ở địa chỉ _addr
    }

    // 2. Đánh thức MPU6050 (Power Management 1 = 0)
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    delay(10); // Đợi chip khởi động lại dao động

    // 3. Cấu hình dải đo Gyro: +/- 2000 deg/s (Hệ số nhạy: 16.4 LSB/(deg/s)) để đo chính xác tốc độ quay cao của xe
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_GYRO_CONFIG);
    Wire.write(0x18); 
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // 3b. Bật lọc thông thấp số (DLPF ~44Hz): giảm nhiễu rung động cơ — nhiễu lớn vượt vùng chết
    //     sẽ bị cộng dồn thành trôi góc
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_CONFIG);
    Wire.write(0x03);
    if (Wire.endTransmission() != 0) {
        return false;
    }

    // 4. Cấu hình dải đo Accel: +/- 8g
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_ACCEL_CONFIG);
    Wire.write(0x10); 
    if (Wire.endTransmission() != 0) {
        return false;
    }

    delay(100);   // chờ gyro ổn định sau khi đánh thức — hiệu chuẩn ngay lúc này cho offset sai
    _lastTime = micros();
    _ok = true;
    return true;
}

bool MPU6050::readRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    Wire.beginTransmission(_addr);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) {
        return false; // I2C NACK hoặc bus bận, tránh gọi requestFrom gây Error 259
    }

    uint8_t bytesReceived = Wire.requestFrom(_addr, (uint8_t)14);
    if (bytesReceived != 14) {
        return false; // Không nhận đủ byte dữ liệu
    }

    *ax = (int16_t)((Wire.read() << 8) | Wire.read());
    *ay = (int16_t)((Wire.read() << 8) | Wire.read());
    *az = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read(); // Bỏ qua giá trị nhiệt độ (Temperature)
    *gx = (int16_t)((Wire.read() << 8) | Wire.read());
    *gy = (int16_t)((Wire.read() << 8) | Wire.read());
    *gz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
}

void MPU6050::calibrate(int samples) {
    if (!_ok) return;
    float sumZ = 0;
    int validSamples = 0;
    int16_t ax, ay, az, gx, gy, gz;
    
    // Lưu ý: Khi hiệu chuẩn, robot phải đặt đứng yên hoàn toàn
    for (int i = 0; i < 30; i++) { readRawData(&ax, &ay, &az, &gx, &gy, &gz); delay(3); }   // bỏ mẫu đầu
    for (int i = 0; i < samples; i++) {
        if (readRawData(&ax, &ay, &az, &gx, &gy, &gz)) {
            sumZ += gz;
            validSamples++;
        }
        delay(3);
    }
    
    // Tính toán độ lệch tĩnh (offset) cho trục Z với dải 2000 deg/s (16.4 LSB/(deg/s))
    if (validSamples > 0) {
        _gyroZOffset = (sumZ / validSamples) / 16.4f;
    }
    _roll = 0.0;
    _pitch = 0.0;
    _yaw = 0.0;
    _yawSetpoint = 0.0;
    _heading = 0.0;
    _rateDps = 0.0;
    _lastTime = micros();
}

void MPU6050::update() {
    if (!_ok) return;
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRawData(&ax, &ay, &az, &gx, &gy, &gz)) {
        return; // Đọc I2C thất bại -> bỏ qua vòng này, không tính toán sai góc
    }

    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0; // Đổi từ micro giây sang giây
    if (_lastTime == 0 || dt > 1.0) {
        dt = 0.01;
    }
    _lastTime = now;

    // Góc nghiêng lấy từ gia tốc kế, tính theo độ.
    _roll = atan2((float)ay, (float)az) * 180.0 / PI;
    _pitch = atan2(-(float)ax, sqrt((float)ay * ay + (float)az * az)) * 180.0 / PI;

    // Đổi giá trị thô sang đơn vị độ/giây (deg/s) với hệ số 16.4 (cho dải 2000 deg/s)
    // Cảm biến đặt úp nên đảo chiều quay quanh trục Z.
    float gyroZRate = -((gz / 16.4f) - _gyroZOffset);

    // tốc độ quay đã lọc (EMA) — 1 mẫu nhiễu lẻ không làm hỏng việc nhận biết đứng yên
    _rateDps += 0.1f * (gyroZRate - _rateDps);

    // KHÓA GÓC KHI ROBOT KHÔNG CHẠY: offset gyro đổi theo nhiệt độ / hiệu chuẩn lúc xe rung => sai
    // vài độ/s là góc trôi mãi. Robot không tự chạy/quay (_moving = false) mà tốc độ quay < GYRO_HAND_DPS
    // thì chắc chắn đó là trôi: KHÔNG cộng góc, và kéo offset về giá trị đang đo để hết trôi.
    // Chỉ khi lấy tay xoay xe (nhanh hơn GYRO_HAND_DPS) thì góc mới đổi.
    if (!_moving && fabsf(_rateDps) < GYRO_HAND_DPS) {
        _gyroZOffset += 0.02f * ((gz / 16.4f) - _gyroZOffset);
        return;
    }

    // Lọc nhiễu tĩnh nhỏ (Deadzone) tránh bị trôi góc khi xe đứng yên
    if (fabsf(gyroZRate) < 0.8f) {
        gyroZRate = 0.0f;
    }

    // Tích phân vận tốc góc theo thời gian để ra góc Yaw. _yawScale bù sai số hệ thống (thường do
    // cảm biến gắn hơi nghiêng so với trục quay thật) khiến gyro báo ít/nhiều hơn góc quay thật.
    _yaw += gyroZRate * dt * _yawScale;
    _heading += gyroZRate * dt * _yawScale;
}

float MPU6050::getRoll() const {
    return _roll;
}

float MPU6050::getPitch() const {
    return _pitch;
}

float MPU6050::getYaw() const {
    return _yaw;
}

float MPU6050::getRelativeYaw() const {
    return _yaw - _yawSetpoint;
}

void MPU6050::setYawSetpoint() {
    _yawSetpoint = _yaw;
}

void MPU6050::resetYaw() {
    _yaw = 0.0;
    _yawSetpoint = 0.0;
}