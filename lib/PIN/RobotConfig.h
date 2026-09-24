#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>

// ================= I2C PINOUT =================
#define SDA_PIN 19
#define SCL_PIN 18
#define I2C_SDA SDA_PIN
#define I2C_SCL SCL_PIN

// ================= VL53L0X XSHUT PINS =================
#define XSHUT_LEFT  20
#define XSHUT_FRONT 9
#define XSHUT_RIGHT 8

// ================= VL53L0X I2C ADDRESSES =================
#define ADDRESS_LEFT  0x30
#define ADDRESS_FRONT 0x31
#define ADDRESS_RIGHT 0x32

// ================= DRV8833 MOTOR DRIVER PINS =================
#define M1_IN1 6   // Động cơ trái
#define M1_IN2 7
#define M2_IN1 14  // Động cơ phải
#define M2_IN2 15
#define MOTOR_LEFT_IN1  M1_IN1
#define MOTOR_LEFT_IN2  M1_IN2
#define MOTOR_RIGHT_IN1 M2_IN1
#define MOTOR_RIGHT_IN2 M2_IN2
#define MOTOR_SLEEP_PIN 21

// ================= ENCODER PINS =================
#define ENC_LEFT_A  4
#define ENC_LEFT_B  5
#define ENC_RIGHT_A 16
#define ENC_RIGHT_B 17

// ================= MOBILITY / NAVIGATION CONSTANTS =================
#define WHEEL_RADIUS_CM 3.2f
#define ENC_PULSES_PER_REV 20.0f
#define WALL_THRESHOLD_MM 140
#define MAZE_SIZE 17
#define CELL_LEN_CM 18.0f
#define MAX_SPEED_PWM 180
#define TURN_MIN_PWM 70
#define TURN_MAX_PWM 220
#define CRAWL_DIST_CM 6.0f
#define CRAWL_PWM 80

// ================= MPU6050 I2C ADDRESS =================
#define MPU6050_ADDR 0x68

#endif
