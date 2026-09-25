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

// Đảo chiều quay động cơ (1 = đảo) — dùng khi dây motor đấu ngược,
// để speed dương luôn là TIẾN.
#define MOTOR_LEFT_INVERT  1
#define MOTOR_RIGHT_INVERT 1

// ================= ENCODER PINS =================
#define ENC_LEFT_A  4
#define ENC_LEFT_B  5
#define ENC_RIGHT_A 16
#define ENC_RIGHT_B 17

// 1 = encoder 1 KÊNH (đĩa 20 khe + LM393, chỉ có chân A): mỗi cạnh lên của A = +1 xung.
//     Code quadrature cũ đọc cả chân B — B bỏ trống bị pull-up => +1 rồi -1 => tổng luôn ~0,
//     robot tưởng chưa đi được gì nên chạy mãi và vị trí trên web không đổi.
//     Chế độ này dùng được cho cả encoder 2 kênh (chỉ mất chiều quay — code chạy không cần chiều).
// 0 = encoder 2 kênh A/B (quadrature) đếm có chiều.
#define ENC_SINGLE_CHANNEL 1
#define ENC_DEBOUNCE_US    50    // bỏ các cạnh cách nhau < 50us (nhiễu dội) — đủ nhỏ để không mất xung khi chạy nhanh

// Đảo dấu đếm encoder (1 = đảo) — chỉ dùng khi ENC_SINGLE_CHANNEL = 0 — chỉnh sao cho robot đi TIẾN thì cả 2 bánh đếm DƯƠNG.
#define ENC_LEFT_INVERT  0
#define ENC_RIGHT_INVERT 0

// ================= KÍCH THƯỚC MÊ CUNG =================
// Chỉnh được trên web (tab Bản đồ) — robot lưu vào flash. Đây chỉ là giá trị MẶC ĐỊNH lần đầu.
// Ô đánh số từ 1: hàng 1 = trên cùng, cột 1 = trái cùng. Robot xuất phát ở ô (1,1), hướng EAST.
//   Mê cung 5x5  : MAZE_CELLS 5,  GOAL_ROW 3, GOAL_COL 3, GOAL_SIZE 1  (đích = ô giữa)
//   Mê cung 16x16: MAZE_CELLS 16, GOAL_ROW 8, GOAL_COL 8, GOAL_SIZE 2  (đích = 4 ô giữa)
#define MAZE_CELLS 5
#define GOAL_ROW   3      // ô trên-trái của vùng đích
#define GOAL_COL   3
#define GOAL_SIZE  1      // vùng đích GOAL_SIZE x GOAL_SIZE ô

#define MAZE_MAX_CELLS 16                       // mảng cấp phát sẵn cho mê cung lớn nhất
#define MAZE_MAX_SIZE  (2 * MAZE_MAX_CELLS + 1) // lưới bàn cờ 2 lớp ô/tường — KHÔNG sửa

// ================= MOBILITY / NAVIGATION CONSTANTS =================
#define WHEEL_RADIUS_CM 3.2f
#define WHEEL_BASE_CM 8.0f      // khoảng cách tâm 2 bánh — ĐO LẠI trên robot (dùng khi quay bằng encoder)
#define ENC_PULSES_PER_REV 20.0f
#define WALL_THRESHOLD_MM 230   // < 230mm là có tường, > 230mm là cửa trống (đo thực tế, lấy từ project mm)
// Khoảng cách đo được tại TÂM Ô khi có tường 2 bên (lấy từ project mm) — PID bám tường giữ robot ở đây
#define WALL_TARGET_LEFT_MM   170.0f
#define WALL_TARGET_RIGHT_MM  149.0f
#define WALL_CENTER_OFFSET_MM (WALL_TARGET_LEFT_MM - WALL_TARGET_RIGHT_MM)
#define WALL_ERR_DEADBAND_MM  5.0f    // lệch dưới 5mm coi như đang đi thẳng
#define FRONT_STOP_MM 60      // tường trước gần hơn mức này => dừng ngay dù encoder chưa đủ quãng
// Độ dài 1 ô = SỐ XUNG encoder (trung bình 2 bánh), KHÔNG quy ra cm — chỉnh live ở Chế độ 3 (SET_CELL),
// robot lưu vào flash. Đi được nửa số xung này thì vị trí trên web sang ô mới.
#define CELL_LEN_PULSES     100    // MẶC ĐỊNH
#define CELL_LEN_MIN_PULSES 30
#define CELL_LEN_MAX_PULSES 300
#define CRAWL_CELL_FRAC     0.33f  // giảm tốc trong 1/3 ô cuối
#define ENC_STALL_MS 1500      // đang chạy mà encoder không tăng quá lâu => dừng, báo lỗi encoder
#define MAX_SPEED_PWM 60       // tốc độ chạy thẳng mặc định (chỉnh live 0-160 ở Chế độ 3: SET_DSPD)
#define DRIVE_SPEED_MAX 160    // giới hạn trên của thanh chỉnh tốc độ
#define TURN_MIN_PWM 70
#define TURN_MAX_PWM 220
#define TURN_DEFAULT_PWM 130   // tốc độ quay mặc định (chỉnh live: SET_SPD)
#define TURN_SLOW_DEG 40.0f    // còn cách đích bấy nhiêu độ thì bắt đầu giảm tốc
#define TURN_TOL_DEG 2.0f      // sai số chấp nhận; lố/thiếu hơn => quay bù chậm
#define TURN_LEFT_COMP_DEG  2.0f   // ngắt động cơ sớm hơn bấy nhiêu độ để bù quán tính (chỉnh live: SET_LC)
#define TURN_RIGHT_COMP_DEG 2.0f   // (chỉnh live: SET_RC)
#define CRAWL_PWM 80

// ================= MPU6050 I2C ADDRESS =================
#define MPU6050_ADDR 0x68
// Dấu gyro Z so với chiều quay: driver MPU6050 (từ mm) đã đảo trục Z vì cảm biến đặt úp
// => quay PHẢI cho yaw DƯƠNG => +1.
// Chỉ là giá trị ban đầu — robot tự học lại dấu sau mỗi lần quay bằng gyro.
#define GYRO_Z_SIGN_DEFAULT 1.0f
// Robot KHÔNG tự chạy/quay mà gyro báo chậm hơn mức này => coi là trôi, giữ nguyên góc và tự chỉnh offset.
// Lấy tay xoay xe nhanh hơn mức này thì đồng hồ trên web vẫn quay theo.
#define GYRO_HAND_DPS 10.0f

#endif
