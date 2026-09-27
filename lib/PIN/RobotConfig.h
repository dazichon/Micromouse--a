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
// Tốc độ đo VL53L0X: thời gian cho 1 lần đo (us) — càng lớn thì cảm biến gom trung bình nội bộ
// nhiều hơn => ít nhiễu/ít giá trị rác hơn (đặc biệt lúc đứng im hoặc dừng giữa các ô ở Chế độ 2),
// nhưng đo chậm hơn => vòng lặp PID (DRIVE_LOOP_MS, tính từ giá trị này) cũng chậm theo.
// Nhỏ quá thư viện từ chối => robot tự quay về 20000 và báo trên Serial.
//   33000 = ~30Hz (mặc định thư viện, ổn định nhất), 20000 = 50Hz, 15000 = ~66Hz, 10000 = 100Hz
#define VL53_TIMING_BUDGET_US 33000
// Lọc nhiễu VL53L0X: raw -> median-3 (loại nhiễu xung/spike) -> EMA (làm mượt).
// Alpha càng lớn -> bám sát mẫu mới càng nhanh nhưng càng ít mượt (0 < alpha <= 1).
#define TOF_EMA_ALPHA 0.35f
// Mẫu raw >= ngưỡng này coi là "ngoài tầm/không thấy vật" (KHÔNG lọc median/EMA cùng mẫu gần —
// xem VL53Sensors::poll()). Chọn lớn hơn mọi bề rộng hành lang mê cung thực tế.
#define TOF_VALID_RANGE_MAX_MM 1200
// Cần bấy nhiêu mẫu GẦN/XA liên tiếp mới đổi trạng thái — chống cảm biến chớp nháy giữa 1 mẫu đo
// đúng và 1 mẫu "ngoài tầm" (từng thấy trên VL53L0X khi vật ở mép tầm đo / nhiễu I2C).
// CỐ Ý LỆCH: tin mẫu GẦN ngay (1 mẫu, có tường/vật là phải biết ngay kẻo đâm), chỉ chốt XA sau
// vài mẫu liên tiếp — nếu không, XA và GẦN chớp nháy đúng 1-1 (trường hợp lỗi cảm biến thực tế)
// sẽ không bao giờ chốt được trạng thái nào (cả 2 phía đều bị mẫu đối lập xen kẽ làm mất streak).
#define TOF_FAR_CONFIRM_SAMPLES  4
#define TOF_NEAR_CONFIRM_SAMPLES 1
// Chu kỳ vòng PID khi chạy thẳng (ms) — bằng thời gian đo VL53 để mỗi vòng có mẫu mới
#define DRIVE_LOOP_MS ((VL53_TIMING_BUDGET_US + 999) / 1000)
#define WALL_THRESHOLD_MM 230  // < 230mm là có tường, > 230mm là cửa trống (đo thực tế, lấy từ project mm)
// Khoảng cách đo được tại TÂM Ô khi có tường 2 bên (lấy từ project mm) — PID bám tường giữ robot ở đây
#define WALL_TARGET_LEFT_MM   170.0f
#define WALL_TARGET_RIGHT_MM  149.0f
#define WALL_CENTER_OFFSET_MM (WALL_TARGET_LEFT_MM - WALL_TARGET_RIGHT_MM)
// Dùng làm ngưỡng "đã thẳng giữa ô" trong alignInCell() (Chế độ 2 - đi từng ô) — KHÔNG dùng để
// triệt sai số nhỏ trong PID bám tường (wallFollowCorrection() để y hệt project mm, không deadband).
#define WALL_ERR_DEADBAND_MM  5.0f
// ================= PID BÁM TƯỜNG — theo đúng project mm =================
// EMA làm mượt khoảng cách 2 bên trước khi tính sai số: còn trong tầm thì bám nhanh (alpha lớn);
// mất tường/lóa thì trôi CHẬM về giá trị "coi như trống" (999) để không bị giật khi mất/được tường.
#define WALL_SMOOTH_ALPHA_NEAR 0.4f
#define WALL_SMOOTH_ALPHA_FAR  0.15f
#define WALL_SMOOTH_FAR_MM     999.0f
// Trộn thêm sai số đồng bộ 2 bánh (chênh lệch xung encoder GIỮA 2 LẦN GỌI GẦN NHẤT — không phải
// cộng dồn từ đầu — đi thẳng đều thì lệch này ≈0) vào sai số bám tường trước khi đưa vào PID —
// bù trôi do 1 bánh/motor yếu hơn mà cảm biến tường (đo mm, cập nhật chậm hơn encoder) chưa kịp
// phát hiện. final_error = W1*wall_error(mm) + W2*(deltaEncL - deltaEncR).
#define WALL_ENC_BLEND_W1 0.7f    // trọng số sai số bám tường (mm)
#define WALL_ENC_BLEND_W2 0.35f   // trọng số sai số đồng bộ encoder (xung/vòng lặp)
// Không có tường 2 bên (ngã tư/ô trống): giữ thẳng bằng gyro + encoder thay vì chỉ 1 nguồn.
#define NOWALL_GYRO_WEIGHT 0.6f
#define NOWALL_ENC_WEIGHT  0.4f
// Lọc thông thấp lần 2 trên sai số ĐÃ TRỘN (chống giật do khe nứt tường / gai nhọn tức thời),
// rồi kẹp trần trước khi đưa vào PID — PID Kp/Ki/Kd chỉnh ở tab Chế độ 2 áp dụng SAU 2 bước này.
#define WALL_ERR_LOWPASS_ALPHA 0.3f
#define WALL_ERR_MAX_MM        20.0f
#define FRONT_STOP_MM 100     // tường trước gần hơn mức này => dừng ngay dù encoder chưa đủ quãng (lấy từ project mm)
#define FRONT_SLOW_MM 150     // tường trước gần hơn mức này => chạy chậm (CRAWL_PWM) để kịp phanh
// Chế độ 2 (đi từng ô): sau khi dừng cuối đoạn, tự xoay chỉnh tại chỗ theo sai số bám tường
// (dùng chung WALL_ERR_DEADBAND_MM làm ngưỡng "đã thẳng") cho tới khi thẳng giữa ô rồi mới đi tiếp.
#define ALIGN_TIMEOUT_MS 1500  // quá thời gian này vẫn chưa canh xong thì bỏ qua, đi tiếp luôn
#define ALIGN_NUDGE_MS   30    // mỗi lần nhích quay bao lâu (ms)
#define ALIGN_SETTLE_MS  120   // đứng yên hẳn bao lâu trước khi đo lại (tránh rung làm ToF sai)
#define ALIGN_PWM_MIN    45    // PWM nhích tối thiểu để bánh thực sự quay (thắng ma sát tĩnh)
#define ALIGN_PWM_MAX    90
// Độ dài 1 ô = SỐ XUNG encoder (trung bình 2 bánh), KHÔNG quy ra cm — chỉnh live ở Chế độ 3 (SET_CELL),
// robot lưu vào flash. Đi được nửa số xung này thì vị trí trên web sang ô mới.
#define CELL_LEN_PULSES     100    // MẶC ĐỊNH
#define CELL_LEN_MIN_PULSES 30
#define CELL_LEN_MAX_PULSES 300
#define CRAWL_CELL_FRAC     0.33f  // giảm tốc trong 1/3 ô cuối
// Explore: sau khi sang ô mới, dừng hẳn bấy nhiêu ms trước khi đo tường/đi tiếp
// — cảm biến hết rung/ổn định số đo, web kịp hiển thị vị trí ô mới.
#define CELL_PAUSE_MS 400
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
// Hệ số hiệu chỉnh góc gyro: gyro báo ÍT hơn góc quay THẬT (vd. quay thật 110° mà gyro báo 90°)
// => tăng hệ số này (vd. 110/90 ≈ 1.22); báo NHIỀU hơn thật thì giảm. Cách chỉnh: quay robot 1 góc
// đã biết (vd. quay tay/động cơ nhiều vòng liên tiếp), đo góc THẬT bằng thước đo góc/đánh dấu sàn,
// so với góc gyro báo (ô "ĐÃ QUAY" ở Chế độ 1), rồi chỉnh SET_YAWSCALE = góc thật / góc gyro báo
// cho tới khi 2 số khớp nhau. Nguyên nhân thường gặp: cảm biến gắn hơi nghiêng so với trục quay
// thật của xe (trục Z không thẳng đứng) => quay tại chỗ chỉ "thấy" 1 phần góc thật.
#define GYRO_YAW_SCALE_DEFAULT 1.0f

#endif
