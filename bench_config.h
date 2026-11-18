#pragma once
#include <Arduino.h>

// 引脚表适用于 ESP32-S3-WROOM-1-N16R8，不适用于经典 ESP32-WROOM。
// 接线按实物开发板排针位置分组，方便核对。
// 避开 PSRAM 占用的 GPIO35/36/37、USB 的 GPIO19/20、启动配置脚 0/3/45/46，
// 以及板载 RGB LED 的 GPIO38/48；GPIO9/10/11 为通用引脚。
constexpr int SDA_PIN = 9;
constexpr int SCL_PIN = 10;
constexpr uint8_t IMU_ADDRESS = 0x68;  // 本 PCB 的 IMU1：AD0 接 GND。
constexpr bool IMU1_LOCAL_I2C = true; // 本 PCB 两块 MPU6050 都直接接主控；旧串口采集板可显式改为 false。
constexpr int IMU1_INT_PIN = 8;
constexpr uint8_t IMU1_SAMPLE_DIVIDER = 4;
// 以下 UART 参数仅在 IMU1_LOCAL_I2C=false 的旧双主控接法中使用。
constexpr int PRIMARY_UART_RX = 16; // 原主控 RX16 ← 采集板 S3 SuperMini 的 TX6。
constexpr int PRIMARY_UART_TX = 17; // 原主控 TX17 → 采集板 S3 SuperMini 的 RX7。
constexpr uint32_t PRIMARY_UART_BAUD = 460800;
// IMU2 可在网页中选为显示与 PID 控制源；默认选择 IMU2。
constexpr uint8_t IMU2_ADDRESS = 0x69;  // IMU2 的 AD0 接 3.3 V 高电平，地址为 0x69。
constexpr int IMU2_INT_PIN = 2; // 原理图 MPU2 的 INT 接 GPIO2，GPIO8 属于 MPU1。
// DLPF_CFG=3 时，MPU6050 寄存器输出频率为 1000/(1+SMPLRT_DIV)。
// 控制源编号：1=IMU1（本 PCB 为本地 0x68），2=主控本地 IMU2（0x69）。
constexpr uint8_t SOURCE_IMU1 = 1;
constexpr uint8_t SOURCE_IMU2 = 2;
constexpr uint8_t DEFAULT_CONTROL_SOURCE = SOURCE_IMU2; // 网页默认选择 IMU2，切换后固件实际生效。
// 同一 PCB、同一面、同向安装：两块 IMU 使用相同的机体安装方位。
// 保留 IMU2 原先的 -45° 作为共同参考；绝对机械方位仍需小角度倾斜核对。
// 下面是“PCB 朝外法线相对机体 +X”的方位角，不是给欧拉倾角直接加减的数。
// 从箭头朝尾部俯视，逆时针为正。竖直静止校准不能辨识绕箭体长轴的安装角。
constexpr float IMU2_MOUNT_YAW_DEG = -45.0f;
constexpr float IMU1_MOUNT_YAW_DEG = IMU2_MOUNT_YAW_DEG;
constexpr float controlMountYawDeg(uint8_t source) {
  return source==SOURCE_IMU1 ? IMU1_MOUNT_YAW_DEG : IMU2_MOUNT_YAW_DEG;
}
constexpr uint16_t IMU2_SAMPLE_HZ = 200;
constexpr uint8_t IMU2_SAMPLE_DIVIDER = 4;
static_assert(1000/(1+IMU2_SAMPLE_DIVIDER)==IMU2_SAMPLE_HZ, "IMU2 divider must match its target rate");
constexpr uint32_t IMU2_TIMEOUT_US = 300000;  // 原始监测有效期；控制姿态另有历史计算窗口。
constexpr uint32_t IMU2_RATE_WINDOW_US = 1000000;  // 按一秒窗口统计采样频率。
constexpr uint32_t IMU2_RETRY_DELAY_US = 100000;  // I2C 读取出错后，等待一段时间再重试。
constexpr uint32_t IMU2_REINIT_INTERVAL_US = 3000000; // 未检测到 IMU2 时，每 3 秒重新探测一次地址。
constexpr uint32_t IMU1_RATE_WINDOW_US = 1000000;  // 未被选为控制源时，IMU1 仍按窗口统计采样频率。
static_assert(IMU2_ADDRESS != IMU_ADDRESS, "The two IMUs need different I2C addresses");
static_assert(IMU1_INT_PIN != IMU2_INT_PIN, "Local IMUs need separate interrupt pins");
static_assert(PRIMARY_UART_RX != PRIMARY_UART_TX, "UART pins must differ");
constexpr int SERVO_PINS[2] = {4, 5};
constexpr int ESC_PIN = 6;
constexpr int CG_SERVO_PIN = 7;
constexpr int CG_SERVO_CHANNEL = 3;
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 1000; // 超过此时间仅报告告警，不停止输出。
// 连接会话是否存在，与是否允许驱动执行器是两种独立状态。
constexpr uint32_t SESSION_TIMEOUT_MS = 5000;
constexpr uint32_t WIFI_TELEMETRY_INTERVAL_MS = 40;
constexpr uint32_t SERIAL_TELEMETRY_INTERVAL_MS = 100;
constexpr uint32_t IMU_FRESH_US = 50000;           // 50ms 内视为实时测量
constexpr uint32_t HISTORY_PREDICTION_US = 150000; // 缺帧后最多外推 150ms，再保持估计姿态
constexpr uint32_t IMU_TIMEOUT_US = 2000000;       // 已有有效样本允许使用历史，连续 2 秒无有效样本才停输出
constexpr uint32_t PID_MAX_TICK_GAP_US = 50000;    // 控制任务本身的运行周期限制，与测量过期不是同一概念
// GPIO4：360°位置舵机，控制方位；GPIO5：270°双轴位置舵机，控制俯仰。
// 双轴指同一舵机的同轴输出，并不增加一个独立控制自由度。
// 500～2500us 暂按标称全行程映射，需按实际型号核对。
constexpr uint16_t SERVO_MIN_US = 500;
constexpr uint16_t SERVO_MAX_US = 2500;
constexpr uint16_t SERVO_CENTER_US = 1500;
// 可动重心位置舵机使用脉宽指令，实物角度和机构端点待标定。
constexpr int CG_SERVO_MIN_US = 500;
constexpr int CG_SERVO_MAX_US = 2000;
constexpr int CG_SERVO_INITIAL_US = 1500;
constexpr int CG_SERVO_SLEW_US = 12;     // 按住时每 10 ms 调整 12 µs。
constexpr uint32_t CG_SERVO_JOG_TIMEOUT_MS = 400; // 未收到持续按住指令时保持当前位置。
constexpr bool ELEVATION_SERVO_REVERSED = true; // 尚未保存标定时的默认俯仰方向。
// 两路 PID 对应机体 X、Y 两个方向的倾斜，不是分别直接对应两只舵机。
// X 为公共机构参考方向（视频向右），Y=Z×X；两块 PCB 先按各自安装角转换到该坐标系。
// PID 输出组成二维偏转向量，再转换为“方位角 + 俯仰偏转”。
// PID 输入为“0－当前倾斜角度”；输出为云台修正角度。
// Kp 单位为舵机度/机体度；Ki 单位为舵机度/(机体度×秒)；
// Kd 单位为舵机度×秒/机体度，具体参数需根据实物机构调整。
// 保留原常量命名：PITCH 配置 X 向，ROLL 配置 Y 向。默认值未经过实物调参。
constexpr float AUTO_PITCH_KP = 0.6f;
constexpr float AUTO_PITCH_KI = 0.0f;
constexpr float AUTO_PITCH_KD = 0.03f;
constexpr float AUTO_ROLL_KP = 0.6f;
constexpr float AUTO_ROLL_KI = 0.0f;
constexpr float AUTO_ROLL_KD = 0.03f;
constexpr float AUTO_MAX_CORRECTION_DEG = 15.0f;
constexpr float AUTO_INTEGRAL_LIMIT_DEG = 3.0f;
constexpr float AUTO_D_FILTER_SECONDS = 0.03f;
// 模型控制必须使用本台架的实测参数；默认关闭。矩阵按行排列，
// q¨ = A*q + B*(v-bias)，q 为机体倾角，v 为二维推力纠偏角。
// 标定完成后填入 bias、A、B 的逆矩阵，并重新整定角加速度 PID。
// 全零矩阵不能作为实物标定；误启用时主控将拒绝无效配置。
constexpr bool AUTO_MODEL_ENABLED = false;
constexpr float AUTO_MODEL_BIAS[2] = {0,0};
constexpr float AUTO_MODEL_GRAVITY[4] = {0,0,0,0};
constexpr float AUTO_MODEL_INVERSE[4] = {0,0,0,0};
constexpr float AUTO_MODEL_ACCEL_LIMIT = 15.0f; // 角加速度上限，度/秒²。
constexpr int SERVO_SLEW_US_PER_TICK = 25;  // 每 10 ms 最多变化 25us，即 2500us/s；手动、自动和预定位共用。
// 预定位到达目标 PWM 后额外等待；这是估计时间，不是舵机位置反馈。
constexpr uint32_t PREPOSITION_SETTLE_MS = 2000;
constexpr uint32_t PREPOSITION_TIMEOUT_MS = 20000;
constexpr float PREPOSITION_MAX_MOTION_DEG = 2.0f;
constexpr uint32_t PWM_HZ = 50;
constexpr uint8_t PWM_BITS = 14;
constexpr uint32_t SERIAL_BAUD = 460800;
constexpr uint8_t PROTOCOL_VERSION = 10;
constexpr char FW_VERSION[] = "tvc-gimbal-2.5.3";
constexpr float GIMBAL_AZIMUTH_ZERO_DEG = 180.0f; // 几何方位零位。
constexpr uint16_t DEFAULT_AZIMUTH_SERVO_CENTER_TENTHS=1868;
constexpr uint16_t DEFAULT_ELEVATION_SERVO_CENTER_DEG=175;
constexpr float GIMBAL_VECTOR_X_SIGN = 1.0f;
constexpr float GIMBAL_VECTOR_Y_SIGN = -1.0f; // 按实物回正方向修正机体 Y 轴纠偏。
// 用户提供的竖直静置读数，单位 g。归一化后作为两路默认安装参考。
struct GravityReference {float up[3];float bias[3];};
constexpr float IMU1_DEFAULT_UP_G[3]={0.18f,0.12f,0.95f};
constexpr float IMU2_DEFAULT_UP_G[3]={1.02f,0.12f,-0.07f};
constexpr int IMU1_OUTWARD_AXIS=0; // +Z 接近竖直，采用 +X 的水平投影作方位基准。
constexpr int IMU2_OUTWARD_AXIS=2;
constexpr float EARTH_GRAVITY_MPS2=9.8f;
constexpr int GRAVITY_STARTUP_SAMPLES=8;
// 辅助监测卡片使用此矩阵，当前控制源可使用默认或手动保存的安装参考。
constexpr float IMU2_SENSOR_TO_BODY[3][3] = {{0,0,1},{0,1,0},{-1,0,0}};
constexpr uint64_t COMMAND_MAX_AGE_US = 500000;
