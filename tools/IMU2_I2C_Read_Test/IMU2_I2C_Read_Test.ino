// ESP32-S3 本地 IMU2 的 I2C 读取测试，不输出舵机或电机信号。
// SDA = GPIO9，SCL = GPIO10；IMU2 的 AD0 接 3.3 V，地址为 0x69。
// 不使用 INT 中断；上传本程序会暂时替换板上的飞控程序。
#include <Arduino.h>
#include <Wire.h>

constexpr int SDA_GPIO = 9;
constexpr int SCL_GPIO = 10;
constexpr uint8_t IMU_ADDR = 0x69;
constexpr uint32_t PRINT_INTERVAL_MS = 100; // 串口输出 10 次/秒
bool ready = false;
uint32_t lastAttemptMs = 0;
uint32_t lastPrintMs = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *data, size_t count) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IMU_ADDR, uint8_t(count)) != count) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool initializeImu() {
  uint8_t id = 0;
  if (!readRegs(0x75, &id, 1)) {
    Serial.println("无法读取 0x69：检查 SDA、SCL、供电和 AD0；2 秒后重试。");
    return false;
  }
  Serial.printf("WHO_AM_I = 0x%02X（当前飞控要求 0x68）\n", id);
  if (id != 0x68) {
    Serial.println("芯片身份与当前飞控预期不符，停止初始化并重试。");
    return false;
  }
  // 与飞控采用相同的滤波、采样分频和量程；此程序不启用数据就绪中断。
  if (!writeReg(0x6B, 0x80)) return false;
  delay(100);
  if (!writeReg(0x6B, 0x01) || !writeReg(0x1A, 3) ||
      !writeReg(0x19, 4) || !writeReg(0x1B, 0x08) ||
      !writeReg(0x1C, 0x08) || !writeReg(0x38, 0)) {
    Serial.println("IMU 寄存器配置失败，2 秒后重试。");
    return false;
  }
  Serial.println("读取成功；加速度单位 g，角速度单位 deg/s，均为传感器原始坐标轴。");
  return true;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_GPIO, SCL_GPIO, 400000);
  Wire.setTimeOut(20);
  delay(1000);
  Serial.println("IMU2 I2C 读取测试：SDA=GPIO9 SCL=GPIO10 地址=0x69");
  ready = initializeImu();
  lastAttemptMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (!ready) {
    if (now - lastAttemptMs >= 2000) {
      lastAttemptMs = now;
      ready = initializeImu();
    }
    delay(10);
    return;
  }
  if (now - lastPrintMs < PRINT_INTERVAL_MS) { delay(2); return; }
  lastPrintMs = now;
  uint8_t bytes[14];
  if (!readRegs(0x3B, bytes, sizeof(bytes))) {
    Serial.println("读取加速度/角速度失败，重新初始化。");
    ready = false;
    lastAttemptMs = now;
    return;
  }
  int16_t raw[7];
  for (int i = 0; i < 7; ++i) raw[i] = int16_t((uint16_t(bytes[2*i]) << 8) | bytes[2*i+1]);
  Serial.printf("A[g] X=%+.3f Y=%+.3f Z=%+.3f | G[deg/s] X=%+.2f Y=%+.2f Z=%+.2f | T=%.1f C\n",
    raw[0]/8192.0f, raw[1]/8192.0f, raw[2]/8192.0f,
    raw[4]/65.5f, raw[5]/65.5f, raw[6]/65.5f, raw[3]/340.0f+36.53f);
}
