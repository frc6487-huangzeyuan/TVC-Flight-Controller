// MotorControl.ino：两个舵机和一个无刷电调的 PWM 输出层。
// 这是 TVC_Electronics 工程的一个标签页，不是另一块主控的独立程序。
// 不要单独运行本文件，也不要直接调用 pulse() 绕过主程序的控制条件。
//
// 通道 0 → GPIO4 → 360°方位位置舵机；通道 1 → GPIO5 → 270°俯仰位置舵机。
// 通道 2 → GPIO6 → 电调细信号线；电调信号地需与 ESP32 的 GND 共地。
// 电调的 U/V/W 三根粗线接电机，不能接到 ESP32 的 GPIO。
// 电调负责驱动电机三相，ESP32 只通过 PWM 发送油门指令。
// 舵机使用外部供电；不要将电调 BEC 电源线接到 GPIO 或 3V3 引脚。
//
// 使用 ESP32 Arduino 2.x 的 LEDC，频率 50 Hz、分辨率 14 位、周期 20000 微秒。
// 当前已验证的双向电调以 1500 微秒为中位停止值，
// 最小值、中位值和最大值仍须由网页在电调解锁前明确配置。
// 状态机、解锁、心跳和停止逻辑统一由 TVC_Electronics.ino 管理。

bool pwmAttached[3] = {false, false, false};

int pwmPin(int channel) { return channel == 2 ? ESC_PIN : SERVO_PINS[channel]; }
void pulse(int channel, int us) {
  if(channel<2)us=limitServoUs(channel,us); // 最后一道硬件输出边界，俯仰始终在中位 ±50°内。
  if (!pwmAttached[channel]) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if(!ledcAttachChannel(pwmPin(channel),PWM_HZ,PWM_BITS,channel)){actuatorPwmFault=true;return;}
#else
    ledcSetup(channel, PWM_HZ, PWM_BITS);
    ledcAttachPin(pwmPin(channel), channel);
#endif
    pwmAttached[channel] = true;
  }
  const uint32_t duty=uint32_t((uint64_t(us)*(1u<<PWM_BITS)+10000)/20000);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  if(!ledcWriteChannel(channel,duty))actuatorPwmFault=true;
#else
  ledcWrite(channel,duty);
#endif
}
void disablePulse(int channel) {
  if (pwmAttached[channel]) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWriteChannel(channel,0);
    ledcDetach(pwmPin(channel));
#else
    ledcWrite(channel, 0);
    ledcDetachPin(pwmPin(channel));
#endif
    pwmAttached[channel] = false;
  }
  pinMode(pwmPin(channel), OUTPUT);
  digitalWrite(pwmPin(channel), LOW);
}
