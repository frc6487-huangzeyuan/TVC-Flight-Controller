// IMU2 原始采集；选中 IMU2 时，sensorTask 将新样本交给姿态估计与 PID。
// 只使用原主控本地 I2C：SDA=9、SCL=10；IMU2 的 AD0=3.3 V、地址=0x69、INT=GPIO8。
// 本模块本身不发送执行器指令。

Imu2MonitorState imu2Monitor;
volatile uint64_t imu2IrqTime = 0;
volatile uint32_t imu2IrqCount = 0;
uint32_t imu2Consumed = 0;
uint64_t imu2ReadAfterUs = 0;
uint64_t imu2ProbeAfterUs = 0;   // 未检测到模块时，下一次重新探测的最早时间。
bool imu2IrqAttached = false;

void IRAM_ATTR imu2Irq() {
  portENTER_CRITICAL_ISR(&irqLock);
  imu2IrqTime = esp_timer_get_time();
  ++imu2IrqCount;
  portEXIT_CRITICAL_ISR(&irqLock);
}

// 只在初始化成功后挂中断；上电时没接好、之后补接的模块也能被重新识别。
void attachImu2Interrupt() {
  if (imu2IrqAttached) return;
  pinMode(IMU2_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU2_INT_PIN), imu2Irq, RISING);
  imu2IrqAttached = true;
}

void beginImu2Monitor() {
  // 初始化发生在 sensorTask 启动之前，此处不存在并发访问 Wire 的问题。
  imu2Monitor.present = initializeImu(IMU2_ADDRESS, IMU2_SAMPLE_DIVIDER);
  if (imu2Monitor.present) attachImu2Interrupt();
  else imu2ProbeAfterUs = uint64_t(esp_timer_get_time()) + IMU2_REINIT_INTERVAL_US;
}

void sampleImu2Monitor() {
  // 仅由 sensorTask 调用，且在主 IMU 数据处理之后执行；Wire 只在这里访问。
  if (!imu2Monitor.present) {
    // 地址探测失败（模块未接、AD0 电平与预期不符等）时按固定间隔重试，
    // 让网页看到“未检测到 → 恢复”的真实过程，而不是停在开机状态。
    if (uint64_t(esp_timer_get_time()) < imu2ProbeAfterUs) return;
    imu2ProbeAfterUs = uint64_t(esp_timer_get_time()) + IMU2_REINIT_INTERVAL_US;
    if (!initializeImu(IMU2_ADDRESS, IMU2_SAMPLE_DIVIDER)) return;
    imu2Monitor.present = true;
    imu2Monitor.valid = false;
    imu2Monitor.tiltValid = false;
    imu2Consumed = imu2IrqCount;  // 丢弃探测前的旧中断计数，只接受之后的新样本。
    attachImu2Interrupt();
    return;
  }
  if (uint64_t(esp_timer_get_time()) < imu2ReadAfterUs) return;
  uint64_t eventTime;
  uint32_t count;
  portENTER_CRITICAL(&irqLock);
  eventTime = imu2IrqTime;
  count = imu2IrqCount;
  portEXIT_CRITICAL(&irqLock);
  if (count == imu2Consumed) return;
  const uint32_t skipped = count - imu2Consumed - 1;
  imu2Consumed = count;
  uint8_t bytes[14];
  const bool good = readRegisters(IMU2_ADDRESS, 0x3b, bytes, sizeof(bytes));
  uint32_t after;
  portENTER_CRITICAL(&irqLock);
  after = imu2IrqCount;
  portEXIT_CRITICAL(&irqLock);
  xSemaphoreTake(stateLock, portMAX_DELAY);
  if (good && after == count)
    imu2Monitor.accept(bytes, eventTime, skipped, IMU2_SENSOR_TO_BODY);
  else
    imu2Monitor.reject(skipped);
  xSemaphoreGive(stateLock);
  // 采集层只有限重试；若 IMU2 是当前控制源，其超时由 safetyTask 统一处理。
  if (!good) imu2ReadAfterUs = uint64_t(esp_timer_get_time()) + IMU2_RETRY_DELAY_US;
}

void appendImu2Telemetry(JsonObject item, uint64_t now) {
  // 调用者已持有 stateLock，此处持锁期间不进行 I2C 访问或通信发送。
  static uint64_t rateAt = 0;
  static uint32_t rateSamples = 0;
  static float sampleHz = 0;
  const auto &sample = imu2Monitor;
  if (!rateAt) { rateAt = now; rateSamples = sample.samples; }
  if (now - rateAt >= IMU2_RATE_WINDOW_US) {
    sampleHz = float(double(uint32_t(sample.samples-rateSamples))*1e6/double(now-rateAt));
    rateAt = now; rateSamples = sample.samples;
  }
  const bool valid = sample.fresh(now, IMU2_TIMEOUT_US);
  item["address"] = IMU2_ADDRESS;
  item["present"] = sample.present;
  item["valid"] = valid;
  // 该字段说明这块模块此刻是否就是固件正在使用的 PID 控制源，由主程序状态决定。
  item["control_source"] = controlSource == SOURCE_IMU2;
  item["status"] = !sample.present ? "NOT_FOUND" : valid ? "OK" :
    sample.errors && !sample.valid ? "READ_ERROR" : sample.samples ? "STALE" : "WAIT_INT";
  item["sample_t_us"] = sample.sampleUs;
  item["sample_age_ms"] = sample.samples && now >= sample.sampleUs ? double(now-sample.sampleUs)/1000 : -1;
  item["sample_timeout_us"] = IMU2_TIMEOUT_US;
  item["target_hz"] = IMU2_SAMPLE_HZ;
  item["sample_hz"] = valid ? roundf(sampleHz*10.0f)/10.0f : 0;
  item["errors"] = sample.errors;
  item["missed"] = sample.missed;
  item["tilt_valid"] = valid && sample.tiltValid;
  auto acceleration=item.createNestedArray("a_g");
  auto gyro=item.createNestedArray("g_dps");
  for(int i=0;i<3;++i){
    if(valid){acceleration.add(sample.accelG[i]);gyro.add(sample.gyroDps[i]);}
    else{acceleration.add(nullptr);gyro.add(nullptr);}
  }
  if (valid) item["temp_c"] = roundf(sample.temperatureC*10.0f)/10.0f;
  else item["temp_c"] = nullptr;
  // 辅助倾角只由重力方向算出，使用的是 IMU2 自己的固定安装矩阵，
  // 仅用于校验显示；它不是 PID 使用的姿态，也不估计偏航角。
  // 选中 IMU2 为控制源时，PID 用的是重新竖直校准得到的安装矩阵。
  if (valid && sample.tiltValid) {
    item["roll_deg"] = roundf(sample.accelRollDeg*10.0f)/10.0f;
    item["pitch_deg"] = roundf(sample.accelPitchDeg*10.0f)/10.0f;
  } else {
    item["roll_deg"] = nullptr; item["pitch_deg"] = nullptr;
  }
}
