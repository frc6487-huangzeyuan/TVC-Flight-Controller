// 本 PCB 的 IMU1 与 IMU2 共用 SDA9/SCL10，IMU1 地址 0x68、INT 接 GPIO8。
// 所有 I2C 访问由 sensorTask 串行执行；中断只记录真实采样事件。
Imu2MonitorState imu1Monitor;
volatile uint64_t imu1IrqTime = 0;
volatile uint32_t imu1IrqCount = 0;
uint32_t imu1Consumed = 0;
uint64_t imu1ReadAfterUs = 0, imu1ProbeAfterUs = 0;
bool imu1IrqAttached = false;

void IRAM_ATTR imu1Irq() {
  portENTER_CRITICAL_ISR(&irqLock);
  imu1IrqTime = esp_timer_get_time();
  ++imu1IrqCount;
  portEXIT_CRITICAL_ISR(&irqLock);
}

void attachImu1Interrupt() {
  if (imu1IrqAttached) return;
  pinMode(IMU1_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU1_INT_PIN), imu1Irq, RISING);
  imu1IrqAttached = true;
}

void beginImu1Monitor() {
  if (!IMU1_LOCAL_I2C) return;
  imu1Monitor.present = initializeImu(IMU_ADDRESS, IMU1_SAMPLE_DIVIDER);
  if (imu1Monitor.present) attachImu1Interrupt();
  else imu1ProbeAfterUs = uint64_t(esp_timer_get_time()) + IMU2_REINIT_INTERVAL_US;
}

void sampleImu1Monitor() {
  if (!IMU1_LOCAL_I2C) return;
  if (!imu1Monitor.present) {
    if (uint64_t(esp_timer_get_time()) < imu1ProbeAfterUs) return;
    imu1ProbeAfterUs = uint64_t(esp_timer_get_time()) + IMU2_REINIT_INTERVAL_US;
    if (!initializeImu(IMU_ADDRESS, IMU1_SAMPLE_DIVIDER)) return;
    xSemaphoreTake(stateLock, portMAX_DELAY);
    imu1Monitor.present = true;
    imu1Monitor.valid = imu1Monitor.tiltValid = false;
    xSemaphoreGive(stateLock);
    imu1Consumed = imu1IrqCount;
    attachImu1Interrupt();
    return;
  }
  if (uint64_t(esp_timer_get_time()) < imu1ReadAfterUs) return;
  uint64_t eventTime;
  uint32_t count;
  portENTER_CRITICAL(&irqLock);
  eventTime = imu1IrqTime;
  count = imu1IrqCount;
  portEXIT_CRITICAL(&irqLock);
  if (count == imu1Consumed) return;
  const uint32_t skipped = count - imu1Consumed - 1;
  imu1Consumed = count;
  uint8_t bytes[14];
  const bool good = readRegisters(IMU_ADDRESS, 0x3b, bytes, sizeof(bytes));
  uint32_t after;
  portENTER_CRITICAL(&irqLock);
  after = imu1IrqCount;
  portEXIT_CRITICAL(&irqLock);
  xSemaphoreTake(stateLock, portMAX_DELAY);
  if (good && after == count)
    imu1Monitor.accept(bytes, eventTime, skipped, IMU2_SENSOR_TO_BODY);
  else
    imu1Monitor.reject(skipped);
  xSemaphoreGive(stateLock);
  if (!good) imu1ReadAfterUs = uint64_t(esp_timer_get_time()) + IMU2_RETRY_DELAY_US;
}
