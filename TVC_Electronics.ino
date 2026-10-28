// TVC 实验平台：两路 MPU6050 都可以作为 PID 控制源，由网页明确切换。
//   IMU1：采集板 ESP32-S3 SuperMini 读取 0x68，经 UART1 发给原主控。
//   IMU2：直接接原主控 SDA9/SCL10，AD0=3.3V，地址 0x69，INT=GPIO8。
// 未被选中的那一路继续采样，只作为校验数据，不参与姿态估计和 PID。
// 在 Arduino IDE 中打开本文件，MotorControl.ino 和 bench_config.h 必须放在同一目录。
// 构建依赖：Espressif ESP32 开发板包 3.3.12、ArduinoJson 7.4.3、WebSockets 2.7.2。
// 原主控选 ESP32S3 Dev Module，QIO 80 MHz，Flash 16 MB，OPI PSRAM。
// 使用原生 USB 时启用 USB CDC On Boot；使用 USB 转串口接口时按对应接口配置。
// Wi-Fi 热点模式在 http://192.168.4.1 提供控制台，通信协议版本为 10。
// 上电时输出保持关闭，不自动扫油门，也不自动校准电调。

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include "PrimaryUartProtocol.h"
#include <esp_timer.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "bench_config.h"
#include "Attitude.h"
#include "AutoLevelPid.h"
#include "GimbalControl.h"
#include "StartupPreposition.h"
#include "HistoryAttitude.h"
#include "TiltThrottle.h"
#include "ImuState.h"
#include "NetworkTransport.h"
#include "Imu2MonitorState.h"
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION_MAJOR != 2 && ESP_ARDUINO_VERSION_MAJOR != 3
#error "Use esp32 by Espressif Systems 2.x or 3.x."
#endif

// 在 MotorControl.ino 标签页中实现，通道 0/1 对应舵机，通道 2 对应电调。
void pulse(int channel, int us);
void disablePulse(int channel);
void beginImu2Monitor();
void sampleImu2Monitor();
void appendImu2Telemetry(JsonObject item, uint64_t now);
void beginPrimaryUart();
void printPrimaryDebug();
bool pollPrimaryUart(PrimaryUart::Sample &sample);
extern Imu2MonitorState imu2Monitor;

Imu primaryImu(IMU_ADDRESS); // IMU1 的独立采集缓存，未选中时仅作为校验数据
uint8_t controlSource=DEFAULT_CONTROL_SOURCE;
Imu imu(DEFAULT_CONTROL_SOURCE==1?IMU_ADDRESS:IMU2_ADDRESS); // 当前选中源的控制状态
// IMU2 通道已被控制逻辑消费到的样本计数，与 imu2Consumed（I2C 读取计数）分开。
// 切换控制源时把它对齐到最新计数，避免把切换前缓存的老样本当成新测量。
uint32_t imu2ControlConsumed=0;
Attitude attitude;
Attitude controlAttitude;
uint64_t lastControlMeasurementUs=0;
bool worldReady = false, correctionActive = false;
bool mountingCalibrated = false, gimbalTiltValid = false;
float mountingMatrix[3][3]={{1,0,0},{0,1,0},{0,0,1}};
uint32_t referenceId = 0;
uint64_t referenceTime = 0;
float bodyA[3] = {0,0,0}, bodyG[3] = {0,0,0};
double accelSum[3] = {0,0,0}, accelSumSq[3] = {0,0,0}, gyroSumSq[3] = {0,0,0};
uint32_t calibrationRejectedSamples = 0;
// -2 表示无控制端，-1 表示 USB，非负数表示 WebSocket 客户端；仅由 loop 任务修改。
int controlOwner = -2;
SemaphoreHandle_t stateLock;
portMUX_TYPE irqLock = portMUX_INITIALIZER_UNLOCKED;
uint32_t primaryReplies=0,primaryBadFrames=0,primaryRejected=0,primaryTimeouts=0;
uint32_t primaryRemoteBoot=0;
bool initialControlCalibrationPending=true;
char bootId[17], sessionId[33] = "";
const char *fault = "";
bool heartbeatWarning=false; // 心跳超时只告警，不属于会停止执行器的 fault。
uint32_t lastCommandSeq = 0, txSeq = 0, heartbeatAt = 0, controllerSeenAt = 0, epoch = 0;
bool servoEnabled = false, escConfigured = false, escArmed = false, escArming = false, autoEnabled = false;
bool escSignalEnabled = false;
StartupPreposition startupPreposition;
bool prepositionOwnedServos=false,prepositionResumeAuto=false;
uint32_t prepositionAt=0;
bool actuatorPwmFault = false;
bool autoMotorAllowed = false; // 每次会话都需明确勾选授权，此标志本身不会启动电调。
bool tiltThrottleEnabled=false;
TiltThrottle tiltThrottle;
bool calibrated = false, calibrating = false;
enum OperatingMode : uint8_t { MODE_ATTITUDE, MODE_MANUAL };
OperatingMode operatingMode = MODE_ATTITUDE;
uint32_t calibrationAt = 0, armAt = 0;
int servoTarget[2] = {1500, 1500}, servoActual[2] = {1500, 1500};
int escStop = 0, escMin = 0, escMax = 0, escTarget = 0, escActual = 0;
float autoZeroRoll = 0, autoZeroPitch = 0, autoErrorRoll = 0, autoErrorPitch = 0;
GimbalPid automaticPid({
  {AUTO_PITCH_KP,AUTO_PITCH_KI,AUTO_PITCH_KD},
  {AUTO_ROLL_KP,AUTO_ROLL_KI,AUTO_ROLL_KD},
  AUTO_MAX_CORRECTION_DEG,AUTO_INTEGRAL_LIMIT_DEG,AUTO_D_FILTER_SECONDS,
  GIMBAL_AZIMUTH_ZERO_DEG,GIMBAL_VECTOR_X_SIGN,GIMBAL_VECTOR_Y_SIGN,
  SERVO_SLEW_US_PER_TICK,PID_MAX_TICK_GAP_US
});
uint32_t armHold = 3000, configRevision = 0, badFrames = 0;

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t c = 0xffffffff;
  for (size_t i = 0; i < length; ++i) {
    c ^= data[i];
    for (int bit = 0; bit < 8; ++bit) c = (c >> 1) ^ ((c & 1) ? 0xedb88320 : 0);
  }
  return c ^ 0xffffffff;
}

// 调用者已持有 stateLock；当前电子平台测试方案会关闭舵机 PWM 输出。
// 安装实物机构后，可能需要根据测试结果选择不同的保持位置或回中策略。
void stopEscOutput();
void stopServoOutput() {
  if(tiltThrottleEnabled)stopEscOutput(); // 联动油门不能在姿态回中已停止时继续运行。
  autoEnabled = false; servoEnabled = false;
  automaticPid.reset();
  disablePulse(0); disablePulse(1);
}
void stopEscOutput() {
  tiltThrottleEnabled=false;tiltThrottle.reset();
  if(startupPreposition.active()){
    startupPreposition.cancel();
    if(prepositionOwnedServos){autoEnabled=false;servoEnabled=false;disablePulse(0);disablePulse(1);}
    else if(servoEnabled && prepositionResumeAuto){automaticPid.reset();autoEnabled=true;}
  }
  prepositionOwnedServos=prepositionResumeAuto=false;
  escArmed = escArming = false;
  if (escConfigured && escSignalEnabled) { escActual = escTarget = escStop; pulse(2, escStop); }
  else { escActual = escTarget = 0; disablePulse(2); }
}
void stopOutputs() { ++epoch; autoMotorAllowed=false; stopServoOutput(); stopEscOutput(); }
void latchFault(const char *reason) {
  if (!fault[0]) { fault = reason; stopOutputs(); }
}
bool freshHeartbeat() { return sessionId[0] && millis() - heartbeatAt < HEARTBEAT_TIMEOUT_MS; }
bool imuUsable(uint64_t now) {
  return imuSampleUsable(imu,now,IMU_TIMEOUT_US);
}
bool imuFresh(uint64_t now) { return imuUsable(now) && now-imu.t<=IMU_FRESH_US; }
bool recoverableImuFault() {
  return !fault[0] || !strcmp(fault,"CALIBRATION_MOVED") || !strcmp(fault,"CALIBRATION_DATA") ||
    !strcmp(fault,"HEADING_UNDEFINED") || !strcmp(fault,"IMU_TIMEOUT") ||
    !strcmp(fault,"ATTITUDE_GAP") || !strcmp(fault,"ATTITUDE_INVALID") || !strcmp(fault,"IMU_RESTART") ||
    !strcmp(fault,"MOUNT_CALIBRATION") || !strcmp(fault,"THROTTLE_INVALID") ||
    !strcmp(fault,"PREPOSITION_MOVED") || !strcmp(fault,"PREPOSITION_TIMEOUT");
}
bool motorModeAllowed() {
  return operatingMode==MODE_MANUAL || (operatingMode==MODE_ATTITUDE && autoMotorAllowed && worldReady);
}
bool commandNeedsEpoch(const char *command) {
  return !strcmp(command,"auto_motor_enable") || !strcmp(command,"servo_enable") || !strcmp(command,"auto_enable") ||
    !strcmp(command,"esc_config") || !strcmp(command,"esc_arm") ||
    !strcmp(command,"servo") || !strcmp(command,"esc") ||
    !strcmp(command,"set_mode") || !strcmp(command,"calibrate") ||
    !strcmp(command,"world_zero") || !strcmp(command,"clear_fault") || !strcmp(command,"pid_config") ||
    !strcmp(command,"select_imu") || !strcmp(command,"tilt_throttle_config");
}
bool controllerLeaseExpired() {
  return controlOwner == -1 && millis()-controllerSeenAt >= SESSION_TIMEOUT_MS;
}
const char *operatingModeName() { return operatingMode == MODE_MANUAL ? "MANUAL" : "ATTITUDE"; }

// 调用者已持有 stateLock；姿态模式会重新建立静止参考坐标系，
// 此过程不会启用任何执行器输出。
bool beginCalibrationLocked() {
  if (servoEnabled || escArmed || escArming || !imu.present || !recoverableImuFault()) return false;
  autoMotorAllowed=false;
  ++epoch;
  automaticPid.reset();
  for(int k=0;k<3;++k){imu.biasSum[k]=0;accelSum[k]=0;accelSumSq[k]=0;gyroSumSq[k]=0;}
  calibrationRejectedSamples=0;
  imu.calibrationSamples=0;imu.calibrated=false;worldReady=false;attitude.invalidate();correctionActive=false;
  // 只有输出已停止时，显式重新校准才能恢复传感器跟踪类故障，
  // 此处不清除配置或网络等其他故障。
  fault="";
  calibrated=false;mountingCalibrated=false;gimbalTiltValid=false;calibrating=true;calibrationAt=millis();
  return true;
}

const char *controlSourceName(uint8_t source) { return source==SOURCE_IMU1 ? "IMU1" : "IMU2"; }
uint8_t controlSourceAddress(uint8_t source) { return source==SOURCE_IMU1 ? IMU_ADDRESS : IMU2_ADDRESS; }
const char *controlSourceLabel(uint8_t source) {
  return source==SOURCE_IMU1 ? "IMU1 · SuperMini · 0x68" : "IMU2 · 本地 I2C · 0x69";
}
const char *controlSourceInterface(uint8_t source) {
  return source==SOURCE_IMU1 ? "SUPERMINI_UART" : "LOCAL_I2C_IMU2";
}
// 该路传感器此刻是否有可用的采集通道。只看“模块是否存在”，
// 不因为暂时没有新样本就拒绝切换，采样年龄由遥测如实报告。
bool controlSourcePresent(uint8_t source) {
  return source==SOURCE_IMU1 ? primaryImu.present : imu2Monitor.present;
}

// 调用者持有 stateLock。切换只改变数据源，不会重新启动任何输出。
// 返回 false 表示目标传感器不存在：此时保持原控制源不变，也不清除故障，
// 由网页显示真实的传感器状态，而不是悄悄换到另一块。
bool selectControlImuLocked(uint8_t source) {
  if(source!=SOURCE_IMU1 && source!=SOURCE_IMU2) return false;
  if(!controlSourcePresent(source)) return false;
  // 先停止执行器并复位 PID：旧来源的积分、微分历史和参考姿态都不能带到新来源。
  stopOutputs();
  const bool sensorFault=recoverableImuFault();
  controlSource=source;
  imu=Imu(controlSourceAddress(source));
  imu.present=true;
  imu2ControlConsumed=imu2Monitor.samples;
  worldReady=calibrated=calibrating=mountingCalibrated=gimbalTiltValid=false;
  attitude=Attitude();controlAttitude=Attitude();
  correctionActive=false;lastControlMeasurementUs=0;
  autoZeroRoll=autoZeroPitch=autoErrorRoll=autoErrorPitch=0;
  for(int i=0;i<3;++i){bodyA[i]=bodyG[i]=0;accelSum[i]=accelSumSq[i]=gyroSumSq[i]=0;}
  calibrationRejectedSamples=0;
  referenceTime=esp_timer_get_time();++referenceId;
  if(sensorFault)fault="";
  // 姿态模式下，新来源的第一份样本会触发一次新的竖直静止校准。
  initialControlCalibrationPending=true;
  return true;
}

void toBody(const float input[3], float output[3]) {
  for(int r=0;r<3;++r){output[r]=0;for(int c=0;c<3;++c)output[r]+=mountingMatrix[r][c]*input[c];}
}
bool mountingValid() {
  for(int r=0;r<3;++r)for(int s=0;s<3;++s){
    float dot=0;for(int c=0;c<3;++c)dot+=mountingMatrix[r][c]*mountingMatrix[s][c];
    if(!isfinite(dot)||fabsf(dot-(r==s?1.0f:0.0f))>0.001f)return false;
  }
  const auto &m=mountingMatrix;
  float d=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
  return fabsf(d-1.0f)<0.001f;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool readRegisters(uint8_t address, uint8_t reg, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, uint8_t(length)) != length) {
    while (Wire.available()) Wire.read(); return false;
  }
  for (size_t i = 0; i < length; ++i) buffer[i] = Wire.read();
  return true;
}
bool initializeImu(uint8_t address, uint8_t sampleDivider) {
  uint8_t id;
  if (!readRegisters(address, 0x75, &id, 1) || id != 0x68) return false;
  if (!writeRegister(address, 0x6b, 0x80)) return false;
  delay(100);
  return writeRegister(address, 0x6b, 0x01) && writeRegister(address, 0x1a, 3)
    && writeRegister(address, 0x19, sampleDivider) && writeRegister(address, 0x1b, 0x08)
    && writeRegister(address, 0x1c, 0x08) && writeRegister(address, 0x37, 0)
    && writeRegister(address, 0x38, 1);
}

// 两种采集通道只在这里汇入当前控制状态，调用者已持有 stateLock。
void acceptControlSample(const float acceleration[3],const float gyro[3],float temperature,
                         uint64_t eventTime,uint32_t dtUs,uint32_t missed) {
      Imu &s=imu;s.missed+=missed;
      const float previousGyro[3]={bodyG[0],bodyG[1],bodyG[2]};
      if (dtUs < 100 || dtUs > IMU_TIMEOUT_US) {
        worldReady=false;attitude.invalidate();correctionActive=false;latchFault("ATTITUDE_GAP");
      }
      for (int k = 0; k < 3; ++k) { s.a[k]=acceleration[k];s.g[k]=gyro[k]; }
      s.temperature=temperature;
      float norm = sqrtf(s.a[0]*s.a[0]+s.a[1]*s.a[1]+s.a[2]*s.a[2]);
      toBody(s.a,bodyA);
      float corrected[3];for(int k=0;k<3;++k)corrected[k]=s.g[k]-s.bias[k];
      toBody(corrected,bodyG);
      // 不要将电机推力或平移产生的加速度，当作重力方向反馈。
      correctionActive=worldReady && !escArmed && !escArming && norm>.9f && norm<1.1f;
      if(worldReady && !acceptHistorySample(attitude,previousGyro,bodyG,bodyA,dtUs,correctionActive)) {
        worldReady=false;attitude.invalidate();correctionActive=false;latchFault("ATTITUDE_INVALID");
      }
      correctionActive=worldReady&&attitude.gravityCorrectionApplied();
      if(worldReady){auto e=attitude.euler();s.roll=e.roll;s.pitch=e.pitch;}
      s.t = eventTime; ++s.count; s.valid = true;
      if (calibrating) {
        // MPU6050 可能出现少量陀螺仪尖峰，以及每秒数度的未校准零偏，
        // 先忽略孤立的异常样本，再依据整个三秒窗口内的
        // 有效样本数和数据方差，判断校准是否通过。
        bool still = norm > .85f && norm < 1.15f && fabsf(s.g[0]) < 15 && fabsf(s.g[1]) < 15 && fabsf(s.g[2]) < 15;
        if (!still) ++calibrationRejectedSamples;
        else { for (int k=0;k<3;++k) {s.biasSum[k]+=s.g[k];gyroSumSq[k]+=s.g[k]*s.g[k];accelSum[k]+=s.a[k];accelSumSq[k]+=s.a[k]*s.a[k];} ++s.calibrationSamples; }
      }
      if(initialControlCalibrationPending){
        initialControlCalibrationPending=false;
        if(operatingMode==MODE_ATTITUDE&&!worldReady&&!calibrating&&!fault[0])beginCalibrationLocked();
      }
}

void sensorTask(void *) {
  for (;;) {
    PrimaryUart::Sample sample;
    if(pollPrimaryUart(sample)) {
      xSemaphoreTake(stateLock,portMAX_DELAY);
      for(int k=0;k<3;++k){primaryImu.a[k]=sample.frame.raw[k]/8192.0f;primaryImu.g[k]=sample.frame.raw[k+4]/65.5f;}
      primaryImu.temperature=sample.frame.raw[3]/340.0f+36.53f;
      primaryImu.t=sample.localUs;primaryImu.valid=true;++primaryImu.count;primaryImu.missed+=sample.missed;
      // 只有被选中的来源才进入姿态估计与 PID；另一路只更新自己的采集缓存。
      if(controlSource==SOURCE_IMU1){
        imu.present=primaryImu.present;
        acceptControlSample(primaryImu.a,primaryImu.g,primaryImu.temperature,sample.localUs,sample.dtUs,sample.missed);
        primaryImu.pitch=imu.pitch;primaryImu.roll=imu.roll;
      }
      xSemaphoreGive(stateLock);
    }
    // 所有 Wire 访问仍由本任务独占；切换命令不在 loop 任务中碰 I2C。
    sampleImu2Monitor();
    xSemaphoreTake(stateLock,portMAX_DELAY);
    if(controlSource==SOURCE_IMU2){
      imu.present=imu2Monitor.present;imu.errors=imu.readErrors=imu2Monitor.errors;
      // samples 是 I2C 读取计数，保证同一份样本只送进控制逻辑一次；
      // 切换控制源时 imu2ControlConsumed 已被对齐到当前计数。
      if(imu2Monitor.samples!=imu2ControlConsumed&&imu2Monitor.fresh(esp_timer_get_time(),IMU2_TIMEOUT_US)){
        imu2ControlConsumed=imu2Monitor.samples;
        const uint64_t elapsed=imu.t&&imu2Monitor.sampleUs>imu.t?imu2Monitor.sampleUs-imu.t:5000;
        const uint32_t dt=elapsed>IMU_TIMEOUT_US?IMU_TIMEOUT_US+1:uint32_t(elapsed);
        acceptControlSample(imu2Monitor.accelG,imu2Monitor.gyroDps,imu2Monitor.temperatureC,imu2Monitor.sampleUs,dt,0);
        imu.missed=imu2Monitor.missed;
      }
    }else{
      imu.present=primaryImu.present;imu.errors=primaryImu.errors;imu.readErrors=primaryImu.readErrors;
    }
    xSemaphoreGive(stateLock);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void safetyTask(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    xSemaphoreTake(stateLock, portMAX_DELAY);
    const uint64_t nowUs = esp_timer_get_time();
    heartbeatWarning=(servoEnabled || escArmed || escArming) && !freshHeartbeat();
    if ((servoEnabled || escArmed || escArming) && !imuUsable(nowUs)) latchFault("IMU_TIMEOUT");
    if (autoEnabled && !worldReady) latchFault("WORLD_NOT_READY");
    if (calibrating && millis()-calibrationAt >= 3000) {
      bool enough = imu.present && imu.calibrationSamples >= 300 && calibrationRejectedSamples <= 60 && imuFresh(nowUs);
      float meanA[3]={0,0,0};
      if(enough)for(int k=0;k<3;++k){
        const double n=imu.calibrationSamples, mg=imu.biasSum[k]/n, ma=accelSum[k]/n;
        enough=enough && gyroSumSq[k]/n-mg*mg<4.0 && accelSumSq[k]/n-ma*ma<0.0025;
        meanA[k]=ma;
      }
      float proposedMount[3][3];
      const bool mountOk=enough && buildUprightMount(meanA,proposedMount,controlMountYawDeg(controlSource));
      if(mountOk)memcpy(mountingMatrix,proposedMount,sizeof(mountingMatrix));
      float meanBody[3];toBody(meanA,meanBody);
      if(enough && mountOk && mountingValid() && attitude.initializeFromAccel(meanBody[0],meanBody[1],meanBody[2])) {
        mountingCalibrated=true;
        for(int k=0;k<3;++k)imu.bias[k]=imu.biasSum[k]/imu.calibrationSamples;
        imu.calibrated=true;calibrated=true;worldReady=true;referenceTime=nowUs;++referenceId;
        if(!strcmp(fault,"CALIBRATION_MOVED") || !strcmp(fault,"CALIBRATION_DATA") || !strcmp(fault,"HEADING_UNDEFINED")) fault="";
        auto e=attitude.euler();imu.roll=e.roll;imu.pitch=e.pitch;
        autoZeroRoll=e.roll;autoZeroPitch=e.pitch;autoErrorRoll=autoErrorPitch=0;
        automaticPid.reset();
        lastControlMeasurementUs=0;
        float corrected[3];for(int k=0;k<3;++k)corrected[k]=imu.g[k]-imu.bias[k];toBody(corrected,bodyG);
      } else {worldReady=false;latchFault(enough&&!mountOk?"MOUNT_CALIBRATION":(calibrationRejectedSamples>60?"CALIBRATION_MOVED":"CALIBRATION_DATA"));}
      calibrating=false;
    }
    if(actuatorPwmFault)latchFault("PWM_INIT");
    if (!fault[0]) {
      gimbalTiltValid=false;
      if (worldReady && imuUsable(nowUs)) {
        const uint64_t ageUs=nowUs>=imu.t?nowUs-imu.t:0;
        const bool newMeasurement=imu.t!=lastControlMeasurementUs;
        gimbalTiltValid=projectHistory(attitude,bodyG,ageUs,HISTORY_PREDICTION_US,controlAttitude) &&
                        gimbalLean(controlAttitude.rotation(),autoErrorPitch,autoErrorRoll);
        if (autoEnabled && servoEnabled) {
          if(!gimbalTiltValid)latchFault("ATTITUDE_INVALID");
          else if (automaticPid.update(nowUs,autoErrorPitch,autoErrorRoll,
                                  servoActual,servoTarget,newMeasurement&&ageUs<=IMU_FRESH_US)==GimbalPid::INVALID)
            latchFault("PID_INVALID");
        }
        lastControlMeasurementUs=imu.t;
      }
      // 自动模式启动前保持电调停止脉宽，仅让舵机向当前姿态的比例纠偏目标移动。
      if(startupPreposition.active()){
        if(!gimbalTiltValid || startupPreposition.moved(autoErrorPitch,autoErrorRoll,PREPOSITION_MAX_MOTION_DEG))
          latchFault("PREPOSITION_MOVED");
        else if(millis()-prepositionAt>=PREPOSITION_TIMEOUT_MS) latchFault("PREPOSITION_TIMEOUT");
        else {
          servoTarget[0]=startupPreposition.goal(0);servoTarget[1]=startupPreposition.goal(1);
          if(startupPreposition.ready(servoActual,millis(),PREPOSITION_SETTLE_MS,imuFresh(nowUs)) && millis()-armAt>=armHold){
            startupPreposition.cancel();prepositionOwnedServos=prepositionResumeAuto=false;
            automaticPid.reset();autoEnabled=true;
          }
        }
      }
      if(escArming && !startupPreposition.active() && !fault[0] && millis()-armAt>=armHold){escArming=false;escArmed=true;}
      if(tiltThrottleEnabled){
        if(!autoEnabled||!servoEnabled||!escArmed||fault[0]||!gimbalTiltValid)stopEscOutput();
        else if(!tiltThrottle.update(nowUs,controlAttitude.rotation().m[2][2],imuFresh(nowUs)))latchFault("THROTTLE_INVALID");
        else escTarget=tiltThrottle.pulse(escStop,escMax);
      }
      if (servoEnabled) for(int i=0;i<2;++i){servoTarget[i]=limitServoUs(i,servoTarget[i]);servoActual[i]=limitServoUs(i,servoActual[i]+constrain(servoTarget[i]-servoActual[i],-SERVO_SLEW_US_PER_TICK,SERVO_SLEW_US_PER_TICK));pulse(i,servoActual[i]);}
      if (escArmed) {escActual+=constrain(escTarget-escActual,-5,5);pulse(2,escActual);}
    }
    xSemaphoreGive(stateLock);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
  }
}

void sendJson(JsonDocument &doc, int destination) {
  String payload; serializeJson(doc,payload);
  char checksum[12]; snprintf(checksum,sizeof(checksum),"\t%08lx\n",(unsigned long)crc32((const uint8_t*)payload.c_str(),payload.length()));
  payload+=checksum;
  // 可能阻塞的网络或 USB 发送期间，不要持有 stateLock。
  if(destination==-1){if(Serial)Serial.print(payload);}
  else if(destination>=0 && !sendNetworkFrame(uint8_t(destination),payload)) disconnectNetworkClient(uint8_t(destination));
}
void reply(uint32_t request, bool ok, const char *reason, int destination, const char *requestSession) {
  StaticJsonDocument<512> doc;
  xSemaphoreTake(stateLock, portMAX_DELAY);
  doc["v"]=PROTOCOL_VERSION; doc["type"]="ack"; doc["seq"]=++txSeq; doc["request_seq"]=request;
  doc["session"]=requestSession; doc["boot"]=bootId; doc["ok"]=ok; doc["reason"]=reason; doc["epoch"]=epoch;
  doc["config_rev"]=configRevision; doc["firmware"]=FW_VERSION;
  xSemaphoreGive(stateLock); sendJson(doc,destination);
}
bool integerIn(JsonVariantConst value, int low, int high) {
  return value.is<int>() && value.as<int>()>=low && value.as<int>()<=high;
}

void handleFrame(char *line, int source) {
  char *separator=strrchr(line,'\t');
  if(!separator || strlen(separator+1)!=8){++badFrames;return;}
  for(char *p=separator+1;*p;++p)if(!isxdigit(*p)){++badFrames;return;}
  const uint32_t expected=strtoul(separator+1,nullptr,16); *separator=0;
  if(crc32((uint8_t*)line,strlen(line))!=expected){++badFrames;return;}
  StaticJsonDocument<1024> doc;
  if(deserializeJson(doc,line) || doc["v"].as<int>()!=int(PROTOCOL_VERSION) || !doc["seq"].is<uint32_t>() || !doc["session"].is<const char*>() || !doc["cmd"].is<const char*>()){++badFrames;return;}
  uint32_t request=doc["seq"]; const char *sid=doc["session"], *command=doc["cmd"];
  if(strlen(sid)<8 || strlen(sid)>32 || request==0){++badFrames;return;}
  xSemaphoreTake(stateLock,portMAX_DELAY);
  bool ok=false; const char *reason="UNKNOWN_COMMAND";
  if(!strcmp(command,"hello")) {
    if(source>=0 && (!doc["token"].is<const char*>() || strcmp(doc["token"].as<const char*>(),getNetworkToken())))reason="AUTH_REQUIRED";
    else if(controlOwner!=-2 && controlOwner!=source)reason="CONTROLLER_BUSY";
    else {
      stopOutputs();controlOwner=source;strlcpy(sessionId,sid,sizeof(sessionId));
      lastCommandSeq=request;heartbeatAt=controllerSeenAt=millis();
      // 获取控制权不会取消正在进行的静止校准。
      if(operatingMode==MODE_ATTITUDE && !worldReady && !calibrating && !fault[0] && imu.present)
        beginCalibrationLocked();
      ok=true;reason="HELLO";
    }
  } else if(controlOwner!=source || strcmp(sid,sessionId) || !doc["boot"].is<const char*>() || strcmp(doc["boot"].as<const char*>(),bootId)) reason="SESSION_MISMATCH";
  else if(request<=lastCommandSeq) reason="OLD_SEQUENCE";
  else if(strcmp(command,"stop") && strcmp(command,"auto_disable") && strcmp(command,"auto_motor_disable") && strcmp(command,"tilt_throttle_disable") &&
    strcmp(command,"esc_stop") && strcmp(command,"servo_disable") && (!doc["device_t_us"].is<uint64_t>() ||
    doc["device_t_us"].as<uint64_t>()>uint64_t(esp_timer_get_time())+100000 ||
    uint64_t(esp_timer_get_time())>doc["device_t_us"].as<uint64_t>()+COMMAND_MAX_AGE_US))reason="STALE_COMMAND";
  else {
    lastCommandSeq=request;controllerSeenAt=millis();
    // 手动开环测试不依赖姿态或世界坐标参考，
    // 但仍用主 IMU 的实时数据作为平台健康监测条件。
    bool ready=!fault[0] && !calibrating && imuUsable(esp_timer_get_time());
    // 停止指令始终可执行；启用输出或修改配置的指令，必须对应
    // 当前控制器已确认的状态版本。
    if(commandNeedsEpoch(command) && (!doc["epoch"].is<uint32_t>() || doc["epoch"].as<uint32_t>()!=epoch))reason="OLD_EPOCH";
    else if(!strcmp(command,"keepalive")){ok=true;reason="KEEPALIVE";}
    else if(!strcmp(command,"heartbeat")){heartbeatAt=millis();ok=true;reason="HEARTBEAT";}
    else if(!strcmp(command,"stop")){stopOutputs();calibrating=false;ok=true;reason="STOPPED";}
    else if(!strcmp(command,"clear_fault")) {
      if(!servoEnabled && !escArmed && !escArming){fault="";stopOutputs();ok=true;reason="CLEARED";}else reason="STOP_OUTPUTS_FIRST";
    } else if(!strcmp(command,"set_mode")) {
      if(!doc["mode"].is<const char*>())reason="INVALID_MODE";
      else if(!strcmp(doc["mode"].as<const char*>(),"ATTITUDE")) {
        stopOutputs();operatingMode=MODE_ATTITUDE;calibrating=false;
        ok=beginCalibrationLocked();reason=ok?"MODE_ATTITUDE_CALIBRATING":"IMU_NOT_READY";
      } else if(!strcmp(doc["mode"].as<const char*>(),"MANUAL")) {
        stopOutputs();calibrating=false;operatingMode=MODE_MANUAL;
        // 手动开环测试不要求世界坐标系已校准；切换到此模式时，
        // 只清除与校准相关的故障。
        if(!strcmp(fault,"CALIBRATION_MOVED") || !strcmp(fault,"CALIBRATION_DATA") ||
          !strcmp(fault,"HEADING_UNDEFINED")) fault="";
        ok=true;reason="MODE_MANUAL";
      } else reason="INVALID_MODE";
    } else if(!strcmp(command,"select_imu")) {
      // 切换会先停止全部输出并清空 PID 历史；目标传感器不存在时保持原来源不变。
      if(!integerIn(doc["source"],SOURCE_IMU1,SOURCE_IMU2))reason="INVALID_IMU_SOURCE";
      else if(uint8_t(doc["source"].as<int>())==controlSource){ok=true;reason="IMU_SOURCE_UNCHANGED";}
      else if(!selectControlImuLocked(uint8_t(doc["source"].as<int>())))reason="IMU_NOT_FOUND";
      else{ok=true;reason="IMU_SOURCE_SELECTED";}
    } else if(!strcmp(command,"calibrate")) {
      if(!imu.present)reason="IMU_NOT_READY";
      else if(servoEnabled||escArmed||escArming)reason="STOP_OUTPUTS_FIRST";
      else if(beginCalibrationLocked()){ok=true;reason="CALIBRATING";}
      else reason="FAULT_REQUIRES_RESTART";
    } else if(!strcmp(command,"world_zero")) {
      if(servoEnabled||escArmed||escArming||calibrating)reason="STOP_OUTPUTS_FIRST";
      else if(!worldReady||!imuFresh(esp_timer_get_time()))reason="WORLD_NOT_READY";
      else if(fabsf(bodyG[0])>2||fabsf(bodyG[1])>2||fabsf(bodyG[2])>2)reason="KEEP_STILL";
      else if(!attitude.resetYaw())reason="HEADING_UNDEFINED";
      else{automaticPid.reset();referenceTime=esp_timer_get_time();++referenceId;++epoch;ok=true;reason="WORLD_HEADING_ZEROED";}
    } else if(!strcmp(command,"pid_config")) {
      if(servoEnabled||escArmed||escArming||calibrating)reason="STOP_OUTPUTS_FIRST";
      else {
        const char *keys[]={"kp_x","ki_x","kd_x","kp_y","ki_y","kd_y","max_tilt_deg","azimuth_zero_deg","sign_x","sign_y"};
        bool numbers=true;
        for(const char *key:keys)numbers=numbers&&doc[key].is<float>()&&isfinite(doc[key].as<float>());
        GimbalConfig next=automaticPid.config();
        if(numbers){
          next.x={doc["kp_x"].as<float>(),doc["ki_x"].as<float>(),doc["kd_x"].as<float>()};
          next.y={doc["kp_y"].as<float>(),doc["ki_y"].as<float>(),doc["kd_y"].as<float>()};
          next.limit=doc["max_tilt_deg"].as<float>();next.integralLimit=fminf(AUTO_INTEGRAL_LIMIT_DEG,next.limit);
          next.azimuthZero=doc["azimuth_zero_deg"].as<float>();next.signX=doc["sign_x"].as<float>();next.signY=doc["sign_y"].as<float>();
        }
        GimbalPid proposed(next);
        if(!numbers||next.x.kp>10||next.y.kp>10||next.x.ki>2||next.y.ki>2||next.x.kd>2||next.y.kd>2||!proposed.configurationValid())reason="PID_CONFIG";
        else{automaticPid=proposed;++epoch;ok=true;reason="PID_CONFIGURED";}
      }
    } else if(!strcmp(command,"servo_enable")) {
      if(operatingMode!=MODE_MANUAL)reason="MODE_LOCKED";
      else if(ready && !servoEnabled){automaticPid.reset();servoEnabled=true;autoEnabled=false;++epoch;for(int i=0;i<2;++i){servoActual[i]=servoTarget[i]=SERVO_CENTER_US;pulse(i,SERVO_CENTER_US);}ok=true;reason="SERVO_ENABLED";}else reason="NOT_READY";
    } else if(!strcmp(command,"servo_disable")) {
      if(operatingMode!=MODE_MANUAL)reason="MODE_LOCKED";
      else{++epoch;stopServoOutput();ok=true;reason="SERVO_DISABLED";}
    } else if(!strcmp(command,"servo")) {
      if(operatingMode!=MODE_MANUAL)reason="MODE_LOCKED";
      else if(!ready || !servoEnabled)reason="NOT_READY";
      else if(!doc["epoch"].is<uint32_t>() || doc["epoch"].as<uint32_t>()!=epoch)reason="OLD_EPOCH";
      else if(!integerIn(doc["x"],SERVO_MIN_US,SERVO_MAX_US))reason="RANGE";
      else if(!integerIn(doc["y"],ELEVATION_MIN_US,ELEVATION_MAX_US))reason="ELEVATION_LIMIT";
      else{servoTarget[0]=doc["x"];servoTarget[1]=doc["y"];ok=true;reason="SERVO_ACCEPTED";}
    } else if(!strcmp(command,"esc_config")) {
      if(!motorModeAllowed())reason="MODE_LOCKED";
      else if(escArmed||escArming)reason="STOP_ESC_FIRST";
      else if(!doc["confirmed"].as<bool>()||!integerIn(doc["stop_us"],900,2100)||!integerIn(doc["min_us"],900,2100)||!integerIn(doc["max_us"],900,2100)||!integerIn(doc["hold_ms"],1000,15000)||doc["hz"].as<int>()!=50)reason="INVALID_ESC_CONFIG";
      else if(doc["min_us"].as<int>()>=doc["stop_us"].as<int>() || doc["max_us"].as<int>()<=doc["stop_us"].as<int>())reason="STOP_NOT_INSIDE_RANGE";
      else {++epoch;stopEscOutput();disablePulse(2);escSignalEnabled=false;escActual=escTarget=0;escStop=doc["stop_us"];escMin=doc["min_us"];escMax=doc["max_us"];armHold=doc["hold_ms"];escConfigured=true;++configRevision;ok=true;reason="ESC_CONFIGURED";}
    } else if(!strcmp(command,"esc_arm")) {
      if(!motorModeAllowed())reason="MODE_LOCKED";
      else if(!ready||!escConfigured)reason="NOT_READY";
      else if(!doc["config_rev"].is<uint32_t>()||doc["config_rev"].as<uint32_t>()!=configRevision)reason="CONFIG_MISMATCH";
      else if(operatingMode==MODE_ATTITUDE && (!worldReady||!mountingCalibrated||!gimbalTiltValid||!imuFresh(esp_timer_get_time())))reason="WORLD_NOT_READY";
      else {
        ++epoch;stopEscOutput();
        if(operatingMode==MODE_ATTITUDE){
          prepositionOwnedServos=!servoEnabled;prepositionResumeAuto=autoEnabled;
          if(!servoEnabled){servoEnabled=true;for(int i=0;i<2;++i){servoActual[i]=servoTarget[i]=SERVO_CENTER_US;pulse(i,SERVO_CENTER_US);}}
          autoEnabled=false;automaticPid.reset();
          if(!startupPreposition.begin(automaticPid.config(),autoErrorPitch,autoErrorRoll,servoActual)){stopOutputs();reason="PID_INVALID";}
          else prepositionAt=millis();
        }
        if(operatingMode==MODE_MANUAL || startupPreposition.active()){
          escSignalEnabled=true;escActual=escTarget=escStop;pulse(2,escStop);escArming=true;armAt=millis();ok=true;reason="ARMING";
        }
      }
    } else if(!strcmp(command,"esc")) {
      if(!motorModeAllowed())reason="MODE_LOCKED";
      else if(tiltThrottleEnabled)reason="AUTO_THROTTLE_ACTIVE";
      else if(!ready||!escArmed)reason="NOT_ARMED";
      else if(!doc["epoch"].is<uint32_t>()||doc["epoch"].as<uint32_t>()!=epoch)reason="OLD_EPOCH";
      else if(!integerIn(doc["us"],escMin,escMax))reason="RANGE";
      else {escTarget=doc["us"];ok=true;reason="ESC_ACCEPTED";}
    } else if(!strcmp(command,"esc_stop")) {
      ++epoch;stopEscOutput();ok=true;reason="ESC_STOPPED";
    } else if(!strcmp(command,"auto_motor_enable")) {
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else if(!ready||!worldReady)reason="NOT_READY";
      else{++epoch;autoMotorAllowed=true;ok=true;reason="AUTO_MOTOR_ENABLED";}
    } else if(!strcmp(command,"auto_motor_disable")) {
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else{++epoch;autoMotorAllowed=false;stopEscOutput();ok=true;reason="AUTO_MOTOR_DISABLED";}
    } else if(!strcmp(command,"tilt_throttle_disable")) {
      ++epoch;stopEscOutput();autoMotorAllowed=false;ok=true;reason="TILT_THROTTLE_DISABLED";
    } else if(!strcmp(command,"tilt_throttle_config")) {
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else if(servoEnabled||escArmed||escArming||calibrating)reason="STOP_OUTPUTS_FIRST";
      else {
        const char *keys[]={"base_pct","gain_pct_deg","max_pct","deadband_deg","ramp_pct_s"};bool numbers=true;
        for(const char *key:keys)numbers=numbers&&doc[key].is<float>()&&isfinite(doc[key].as<float>());
        TiltThrottle proposed;proposed.config={doc["base_pct"].as<float>(),doc["gain_pct_deg"].as<float>(),doc["max_pct"].as<float>(),doc["deadband_deg"].as<float>(),doc["ramp_pct_s"].as<float>()};
        if(!numbers||!proposed.valid())reason="THROTTLE_CONFIG";
        else{tiltThrottle=proposed;++epoch;ok=true;reason="THROTTLE_CONFIGURED";}
      }
    } else if(!strcmp(command,"auto_enable")) {
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else if(startupPreposition.active())reason="PREPOSITION_ACTIVE";
      else if(!doc["tilt_throttle"].isNull()&&!doc["tilt_throttle"].is<bool>())reason="THROTTLE_CONFIG";
      else if(!ready||!worldReady||!mountingCalibrated||!gimbalTiltValid||((escArmed||escArming)&&!autoMotorAllowed))reason="NOT_READY";
      else if(doc["tilt_throttle"].as<bool>()&&(!autoMotorAllowed||!escArmed||escArming||!escConfigured||!tiltThrottle.valid()))reason="THROTTLE_NOT_READY";
      else{tiltThrottle.reset();tiltThrottleEnabled=doc["tilt_throttle"].as<bool>();++epoch;
        if(!autoEnabled){automaticPid.reset();autoEnabled=true;if(!servoEnabled){servoEnabled=true;for(int i=0;i<2;++i){servoActual[i]=servoTarget[i]=SERVO_CENTER_US;pulse(i,SERVO_CENTER_US);}}}
        ok=true;reason="AUTO_ENABLED";}
    } else if(!strcmp(command,"auto_disable")) {
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else{stopOutputs();ok=true;reason="AUTO_DISABLED";}
    }
  }
  xSemaphoreGive(stateLock); reply(request,ok,reason,source,sid);
}

void appendPidTelemetry(JsonObject item, const AnglePid &pid) {
  const auto &t=pid.terms();const auto gains=pid.gains();
  item["error_deg"]=t.error;item["p_deg"]=t.p;item["i_deg"]=t.i;
  item["d_deg"]=t.d;item["output_deg"]=t.output;
  item["kp"]=gains.kp;item["ki"]=gains.ki;item["kd"]=gains.kd;
}

void appendImuSource(JsonObject item,const Imu &s,uint8_t source,uint64_t now,float rateHz) {
  const bool usable=imuSampleUsable(s,now,IMU_TIMEOUT_US);
  const uint64_t age=s.t&&now>=s.t?now-s.t:IMU_TIMEOUT_US;
  const bool local=source==SOURCE_IMU2;
  item["address"]=s.address;item["source_id"]=source;
  item["source"]=controlSourceInterface(source);
  // 同一时刻只有一路为 true；网页据此标注“控制源 / 校验”。
  item["control_source"]=source==controlSource;
  item["present"]=s.present;item["valid"]=usable;item["calibrated"]=s.calibrated;
  item["sample_t_us"]=s.t;item["samples"]=s.count;item["errors"]=s.errors;item["missed"]=s.missed;
  item["read_errors"]=s.readErrors;item["race_drops"]=s.raceDrops;
  item["sample_timeout_us"]=IMU_TIMEOUT_US;item["fresh_window_us"]=IMU_FRESH_US;
  item["sample_age_ms"]=double(age)/1000;
  item["sample_hz"]=usable?roundf(rateHz*10.0f)/10.0f:0;
  item["target_hz"]=200;
  item["history_state"]=!usable?"UNAVAILABLE":age<=IMU_FRESH_US?"LIVE":age<=HISTORY_PREDICTION_US?"PREDICTED":"HELD";
  // 状态要区分“模块没接”“还没收到第一帧”“数据过期”和“读取出错”，
  // 不能用同一个 STALE 掩盖真实原因。
  item["status"]=!s.present?(local?"NOT_FOUND":(primaryReplies?"C3_NOT_FOUND":"WAIT_C3"))
    :usable?"OK":s.count?"STALE":(local?(imu2Monitor.errors?"READ_ERROR":"WAIT_INT"):"WAIT_SAMPLE");
  item["pitch"]=s.pitch;item["roll"]=s.roll;item["temperature"]=s.temperature;
  item["calibration_samples"]=s.calibrationSamples;
  item["calibration_rejected"]=source==controlSource?calibrationRejectedSamples:0;
  if(source==SOURCE_IMU1){
    item["uart_replies"]=primaryReplies;item["uart_bad_frames"]=primaryBadFrames;
    item["uart_rejected"]=primaryRejected;item["uart_timeouts"]=primaryTimeouts;item["c3_boot"]=primaryRemoteBoot;
  }else{
    item["i2c_errors"]=imu2Monitor.errors;item["irq_missed"]=imu2Monitor.missed;
  }
  auto a=item.createNestedArray("a"),g=item.createNestedArray("g"),bias=item.createNestedArray("bias");
  for(int k=0;k<3;++k){a.add(s.a[k]);g.add(s.g[k]-s.bias[k]);bias.add(s.bias[k]);}
}

void telemetry() {
  static StaticJsonDocument<6144> doc;
  doc.clear();
  xSemaphoreTake(stateLock,portMAX_DELAY);
  const uint64_t now=esp_timer_get_time();
  const int destination=controlOwner;
  static uint64_t rateAt=0;static uint32_t rateSamples=0;static float sampleHz=0;static uint8_t rateSource=0;
  if(!rateAt||rateSource!=controlSource||imu.count<rateSamples){rateAt=now;rateSamples=imu.count;sampleHz=0;rateSource=controlSource;}
  if(now-rateAt>=1000000){sampleHz=float(double(uint32_t(imu.count-rateSamples))*1e6/double(now-rateAt));rateAt=now;rateSamples=imu.count;}
  // IMU1 通道即使没有被选为控制源，也按自己的样本计数统计采样率，供校验卡片显示。
  static uint64_t primaryRateAt=0;static uint32_t primaryRateSamples=0;static float primaryHz=0;
  if(!primaryRateAt||primaryImu.count<primaryRateSamples){primaryRateAt=now;primaryRateSamples=primaryImu.count;primaryHz=0;}
  if(now-primaryRateAt>=IMU1_RATE_WINDOW_US){primaryHz=float(double(uint32_t(primaryImu.count-primaryRateSamples))*1e6/double(now-primaryRateAt));primaryRateAt=now;primaryRateSamples=primaryImu.count;}
  doc["v"]=PROTOCOL_VERSION;doc["type"]="telemetry";doc["seq"]=++txSeq;doc["session"]=sessionId;doc["boot"]=bootId;doc["t_us"]=now;
  doc["firmware"]=FW_VERSION;doc["fault"]=fault;doc["epoch"]=epoch;doc["config_rev"]=configRevision;
  doc["warning"]=heartbeatWarning?"HEARTBEAT_TIMEOUT":"";
  doc["heartbeat_age_ms"]=heartbeatAt?int64_t(uint32_t(millis()-heartbeatAt)):int64_t(-1);
  doc["state"]=fault[0]?"FAULT":(escArming?"ARMING":(escArmed?"ARMED":(servoEnabled?"SERVO_TEST":"DISARMED")));
  doc["operating_mode"]=operatingModeName();
  doc["calibrating"]=calibrating;doc["calibrated"]=calibrated;doc["sample_hz"]=imuFresh(now)?roundf(sampleHz*10)/10:0;doc["target_hz"]=200;doc["bad_frames"]=badFrames;
  doc["imu_count"]=1;
  // 固件实际生效的控制源。网页的“页面选择”只用于发起切换，
  // 显示与 PID 都以这里回传的值为准，避免前端显示和固件不一致。
  doc["control_source"]=controlSource;
  doc["control_source_name"]=controlSourceName(controlSource);
  doc["control_source_label"]=controlSourceLabel(controlSource);
  auto outputs=doc.createNestedObject("outputs");outputs["servo_enabled"]=servoEnabled;
  auto servo=outputs.createNestedArray("servo_us");servo.add(servoActual[0]);servo.add(servoActual[1]);
  outputs["esc_configured"]=escConfigured;outputs["esc_armed"]=escArmed;outputs["esc_arming"]=escArming;outputs["esc_us"]=escActual;
  outputs["esc_signal_enabled"]=escSignalEnabled;
  outputs["esc_stop_us"]=escStop;outputs["esc_min_us"]=escMin;outputs["esc_max_us"]=escMax;
  auto control=doc.createNestedObject("control");control["auto_enabled"]=autoEnabled;control["auto_motor_allowed"]=autoMotorAllowed;
  control["preposition_active"]=startupPreposition.active();control["preposition_wait_ms"]=PREPOSITION_SETTLE_MS;
  control["source_id"]=controlSource;
  control["source_address"]=controlSourceAddress(controlSource);
  control["source_label"]=controlSourceLabel(controlSource);
  auto error=control.createNestedArray("error_deg");error.add(autoErrorRoll);error.add(autoErrorPitch);
  auto zero=control.createNestedArray("zero_deg");zero.add(autoZeroRoll);zero.add(autoZeroPitch);
  control["algorithm"]="VECTOR_PID";control["geometry"]="AZIMUTH_ELEVATION";
  control["mount_ready"]=mountingCalibrated;control["tilt_valid"]=gimbalTiltValid;
  control["mount_yaw_deg"]=controlMountYawDeg(controlSource);
  auto ranges=outputs.createNestedArray("servo_range_deg");ranges.add(360);ranges.add(270);
  auto elevationLimits=outputs.createNestedArray("elevation_limits_deg");elevationLimits.add(ELEVATION_MIN_DEG);elevationLimits.add(ELEVATION_MAX_DEG);
  auto throttle=control.createNestedObject("tilt_throttle");
  throttle["enabled"]=tiltThrottleEnabled;throttle["tilt_deg"]=tiltThrottle.tilt;throttle["target_pct"]=tiltThrottle.requested;throttle["output_pct"]=tiltThrottle.percent;
  throttle["base_pct"]=tiltThrottle.config.base;throttle["gain_pct_deg"]=tiltThrottle.config.gain;throttle["max_pct"]=tiltThrottle.config.maximum;throttle["deadband_deg"]=tiltThrottle.config.deadband;throttle["ramp_pct_s"]=tiltThrottle.config.ramp;
  auto angles=outputs.createNestedArray("servo_deg");for(int k=0;k<2;++k)angles.add(GimbalPid::usToAngle(servoActual[k],k));
  const auto &cfg=automaticPid.config();
  control["azimuth_zero_deg"]=cfg.azimuthZero;control["sign_x"]=cfg.signX;control["sign_y"]=cfg.signY;
  auto mount=control.createNestedArray("mount_matrix");for(int r=0;r<3;++r)for(int c=0;c<3;++c)mount.add(mountingMatrix[r][c]);
  auto pid=control.createNestedObject("pid");pid["dt_s"]=automaticPid.dt();
  pid["output_limit_deg"]=cfg.limit;pid["limited"]=automaticPid.limited();
  appendPidTelemetry(pid.createNestedObject("x"),automaticPid.x());
  appendPidTelemetry(pid.createNestedObject("y"),automaticPid.y());
  // control_imu 明确承载当前 PID 源；imus 保留原 IMU1 采集通道，imu2 为本地原始数据。
  auto list=doc.createNestedArray("imus");
  appendImuSource(list.createNestedObject(),primaryImu,SOURCE_IMU1,now,primaryHz);
  appendImuSource(doc.createNestedObject("control_imu"),imu,controlSource,now,sampleHz);
  auto world=doc.createNestedObject("world");
  world["valid"]=worldReady&&imuUsable(now);
  world["frame"]="LOCAL_Z_UP";world["reference_id"]=referenceId;world["heading_absolute"]=false;
  world["correction_active"]=correctionActive;world["reference_age_s"]=referenceId?double(now-referenceTime)/1e6:0;
  Attitude displayAttitude;
  const uint64_t displayAge=imu.t&&now>=imu.t?now-imu.t:0;
  projectHistory(attitude,bodyG,displayAge,HISTORY_PREDICTION_US,displayAttitude);
  world["prediction_age_ms"]=double(displayAge>HISTORY_PREDICTION_US?HISTORY_PREDICTION_US:displayAge)/1000;
  world["history_mode"]=displayAge>IMU_FRESH_US;
  auto q=displayAttitude.quaternion();auto e=displayAttitude.euler();auto r=displayAttitude.rotation();
  world["heading_defined"]=e.headingDefined;
  auto wq=world.createNestedArray("q_bw");wq.add(q.w);wq.add(q.x);wq.add(q.y);wq.add(q.z);
  auto we=world.createNestedArray("euler_deg");we.add(e.roll);we.add(e.pitch);we.add(e.yaw);
  auto wr=world.createNestedArray("r_bw");for(int i=0;i<3;++i)for(int j=0;j<3;++j)wr.add(r.m[i][j]);
  auto ba=world.createNestedArray("a_body_g");auto bg=world.createNestedArray("gyro_body_dps");
  for(int i=0;i<3;++i){ba.add(bodyA[i]);bg.add(bodyG[i]);}
  auto fw=displayAttitude.worldVector(bodyA[0],bodyA[1],bodyA[2]);
  auto la=displayAttitude.linearAcceleration(bodyA[0],bodyA[1],bodyA[2]);
  auto f=world.createNestedArray("f_world_g");f.add(fw.x);f.add(fw.y);f.add(fw.z);
  auto linear=world.createNestedArray("linear_world_mps2");linear.add(la.x);linear.add(la.y);linear.add(la.z);
  // 保留 IMU2 原始监测数据，与所选源的融合姿态区分显示。
  appendImu2Telemetry(doc.createNestedObject("imu2"),now);
  xSemaphoreGive(stateLock);
  if(!doc.overflowed())sendJson(doc,destination);
}

void onNetworkFrame(uint8_t client, char *line) { handleFrame(line,int(client)); }
void onNetworkDisconnect(uint8_t client) {
  xSemaphoreTake(stateLock,portMAX_DELAY);
  if(controlOwner==int(client)){
    stopOutputs();heartbeatAt=controllerSeenAt=0;sessionId[0]=0;controlOwner=-2;
  }
  xSemaphoreGive(stateLock);
}

void setup() {
  for(int i=0;i<3;++i)disablePulse(i);
  Serial.begin(SERIAL_BAUD);
  snprintf(bootId,sizeof(bootId),"%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random());
  stateLock=xSemaphoreCreateMutex();
  if(!automaticPid.configurationValid())latchFault("PID_CONFIG");
  Wire.begin(SDA_PIN,SCL_PIN,400000);Wire.setTimeOut(3);
  beginPrimaryUart();
  beginImu2Monitor();
  xTaskCreatePinnedToCore(sensorTask,"sensors",4096,nullptr,2,nullptr,1);
  xTaskCreatePinnedToCore(safetyTask,"safety",4096,nullptr,3,nullptr,0);
  if(!mountingValid()){xSemaphoreTake(stateLock,portMAX_DELAY);latchFault("MOUNT_MATRIX");xSemaphoreGive(stateLock);}
  if(!beginNetwork()){xSemaphoreTake(stateLock,portMAX_DELAY);latchFault("WIFI_INIT");xSemaphoreGive(stateLock);}
}

void loop() {
  pollNetwork();
  // 串口桥接无法直接报告线缆断开，因此要释放已超时的 USB 控制会话。
  xSemaphoreTake(stateLock,portMAX_DELAY);
  if(controllerLeaseExpired()){stopOutputs();heartbeatAt=controllerSeenAt=0;sessionId[0]=0;controlOwner=-2;}
  xSemaphoreGive(stateLock);
  static char line[1024];static size_t length=0;static bool overflow=false;
  static uint32_t telemetryAt=0;
  // 每轮限制接收数据的处理量，避免大量数据挤占遥测发送时间。
  for(int i=0;i<512&&Serial.available();++i){char c=Serial.read();
    if(c=='\n'){if(!overflow){line[length]=0;if(length)handleFrame(line,-1);}else ++badFrames;length=0;overflow=false;}
    else if(c!='\r'){if(length<sizeof(line)-1)line[length++]=c;else overflow=true;}
  }
  const uint32_t interval=controlOwner==-1?SERIAL_TELEMETRY_INTERVAL_MS:WIFI_TELEMETRY_INTERVAL_MS;
  if(millis()-telemetryAt>=interval){telemetryAt=millis();if(controlOwner!=-2)telemetry();}
  printPrimaryDebug();
  delay(1);
}
