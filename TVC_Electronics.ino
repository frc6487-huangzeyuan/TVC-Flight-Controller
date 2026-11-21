// 版本：tvc-gimbal-2.5.11
// TVC 实验平台：两路 MPU6050 都可以作为 PID 控制源，由网页明确切换。
//   本 PCB 两块 MPU6050 共用主控 SDA9/SCL10。
//   IMU1：AD0=GND，地址 0x68，INT=GPIO8；IMU2：AD0=3.3V，地址 0x69，INT=GPIO2。
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
#include <Preferences.h>
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
#include "ThrustTorqueControl.h"
#include "ServoCalibration.h"
#include "HistoryAttitude.h"
#include "TiltThrottle.h"
#include "ImuState.h"
#include "NetworkTransport.h"
#include "Imu2MonitorState.h"
#include <esp_arduino_version.h>

#if ESP_ARDUINO_VERSION_MAJOR != 2 && ESP_ARDUINO_VERSION_MAJOR != 3
#error "Use esp32 by Espressif Systems 2.x or 3.x."
#endif

// 在 MotorControl.ino 标签页中实现，通道 0/1 对应云台，2 对应电调，3 对应重心舵机。
int servoOutputUs(int channel, int commandUs);
void writePwmUs(int channel, int us);
void pulse(int channel, int us);
void disablePulse(int channel);
void beginImu2Monitor();
void sampleImu2Monitor();
void beginImu1Monitor();
void sampleImu1Monitor();
void appendImu2Telemetry(JsonObject item, uint64_t now);
void beginPrimaryUart();
void printPrimaryDebug();
bool pollPrimaryUart(PrimaryUart::Sample &sample);
extern Imu2MonitorState imu2Monitor;
extern Imu2MonitorState imu1Monitor;
extern volatile uint32_t imu1IrqCount, imu2IrqCount;

Imu primaryImu(IMU_ADDRESS); // IMU1 的独立采集缓存，未选中时仅作为校验数据
uint8_t controlSource=DEFAULT_CONTROL_SOURCE;
Imu imu(DEFAULT_CONTROL_SOURCE==1?IMU_ADDRESS:IMU2_ADDRESS); // 当前选中源的控制状态
// IMU2 通道已被控制逻辑消费到的样本计数，与 imu2Consumed（I2C 读取计数）分开。
// 切换控制源时把它对齐到最新计数，避免把切换前缓存的老样本当成新测量。
uint32_t imu2ControlConsumed=0;
uint32_t imu1ControlConsumed=0;
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
bool savedGravityReference=false;
struct GuidedGravityReference {GravityReference gravity;float outward[3];};
bool guidedMount=false;
uint8_t gravityStartupSamples=0;
float gravityStartupFirst[3]={},gravityStartupSum[3]={};
char bootId[17], sessionId[33] = "";
const char *fault = "";
bool heartbeatWarning=false; // 心跳超时只告警，不属于会停止执行器的 fault。
uint32_t lastCommandSeq = 0, txSeq = 0, heartbeatAt = 0, controllerSeenAt = 0, epoch = 0;
bool servoEnabled = false, escConfigured = false, escArmed = false, escArming = false, autoEnabled = false;
bool escSignalEnabled = false;
ThrustTorqueControl thrustTorque;
bool torquePrepared=false,torqueActive=false;
bool actuatorPwmFault = false;
bool autoMotorAllowed = false; // 每次会话都需明确勾选授权，此标志本身不会启动电调。
bool tiltThrottleEnabled=false;
TiltThrottle tiltThrottle;
bool calibrated = false, calibrating = false;
enum OperatingMode : uint8_t { MODE_ATTITUDE, MODE_MANUAL };
OperatingMode operatingMode = MODE_ATTITUDE;
uint32_t calibrationAt = 0, armAt = 0;
const int defaultAzimuthCenterUs=GimbalPid::angleToUs(GIMBAL_AZIMUTH_ZERO_DEG,0);
int servoTarget[2] = {defaultAzimuthCenterUs, SERVO_CENTER_US};
int servoActual[2] = {defaultAzimuthCenterUs, SERVO_CENTER_US};
bool cgServoEnabled = false;
int cgServoUs = CG_SERVO_INITIAL_US, cgServoDirection = 0;
uint32_t cgServoJogAt = 0;
int escStop = 0, escMin = 0, escMax = 0, escTarget = 0, escActual = 0;
float autoZeroRoll = 0, autoZeroPitch = 0, autoErrorRoll = 0, autoErrorPitch = 0;
GimbalConfig initialGimbalConfig() {
  GimbalConfig c{{AUTO_PITCH_KP,AUTO_PITCH_KI,AUTO_PITCH_KD},
    {AUTO_ROLL_KP,AUTO_ROLL_KI,AUTO_ROLL_KD},
    AUTO_MAX_CORRECTION_DEG,AUTO_INTEGRAL_LIMIT_DEG,AUTO_D_FILTER_SECONDS,
    GIMBAL_AZIMUTH_ZERO_DEG,GIMBAL_VECTOR_X_SIGN,GIMBAL_VECTOR_Y_SIGN,
    SERVO_SLEW_US_PER_TICK,PID_MAX_TICK_GAP_US};
  c.model.enabled=AUTO_MODEL_ENABLED;
  for(int i=0;i<2;++i)c.model.bias[i]=AUTO_MODEL_BIAS[i];
  for(int i=0;i<4;++i){c.model.gravity[i]=AUTO_MODEL_GRAVITY[i];c.model.inverse[i]=AUTO_MODEL_INVERSE[i];}
  c.model.accelerationLimit=AUTO_MODEL_ACCEL_LIMIT;
  return c;
}
GimbalPid automaticPid(initialGimbalConfig());
uint32_t armHold = 3000, configRevision = 0, badFrames = 0;
ServoCalibration servoCalibration{DEFAULT_AZIMUTH_SERVO_CENTER_TENTHS,DEFAULT_ELEVATION_SERVO_CENTER_DEG,0,ELEVATION_SERVO_REVERSED};
int servoCenterUs[2]={SERVO_CENTER_US,SERVO_CENTER_US};
ServoTravelSweep servoSweep;

void applyServoCalibration(const ServoCalibration &calibration) {
  servoCalibration=calibration;
  servoCenterUs[0]=GimbalPid::angleToUs(calibration.azimuthTenths/10.0f,0);
  servoCenterUs[1]=constrain(GimbalPid::angleToUs(calibration.elevationDeg,1),ELEVATION_PHYSICAL_MIN_US,ELEVATION_PHYSICAL_MAX_US);
  for(int axis=0;axis<2;++axis)if(calibration.travel[axis].zero)servoCenterUs[axis]=calibration.travel[axis].zero;
  GimbalConfig config=automaticPid.config();
  config.azimuthZero=GIMBAL_AZIMUTH_ZERO_DEG;
  config.travel=servoCalibration.commandTravel(servoCenterUs);
  automaticPid=GimbalPid(config);
  servoActual[0]=servoTarget[0]=GimbalPid::angleToUs(config.azimuthZero,0);
  servoActual[1]=servoTarget[1]=SERVO_CENTER_US;
}

void loadServoCalibration() {
  Preferences storage;
  ServoCalibration calibration=servoCalibration;
  ServoCalibration saved{};
  if(storage.begin("tvc-servo",true)){
    constexpr size_t legacySize=offsetof(ServoCalibration,travel);
    if(storage.getBytesLength("cal-v7")==sizeof(saved)&&
       storage.getBytes("cal-v7",&saved,sizeof(saved))==sizeof(saved)&&saved.valid()){
      calibration=saved;
    } else {
      for(int version=6;version>=1;--version){
        char key[]="cal-v6";
        key[5]='0'+version;
        saved={};
        if(storage.getBytesLength(key)==legacySize&&storage.getBytes(key,&saved,legacySize)==legacySize&&saved.valid()){
          if(version<6)saved.azimuthTenths=calibration.azimuthTenths;
          calibration=saved;
          break;
        }
      }
    }
    storage.end();
  }
  applyServoCalibration(calibration);
}

bool saveServoCalibration(const ServoCalibration &calibration) {
  Preferences storage;
  if(!storage.begin("tvc-servo",false))return false;
  const bool saved=storage.putBytes("cal-v7",&calibration,sizeof(calibration))==sizeof(calibration);
  storage.end();
  return saved;
}

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t c = 0xffffffff;
  for (size_t i = 0; i < length; ++i) {
    c ^= data[i];
    for (int bit = 0; bit < 8; ++bit) c = (c >> 1) ^ ((c & 1) ? 0xedb88320 : 0);
  }
  return c ^ 0xffffffff;
}

// 调用者已持有 stateLock。方位启用位置与当前云台解算的零位一致。
void enableServosAtCenter() {
  servoEnabled=true;
  servoActual[0]=servoTarget[0]=GimbalPid::angleToUs(automaticPid.config().azimuthZero,0);
  servoActual[1]=servoTarget[1]=SERVO_CENTER_US;
  pulse(0,servoActual[0]);
  pulse(1,servoActual[1]);
}

// 调用者已持有 stateLock；当前电子平台测试方案会关闭舵机 PWM 输出。
// 安装实物机构后，可能需要根据测试结果选择不同的保持位置或回中策略。
void stopEscOutput();
void stopServoOutput() {
  if(tiltThrottleEnabled||torquePrepared)stopEscOutput();
  autoEnabled = false; servoEnabled = false;
  servoSweep={};
  automaticPid.reset();
  disablePulse(0); disablePulse(1);
}
void stopEscOutput() {
  if(torquePrepared){
    autoEnabled=false;servoEnabled=false;
    disablePulse(0);disablePulse(1);
  }
  torquePrepared=torqueActive=false;
  thrustTorque.reset();
  tiltThrottleEnabled=false;tiltThrottle.reset();
  escArmed = escArming = false;
  if (escConfigured && escSignalEnabled) { escActual = escTarget = escStop; pulse(2, escStop); }
  else { escActual = escTarget = 0; disablePulse(2); }
}
void stopCgServoOutput() {
  cgServoEnabled=false;
  cgServoDirection=0;
  disablePulse(CG_SERVO_CHANNEL);
}
void stopOutputs() { ++epoch; autoMotorAllowed=false; stopServoOutput(); stopEscOutput(); stopCgServoOutput(); }
void latchFault(const char *reason) {
  if (!fault[0]) { fault = reason; stopOutputs(); }
}
bool freshHeartbeat() { return sessionId[0] && millis() - heartbeatAt < HEARTBEAT_TIMEOUT_MS; }
bool imuUsable(uint64_t now) {
  return imuSampleUsable(imu,now,IMU_TIMEOUT_US);
}
bool imuFresh(uint64_t now) { return imuUsable(now) && now-imu.t<=IMU_FRESH_US; }
bool recoverableImuFault() {
  return !fault[0] || !strcmp(fault,"CALIBRATION_MOVED") || !strcmp(fault,"CALIBRATION_DATA") || !strcmp(fault,"CALIBRATION_STORAGE") ||
    !strcmp(fault,"HEADING_UNDEFINED") || !strcmp(fault,"IMU_TIMEOUT") ||
    !strcmp(fault,"ATTITUDE_GAP") || !strcmp(fault,"ATTITUDE_INVALID") || !strcmp(fault,"IMU_RESTART") ||
    !strcmp(fault,"MOUNT_CALIBRATION") || !strcmp(fault,"THROTTLE_INVALID");
}
bool motorModeAllowed() {
  return operatingMode==MODE_MANUAL || (operatingMode==MODE_ATTITUDE && autoMotorAllowed && worldReady);
}
bool commandNeedsEpoch(const char *command) {
  return !strcmp(command,"auto_motor_enable") || !strcmp(command,"servo_enable") || !strcmp(command,"auto_enable") ||
    !strcmp(command,"esc_config") || !strcmp(command,"esc_arm") ||
    !strcmp(command,"servo") || !strcmp(command,"esc") ||
    !strcmp(command,"cg_servo_enable") || !strcmp(command,"cg_servo_jog") ||
    !strcmp(command,"set_mode") || !strcmp(command,"calibrate") || !strcmp(command,"restore_calibration") || !strcmp(command,"mount_config") ||
    !strcmp(command,"world_zero") || !strcmp(command,"clear_fault") || !strcmp(command,"pid_config") ||
    !strcmp(command,"select_imu") || !strcmp(command,"tilt_throttle_config") || !strcmp(command,"servo_calibrate") ||
    !strcmp(command,"torque_config") || !strcmp(command,"attitude_target") ||
    !strcmp(command,"servo_travel_start") || !strcmp(command,"servo_travel_step") || !strcmp(command,"servo_travel_jog") ||
    !strcmp(command,"servo_travel_zero") || !strcmp(command,"servo_travel_capture") || !strcmp(command,"servo_travel_save");
}
bool controllerLeaseExpired() {
  return controlOwner == -1 && millis()-controllerSeenAt >= SESSION_TIMEOUT_MS;
}
const char *operatingModeName() { return operatingMode == MODE_MANUAL ? "MANUAL" : "ATTITUDE"; }

// 在用户明确要求时重新测量静止参考坐标系。
bool beginCalibrationLocked() {
  if (servoEnabled || cgServoEnabled || escArmed || escArming || !imu.present || !recoverableImuFault()) return false;
  autoMotorAllowed=false;
  ++epoch;
  automaticPid.reset();
  for(int k=0;k<3;++k){imu.biasSum[k]=0;accelSum[k]=0;accelSumSq[k]=0;gyroSumSq[k]=0;}
  calibrationRejectedSamples=0;
  gravityStartupSamples=0;
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
  return source==SOURCE_IMU1 ? (IMU1_LOCAL_I2C ? "IMU1 · 本地 I2C · 0x68" : "IMU1 · SuperMini · 0x68") : "IMU2 · 本地 I2C · 0x69";
}
const char *controlSourceInterface(uint8_t source) {
  return source==SOURCE_IMU1 ? (IMU1_LOCAL_I2C ? "LOCAL_I2C_IMU1" : "SUPERMINI_UART") : "LOCAL_I2C_IMU2";
}
// 该路传感器此刻是否有可用的采集通道。只看“模块是否存在”，
// 不因为暂时没有新样本就拒绝切换，采样年龄由遥测如实报告。
bool controlSourcePresent(uint8_t source) {
  return source==SOURCE_IMU1 ? primaryImu.present : imu2Monitor.present;
}

const char *gravityReferenceKey(){return controlSource==SOURCE_IMU1?"imu1-v1":"imu2-v1";}
int controlOutwardAxis(){return controlSource==SOURCE_IMU1?IMU1_OUTWARD_AXIS:IMU2_OUTWARD_AXIS;}

bool applyGravityReference(const GravityReference &reference,const float *outward=nullptr) {
  for(float bias:reference.bias)if(!isfinite(bias)||fabsf(bias)>=15)return false;
  float matrix[3][3];
  const bool valid=outward?buildMountBasis(reference.up,outward,matrix):
    buildUprightMount(reference.up,matrix,controlMountYawDeg(controlSource),controlOutwardAxis());
  if(!valid)return false;
  memcpy(mountingMatrix,matrix,sizeof(mountingMatrix));
  memcpy(imu.bias,reference.bias,sizeof(imu.bias));
  guidedMount=outward!=nullptr;
  return true;
}

void loadGravityReference() {
  Preferences storage;
  GravityReference reference{};
  savedGravityReference=false;
  if(storage.begin("tvc-gravity",true)){
    const char *key=gravityReferenceKey();
    const size_t size=storage.getBytesLength(key);
    if(size==sizeof(GuidedGravityReference)){
      GuidedGravityReference guided{};
      savedGravityReference=storage.getBytes(key,&guided,sizeof(guided))==sizeof(guided)&&
        applyGravityReference(guided.gravity,guided.outward);
    }else if(size==sizeof(reference)){
      savedGravityReference=storage.getBytes(key,&reference,sizeof(reference))==sizeof(reference)&&
        applyGravityReference(reference);
    }
    storage.end();
  }
  if(!savedGravityReference){
    reference={};
    const float *up=controlSource==SOURCE_IMU1?IMU1_DEFAULT_UP_G:IMU2_DEFAULT_UP_G;
    memcpy(reference.up,up,sizeof(reference.up));
    applyGravityReference(reference);
  }
  gravityStartupSamples=0;
}

bool saveGravityReference(const GravityReference &reference,const float *outward=nullptr) {
  Preferences storage;
  if(!storage.begin("tvc-gravity",false))return false;
  bool saved;
  if(outward){
    const GuidedGravityReference guided{reference,{outward[0],outward[1],outward[2]}};
    saved=storage.putBytes(gravityReferenceKey(),&guided,sizeof(guided))==sizeof(guided);
  }else saved=storage.putBytes(gravityReferenceKey(),&reference,sizeof(reference))==sizeof(reference);
  storage.end();
  return saved;
}

void invalidateGravityState() {
  gravityStartupSamples=0;
  fault="";
  worldReady=calibrated=imu.calibrated=mountingCalibrated=gimbalTiltValid=correctionActive=false;
  imu.calibrationSamples=calibrationRejectedSamples=0;
  attitude.invalidate();
  controlAttitude.invalidate();
  automaticPid.reset();
  lastControlMeasurementUs=0;
}

bool restoreGravityReference() {
  Preferences storage;
  if(!storage.begin("tvc-gravity",false))return false;
  const char *key=gravityReferenceKey();
  const bool removed=!storage.isKey(key)||storage.remove(key);
  storage.end();
  if(!removed)return false;
  loadGravityReference();
  invalidateGravityState();
  ++epoch;
  return true;
}

// 调用者持有 stateLock。切换只改变数据源，不会重新启动任何输出.
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
  imu1ControlConsumed=imu1Monitor.samples;
  worldReady=calibrated=calibrating=mountingCalibrated=gimbalTiltValid=false;
  attitude=Attitude();controlAttitude=Attitude();
  correctionActive=false;lastControlMeasurementUs=0;
  autoZeroRoll=autoZeroPitch=autoErrorRoll=autoErrorPitch=0;
  for(int i=0;i<3;++i){bodyA[i]=bodyG[i]=0;accelSum[i]=accelSumSq[i]=gyroSumSq[i]=0;}
  calibrationRejectedSamples=0;
  referenceTime=esp_timer_get_time();++referenceId;
  if(sensorFault)fault="";
  loadGravityReference();
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

void markGravityReady(uint64_t sampleTime) {
  mountingCalibrated=imu.calibrated=calibrated=worldReady=true;
  referenceTime=sampleTime;
  ++referenceId;
  auto e=attitude.euler();
  imu.roll=e.roll;imu.pitch=e.pitch;
  autoZeroRoll=autoZeroPitch=autoErrorRoll=autoErrorPitch=0;
  automaticPid.reset();
  lastControlMeasurementUs=0;
}

void initializeFromGravity(float normG,uint32_t dtUs) {
  if(worldReady||calibrating||fault[0]||servoEnabled||cgServoEnabled||escArmed||escArming)return;
  const float normMps2=normG*EARTH_GRAVITY_MPS2;
  const bool still=fabsf(normMps2-EARTH_GRAVITY_MPS2)<=0.1f*EARTH_GRAVITY_MPS2&&
    fabsf(bodyG[0])<15&&fabsf(bodyG[1])<15&&fabsf(bodyG[2])<15;
  if(!still||dtUs>50000){gravityStartupSamples=0;return;}
  if(!gravityStartupSamples){
    memcpy(gravityStartupFirst,bodyA,sizeof(gravityStartupFirst));
    memset(gravityStartupSum,0,sizeof(gravityStartupSum));
  }
  for(int i=0;i<3;++i){
    if(fabsf(bodyA[i]-gravityStartupFirst[i])>.06f){gravityStartupSamples=0;return;}
  }
  for(int i=0;i<3;++i)gravityStartupSum[i]+=bodyA[i];
  if(++gravityStartupSamples<GRAVITY_STARTUP_SAMPLES)return;
  const float scale=1.0f/GRAVITY_STARTUP_SAMPLES;
  if(attitude.initializeFromAccel(scale*gravityStartupSum[0],scale*gravityStartupSum[1],scale*gravityStartupSum[2]))markGravityReady(imu.t);
  gravityStartupSamples=0;
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
      initializeFromGravity(norm,dtUs);
}

void sensorTask(void *) {
  for (;;) {
    PrimaryUart::Sample sample;
    if(!IMU1_LOCAL_I2C && pollPrimaryUart(sample)) {
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
    if(IMU1_LOCAL_I2C){
      sampleImu1Monitor();
      xSemaphoreTake(stateLock,portMAX_DELAY);
      primaryImu.present=imu1Monitor.present;
      primaryImu.errors=primaryImu.readErrors=imu1Monitor.errors;
      primaryImu.missed=imu1Monitor.missed;
      if(imu1Monitor.samples!=imu1ControlConsumed && imu1Monitor.fresh(esp_timer_get_time(),IMU2_TIMEOUT_US)){
        imu1ControlConsumed=imu1Monitor.samples;
        for(int k=0;k<3;++k){primaryImu.a[k]=imu1Monitor.accelG[k];primaryImu.g[k]=imu1Monitor.gyroDps[k];}
        primaryImu.temperature=imu1Monitor.temperatureC;
        primaryImu.t=imu1Monitor.sampleUs;primaryImu.valid=true;primaryImu.count=imu1Monitor.samples;
        if(controlSource==SOURCE_IMU1){
          imu.present=primaryImu.present;
          const uint64_t elapsed=imu.t&&primaryImu.t>imu.t?primaryImu.t-imu.t:5000;
          const uint32_t dt=elapsed>IMU_TIMEOUT_US?IMU_TIMEOUT_US+1:uint32_t(elapsed);
          acceptControlSample(primaryImu.a,primaryImu.g,primaryImu.temperature,primaryImu.t,dt,0);
          imu.missed=primaryImu.missed;
          primaryImu.pitch=imu.pitch;primaryImu.roll=imu.roll;
        }
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
    if(torquePrepared && heartbeatWarning)latchFault("TORQUE_HEARTBEAT");
    if(servoSweep.axis>=0 && heartbeatWarning)stopOutputs();
    if ((servoEnabled || cgServoEnabled || escArmed || escArming) && !imuUsable(nowUs)) latchFault("IMU_TIMEOUT");
    if (autoEnabled && !worldReady) latchFault("WORLD_NOT_READY");
    if (calibrating && millis()-calibrationAt >= 3000) {
      bool enough = imu.present && imu.calibrationSamples >= 300 && calibrationRejectedSamples <= 60 && imuFresh(nowUs);
      float meanA[3]={0,0,0};
      if(enough)for(int k=0;k<3;++k){
        const double n=imu.calibrationSamples, mg=imu.biasSum[k]/n, ma=accelSum[k]/n;
        enough=enough && gyroSumSq[k]/n-mg*mg<4.0 && accelSumSq[k]/n-ma*ma<0.0025;
        meanA[k]=ma;
      }
      GravityReference proposed{};
      if(enough)for(int k=0;k<3;++k){
        proposed.up[k]=meanA[k];
        proposed.bias[k]=imu.biasSum[k]/imu.calibrationSamples;
      }
      if(!enough)latchFault(calibrationRejectedSamples>60?"CALIBRATION_MOVED":"CALIBRATION_DATA");
      else if(!applyGravityReference(proposed,guidedMount?mountingMatrix[0]:nullptr))latchFault("MOUNT_CALIBRATION");
      else {
        float meanBody[3];toBody(meanA,meanBody);
        if(!attitude.initializeFromAccel(meanBody[0],meanBody[1],meanBody[2]))latchFault("MOUNT_CALIBRATION");
        else {
          savedGravityReference=saveGravityReference(proposed,guidedMount?mountingMatrix[0]:nullptr);
          if(savedGravityReference){
            markGravityReady(nowUs);
            float corrected[3];for(int k=0;k<3;++k)corrected[k]=imu.g[k]-imu.bias[k];
            toBody(corrected,bodyG);
          } else {
            loadGravityReference();
            attitude.invalidate();
            latchFault("CALIBRATION_STORAGE");
          }
        }
      }
      calibrating=false;
    }
    if(actuatorPwmFault)latchFault("PWM_INIT");
    if (!fault[0]) {
      gimbalTiltValid=false;
      if (worldReady && imuUsable(nowUs)) {
        const uint64_t ageUs=nowUs>=imu.t?nowUs-imu.t:0;
        const bool newMeasurement=imu.t!=lastControlMeasurementUs;
        gimbalTiltValid=projectHistory(attitude,bodyG,ageUs,HISTORY_PREDICTION_US,controlAttitude) &&
                        gimbalLean(controlAttitude.rotation(),autoErrorPitch,autoErrorRoll) &&
                         gimbalControlUpright(controlAttitude.rotation());
        if(torqueActive){
          auto geometry=automaticPid.config();
          geometry.limit=ELEVATION_MAX_DEFLECTION_DEG;
          const TorqueEscStep esc{escStop,escMax,escActual,5};
          if(!imuFresh(nowUs))latchFault("TORQUE_IMU_STALE");
          else if(!gimbalTiltValid || !thrustTorque.update(geometry,nowUs,
                     controlAttitude.rotation(),bodyG,servoActual,servoTarget,&esc))latchFault("TORQUE_CONTROL_INVALID");
          else escTarget=esc.targetUs(thrustTorque.throttlePct);
        } else if (autoEnabled && servoEnabled) {
          if(!gimbalTiltValid)latchFault("ATTITUDE_INVALID");
          else if (automaticPid.update(nowUs,autoErrorPitch,autoErrorRoll,
                                  servoActual,servoTarget,newMeasurement&&ageUs<=IMU_FRESH_US)==GimbalPid::INVALID)
            latchFault("PID_INVALID");
        }
        lastControlMeasurementUs=imu.t;
      }
      // 解锁只等待电调停止脉宽保持时间。姿态模式仍检查有效且新鲜的控制 IMU。
      if(escArming && !fault[0]){
        if(operatingMode==MODE_ATTITUDE && !gimbalTiltValid)latchFault("ATTITUDE_INVALID");
        else if(millis()-armAt>=armHold){
          if(operatingMode==MODE_ATTITUDE && !imuFresh(nowUs))latchFault("IMU_TIMEOUT");
          else{
            if(operatingMode==MODE_ATTITUDE){
              if(!servoEnabled)enableServosAtCenter();
              if(!torquePrepared && !autoEnabled){automaticPid.reset();autoEnabled=true;}
            }
            escArming=false;escArmed=true;
          }
        }
      }
      if(tiltThrottleEnabled){
        if(!autoEnabled||!servoEnabled||!escArmed||fault[0]||!gimbalTiltValid)stopEscOutput();
        else {
          const auto r=controlAttitude.rotation();
          if(!tiltThrottle.update(nowUs,r.m[2][2],imuFresh(nowUs),outwardTiltRateDps(r.m[2],bodyG)))
            latchFault("THROTTLE_INVALID");
          else escTarget=tiltThrottle.pulse(escStop,escMax);
        }
      }
      if(servoSweep.axis>=0){
        servoSweep.tick(millis());
        writePwmUs(servoSweep.axis,servoSweep.current);
      } else if (servoEnabled) {
        const auto &travel=automaticPid.config().travel;
        for(int i=0;i<2;++i){
          servoTarget[i]=travel.clamp(i,servoTarget[i]);
          const int step=constrain(servoTarget[i]-servoActual[i],-SERVO_SLEW_US_PER_TICK,SERVO_SLEW_US_PER_TICK);
          servoActual[i]=travel.clamp(i,servoActual[i]+step);
          pulse(i,servoActual[i]);
        }
      }
      if (cgServoEnabled && cgServoDirection) {
        if (millis()-cgServoJogAt>=CG_SERVO_JOG_TIMEOUT_MS) cgServoDirection=0;
        else {
          cgServoUs=constrain(cgServoUs+cgServoDirection*CG_SERVO_SLEW_US,CG_SERVO_MIN_US,CG_SERVO_MAX_US);
          pulse(CG_SERVO_CHANNEL,cgServoUs);
          if((cgServoDirection>0&&cgServoUs==CG_SERVO_MAX_US)||(cgServoDirection<0&&cgServoUs==CG_SERVO_MIN_US))cgServoDirection=0;
        }
      }
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

bool readMountVector(JsonArrayConst values,float result[3]) {
  if(values.size()!=3)return false;
  for(int i=0;i<3;++i){
    if(!values[i].is<float>())return false;
    result[i]=values[i].as<float>();
    if(!isfinite(result[i]))return false;
  }
  return true;
}

const char *saveGuidedMount(JsonVariantConst doc) {
  if(servoEnabled||cgServoEnabled||escArmed||escArming||calibrating)return "STOP_OUTPUTS_FIRST";
  if(operatingMode!=MODE_ATTITUDE)return "MODE_LOCKED";
  if(!recoverableImuFault())return "FAULT_REQUIRES_RESTART";
  if(!imuFresh(esp_timer_get_time()))return "IMU_NOT_READY";
  if(!integerIn(doc["source_id"],SOURCE_IMU1,SOURCE_IMU2)||doc["source_id"].as<int>()!=controlSource)
    return "INVALID_IMU_SOURCE";
  GravityReference proposed{};
  float leaned[3],outward[3];
  if(!integerIn(doc["direction"],0,3)||
     !readMountVector(doc["upright"].as<JsonArrayConst>(),proposed.up)||
     !readMountVector(doc["leaned"].as<JsonArrayConst>(),leaned)||
     !readMountVector(doc["bias"].as<JsonArrayConst>(),proposed.bias)||
     !guidedMountOutward(proposed.up,leaned,doc["direction"].as<int>(),outward))return "MOUNT_POSE_RANGE";
  for(float bias:proposed.bias)if(fabsf(bias)>=15)return "MOUNT_POSE_RANGE";
  // 仅在返回第一次采样的竖直姿态且保持静止时提交。
  const float norm=std::sqrt(dotMountVector(imu.a,imu.a));
  if(norm<.85f||norm>1.15f||dotMountVector(proposed.up,imu.a)<
     .99863f*std::sqrt(dotMountVector(proposed.up,proposed.up))*norm)return "MOUNT_RETURN_UPRIGHT";
  for(int i=0;i<3;++i)if(fabsf(imu.g[i]-proposed.bias[i])>5)return "MOUNT_RETURN_UPRIGHT";
  if(!saveGravityReference(proposed,outward))return "CALIBRATION_STORAGE";
  applyGravityReference(proposed,outward);
  savedGravityReference=true;
  invalidateGravityState();
  autoMotorAllowed=false;
  ++epoch;
  ++configRevision;
  return nullptr;
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
      ok=true;reason="HELLO";
    }
  } else if(controlOwner!=source || strcmp(sid,sessionId) || !doc["boot"].is<const char*>() || strcmp(doc["boot"].as<const char*>(),bootId)) reason="SESSION_MISMATCH";
  else if(request<=lastCommandSeq) reason="OLD_SEQUENCE";
  else if(strcmp(command,"stop") && strcmp(command,"auto_disable") && strcmp(command,"auto_motor_disable") && strcmp(command,"tilt_throttle_disable") &&
    strcmp(command,"esc_stop") && strcmp(command,"servo_disable") && strcmp(command,"servo_travel_hold") &&
    strcmp(command,"cg_servo_hold") && strcmp(command,"cg_servo_disable") && (!doc["device_t_us"].is<uint64_t>() ||
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
    else if(!strcmp(command,"servo_travel_start")){
      if(servoEnabled||cgServoEnabled||escArmed||escArming)reason="STOP_OUTPUTS_FIRST";
      else if(!ready||!freshHeartbeat())reason="NOT_READY";
      else if(!integerIn(doc["axis"],0,1))reason="SERVO_CALIBRATION_RANGE";
      else {
        const int axis=doc["axis"].as<int>();
        stopOutputs();
        servoSweep.begin(axis,servoCenterUs[axis],doc["explicit_endpoints"]|false);
        servoEnabled=true;
        ok=true;reason="SERVO_TRAVEL_STARTED";
      }
    } else if(!strcmp(command,"servo_travel_hold")){
      servoSweep.hold();ok=true;reason="SERVO_TRAVEL_HOLD";
    } else if(!strcmp(command,"servo_travel_jog")){
      const auto direction=doc["direction"];
      if(servoSweep.axis<0||!ready||!freshHeartbeat())reason="NOT_READY";
      else if(!integerIn(doc["axis"],0,1)||doc["axis"].as<int>()!=servoSweep.axis||
              !direction.is<int>()||(direction.as<int>()!=1&&direction.as<int>()!=-1))reason="SERVO_CALIBRATION_RANGE";
      else{servoSweep.jog(direction.as<int>(),millis());ok=true;reason="SERVO_TRAVEL_JOG";}
    } else if(!strcmp(command,"servo_travel_step") || !strcmp(command,"servo_travel_zero") ||
              !strcmp(command,"servo_travel_capture") || !strcmp(command,"servo_travel_save")){
      if(servoSweep.axis<0||!ready||!freshHeartbeat())reason="NOT_READY";
      else if(servoSweep.jogging||servoSweep.current!=servoSweep.target)reason="SERVO_TRAVEL_MOVING";
      else if(!strcmp(command,"servo_travel_step")){
        if(!integerIn(doc["delta_us"],-ServoTravelSweep::maxStepUs,ServoTravelSweep::maxStepUs)||!doc["delta_us"].as<int>())reason="SERVO_CALIBRATION_RANGE";
        else{servoSweep.nudge(doc["delta_us"].as<int>());++epoch;ok=true;reason="SERVO_TRAVEL_STEP";}
      } else if(!strcmp(command,"servo_travel_zero")){
        servoSweep.captureZero();++epoch;ok=true;reason="SERVO_TRAVEL_ZERO";
      } else if(!strcmp(command,"servo_travel_capture")){
        const char *endpoint=doc["endpoint"]|"";
        if((strcmp(endpoint,"min")&&strcmp(endpoint,"max"))||
           !servoSweep.captureEndpoint(!strcmp(endpoint,"max")))reason="SERVO_TRAVEL_ENDPOINT_SIDE";
        else{++epoch;ok=true;reason="SERVO_TRAVEL_ENDPOINT_RECORDED";}
      } else if(!servoSweep.range.valid(servoSweep.axis))reason="SERVO_TRAVEL_ENDPOINTS";
      else {
        ServoCalibration proposed=servoCalibration;
        const int axis=servoSweep.axis, zero=servoSweep.range.zero;
        proposed.travel[axis]=servoSweep.range;
        if(axis==0)proposed.azimuthTenths=lroundf(GimbalPid::usToAngle(zero,0)*10);
        else proposed.elevationDeg=lroundf(GimbalPid::usToAngle(zero,1));
        if(!saveServoCalibration(proposed))reason="SERVO_CALIBRATION_SAVE";
        else {
          stopOutputs();
          applyServoCalibration(proposed);
          ++configRevision;
          ok=true;reason="SERVO_CALIBRATION_SAVED";
        }
      }
    } else if(servoSweep.axis>=0 && !strcmp(command,"servo_disable")){
      stopOutputs();ok=true;reason="SERVO_DISABLED";
    } else if(servoSweep.axis>=0 && strcmp(command,"set_mode") && strcmp(command,"esc_stop"))
      reason="SERVO_TRAVEL_ACTIVE";
    else if(!strcmp(command,"clear_fault")) {
      if(!servoEnabled && !escArmed && !escArming){fault="";stopOutputs();ok=true;reason="CLEARED";}else reason="STOP_OUTPUTS_FIRST";
    } else if(!strcmp(command,"set_mode")) {
      if(!doc["mode"].is<const char*>())reason="INVALID_MODE";
      else if(!strcmp(doc["mode"].as<const char*>(),"ATTITUDE")) {
        stopOutputs();operatingMode=MODE_ATTITUDE;
        ok=true;reason="MODE_ATTITUDE";
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
    } else if(!strcmp(command,"mount_config")) {
      const char *error=saveGuidedMount(doc.as<JsonVariantConst>());
      ok=error==nullptr;reason=ok?"MOUNT_REFERENCE_SAVED":error;
    } else if(!strcmp(command,"calibrate")) {
      if(!imu.present)reason="IMU_NOT_READY";
      else if(servoEnabled||cgServoEnabled||escArmed||escArming)reason="STOP_OUTPUTS_FIRST";
      else if(beginCalibrationLocked()){ok=true;reason="CALIBRATING";}
      else reason="FAULT_REQUIRES_RESTART";
    } else if(!strcmp(command,"restore_calibration")) {
      if(servoEnabled||cgServoEnabled||escArmed||escArming||calibrating)reason="STOP_OUTPUTS_FIRST";
      else if(!recoverableImuFault())reason="FAULT_REQUIRES_RESTART";
      else if(restoreGravityReference()){ok=true;reason="CALIBRATION_DEFAULT_RESTORED";}
      else reason="CALIBRATION_STORAGE";
    } else if(!strcmp(command,"world_zero")) {
      if(servoEnabled||escArmed||escArming||calibrating)reason="STOP_OUTPUTS_FIRST";
      else if(!worldReady||!imuFresh(esp_timer_get_time()))reason="WORLD_NOT_READY";
      else if(fabsf(bodyG[0])>2||fabsf(bodyG[1])>2||fabsf(bodyG[2])>2)reason="KEEP_STILL";
      else if(!attitude.resetYaw())reason="HEADING_UNDEFINED";
      else{automaticPid.reset();referenceTime=esp_timer_get_time();++referenceId;++epoch;ok=true;reason="WORLD_HEADING_ZEROED";}
    } else if(!strcmp(command,"servo_calibrate")) {
      if(servoEnabled||cgServoEnabled||escArmed||escArming||calibrating)reason="STOP_OUTPUTS_FIRST";
      else if(!doc["azimuth_deg"].is<float>()||!isfinite(doc["azimuth_deg"].as<float>())||doc["azimuth_deg"].as<float>()<0||doc["azimuth_deg"].as<float>()>360||!integerIn(doc["elevation_deg"],140,210)||
              !doc["azimuth_reversed"].is<bool>()||!doc["elevation_reversed"].is<bool>())reason="SERVO_CALIBRATION_RANGE";
      else {
        ServoCalibration proposed{uint16_t(lroundf(doc["azimuth_deg"].as<float>()*10)),doc["elevation_deg"].as<uint16_t>(),
          doc["azimuth_reversed"].as<bool>(),doc["elevation_reversed"].as<bool>()};
        proposed.travel[0]=servoCalibration.travel[0];proposed.travel[1]=servoCalibration.travel[1];
        if((proposed.travel[0].zero&&proposed.azimuthTenths!=servoCalibration.azimuthTenths)||
           (proposed.travel[1].zero&&proposed.elevationDeg!=servoCalibration.elevationDeg))reason="SERVO_TRAVEL_ZERO_LOCKED";
        else if(!proposed.elevationHasBothDirections())reason="ELEVATION_CENTER_AT_LIMIT";
        else if(!saveServoCalibration(proposed))reason="SERVO_CALIBRATION_SAVE";
        else {
          applyServoCalibration(proposed);
          ++epoch;++configRevision;
          ok=true;reason="SERVO_CALIBRATION_SAVED";
        }
      }
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
          const char *trimKeys[]={"trim_x_deg","trim_y_deg"};
          for(int k=0;k<2;++k)if(doc.containsKey(trimKeys[k])){
            numbers=numbers && doc[trimKeys[k]].is<float>() && !next.model.enabled;
            next.trim[k]=doc[trimKeys[k]].as<float>();
          }
        }
        GimbalPid proposed(next);
        if(!numbers||next.x.kp>10||next.y.kp>10||next.x.ki>2||next.y.ki>2||next.x.kd>2||next.y.kd>2||!proposed.configurationValid())reason="PID_CONFIG";
        else if(next.azimuthZero!=GIMBAL_AZIMUTH_ZERO_DEG)reason="USE_SERVO_CALIBRATION";
        else{automaticPid=proposed;++epoch;ok=true;reason="PID_CONFIGURED";}
      }
    } else if(!strcmp(command,"torque_config")) {
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else if(servoEnabled||escArmed||escArming||calibrating)reason="STOP_OUTPUTS_FIRST";
      else {
        const char *keys[]={"full_torque_nm","base_pct","max_pct","kp_x","ki_x","kd_x","kp_y","ki_y","kd_y"};
        bool numbers=true;
        for(const char *key:keys)numbers=numbers&&doc[key].is<float>();
        ThrustTorqueConfig next;
        next.fullTorqueNm=doc["full_torque_nm"].as<float>();
        next.basePct=doc["base_pct"].as<float>();next.maximumPct=doc["max_pct"].as<float>();
        next.gains[0]={doc["kp_x"].as<float>(),doc["ki_x"].as<float>(),doc["kd_x"].as<float>()};
        next.gains[1]={doc["kp_y"].as<float>(),doc["ki_y"].as<float>(),doc["kd_y"].as<float>()};
        if(doc.containsKey("feedforward")){
          const auto values=doc["feedforward"].as<JsonArrayConst>();
          numbers=numbers&&values.size()==6;
          for(int i=0;i<6;++i)numbers=numbers&&values[i].is<float>();
          for(int k=0;k<2;++k){
            next.trimNm[k]=values[k].as<float>();
            for(int j=0;j<2;++j)next.gravityNmPerDeg[k][j]=values[2+2*k+j].as<float>();
          }
        }
        if(!numbers||!thrustTorque.configure(next))reason="TORQUE_CONFIG";
        else{++epoch;ok=true;reason="TORQUE_CONFIGURED";}
      }
    } else if(!strcmp(command,"attitude_target")) {
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else if(!ready || !worldReady || !gimbalTiltValid)reason="NOT_READY";
      else if((escArmed||escArming)&&!torquePrepared)reason="STOP_ESC_FIRST";
      else if(torquePrepared&&!torqueActive)reason="TORQUE_NOT_ACTIVE";
      else if(!doc["x_deg"].is<float>() || !doc["y_deg"].is<float>() ||
              !thrustTorque.setTarget(doc["x_deg"].as<float>(),doc["y_deg"].as<float>()))reason="TARGET_RANGE";
      else{++epoch;ok=true;reason="TARGET_ACCEPTED";}
    } else if(!strcmp(command,"servo_enable")) {
      if(operatingMode!=MODE_MANUAL)reason="MODE_LOCKED";
      else if(ready && !servoEnabled){automaticPid.reset();autoEnabled=false;++epoch;enableServosAtCenter();ok=true;reason="SERVO_ENABLED";}else reason="NOT_READY";
    } else if(!strcmp(command,"servo_disable")) {
      if(operatingMode!=MODE_MANUAL)reason="MODE_LOCKED";
      else{++epoch;stopServoOutput();ok=true;reason="SERVO_DISABLED";}
    } else if(!strcmp(command,"servo")) {
      if(operatingMode!=MODE_MANUAL)reason="MODE_LOCKED";
      else if(!ready || !servoEnabled)reason="NOT_READY";
      else if(!doc["epoch"].is<uint32_t>() || doc["epoch"].as<uint32_t>()!=epoch)reason="OLD_EPOCH";
      else if(!integerIn(doc["x"],std::min<int>(SERVO_MIN_US,automaticPid.config().travel.low[0]),
                                  std::max<int>(SERVO_MAX_US,automaticPid.config().travel.high[0])))reason="RANGE";
      else if(!integerIn(doc["y"],ELEVATION_MIN_US,ELEVATION_MAX_US))reason="ELEVATION_LIMIT";
      else{servoTarget[0]=doc["x"];servoTarget[1]=doc["y"];ok=true;reason="SERVO_ACCEPTED";}
    } else if(!strcmp(command,"cg_servo_enable")) {
      if(escArmed||escArming)reason="STOP_ESC_FIRST";
      else if(!ready)reason="NOT_READY";
      else {
        pulse(CG_SERVO_CHANNEL,cgServoUs);
        if(actuatorPwmFault){latchFault("PWM_INIT");reason="PWM_INIT";}
        else{cgServoEnabled=true;ok=true;reason="CG_SERVO_ENABLED";}
      }
    } else if(!strcmp(command,"cg_servo_jog")) {
      const auto direction=doc["direction"];
      if(escArmed||escArming)reason="STOP_ESC_FIRST";
      else if(!ready||!cgServoEnabled)reason="NOT_READY";
      else if(!direction.is<int>()||(direction.as<int>()!=1&&direction.as<int>()!=-1))reason="RANGE";
      else {
        cgServoDirection=direction.as<int>();
        if((cgServoDirection>0&&cgServoUs>=CG_SERVO_MAX_US)||(cgServoDirection<0&&cgServoUs<=CG_SERVO_MIN_US))cgServoDirection=0;
        cgServoJogAt=millis();
        ok=true;reason="CG_SERVO_JOG";
      }
    } else if(!strcmp(command,"cg_servo_hold")) {
      cgServoDirection=0;
      ok=true;reason="CG_SERVO_HOLD";
    } else if(!strcmp(command,"cg_servo_disable")) {
      if(escArmed||escArming)reason="STOP_ESC_FIRST";
      else{stopCgServoOutput();ok=true;reason="CG_SERVO_DISABLED";}
    } else if(!strcmp(command,"esc_config")) {
      if(!motorModeAllowed())reason="MODE_LOCKED";
      else if(escArmed||escArming)reason="STOP_ESC_FIRST";
      else if(!doc["confirmed"].as<bool>()||!integerIn(doc["stop_us"],900,2100)||!integerIn(doc["min_us"],900,2100)||!integerIn(doc["max_us"],900,2100)||!integerIn(doc["hold_ms"],1000,15000)||doc["hz"].as<int>()!=50)reason="INVALID_ESC_CONFIG";
      else if(doc["min_us"].as<int>()>=doc["stop_us"].as<int>() || doc["max_us"].as<int>()<=doc["stop_us"].as<int>())reason="STOP_NOT_INSIDE_RANGE";
      else {
        const int requestedStop=doc["stop_us"].as<int>();
        ++epoch;stopEscOutput();
        // 已输出的停止中位相同则保持 PWM，避免每次重新配置都让电调丢失信号。
        // 更改停止中位后，只有显式解锁才开始输出新的中位。
        if(escSignalEnabled && requestedStop!=escStop){
          disablePulse(2);escSignalEnabled=false;escActual=escTarget=0;
        }
        escStop=requestedStop;escMin=doc["min_us"];escMax=doc["max_us"];armHold=doc["hold_ms"];
        escConfigured=true;++configRevision;ok=true;reason="ESC_CONFIGURED";
      }
    } else if(!strcmp(command,"esc_arm")) {
      const bool useTorque=doc["thrust_torque"].as<bool>();
      if(!motorModeAllowed())reason="MODE_LOCKED";
      else if(doc.containsKey("thrust_torque")&&!doc["thrust_torque"].is<bool>())reason="TORQUE_CONFIG";
      else if(useTorque&&(operatingMode!=MODE_ATTITUDE||!thrustTorque.config().valid()))reason="TORQUE_CONFIG";
      else if(useTorque&&!freshHeartbeat())reason="TORQUE_NOT_READY";
      else if(cgServoDirection)reason="CG_SERVO_MOVING";
      else if(!ready||!escConfigured)reason="NOT_READY";
      else if(!doc["config_rev"].is<uint32_t>()||doc["config_rev"].as<uint32_t>()!=configRevision)reason="CONFIG_MISMATCH";
      else if(operatingMode==MODE_ATTITUDE && (!worldReady||!mountingCalibrated||!gimbalTiltValid||!imuFresh(esp_timer_get_time())))reason="WORLD_NOT_READY";
      else {
        ++epoch;stopEscOutput();
        if(useTorque){
          autoEnabled=false;automaticPid.reset();
          torquePrepared=true;
        }
        escSignalEnabled=true;escActual=escTarget=escStop;pulse(2,escStop);escArming=true;armAt=millis();ok=true;reason="ARMING";
      }
    } else if(!strcmp(command,"esc")) {
      if(!motorModeAllowed())reason="MODE_LOCKED";
      else if(tiltThrottleEnabled||torquePrepared)reason="AUTO_THROTTLE_ACTIVE";
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
        const auto rateGain=doc["rate_gain_pct_dps"];
        numbers=numbers&&(!doc.containsKey("rate_gain_pct_dps")||rateGain.is<float>());
        TiltThrottle proposed;proposed.config={doc["base_pct"].as<float>(),doc["gain_pct_deg"].as<float>(),doc["max_pct"].as<float>(),doc["deadband_deg"].as<float>(),doc["ramp_pct_s"].as<float>(),rateGain.as<float>()};
        if(!numbers||!proposed.valid())reason="THROTTLE_CONFIG";
        else{tiltThrottle=proposed;++epoch;ok=true;reason="THROTTLE_CONFIGURED";}
      }
    } else if(!strcmp(command,"auto_enable")) {
      const bool useTorque=doc["thrust_torque"].as<bool>();
      if(operatingMode!=MODE_ATTITUDE)reason="MODE_LOCKED";
      else if(escArming)reason="ESC_ARMING";
      else if(doc.containsKey("thrust_torque")&&!doc["thrust_torque"].is<bool>())reason="TORQUE_CONFIG";
      else if(useTorque!=torquePrepared)reason="TORQUE_NOT_READY";
      else if(useTorque&&(!escArmed||!servoEnabled||!imuFresh(esp_timer_get_time())||
                          doc["tilt_throttle"].as<bool>()))reason="TORQUE_NOT_READY";
      else if(!doc["tilt_throttle"].isNull()&&!doc["tilt_throttle"].is<bool>())reason="THROTTLE_CONFIG";
      else if(!ready||!worldReady||!mountingCalibrated||!gimbalTiltValid||((escArmed||escArming)&&!autoMotorAllowed))reason="NOT_READY";
      else if(doc["tilt_throttle"].as<bool>()&&(!autoMotorAllowed||!escArmed||escArming||!escConfigured||!tiltThrottle.valid()))reason="THROTTLE_NOT_READY";
      else{tiltThrottle.reset();tiltThrottleEnabled=doc["tilt_throttle"].as<bool>();++epoch;
        if(useTorque&&!torqueActive){thrustTorque.reset();torqueActive=true;}
        if(!autoEnabled){automaticPid.reset();autoEnabled=true;if(!servoEnabled)enableServosAtCenter();}
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
  const bool local=source==SOURCE_IMU2 || IMU1_LOCAL_I2C;
  const auto &monitor=source==SOURCE_IMU1?imu1Monitor:imu2Monitor;
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
    :usable?"OK":s.count?"STALE":(local?(monitor.errors?"READ_ERROR":"WAIT_INT"):"WAIT_SAMPLE");
  item["pitch"]=s.pitch;item["roll"]=s.roll;item["temperature"]=s.temperature;
  item["calibration_samples"]=s.calibrationSamples;
  item["calibration_rejected"]=source==controlSource?calibrationRejectedSamples:0;
  if(source==SOURCE_IMU1 && !IMU1_LOCAL_I2C){
    item["uart_replies"]=primaryReplies;item["uart_bad_frames"]=primaryBadFrames;
    item["uart_rejected"]=primaryRejected;item["uart_timeouts"]=primaryTimeouts;item["c3_boot"]=primaryRemoteBoot;
  }else{
    item["i2c_errors"]=monitor.errors;item["irq_missed"]=monitor.missed;
    item["int_pin"]=source==SOURCE_IMU1?IMU1_INT_PIN:IMU2_INT_PIN;
    item["irq_count"]=source==SOURCE_IMU1?uint32_t(imu1IrqCount):uint32_t(imu2IrqCount);
    item["sda_pin"]=SDA_PIN;item["scl_pin"]=SCL_PIN;
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
  // servo_us 保留逻辑舵角指令的脉宽表示，供 Studio 和历史记录计算角度。
  auto servoPwm=outputs.createNestedArray("servo_pwm_us");
  for(int k=0;k<2;++k)servoPwm.add(servoSweep.axis>=0?(servoSweep.axis==k?servoSweep.current:0):
      (servoEnabled?servoOutputUs(k,servoActual[k]):0));
  auto cgServo=outputs.createNestedObject("cg_servo");
  cgServo["enabled"]=cgServoEnabled;
  cgServo["command_us"]=cgServoUs;
  cgServo["max_us"]=CG_SERVO_MAX_US;
  cgServo["direction"]=cgServoDirection;
  outputs["esc_configured"]=escConfigured;outputs["esc_armed"]=escArmed;outputs["esc_arming"]=escArming;outputs["esc_us"]=escActual;
  outputs["esc_signal_enabled"]=escSignalEnabled;
  outputs["esc_stop_us"]=escStop;outputs["esc_min_us"]=escMin;outputs["esc_max_us"]=escMax;
  auto control=doc.createNestedObject("control");control["auto_enabled"]=autoEnabled;control["auto_motor_allowed"]=autoMotorAllowed;
  control["preposition_active"]=false; // 协议 10 的旧客户端仍依赖此字段。
  control["source_id"]=controlSource;
  control["source_address"]=controlSourceAddress(controlSource);
  control["source_label"]=controlSourceLabel(controlSource);
  auto error=control.createNestedArray("error_deg");error.add(autoErrorRoll);error.add(autoErrorPitch);
  auto zero=control.createNestedArray("zero_deg");zero.add(autoZeroRoll);zero.add(autoZeroPitch);
  control["algorithm"]=torqueActive?"THRUST_TORQUE":"VECTOR_PID";control["geometry"]="AZIMUTH_ELEVATION";
  auto torque=control["thrust_torque"].to<JsonObject>();
  torque["active"]=torqueActive;torque["prepared"]=torquePrepared;
  torque["limited"]=thrustTorque.limited;
  torque["travel_limited"]=thrustTorque.travelLimited;
  torque["azimuth_rewind"]=thrustTorque.rewinding();
  torque["throttle_target_pct"]=thrustTorque.throttlePct;
  auto target=torque["target_deg"].to<JsonArray>();
  auto lean=torque["lean_deg"].to<JsonArray>();
  auto demand=torque["requested_nm"].to<JsonArray>();
  auto allocated=torque["allocated_nm"].to<JsonArray>();
  lean.add(autoErrorPitch);lean.add(autoErrorRoll);
  for(int k=0;k<2;++k){
    target.add(thrustTorque.target()[k]);demand.add(thrustTorque.requestedNm[k]);allocated.add(thrustTorque.allocatedNm[k]);
  }
  control["mount_ready"]=mountingCalibrated;control["tilt_valid"]=gimbalTiltValid;
  control["gravity_reference"]=savedGravityReference?"MANUAL":"DEFAULT";
  control["mount_method"]="GUIDED_TILT";
  control["mount_reference_ready"]=guidedMount;
  control["mount_yaw_deg"]=controlMountYawDeg(controlSource);
  auto ranges=outputs.createNestedArray("servo_range_deg");ranges.add(360);ranges.add(270);
  const auto &commandTravel=automaticPid.config().travel;
  auto azimuthLimits=outputs.createNestedArray("azimuth_limits_deg");
  azimuthLimits.add(GimbalPid::usToAngle(commandTravel.low[0],0));
  azimuthLimits.add(GimbalPid::usToAngle(commandTravel.high[0],0));
  auto elevationLimits=outputs.createNestedArray("elevation_limits_deg");
  elevationLimits.add(commandTravel.calibrated?GimbalPid::usToAngle(commandTravel.low[1],1):ELEVATION_MIN_DEG);
  elevationLimits.add(commandTravel.calibrated?GimbalPid::usToAngle(commandTravel.high[1],1):ELEVATION_MAX_DEG);
  auto throttle=control.createNestedObject("tilt_throttle");
  throttle["enabled"]=tiltThrottleEnabled;throttle["tilt_deg"]=tiltThrottle.tilt;throttle["target_pct"]=tiltThrottle.requested;throttle["output_pct"]=tiltThrottle.percent;
  throttle["rate_dps"]=tiltThrottle.rate;throttle["rate_mode"]="OUTWARD";throttle["rate_gain_pct_dps"]=tiltThrottle.config.rateGain;
  throttle["base_pct"]=tiltThrottle.config.base;throttle["gain_pct_deg"]=tiltThrottle.config.gain;throttle["max_pct"]=tiltThrottle.config.maximum;throttle["deadband_deg"]=tiltThrottle.config.deadband;throttle["ramp_pct_s"]=tiltThrottle.config.ramp;
  auto angles=outputs.createNestedArray("servo_deg");for(int k=0;k<2;++k)angles.add(GimbalPid::usToAngle(servoActual[k],k));
  const auto &cfg=automaticPid.config();
  auto calibration=control.createNestedObject("servo_calibration");
  calibration["azimuth_deg"]=servoCalibration.azimuthTenths/10.0f;
  calibration["elevation_deg"]=servoCalibration.elevationDeg;
  calibration["azimuth_reversed"]=bool(servoCalibration.azimuthReversed);
  calibration["elevation_reversed"]=bool(servoCalibration.elevationReversed);
  auto travel=control.createNestedObject("servo_travel");
  travel["axis"]=servoSweep.axis;
  travel["capture_endpoints"]=true;travel["explicit_endpoints"]=servoSweep.explicitEndpoints;
  travel["jogging"]=servoSweep.jogging;
  travel["current_us"]=servoSweep.current;travel["target_us"]=servoSweep.target;
  travel["min_us"]=servoSweep.range.min;travel["zero_us"]=servoSweep.range.zero;travel["max_us"]=servoSweep.range.max;
  auto savedTravel=travel.createNestedArray("saved_us");
  for(const auto &range:servoCalibration.travel){
    auto points=savedTravel.createNestedArray();
    points.add(range.min);points.add(range.zero);points.add(range.max);
  }
  auto effectiveTravel=travel.createNestedArray("effective_us");
  for(int axis=0;axis<2;++axis){
    const auto range=servoCalibration.outputRange(axis,servoCenterUs[axis]);
    auto points=effectiveTravel.createNestedArray();
    points.add(range.min);points.add(range.zero);points.add(range.max);
  }
  control["model_enabled"]=cfg.model.enabled;
  control["azimuth_zero_deg"]=cfg.azimuthZero;control["sign_x"]=cfg.signX;control["sign_y"]=cfg.signY;
  auto mount=control.createNestedArray("mount_matrix");for(int r=0;r<3;++r)for(int c=0;c<3;++c)mount.add(mountingMatrix[r][c]);
  auto pid=control.createNestedObject("pid");pid["dt_s"]=automaticPid.dt();
  pid["output_limit_deg"]=cfg.limit;pid["limited"]=automaticPid.limited();
  pid["trim_x_deg"]=cfg.trim[0];pid["trim_y_deg"]=cfg.trim[1];
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
  for(int i=0;i<=CG_SERVO_CHANNEL;++i)disablePulse(i);
  Serial.begin(SERIAL_BAUD);
  snprintf(bootId,sizeof(bootId),"%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random());
  stateLock=xSemaphoreCreateMutex();
  loadServoCalibration();
  loadGravityReference();
  if(!automaticPid.configurationValid())latchFault("PID_CONFIG");
  Wire.begin(SDA_PIN,SCL_PIN,400000);Wire.setTimeOut(3);
  if(IMU1_LOCAL_I2C)beginImu1Monitor();
  else beginPrimaryUart();
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
  if(!IMU1_LOCAL_I2C)printPrimaryDebug();
  delay(1);
}
