#pragma once
#include <cmath>
#include <cstdint>

// 不依赖 Arduino 的 PID 运算；输入为机体欧拉角，单位为度，时间单位为秒。
// 输出以及 P/I/D 各项的贡献均为舵机修正角度，尚未乘方向系数。
struct PidGains { float kp, ki, kd; };
struct PidTerms { float error, p, i, d, output; };

class AnglePid {
 public:
  AnglePid(PidGains gains, float limit, float integralLimit, float filterSeconds)
      : gains_(gains), limit_(limit), integralLimit_(integralLimit),
        filterSeconds_(filterSeconds) { reset(); }

  void reset() {
    terms_ = {0,0,0,0,0}; previous_ = filteredRate_ = 0; primed_ = false;
  }
  void prime(float measurement) {
    reset(); previous_ = measurement; primed_ = true;
  }
  const PidTerms &terms() const { return terms_; }
  PidGains gains() const { return gains_; }

  bool configurationValid() const {
    return finite(gains_.kp) && finite(gains_.ki) && finite(gains_.kd) &&
           gains_.kp >= 0 && gains_.ki >= 0 && gains_.kd >= 0 &&
           finite(limit_) && limit_ > 0 && finite(integralLimit_) &&
           integralLimit_ >= 0 && integralLimit_ <= limit_ &&
           finite(filterSeconds_) && filterSeconds_ >= 0;
  }

  // 调用者提供执行器在本次控制周期内能够到达的输出范围，
  // 条件积分同时考虑总行程限制和单周期转动速度限制。
  bool update(float target, float measurement, float dt,
              float reachableLow, float reachableHigh, bool allowIntegral = true) {
    if (!configurationValid() || !finite(target) || !finite(measurement) ||
        !finite(dt) || dt < 0.0001f || dt > 0.05f ||
        !finite(reachableLow) || !finite(reachableHigh)) return false;
    const float low = maximum(-limit_, reachableLow);
    const float high = minimum(limit_, reachableHigh);
    if (low > high) return false;

    const float error = angleDifference(target, measurement);     //误差
    const float rate = primed_ ? angleDifference(measurement, previous_) / dt : 0;
    const float alpha = filterSeconds_ > 0 ? dt / (filterSeconds_ + dt) : 1;
    const float filtered = filteredRate_ + alpha * (rate - filteredRate_);
    const float p = gains_.kp * error;           //P
    // 对测量值求微分；目标不变时，误差变化率 de/dt = -dy/dt。
    // 因此改变目标值不会产生微分冲击。
    const float d = -gains_.kd * filtered;            //D
    const float increment = gains_.ki * error * dt;   //I
    const float candidate = clamp(terms_.i + increment,      
                                  -integralLimit_, integralLimit_);
    if (!finite(error) || !finite(filtered) || !finite(p) || !finite(d) ||
        !finite(increment) || !finite(candidate) || !finite(p+candidate+d))
      return false;
    float i = terms_.i;
    const float requested = p + candidate + d;
    // 只有积分会把已受限的输出继续推向限幅外侧时，才拒绝这次积分，
    // 反方向的积分仍然可以消除之前积累的偏置。
    if ((allowIntegral || candidate * i < 0 || std::fabs(candidate) <= std::fabs(i)) &&
        !((requested > high && candidate > i) ||
          (requested < low && candidate < i))) i = candidate;
    terms_ = {error, p, i, d, clamp(p+i+d, low, high)};
    previous_ = measurement; filteredRate_ = filtered; primed_ = true;
    return true;
  }

  static float angleDifference(float target, float measurement) {
    return std::remainder(target-measurement, 360.0f);
  }

 private:
  static bool finite(float x) { return std::isfinite(x); }
  static float minimum(float a, float b) { return a < b ? a : b; }
  static float maximum(float a, float b) { return a > b ? a : b; }
  static float clamp(float x, float low, float high) {
    return minimum(maximum(x, low), high);
  }
  PidGains gains_;
  float limit_, integralLimit_, filterSeconds_, previous_, filteredRate_;
  bool primed_;
  PidTerms terms_;
};

struct AutoLevelPidConfig {
  PidGains pitch, roll;
  float correctionLimitDeg, integralLimitDeg, derivativeFilterSeconds;
  int servoMinUs, servoMaxUs, servoCenterUs, servoStepUs;
  // 保留原方向配置的含义：方向系数乘以“当前角度－参考角度”。
  float servoXSign, servoYSign;
  uint32_t maxSampleGapUs;
};

class AutoLevelPid {
 public:
  enum Result { WAITING, UPDATED, INVALID };
  explicit AutoLevelPid(const AutoLevelPidConfig &config)
      : config_(config),
        pitch_(config.pitch,config.correctionLimitDeg,config.integralLimitDeg,
               config.derivativeFilterSeconds),
        roll_(config.roll,config.correctionLimitDeg,config.integralLimitDeg,
              config.derivativeFilterSeconds) { reset(); }

  void reset() { pitch_.reset(); roll_.reset(); sampleUs_ = 0; dt_ = 0; }
  const AnglePid &pitch() const { return pitch_; }
  const AnglePid &roll() const { return roll_; }
  float dt() const { return dt_; }

  bool configurationValid() const {
    const float scale = usPerDegree();
    const float travel = config_.correctionLimitDeg * scale;
    return pitch_.configurationValid() && roll_.configurationValid() &&
      config_.servoMinUs > 0 && config_.servoMaxUs < 20000 &&
      config_.servoMaxUs > config_.servoMinUs && config_.servoStepUs > 0 &&
      config_.servoCenterUs-travel >= config_.servoMinUs &&
      config_.servoCenterUs+travel <= config_.servoMaxUs &&
      (config_.servoXSign == -1 || config_.servoXSign == 1) &&
      (config_.servoYSign == -1 || config_.servoYSign == 1) &&
      config_.maxSampleGapUs >= 100 && config_.maxSampleGapUs <= 50000;
  }

  // 由现有的 100 Hz 输出任务在持有 stateLock 时调用，
  // 使用最新的 IMU1 时间戳；重复样本不会再次积分或微分。
  // 复位后的第一个样本只初始化微分项，目标舵机位置保持在中位。
  Result update(uint64_t sampleUs, float roll, float pitch,
                float zeroRoll, float zeroPitch,
                const int actualUs[2], int targetUs[2]) {
    if (!configurationValid() || !sampleUs || !std::isfinite(roll) ||
        !std::isfinite(pitch) || !std::isfinite(zeroRoll) ||
        !std::isfinite(zeroPitch)) return INVALID;
    for (int k=0;k<2;++k)
      if (actualUs[k] < config_.servoMinUs || actualUs[k] > config_.servoMaxUs)
        return INVALID;
    if (sampleUs_ && sampleUs < sampleUs_) return INVALID;
    if (sampleUs == sampleUs_) return WAITING;
    if (!sampleUs_) {
      pitch_.prime(pitch); roll_.prime(roll); sampleUs_ = sampleUs;
      return WAITING;
    }
    const uint64_t elapsed = sampleUs-sampleUs_;
    if (elapsed < 100 || elapsed > config_.maxSampleGapUs) return INVALID;
    const float dt = float(elapsed)/1e6f;
    const float scale = usPerDegree(), step = config_.servoStepUs/scale;
    // PID 使用“参考角度－当前角度”，因此这里对配置的方向系数取负，
    // 使 Ki=Kd=0、Kp=1 时的比例修正方向与原方向配置一致。
    const float xDirection = -config_.servoXSign;
    const float yDirection = -config_.servoYSign;
    const float x = xDirection*(actualUs[0]-config_.servoCenterUs)/scale;
    const float y = yDirection*(actualUs[1]-config_.servoCenterUs)/scale;
    if (!pitch_.update(zeroPitch,pitch,dt,x-step,x+step) ||
        !roll_.update(zeroRoll,roll,dt,y-step,y+step)) return INVALID;
    targetUs[0] = servoTarget(xDirection*pitch_.terms().output);
    targetUs[1] = servoTarget(yDirection*roll_.terms().output);
    sampleUs_ = sampleUs; dt_ = dt;
    return UPDATED;
  }

 private:
  float usPerDegree() const {
    return float(config_.servoMaxUs-config_.servoMinUs)/270.0f;
  }
  int servoTarget(float correctionDeg) const {
    const int us = int(std::lround(config_.servoCenterUs+
                                    correctionDeg*usPerDegree()));
    return us < config_.servoMinUs ? config_.servoMinUs :
           (us > config_.servoMaxUs ? config_.servoMaxUs : us);
  }
  AutoLevelPidConfig config_;
  AnglePid pitch_, roll_;
  uint64_t sampleUs_;
  float dt_;
};
