#pragma once
#include <cmath>
#include <cstdint>

// 纯数据处理，不依赖执行器状态或主 IMU 的状态。
struct Imu2MonitorState {
  bool present = false;
  bool valid = false;
  bool tiltValid = false;
  uint64_t sampleUs = 0;
  uint32_t samples = 0, errors = 0, missed = 0;
  float accelG[3] = {0,0,0}, gyroDps[3] = {0,0,0};
  float temperatureC = 0, accelRollDeg = 0, accelPitchDeg = 0;

  bool fresh(uint64_t nowUs, uint32_t timeoutUs) const {
    return present && valid && samples != 0 && nowUs >= sampleUs &&
           nowUs - sampleUs < timeoutUs;
  }

  void reject(uint32_t skipped) {
    missed += skipped;
    ++errors;
    valid = false;
    tiltValid = false;
  }

  void accept(const uint8_t bytes[14], uint64_t timestampUs,
              uint32_t skipped, const float sensorToBody[3][3]) {
    int16_t raw[7];
    for (int i=0;i<7;++i)
      raw[i] = int16_t((uint16_t(bytes[2*i]) << 8) | bytes[2*i+1]);
    for (int i=0;i<3;++i) {
      accelG[i] = raw[i] / 8192.0f;  // 加速度量程为 ±4 g，与 IMU1 相同。
      gyroDps[i] = raw[i+4] / 65.5f; // 角速度量程为 ±500°/秒，保留未经校正的传感器轴数据。
    }
    temperatureC = raw[3] / 340.0f + 36.53f;
    float body[3] = {0,0,0};
    for (int r=0;r<3;++r)
      for (int c=0;c<3;++c) body[r] += sensorToBody[r][c] * accelG[c];
    const float norm = std::sqrt(body[0]*body[0]+body[1]*body[1]+body[2]*body[2]);
    tiltValid = std::isfinite(norm) && norm > 0.85f && norm < 1.15f;
    if (tiltValid) {
      const float degrees = 57.29577951308232f;
      accelRollDeg = std::atan2(body[1], body[2]) * degrees;
      accelPitchDeg = std::atan2(-body[0], std::hypot(body[1],body[2])) * degrees;
    }
    sampleUs = timestampUs;
    missed += skipped;
    ++samples;
    valid = true;
  }
};

