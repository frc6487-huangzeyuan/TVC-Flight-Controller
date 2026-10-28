#pragma once
#include <cstdint>

// 真实采集缓存与当前控制状态使用相同的数据结构，但互不覆盖。
struct Imu {
  uint8_t address;
  explicit Imu(uint8_t addr) : address(addr) {}
  bool present=false,valid=false,initialized=false,calibrated=false;
  uint64_t t=0;
  uint32_t count=0,errors=0,missed=0,readErrors=0,raceDrops=0;
  float a[3]={0,0,0},g[3]={0,0,0},bias[3]={0,0,0};
  float pitch=0,roll=0,temperature=0;
  double biasSum[3]={0,0,0};
  uint32_t calibrationSamples=0;
};
inline bool imuSampleUsable(const Imu &s,uint64_t now,uint32_t timeoutUs) {
  return s.present&&s.valid&&s.t&&now>=s.t&&now-s.t<timeoutUs;
}
