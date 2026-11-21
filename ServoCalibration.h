#pragma once
#include <cstdint>
#include "ServoLimits.h"

// 所有位置均为舵机线上输出的脉宽（µs）。
struct ServoTravelRange {
  uint16_t min=0, zero=0, max=0;

  static int lower(int axis) { return axis==0?500:ELEVATION_PHYSICAL_MIN_US; }
  static int upper(int axis) { return axis==0?2500:ELEVATION_PHYSICAL_MAX_US; }
  bool valid(int axis) const {
    return min>=lower(axis) && min<zero && zero<max && max<=upper(axis);
  }
  bool empty() const { return min==0 && zero==0 && max==0; }
};

struct ServoTravelSweep {
  static constexpr int maxStepUs=50;
  static constexpr int slewUsPerTick=40; // 标定独立限速，每 10 ms 最多走 40 µs。
  static constexpr uint32_t jogTimeoutMs=250; // 按住期间须续期，松开或掉线时保持当前脉宽。
  int axis=-1, current=0, target=0;
  ServoTravelRange range{};
  bool explicitEndpoints=false, jogging=false;
  uint32_t jogAt=0;

  void begin(int channel,int center,bool captureEndpoints=false) {
    axis=channel;current=target=center;range={};explicitEndpoints=captureEndpoints;jogging=false;
  }
  void nudge(int delta) {
    const int next=current+delta, low=ServoTravelRange::lower(axis), high=ServoTravelRange::upper(axis);
    target=next<low?low:next>high?high:next;
  }
  void jog(int direction,uint32_t now) {
    target=direction>0?ServoTravelRange::upper(axis):ServoTravelRange::lower(axis);
    jogAt=now;
    jogging=true;
  }
  void hold() {
    target=current;
    jogging=false;
  }
  void tick(uint32_t now=0) {
    if(jogging&&now-jogAt>=jogTimeoutMs)hold();
    const int delta=target-current;
    current+=delta<-slewUsPerTick?-slewUsPerTick:delta>slewUsPerTick?slewUsPerTick:delta;
    if(range.zero&&!explicitEndpoints){
      if(current<range.min)range.min=current;
      if(current>range.max)range.max=current;
    }
  }
  void captureZero() { range={uint16_t(current),uint16_t(current),uint16_t(current)}; }
  bool captureEndpoint(bool high) {
    if(!explicitEndpoints||!range.zero||(high?current<=range.zero:current>=range.zero))return false;
    if(high)range.max=uint16_t(current);
    else range.min=uint16_t(current);
    return true;
  }
};

struct ServoCalibration {
  uint16_t azimuthTenths, elevationDeg;
  uint16_t azimuthReversed, elevationReversed;
  ServoTravelRange travel[2]{};

  bool valid() const {
    return azimuthTenths<=3600 && elevationDeg>=140 && elevationDeg<=210 &&
           azimuthReversed<=1 && elevationReversed<=1 &&
           (travel[0].empty()||travel[0].valid(0)) && (travel[1].empty()||travel[1].valid(1));
  }

  // 普通输出的物理 PWM 范围为实测端点与硬限位的交集。
  ServoTravelRange outputRange(int channel,int centerUs) const {
    int physicalLow=servoLowerUs(channel),physicalHigh=servoUpperUs(channel);
    if(channel==1) {
      const int minUs=centerUs-ELEVATION_DEFLECTION_US,maxUs=centerUs+ELEVATION_DEFLECTION_US;
      physicalLow=minUs<ELEVATION_PHYSICAL_MIN_US?ELEVATION_PHYSICAL_MIN_US:minUs;
      physicalHigh=maxUs>ELEVATION_PHYSICAL_MAX_US?ELEVATION_PHYSICAL_MAX_US:maxUs;
    }
    if(travel[channel].zero){
      if(travel[channel].min>physicalLow)physicalLow=travel[channel].min;
      if(travel[channel].max<physicalHigh)physicalHigh=travel[channel].max;
    }
    return {uint16_t(physicalLow),uint16_t(centerUs),uint16_t(physicalHigh)};
  }

  int direction(int channel) const {
    return (channel==0?azimuthReversed:elevationReversed)?-1:1;
  }

  bool elevationHasBothDirections() const {
    if(travel[1].valid(1))return true;
    // 140°、210° 对应绝对物理端点，不能作为新的双向机械中位。
    return elevationDeg>140 && elevationDeg<210;
  }

  GimbalTravel commandTravel(const int centerUs[2]) const {
    GimbalTravel result;
    for(int axis=0;axis<2;++axis) {
      if(axis==0 && !travel[axis].valid(axis))continue;
      const auto range=outputRange(axis,centerUs[axis]);
      const int sense=direction(axis);
      result.low[axis]=1500+sense*((sense>0?range.min:range.max)-centerUs[axis]);
      result.high[axis]=1500+sense*((sense>0?range.max:range.min)-centerUs[axis]);
      result.calibrated |= travel[axis].valid(axis) ||
          result.low[axis]!=servoLowerUs(axis) || result.high[axis]!=servoUpperUs(axis);
    }
    return result;
  }

  // 俯仰和实测轴使用同一脉宽比例；物理端点直接约束控制器有效行程。
  int outputUs(int channel,int commandUs,int centerUs) const {
    const auto range=outputRange(channel,centerUs);
    const int low=servoLowerUs(channel),high=servoUpperUs(channel);
    const int sense=direction(channel);
    if(channel==1 || travel[channel].valid(channel)) {
      const int physical=centerUs+sense*(commandUs-1500);
      return physical<range.min?range.min:physical>range.max?range.max:physical;
    }
    const int offset=sense*(limitServoUs(channel,commandUs)-1500);
    const int inputSpan=offset<0?1500-low:high-1500;
    const int outputSpan=offset<0?centerUs-range.min:range.max-centerUs;
    const int magnitude=((offset<0?-offset:offset)*outputSpan+inputSpan/2)/inputSpan;
    return centerUs+(offset<0?-magnitude:magnitude);
  }
};
