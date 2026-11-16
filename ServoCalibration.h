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
  int axis=-1, current=0, target=0;
  ServoTravelRange range{};

  void begin(int channel,int center) {
    axis=channel;current=target=center;range={};
  }
  void nudge(int delta) {
    const int next=current+delta, low=ServoTravelRange::lower(axis), high=ServoTravelRange::upper(axis);
    target=next<low?low:next>high?high:next;
  }
  void tick() {
    const int delta=target-current;
    current+=delta<-4?-4:delta>4?4:delta;
    if(range.zero){
      if(current<range.min)range.min=current;
      if(current>range.max)range.max=current;
    }
  }
  void captureZero() { range={uint16_t(current),uint16_t(current),uint16_t(current)}; }
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

  // 指令中位固定为 1500 μs，两侧分别映射到物理行程端点。
  int outputUs(int channel,int commandUs,int centerUs) const {
    const int low=servoLowerUs(channel),high=servoUpperUs(channel);
    int physicalLow=low,physicalHigh=high;
    if(channel==1) {
      const int minUs=centerUs-ELEVATION_DEFLECTION_US,maxUs=centerUs+ELEVATION_DEFLECTION_US;
      physicalLow=minUs<ELEVATION_PHYSICAL_MIN_US?ELEVATION_PHYSICAL_MIN_US:minUs;
      physicalHigh=maxUs>ELEVATION_PHYSICAL_MAX_US?ELEVATION_PHYSICAL_MAX_US:maxUs;
    }
    if(travel[channel].zero){
      if(travel[channel].min>physicalLow)physicalLow=travel[channel].min;
      if(travel[channel].max<physicalHigh)physicalHigh=travel[channel].max;
    }
    const bool reverse=channel==0?azimuthReversed:elevationReversed;
    int offset=limitServoUs(channel,commandUs)-1500;
    if(reverse)offset=-offset;
    const int inputSpan=offset<0?1500-low:high-1500;
    const int outputSpan=offset<0?centerUs-physicalLow:physicalHigh-centerUs;
    const int magnitude=((offset<0?-offset:offset)*outputSpan+inputSpan/2)/inputSpan;
    return centerUs+(offset<0?-magnitude:magnitude);
  }
};
