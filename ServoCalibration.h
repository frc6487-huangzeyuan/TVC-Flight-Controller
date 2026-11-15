#pragma once
#include <cstdint>
#include "ServoLimits.h"

struct ServoCalibration {
  uint16_t azimuthTenths, elevationDeg;
  uint16_t azimuthReversed, elevationReversed;

  bool valid() const {
    return azimuthTenths<=3600 && elevationDeg>=140 && elevationDeg<=210 &&
           azimuthReversed<=1 && elevationReversed<=1;
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
    const bool reverse=channel==0?azimuthReversed:elevationReversed;
    int offset=limitServoUs(channel,commandUs)-1500;
    if(reverse)offset=-offset;
    const int inputSpan=offset<0?1500-low:high-1500;
    const int outputSpan=offset<0?centerUs-physicalLow:physicalHigh-centerUs;
    const int magnitude=((offset<0?-offset:offset)*outputSpan+inputSpan/2)/inputSpan;
    return centerUs+(offset<0?-magnitude:magnitude);
  }
};
