#pragma once
// 逻辑俯仰中位 135°，相对中位最多偏转 25°。
// 脉宽边界向行程内部取整，确保实际指令不超过角度限位。
constexpr float ELEVATION_CENTER_DEG=135.0f;
constexpr float ELEVATION_MAX_DEFLECTION_DEG=25.0f;
constexpr float ELEVATION_MIN_DEG=ELEVATION_CENTER_DEG-ELEVATION_MAX_DEFLECTION_DEG;
constexpr float ELEVATION_MAX_DEG=ELEVATION_CENTER_DEG+ELEVATION_MAX_DEFLECTION_DEG;
constexpr int ELEVATION_DEFLECTION_US=int(ELEVATION_MAX_DEFLECTION_DEG*2000.0f/270.0f);
constexpr int ELEVATION_MIN_US=1500-ELEVATION_DEFLECTION_US,ELEVATION_MAX_US=1500+ELEVATION_DEFLECTION_US;
// 实物绝对行程 140°～210°，保存的机械中位另加 ±25° 窗口。
constexpr int ELEVATION_PHYSICAL_MIN_US=1538,ELEVATION_PHYSICAL_MAX_US=2055;
inline int servoLowerUs(int channel){return channel==1?ELEVATION_MIN_US:500;}
inline int servoUpperUs(int channel){return channel==1?ELEVATION_MAX_US:2500;}
inline int limitServoUs(int channel,int us){return us<servoLowerUs(channel)?servoLowerUs(channel):us>servoUpperUs(channel)?servoUpperUs(channel):us;}

struct GimbalTravel {
  int low[2]={500,ELEVATION_MIN_US},high[2]={2500,ELEVATION_MAX_US};
  bool calibrated=false;

  int clamp(int axis,int us) const {
    return us<low[axis]?low[axis]:us>high[axis]?high[axis]:us;
  }
};
