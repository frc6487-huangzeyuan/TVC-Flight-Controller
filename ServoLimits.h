#pragma once
// 270°俯仰舵机的机械中位为 135°，允许偏转 −50°～+50°。
// 脉宽边界向行程内部取整，确保实际指令不超过角度限位。
constexpr float ELEVATION_CENTER_DEG=135.0f;
constexpr float ELEVATION_MIN_DEG=85.0f,ELEVATION_MAX_DEG=185.0f;
constexpr int ELEVATION_MIN_US=1130,ELEVATION_MAX_US=1870;
inline int servoLowerUs(int channel){return channel==1?ELEVATION_MIN_US:500;}
inline int servoUpperUs(int channel){return channel==1?ELEVATION_MAX_US:2500;}
inline int limitServoUs(int channel,int us){return us<servoLowerUs(channel)?servoLowerUs(channel):us>servoUpperUs(channel)?servoUpperUs(channel):us;}
