#pragma once
#include "GimbalControl.h"

// 只跟踪已发送的 PWM；没有位置传感器时，等待时间不能证明实物已到位。
class StartupPreposition {
 public:
  bool begin(const GimbalConfig &c, float leanX, float leanY, const int sent[2]) {
    if (!GimbalPid(c).configurationValid() || !std::isfinite(leanX) || !std::isfinite(leanY)) return false;
    float vx,vy;GimbalPid::initialVector(c,leanX,leanY,vx,vy);
    if(!std::isfinite(vx)||!std::isfinite(vy))return false;
    float azimuth,elevation;
    const float current=GimbalPid::usToAngle(sent[0],0);
    GimbalPid::vectorToAngles(vx,vy,GimbalPid::initialAzimuthReference(c,current,vx,vy),c.azimuthZero,azimuth,elevation);
    goal_[0]=limitServoUs(0,GimbalPid::angleToUs(azimuth,0));
    goal_[1]=limitServoUs(1,GimbalPid::angleToUs(elevation,1));
    lean_[0]=leanX;lean_[1]=leanY;atGoal_=false;active_=true;return true;
  }
  void cancel(){active_=false;atGoal_=false;}
  bool active()const{return active_;}
  int goal(int channel)const{return goal_[channel];}
  bool moved(float x,float y,float tolerance)const {
    return !std::isfinite(x)||!std::isfinite(y)||std::hypot(x-lean_[0],y-lean_[1])>tolerance;
  }
  bool ready(const int sent[2],uint32_t now,uint32_t settleMs,bool fresh){
    if(!active_ || !fresh || sent[0]!=goal_[0] || sent[1]!=goal_[1]){atGoal_=false;return false;}
    if(!atGoal_){atGoal_=true;at_=now;}
    return uint32_t(now-at_)>=settleMs;
  }
 private:
  int goal_[2]={1500,1500};float lean_[2]={0,0};
  bool active_=false,atGoal_=false;uint32_t at_=0;
};
