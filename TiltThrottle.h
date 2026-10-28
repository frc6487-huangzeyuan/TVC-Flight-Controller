#pragma once
#include <cmath>
#include <cstdint>

// 油门百分比是“停止脉宽→正向最大脉宽”的比例，不是测得的 RPM 或推力。
struct TiltThrottleConfig {float base=0,gain=1,maximum=30,deadband=2,ramp=10;};
class TiltThrottle {
 public:
  TiltThrottleConfig config;
  float requested=0,percent=0,tilt=0;
  bool valid()const{
    const auto &c=config;
    return std::isfinite(c.base)&&std::isfinite(c.gain)&&std::isfinite(c.maximum)&&std::isfinite(c.deadband)&&std::isfinite(c.ramp)&&
      c.base>=0&&c.base<=c.maximum&&c.maximum<=100&&c.maximum>0&&c.gain>=0&&c.gain<=5&&c.deadband>=0&&c.deadband<=30&&c.ramp>=1&&c.ramp<=100;
  }
  void reset(){requested=percent=tilt=0;previousUs_=0;}
  bool update(uint64_t now,float bodyZWorldZ,bool fresh){
    if(!valid()||!std::isfinite(bodyZWorldZ)||!now)return false;
    const float z=bodyZWorldZ<-1?-1:bodyZWorldZ>1?1:bodyZWorldZ;
    tilt=std::acos(z)*57.29577951308232f;
    const float error=tilt>config.deadband?tilt-config.deadband:0;
    requested=std::fmin(config.maximum,config.base+config.gain*error);
    if(!previousUs_){previousUs_=now;return true;}
    if(now<previousUs_||now-previousUs_>50000)return false;
    const float step=config.ramp*float(now-previousUs_)/1e6f;
    // 历史姿态可保持当前指令或降油门，但不能在缺少新测量时继续加油门。
    const float target=fresh?requested:std::fmin(requested,percent);
    percent+=std::fmax(-step,std::fmin(step,target-percent));
    previousUs_=now;return true;
  }
  int pulse(int stop,int forwardMax)const{return int(std::lround(stop+(forwardMax-stop)*percent/100.0f));}
 private:
  uint64_t previousUs_=0;
};
