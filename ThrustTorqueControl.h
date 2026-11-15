#pragma once
#include "GimbalControl.h"
#define TVC_AZIMUTH_REWIND_API 1

struct ThrustTorqueConfig {
  // 100% 正向油门的推力（N）乘有效轴向力臂（m）。油门模型为 F ∝ u²。
  float fullTorqueNm=0, basePct=0, maximumPct=0;
  PidGains gains[2]{};
  // 横向在零倾角时的配平力矩与随倾角变化的重力补偿矩阵，坐标与 PID 一致。
  float trimNm[2]{}, gravityNmPerDeg[2][2]{};

  bool valid() const {
    if (!(fullTorqueNm>0 && fullTorqueNm<=1000 && basePct>0 &&
          basePct<=maximumPct && maximumPct<=100)) return false;
    for(int k=0;k<2;++k) {
      const auto &g=gains[k];
      if (!(g.kp>0 && g.kp<=100 && g.ki>=0 && g.ki<=100 && g.kd>=0 && g.kd<=100) ||
          !(std::fabs(trimNm[k])<=fullTorqueNm) ||
          !(std::fabs(gravityNmPerDeg[k][0])<=fullTorqueNm) ||
          !(std::fabs(gravityNmPerDeg[k][1])<=fullTorqueNm)) return false;
    }
    return true;
  }
};

class ThrustTorqueControl {
 public:
  static constexpr float TARGET_LIMIT_DEG=20;

  bool configure(const ThrustTorqueConfig &config) {
    if (!config.valid()) return false;
    config_=config;
    reset();
    return true;
  }
  const ThrustTorqueConfig &config() const { return config_; }
  bool setTarget(float x,float y) {
    if (!(std::fabs(x)<=TARGET_LIMIT_DEG && std::fabs(y)<=TARGET_LIMIT_DEG)) return false;
    target_[0]=x;target_[1]=y;
    return true;
  }
  const float *target() const { return target_; }

  void reset() {
    previousUs_=0;
    rewindActive_=false;waitSeconds_=0;
    limited=false;
    throttlePct=0;
    for(int k=0;k<2;++k) { terms[k]={};rate_[k]=0;requestedNm[k]=0;allocatedNm[k]=0; }
  }

  // 预定位使用同一推力分配，取静态 P 项和重力补偿，电调由状态机保持停转。
  bool prepare(const GimbalConfig &g,float x,float y,const int sent[2],int goal[2]) const {
    if (!config_.valid() || !std::isfinite(x) || !std::isfinite(y)) return false;
    ThrustTorqueControl preview=*this;
    preview.reset();
    preview.reference_[0]=x;preview.reference_[1]=y;
    preview.calculate(g,x,y,0,0,0,sent,goal,true);
    return true;
  }

  bool update(const GimbalConfig &g,uint64_t now,const AttitudeMatrix &r,const float gyro[3],
              const int sent[2],int goal[2]) {
    if (!config_.valid() || !now || (previousUs_ && (now<=previousUs_ || now-previousUs_>g.maxGapUs)))
      return false;
    float x,y;
    if (!gimbalLean(r,x,y)) return false;
    // 竖直单位向量在机体系中的导数为 -ω×u；据此求两路倾角的真实角速度。
    const auto &u=r.m[2];
    const float dz=gyro[1]*u[0]-gyro[0]*u[1];
    const float dx=gyro[2]*u[1]-gyro[1]*u[2];
    const float dy=gyro[0]*u[2]-gyro[2]*u[0];
    const float denX=u[0]*u[0]+u[2]*u[2],denY=u[1]*u[1]+u[2]*u[2];
    if (denX<1e-6f || denY<1e-6f) return false;
    const float vx=(u[0]*dz-u[2]*dx)/denX, vy=(u[1]*dz-u[2]*dy)/denY;
    if (!std::isfinite(vx) || !std::isfinite(vy)) return false;
    const float dt=previousUs_ ? float(now-previousUs_)*1e-6f : 0;
    previousUs_=now;
    // 启动及改目标时限制参考角移动速度，扰动反馈仍按每拍实测姿态计算。
    if (dt==0) { reference_[0]=x;reference_[1]=y; }
    const float move[2]={target_[0]-reference_[0],target_[1]-reference_[1]};
    const float distance=std::hypot(move[0],move[1]);
    if (distance>0 && !rewinding()) {
      const float fraction=std::min(1.0f,TARGET_RATE_DPS*dt/distance);
      for(int k=0;k<2;++k)reference_[k]+=move[k]*fraction;
    }
    const float alpha=dt>0 ? dt/(dt+g.derivativeSeconds) : 1;
    rate_[0]+=alpha*(vx-rate_[0]);rate_[1]+=alpha*(vy-rate_[1]);
    calculate(g,x,y,rate_[0],rate_[1],dt,sent,goal,false);
    return true;
  }

  PidTerms terms[2]{};
  float throttlePct=0, requestedNm[2]{}, allocatedNm[2]{};
  bool limited=false;
  bool rewinding() const { return rewindActive_; }

 private:
  static constexpr float RAD=0.0174532925199433f;
  static constexpr float TARGET_RATE_DPS=4;
  // 没有舵机位置反馈。回位时按约 180°/s 和舵机响应估算当前方位，
  // 避免把快速发出的 PWM 提前当作已经到达的位置。
  static constexpr float HOME_SPEED_US_S=1000, HOME_RESPONSE_S=.06f;
  static constexpr float HOME_TOLERANCE_US=5, HOME_SETTLE_S=.15f;
  ThrustTorqueConfig config_{};
  float target_[2]{},reference_[2]{},rate_[2]{};
  uint64_t previousUs_=0;
  bool rewindActive_=false;
  float estimatedAzimuthUs_=0,waitSeconds_=0;

  void calculate(const GimbalConfig &g,float x,float y,float vx,float vy,float dt,
                 const int sent[2],int goal[2],bool preparing) {
    const float limit=std::min(g.limit,ELEVATION_MAX_DEG-ELEVATION_CENTER_DEG)*RAD;
    const float maxFraction=config_.maximumPct*.01f;
    const float capacity=config_.fullTorqueNm*maxFraction*maxFraction*std::sin(limit);
    const float lean[2]={x,y}, speed[2]={vx,vy};
    float baseNm[2], candidateI[2], increment[2];
    for(int k=0;k<2;++k) {
      const auto &gain=config_.gains[k];
      auto &t=terms[k];
      t.error=AnglePid::angleDifference(reference_[k],lean[k]);
      t.p=gain.kp*t.error;
      t.d=-gain.kd*speed[k];
      candidateI[k]=std::max(-capacity,std::min(capacity,t.i+gain.ki*t.error*dt));
      increment[k]=candidateI[k]-t.i;
      baseNm[k]=t.p+t.d+config_.trimNm[k]+config_.gravityNmPerDeg[k][0]*x+
                config_.gravityNmPerDeg[k][1]*y;
    }
    const float desiredX=baseNm[0]+terms[0].i,desiredY=baseNm[1]+terms[1].i;
    if (!preparing && (rewinding() || crossesAzimuthStop(g,sent[0],desiredX,desiredY) ||
        crossesAzimuthStop(g,sent[0],baseNm[0]+candidateI[0],baseNm[1]+candidateI[1]))) {
      requestedNm[0]=desiredX;requestedNm[1]=desiredY;
      for(int k=0;k<2;++k)
        terms[k].output=terms[k].p+terms[k].i+terms[k].d;
      const float azimuth=rewindToHome(g,dt,sent,goal);
      // 返回途中沿可用方位投影，保留能实现的纠偏。冻结积分，直到正常跟踪恢复。
      allocateElevation(g,desiredX,desiredY,limit,azimuth,sent,goal,false);
      limited=true;
      return;
    }
    allocate(g,baseNm[0]+candidateI[0],baseNm[1]+candidateI[1],limit,sent,goal,preparing);
    // 只阻止把不可实现力矩继续推大的积分。消除稳态偏置的反向积分仍然生效。
    bool recalculate=false;
    for(int k=0;k<2;++k) {
      if (limited && increment[k]*(requestedNm[k]-allocatedNm[k])>0) recalculate=true;
      else terms[k].i=candidateI[k];
      terms[k].output=terms[k].p+terms[k].i+terms[k].d;
    }
    if (recalculate)
      allocate(g,baseNm[0]+terms[0].i,baseNm[1]+terms[1].i,limit,sent,goal,preparing);
  }

  bool crossesAzimuthStop(const GimbalConfig &g,int azimuthUs,float x,float y) const {
    const float base=config_.basePct*.01f;
    if (std::hypot(x,y)<config_.fullTorqueNm*base*base*std::sin(.15f*RAD)) return false;
    const float current=GimbalPid::usToAngle(azimuthUs,0);
    const float direction=std::atan2(y*g.signY,x*g.signX)/RAD+g.azimuthZero;
    // 有符号俯仰使 180° 周期的方向等价；先检查连续跟随是否会越过机械端点。
    const float continuous=current+std::remainder(direction-current,180.f);
    return continuous<-.01f || continuous>360.01f;
  }

  float rewindToHome(const GimbalConfig &g,float dt,const int sent[2],int goal[2]) {
    const int home=GimbalPid::angleToUs(g.azimuthZero,0);
    if (!rewindActive_) {
      rewindActive_=true;estimatedAzimuthUs_=float(sent[0]);waitSeconds_=0;
    }
    goal[0]=approach(sent[0],home,g.stepUs);
    const float rateStep=HOME_SPEED_US_S*dt;
    const float motion=(sent[0]-estimatedAzimuthUs_)*dt/(HOME_RESPONSE_S+dt);
    estimatedAzimuthUs_+=std::max(-rateStep,std::min(rateStep,motion));
    if (sent[0]==home && std::fabs(estimatedAzimuthUs_-home)<=HOME_TOLERANCE_US) {
      waitSeconds_+=dt;
      if (waitSeconds_>=HOME_SETTLE_S) rewindActive_=false;
    }
    return (estimatedAzimuthUs_-500)*.18f;
  }

  void allocate(const GimbalConfig &g,float x,float y,float limit,const int sent[2],int goal[2],bool preparing) {
    requestedNm[0]=x;requestedNm[1]=y;
    const float demand=std::hypot(x,y);
    const float requiredPct=100*std::sqrt(demand/(config_.fullTorqueNm*std::sin(limit)));
    throttlePct=std::min(config_.maximumPct,std::max(config_.basePct,requiredPct));
    const float fraction=throttlePct*.01f;
    const float axial=config_.fullTorqueNm*fraction*fraction;
    const float angle=std::asin(std::min(std::sin(limit),demand/axial))/RAD;
    const float current=GimbalPid::usToAngle(sent[0],0);
    float azimuth=current,elevation=ELEVATION_CENTER_DEG;
    // 原逆运动学用于选择等价方位；转向时沿下一步可达方位投影，连续保留可用纠偏。
    if (angle>=.15f)
      GimbalPid::vectorToAngles(x*g.signX*angle/demand,y*g.signY*angle/demand,current,g.azimuthZero,azimuth,elevation);
    const int azUs=GimbalPid::angleToUs(azimuth,0);
    goal[0]=preparing?azUs:approach(sent[0],azUs,g.stepUs);
    const bool elevationSlewing=allocateElevation(
      g,x,y,limit,GimbalPid::usToAngle(goal[0],0),sent,goal,preparing,axial);
    limited=requiredPct>config_.maximumPct || goal[0]!=azUs || elevationSlewing;
  }

  bool allocateElevation(const GimbalConfig &g,float x,float y,float limit,float azimuth,const int sent[2],
                         int goal[2],bool preparing,float axial=0) {
    const float heading=(azimuth-g.azimuthZero)*RAD;
    const float basis[2]={std::cos(heading)*g.signX,std::sin(heading)*g.signY};
    const float projection=x*basis[0]+y*basis[1];
    if (axial==0) {
      const float requiredPct=100*std::sqrt(std::fabs(projection)/(config_.fullTorqueNm*std::sin(limit)));
      throttlePct=std::min(config_.maximumPct,std::max(config_.basePct,requiredPct));
      const float fraction=throttlePct*.01f;
      axial=config_.fullTorqueNm*fraction*fraction;
    }
    const float bounded=std::max(-std::sin(limit),std::min(std::sin(limit),projection/axial));
    const int elUs=limitServoUs(1,GimbalPid::angleToUs(ELEVATION_CENTER_DEG+std::asin(bounded)/RAD,1));
    goal[1]=preparing?elUs:approach(sent[1],elUs,g.stepUs);
    const float usable=axial*std::sin((GimbalPid::usToAngle(goal[1],1)-ELEVATION_CENTER_DEG)*RAD);
    for(int k=0;k<2;++k) allocatedNm[k]=usable*basis[k];
    return goal[1]!=elUs;
  }

  static int approach(int current,int target,int step) {
    return current+std::max(-step,std::min(step,target-current));
  }
};
