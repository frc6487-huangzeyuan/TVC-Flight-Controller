#pragma once
#include "GimbalControl.h"
#define TVC_AZIMUTH_REWIND_API 1
#define TVC_TORQUE_ESC_STEP_API 1

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

// 使用后级实际会送出的下一拍 ESC 指令计算可分配力矩。仿真可不传硬件脉宽约束。
struct TorqueEscStep {
  int stop,max,sent,step;

  int targetUs(float percent) const {
    return stop+int(std::lround((max-stop)*percent*.01f));
  }

  int nextUs(float target) const {
    return sent+std::max(-step,std::min(step,targetUs(target)-sent));
  }

  float nextPercent(float target) const {
    return max>stop?100.0f*std::max(0,std::min(max-stop,nextUs(target)-stop))/(max-stop):0;
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
    limited=false;travelLimited=false;
    throttlePct=0;
    for(int k=0;k<2;++k) { terms[k]={};rate_[k]=0;requestedNm[k]=0;allocatedNm[k]=0; }
  }

  bool update(const GimbalConfig &g,uint64_t now,const AttitudeMatrix &r,const float gyro[3],
              const int sent[2],int goal[2],const TorqueEscStep *esc=nullptr) {
    if (!config_.valid() || !now || (previousUs_ && (now<=previousUs_ || now-previousUs_>g.maxGapUs)))
      return false;
    float x,y;
    if (!gimbalControlUpright(r) || !gimbalLean(r,x,y)) return false;
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
    // 启动立即对已确认目标纠偏；运行中改目标再按参考角速度限幅。
    if (dt==0) { reference_[0]=target_[0];reference_[1]=target_[1]; }
    const float move[2]={target_[0]-reference_[0],target_[1]-reference_[1]};
    const float distance=std::hypot(move[0],move[1]);
    if (distance>0 && !rewinding()) {
      const float fraction=std::min(1.0f,TARGET_RATE_DPS*dt/distance);
      for(int k=0;k<2;++k)reference_[k]+=move[k]*fraction;
    }
    const float alpha=dt>0 ? dt/(dt+g.derivativeSeconds) : 1;
    rate_[0]+=alpha*(vx-rate_[0]);rate_[1]+=alpha*(vy-rate_[1]);
    calculate(g,x,y,rate_[0],rate_[1],dt,sent,goal,esc);
    return true;
  }

  PidTerms terms[2]{};
  float throttlePct=0, requestedNm[2]{}, allocatedNm[2]{};
  bool limited=false,travelLimited=false;
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
                 const int sent[2],int goal[2],const TorqueEscStep *esc) {
    const float limit=std::min(g.limit,ELEVATION_MAX_DEG-ELEVATION_CENTER_DEG)*RAD;
    const float maxFraction=config_.maximumPct*.01f;
    const float capacity=config_.fullTorqueNm*maxFraction*maxFraction*
      (g.travel.calibrated?std::max(-travelSine(g,limit,false),travelSine(g,limit,true)):std::sin(limit));
    travelLimited=false;
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
    if (rewinding() || crossesAzimuthStop(g,limit,sent[0],desiredX,desiredY) ||
        crossesAzimuthStop(g,limit,sent[0],baseNm[0]+candidateI[0],baseNm[1]+candidateI[1])) {
      requestedNm[0]=desiredX;requestedNm[1]=desiredY;
      for(int k=0;k<2;++k)
        terms[k].output=terms[k].p+terms[k].i+terms[k].d;
      const float azimuth=rewindToHome(g,dt,sent,goal);
      // 返回途中沿可用方位投影，保留能实现的纠偏。冻结积分，直到正常跟踪恢复。
      allocateElevation(g,desiredX,desiredY,limit,azimuth,sent,goal,config_.basePct,esc);
      limited=true;
      return;
    }
    allocate(g,baseNm[0]+candidateI[0],baseNm[1]+candidateI[1],limit,sent,goal,esc);
    // 只阻止把不可实现力矩继续推大的积分。消除稳态偏置的反向积分仍然生效。
    bool recalculate=false;
    for(int k=0;k<2;++k) {
      if (limited && std::fabs(requestedNm[k]-allocatedNm[k])>capacity*.005f &&
          increment[k]*(requestedNm[k]-allocatedNm[k])>0) recalculate=true;
      else terms[k].i=candidateI[k];
      terms[k].output=terms[k].p+terms[k].i+terms[k].d;
    }
    if (recalculate)
      allocate(g,baseNm[0]+terms[0].i,baseNm[1]+terms[1].i,limit,sent,goal,esc);
  }

  float travelSine(const GimbalConfig &g,float limit,bool positive) const {
    const int endpoint=positive?g.travel.high[1]:g.travel.low[1];
    const float elevation=(GimbalPid::usToAngle(endpoint,1)-ELEVATION_CENTER_DEG)*RAD;
    return std::sin(positive?std::min(limit,elevation):std::max(-limit,elevation));
  }

  GimbalPid::TravelSolution travelSolution(const GimbalConfig &g,float x,float y,float limit,float current) const {
    const float maxFraction=config_.maximumPct*.01f;
    const float axial=config_.fullTorqueNm*maxFraction*maxFraction;
    return GimbalPid::selectTravel(x*g.signX,y*g.signY,current,g.azimuthZero,
      GimbalPid::usToAngle(g.travel.low[0],0),GimbalPid::usToAngle(g.travel.high[0],0),
      axial*travelSine(g,limit,false),axial*travelSine(g,limit,true));
  }

  bool crossesAzimuthStop(const GimbalConfig &g,float limit,int azimuthUs,float x,float y) const {
    const float base=config_.basePct*.01f;
    if (std::hypot(x,y)<config_.fullTorqueNm*base*base*std::sin(.15f*RAD)) return false;
    const float current=GimbalPid::usToAngle(azimuthUs,0);
    const float direction=std::atan2(y*g.signY,x*g.signX)/RAD+g.azimuthZero;
    // 有符号俯仰使 180° 周期的方向等价；先检查连续跟随是否会越过机械端点。
    const float continuous=current+std::remainder(direction-current,180.f);
    if(!g.travel.calibrated)return continuous<-.01f || continuous>360.01f;
    const float lo=GimbalPid::usToAngle(g.travel.low[0],0),hi=GimbalPid::usToAngle(g.travel.high[0],0);
    if(continuous>=lo-.01f && continuous<=hi+.01f)return false;
    const auto alternative=travelSolution(g,x,y,limit,current);
    // 只有替代解确实在初始位的另一侧才执行回位。不可达方向留在可用边缘。
    return (current-g.azimuthZero)*(alternative.azimuth-g.azimuthZero)<0 &&
           std::fabs(alternative.azimuth-current)>5;
  }

  float rewindToHome(const GimbalConfig &g,float dt,const int sent[2],int goal[2]) {
    const int home=g.travel.clamp(0,GimbalPid::angleToUs(g.azimuthZero,0));
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

  void allocate(const GimbalConfig &g,float x,float y,float limit,const int sent[2],int goal[2],
                const TorqueEscStep *esc) {
    requestedNm[0]=x;requestedNm[1]=y;
    const float current=GimbalPid::usToAngle(sent[0],0);
    const float base=config_.basePct*.01f;
    const float deadband=config_.fullTorqueNm*base*base*std::sin(.15f*RAD);
    const auto solution=std::hypot(x,y)>=deadband?travelSolution(g,x,y,limit,current):
                                               GimbalPid::TravelSolution{current,0,false};
    const int azUs=g.travel.clamp(0,GimbalPid::angleToUs(solution.azimuth,0));
    goal[0]=g.travel.clamp(0,approach(sent[0],azUs,g.stepUs));
    const bool turning=goal[0]!=azUs;
    // 换向阶段不因尚不可实现的横向力矩提高转速。
    const bool elevationLimited=allocateElevation(g,x,y,limit,GimbalPid::usToAngle(goal[0],0),
        sent,goal,turning?config_.basePct:config_.maximumPct,esc);
    travelLimited=g.travel.calibrated && solution.limited;
    limited=solution.limited || turning || elevationLimited;
  }

  bool allocateElevation(const GimbalConfig &g,float x,float y,float limit,float azimuth,const int sent[2],
                         int goal[2],float throttleCeiling,const TorqueEscStep *esc) {
    const float heading=(azimuth-g.azimuthZero)*RAD;
    const float basis[2]={std::cos(heading)*g.signX,std::sin(heading)*g.signY};
    const float projection=x*basis[0]+y*basis[1];
    const float low=travelSine(g,limit,false);
    const float high=travelSine(g,limit,true);
    const float capacity=config_.fullTorqueNm*(projection<0?-low:high);
    const float required=capacity>0?100*std::sqrt(std::fabs(projection)/capacity):
                                   (projection==0?config_.basePct:config_.maximumPct);
    throttlePct=std::min(throttleCeiling,std::max(config_.basePct,required));
    const float fraction=throttlePct*.01f;
    const float targetAxial=config_.fullTorqueNm*fraction*fraction;
    const float bounded=targetAxial>0?std::max(low,std::min(high,projection/targetAxial)):0;
    const int elUs=g.travel.clamp(1,GimbalPid::angleToUs(ELEVATION_CENTER_DEG+std::asin(bounded)/RAD,1));
    goal[1]=g.travel.clamp(1,approach(sent[1],elUs,g.stepUs));
    const float elevationSine=std::sin((GimbalPid::usToAngle(goal[1],1)-ELEVATION_CENTER_DEG)*RAD);
    // 舵机来不及越过中位时先撤油门请求。已送出的 ESC 脉宽仍按限速下降。
    const bool opposing=projection*elevationSine<0;
    if (opposing)throttlePct=0;
    const float issuedFraction=(esc?esc->nextPercent(throttlePct):throttlePct)*.01f;
    const float usable=config_.fullTorqueNm*issuedFraction*issuedFraction*elevationSine;
    for(int k=0;k<2;++k) allocatedNm[k]=usable*basis[k];
    return goal[1]!=elUs || required>throttleCeiling || opposing ||
           (esc && esc->nextUs(throttlePct)!=esc->targetUs(throttlePct));
  }

  static int approach(int current,int target,int step) {
    return current+std::max(-step,std::min(step,target-current));
  }
};
