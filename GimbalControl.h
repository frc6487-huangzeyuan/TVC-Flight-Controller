#pragma once
#include "Attitude.h"
#include "AutoLevelPid.h"
#include "ServoLimits.h"
#include <algorithm>

// 照片中的安装：PCB 板面沿箭体竖直，元件面朝外。
// 操作者必须把箭体轴竖直向上再校准。本函数利用此已知姿态建立安装矩阵，
// 不能从任意静止姿态自动识别箭头方向，也不能用重力识别绕箭体长轴的安装角。
// yawDegrees 是 PCB 朝外法线在公共机体 XY 平面内的方位：从箭头向尾部看，逆时针为正。
// 先建立局部 Z=箭体向上、X=PCB 朝外，再用安装角把两块 IMU 转到同一机体坐标系。
inline bool buildUprightMount(const float gravity[3], float matrix[3][3], float yawDegrees=0) {
  if (!std::isfinite(yawDegrees) || yawDegrees < -180 || yawDegrees > 180) return false;
  float length = std::sqrt(gravity[0]*gravity[0]+gravity[1]*gravity[1]+gravity[2]*gravity[2]);
  if (!std::isfinite(length) || length < .85f || length > 1.15f) return false;
  float z[3] = {gravity[0]/length,gravity[1]/length,gravity[2]/length};
  float x[3] = {-z[2]*z[0],-z[2]*z[1],1-z[2]*z[2]};
  length = std::sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]);
  // PCB 法线与重力方向过于接近，说明不符合“板面竖直”的校准条件。
  if (!std::isfinite(length) || length < .7f) return false;
  for (int i=0;i<3;++i) { matrix[0][i]=x[i]/length;matrix[2][i]=z[i]; }
  matrix[1][0]=z[1]*matrix[0][2]-z[2]*matrix[0][1];
  matrix[1][1]=z[2]*matrix[0][0]-z[0]*matrix[0][2];
  matrix[1][2]=z[0]*matrix[0][1]-z[1]*matrix[0][0];
  const float radians=yawDegrees*0.0174532925199433f;
  const float cs=std::cos(radians),sn=std::sin(radians);
  for(int column=0;column<3;++column){
    const float outward=matrix[0][column],lateral=matrix[1][column];
    matrix[0][column]=cs*outward-sn*lateral;
    matrix[1][column]=sn*outward+cs*lateral;
  }
  return true;
}

// 用“世界竖直方向在机体中的分量”计算两向倾斜，竖直姿态没有欧拉航向奇异点。
// 绕箭体自身 Z 轴的自旋不属于这两路云台能独立控制的自由度。
inline bool gimbalLean(const AttitudeMatrix &r, float &x, float &y) {
  if (!std::isfinite(r.m[2][0]) || !std::isfinite(r.m[2][1]) ||
      !std::isfinite(r.m[2][2])) return false;
  // 不再按箭体倾角大小停机；仅拒绝不能用于计算的非有限姿态数据。
  constexpr float degrees=57.29577951308232f;
  x=std::atan2(-r.m[2][0],r.m[2][2])*degrees;
  y=std::atan2(-r.m[2][1],r.m[2][2])*degrees;
  return true;
}

struct GimbalConfig {
  PidGains x,y;
  float limit,integralLimit,derivativeSeconds;
  float azimuthZero,signX,signY;
  int stepUs;
  uint32_t maxGapUs;
};

// 方位位置舵机 0..360°；俯仰位置舵机 0..270°、135°为无偏转。
// 脉宽 500..2500us 是需要按实际舵机规格核对的映射，actualUs 只表示已发送指令。
class GimbalPid {
 public:
  enum Result { WAITING,UPDATED,INVALID };
  explicit GimbalPid(const GimbalConfig &c)
      : config_(c),x_(c.x,c.limit,c.integralLimit,c.derivativeSeconds),
        y_(c.y,c.limit,c.integralLimit,c.derivativeSeconds) { reset(); }
  bool configurationValid() const {
    return x_.configurationValid() && y_.configurationValid() &&
      config_.limit>=1 && config_.limit<=50 && std::isfinite(config_.azimuthZero) &&
      config_.azimuthZero>=0 && config_.azimuthZero<=360 &&
      (config_.signX==1 || config_.signX==-1) && (config_.signY==1 || config_.signY==-1) &&
      config_.stepUs>0 && config_.maxGapUs>=100 && config_.maxGapUs<=50000;
  }
  void reset() { x_.reset();y_.reset();sampleUs_=0;dt_=0;limited_=false; }
  const AnglePid &x() const { return x_; }
  const AnglePid &y() const { return y_; }
  float dt() const { return dt_; }
  bool limited() const { return limited_; }
  const GimbalConfig &config() const { return config_; }
  static int angleToUs(float angle,int channel) {
    return int(std::lround(500+angle*2000/(channel==0?360.0f:270.0f)));
  }
  static float usToAngle(int us,int channel) {
    return (us-500)*(channel==0?360.0f:270.0f)/2000;
  }
  // 低偏转时保持方位；同一向量存在“方位+180°、俯仰取反”的等价解，
  // 按有限行程的实际指令距离选解，避免在 0/360°处跳到另一端。
  static void vectorToAngles(float x,float y,float currentAzimuth,float zero,
                             float &azimuth,float &elevation) {
    float magnitude=std::hypot(x,y);
    azimuth=currentAzimuth;elevation=135;
    if (magnitude<.15f) return;
    constexpr float degrees=57.29577951308232f;
    float direction=std::atan2(y,x)*degrees+zero;
    float best=10000;
    for (int flip=0;flip<2;++flip) {
      for (int turn=-2;turn<=2;++turn) {
        float candidate=direction+180*flip+360*turn;
        if(candidate<0 || candidate>360) continue;
        float distance=std::fabs(candidate-currentAzimuth);
        if(distance<best) { best=distance;azimuth=candidate;elevation=135+(flip?-magnitude:magnitude); }
      }
    }
  }
  Result update(uint64_t sampleUs,float leanX,float leanY,
                const int actualUs[2],int targetUs[2],bool allowIntegral=true) {
    if(!configurationValid() || !sampleUs || !std::isfinite(leanX) || !std::isfinite(leanY)) return INVALID;
    for(int k=0;k<2;++k) if(actualUs[k]<500 || actualUs[k]>2500) return INVALID;
    if(sampleUs_ && sampleUs<sampleUs_) return INVALID;
    if(sampleUs==sampleUs_) return WAITING;
    if(!sampleUs_) { x_.prime(leanX);y_.prime(leanY);sampleUs_=sampleUs;return WAITING; }
    uint64_t elapsed=sampleUs-sampleUs_;
    if(elapsed<100 || elapsed>config_.maxGapUs) return INVALID;
    dt_=float(elapsed)/1e6f;
    if(!x_.update(0,leanX,dt_,-config_.limit,config_.limit,!limited_&&allowIntegral) ||
       !y_.update(0,leanY,dt_,-config_.limit,config_.limit,!limited_&&allowIntegral)) return INVALID;
    float vx=config_.signX*x_.terms().output,vy=config_.signY*y_.terms().output;
    float magnitude=std::hypot(vx,vy);
    bool saturated=magnitude>config_.limit;
    if(saturated) { vx*=config_.limit/magnitude;vy*=config_.limit/magnitude; }
    float azimuth,elevation;
    float current=usToAngle(actualUs[0],0);
    vectorToAngles(vx,vy,current,config_.azimuthZero,azimuth,elevation);
    // 方位指令还在大幅转向时，先命令俯仰靠近中位，避免指向错误方向。
    // 这是指令状态限制，不是舵机位置反馈，实际跟随速度必须实物核对。
    bool turning=std::fabs(azimuth-current)>5;
    if(turning) elevation=135;
    int desired[2]={angleToUs(azimuth,0),angleToUs(elevation,1)};
    bool slew=false;
    for(int k=0;k<2;++k) {
      int difference=desired[k]-actualUs[k];
      slew=slew || std::abs(difference)>config_.stepUs;
      targetUs[k]=actualUs[k]+std::max(-config_.stepUs,std::min(config_.stepUs,difference));
      targetUs[k]=limitServoUs(k,targetUs[k]);
    }
    limited_=saturated || turning || slew;
    sampleUs_=sampleUs;
    return UPDATED;
  }
 private:
  GimbalConfig config_;
  AnglePid x_,y_;
  uint64_t sampleUs_;
  float dt_;
  bool limited_;
};
