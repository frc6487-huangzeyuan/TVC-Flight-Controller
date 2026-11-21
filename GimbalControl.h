#pragma once
#include "Attitude.h"
#include "AutoLevelPid.h"
#include "ServoLimits.h"
#include <algorithm>

inline float dotMountVector(const float a[3],const float b[3]) {
  return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

inline void crossMountVector(const float a[3],const float b[3],float result[3]) {
  result[0]=a[1]*b[2]-a[2]*b[1];
  result[1]=a[2]*b[0]-a[0]*b[2];
  result[2]=a[0]*b[1]-a[1]*b[0];
}

// 矩阵三行是机体 X、Y、Z 在 IMU 原始坐标中的方向。
inline bool buildMountBasis(const float gravity[3],const float outward[3],float matrix[3][3]) {
  const float norm=std::sqrt(dotMountVector(gravity,gravity));
  if(!std::isfinite(norm)||norm<.85f||norm>1.15f)return false;
  float *x=matrix[0],*z=matrix[2];
  for(int i=0;i<3;++i)z[i]=gravity[i]/norm;
  const float projection=dotMountVector(outward,z);
  for(int i=0;i<3;++i)x[i]=outward[i]-projection*z[i];
  const float length=std::sqrt(dotMountVector(x,x));
  if(!std::isfinite(length)||length<.7f)return false;
  for(int i=0;i<3;++i)x[i]/=length;
  crossMountVector(z,x,matrix[1]);
  return true;
}

// 用竖直时的加速度和已知外向轴构造安装矩阵，默认外向轴是 PCB +Z。
// 操作者必须把箭体轴竖直向上再校准。本函数利用此已知姿态建立安装矩阵，
// 不能从任意静止姿态自动识别箭头方向，也不能用重力识别绕箭体长轴的安装角。
// yawDegrees 是 PCB 朝外法线在公共机体 XY 平面内的方位：从箭头向尾部看，逆时针为正。
// 先建立局部 Z=箭体向上、X=PCB 朝外，再用安装角把两块 IMU 转到同一机体坐标系。
inline bool buildUprightMount(const float gravity[3], float matrix[3][3], float yawDegrees=0,int outwardAxis=2) {
  if(outwardAxis<0 || outwardAxis>2)return false;
  if (!std::isfinite(yawDegrees) || yawDegrees < -180 || yawDegrees > 180) return false;
  float outward[3]={};
  outward[outwardAxis]=1;
  if(!buildMountBasis(gravity,outward,matrix))return false;
  const float radians=yawDegrees*0.0174532925199433f;
  const float cs=std::cos(radians),sn=std::sin(radians);
  for(int column=0;column<3;++column){
    const float outward=matrix[0][column],lateral=matrix[1][column];
    matrix[0][column]=cs*outward-sn*lateral;
    matrix[1][column]=sn*outward+cs*lateral;
  }
  return true;
}

// direction 顺序为 +X、-X、+Y、-Y，指箭头移动方向。
// 倾向正轴时，加速度计测得的世界 +Z 向该机体轴的负侧投影。
inline bool guidedMountOutward(const float upright[3],const float leaned[3],int direction,float outward[3]) {
  if(direction<0||direction>3)return false;
  const float upNorm=std::sqrt(dotMountVector(upright,upright));
  const float leanNorm=std::sqrt(dotMountVector(leaned,leaned));
  if(!std::isfinite(upNorm)||upNorm<.85f||upNorm>1.15f||
     !std::isfinite(leanNorm)||leanNorm<.85f||leanNorm>1.15f)return false;
  const float cosine=dotMountVector(upright,leaned)/(upNorm*leanNorm);
  constexpr float radians=.0174532925199433f;
  if(cosine>std::cos(8*radians)||cosine<std::cos(30*radians))return false;
  const float sign=direction%2 ? 1.0f : -1.0f;
  float axis[3];
  for(int k=0;k<3;++k)axis[k]=sign*(leaned[k]/leanNorm-cosine*upright[k]/upNorm);
  const float norm=std::sqrt(dotMountVector(axis,axis));
  for(float &component:axis)component/=norm;
  if(direction<2)for(int k=0;k<3;++k)outward[k]=axis[k];
  else {
    const float z[3]={upright[0]/upNorm,upright[1]/upNorm,upright[2]/upNorm};
    crossMountVector(axis,z,outward);
  }
  return true;
}

// 用“世界竖直方向在机体中的分量”计算两向倾斜，竖直姿态没有欧拉航向奇异点。
// 绕箭体自身 Z 轴的自旋不属于这两路云台能独立控制的自由度。
inline bool gimbalLean(const AttitudeMatrix &r, float &x, float &y) {
  if (!std::isfinite(r.m[2][0]) || !std::isfinite(r.m[2][1]) ||
      !std::isfinite(r.m[2][2])) return false;
  // 遥测也使用此角度表达；闭环在 gimbalControlUpright 另行校验适用范围。
  constexpr float degrees=57.29577951308232f;
  x=std::atan2(-r.m[2][0],r.m[2][2])*degrees;
  y=std::atan2(-r.m[2][1],r.m[2][2])*degrees;
  return true;
}

// 该倾角坐标仅在重力方向处于机体上半球且远离奇异点时用于闭环。
#define TVC_GIMBAL_UPRIGHT_API 1
inline bool gimbalControlUpright(const AttitudeMatrix &r) {
  return std::isfinite(r.m[2][2]) && r.m[2][2]>.001f;
}

// 可选的已标定小角度动力学：q¨ = A q + B (v - bias)。
// q 为机体倾角（度），v 为二维纠偏角（度）；时间单位为秒。
// 未提供标定时保持普通 PID。不得把仿真 CAD 参数当成实物标定。
#define TVC_MODEL_COMPENSATION_API 1
struct GimbalModel {
  bool enabled=false;
  float bias[2]={0,0}, gravity[4]={0,0,0,0}, inverse[4]={0,0,0,0};
  float accelerationLimit=15; // 模型模式下 PID 输出的角加速度上限，度/秒²。
  bool valid() const {
    if(!enabled)return true;
    for(int i=0;i<2;++i)if(!std::isfinite(bias[i]))return false;
    for(int i=0;i<4;++i)if(!std::isfinite(gravity[i])||!std::isfinite(inverse[i]))return false;
    return std::isfinite(accelerationLimit) && accelerationLimit>0 &&
      std::fabs(inverse[0]*inverse[3]-inverse[1]*inverse[2])>1e-9f;
  }
};
struct GimbalConfig {
  PidGains x,y;
  float limit,integralLimit,derivativeSeconds;
  float azimuthZero,signX,signY;
  int stepUs;
  uint32_t maxGapUs;
  GimbalModel model{};
  GimbalTravel travel{};
  float trim[2]={0,0}; // 非模型角度 PID 的实测配平偏转向量，单位度。
};

// 方位位置舵机 0..360°；俯仰位置舵机 0..270°、135°为无偏转。
// 脉宽 500..2500us 是需要按实际舵机规格核对的映射，actualUs 只表示已发送指令。
class GimbalPid {
 public:
  enum Result { WAITING,UPDATED,INVALID };
  explicit GimbalPid(const GimbalConfig &c)
      : config_(c),x_(c.x,controlLimit(c),c.integralLimit,c.derivativeSeconds),
        y_(c.y,controlLimit(c),c.integralLimit,c.derivativeSeconds) { reset(); }
  bool configurationValid() const {
    return config_.model.valid() && x_.configurationValid() && y_.configurationValid() &&
      config_.limit>=1 && config_.limit<=50 && std::isfinite(config_.azimuthZero) &&
      std::hypot(config_.trim[0],config_.trim[1])<=std::min(config_.limit,ELEVATION_MAX_DEFLECTION_DEG) &&
      config_.azimuthZero>=0 && config_.azimuthZero<=360 &&
      (config_.signX==1 || config_.signX==-1) && (config_.signY==1 || config_.signY==-1) &&
      config_.stepUs>0 && config_.maxGapUs>=100 && config_.maxGapUs<=50000;
  }
  void reset() {
    x_.reset();y_.reset();sampleUs_=0;dt_=0;limited_=false;
    residual_[0]=residual_[1]=0;
  }
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
  static float controlLimit(const GimbalConfig &c) {
    return c.model.enabled ? c.model.accelerationLimit : c.limit;
  }
  // 推力关闭且台架固定的启动准备阶段，模型模式优先选择远离方位行程两端
  // 的等价解，减少释放后接近 0°/360° 时需要跨越半圈的换向。
  // 普通 PID 保留原有最近当前位置规则；运行阶段仍按当前指令选解。
  static float initialAzimuthReference(const GimbalConfig &c,float currentAzimuth,float vx,float vy) {
    return c.model.enabled && std::hypot(vx,vy)>=.15f ? 180.0f : currentAzimuth;
  }
  // 模型模式先把期望角加速度换算成纠偏向量，再由同一逆运动学解算舵角。
  static void correctionVector(const GimbalConfig &c,float leanX,float leanY,
                               float ux,float uy,float &vx,float &vy) {
    if(c.model.enabled) {
      const auto &m=c.model;
      const float ax=ux-m.gravity[0]*leanX-m.gravity[1]*leanY;
      const float ay=uy-m.gravity[2]*leanX-m.gravity[3]*leanY;
      vx=m.bias[0]+m.inverse[0]*ax+m.inverse[1]*ay;
      vy=m.bias[1]+m.inverse[2]*ax+m.inverse[3]*ay;
    } else { vx=c.signX*ux+c.trim[0];vy=c.signY*uy+c.trim[1]; }
  }
  static void initialVector(const GimbalConfig &c,float x,float y,float &vx,float &vy) {
    const float limit=controlLimit(c);
    correctionVector(c,x,y,std::max(-limit,std::min(limit,-c.x.kp*x)),
                     std::max(-limit,std::min(limit,-c.y.kp*y)),vx,vy);
    const float magnitude=std::hypot(vx,vy);
    const float limitDeg=std::min(c.limit,ELEVATION_MAX_DEFLECTION_DEG);
    if(magnitude>limitDeg){vx*=limitDeg/magnitude;vy*=limitDeg/magnitude;}
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
  struct TravelSolution { float azimuth,radial;bool limited; };

  // 两条等效射线分别取行程内最近方向，并沿该方向投影到允许的幅度。
  // 量纲由调用方决定：角度 PID 使用偏转角，推力模式使用横向力矩。
  static TravelSolution selectTravel(float x,float y,float current,float zero,
                                     float azMin,float azMax,float negative,float positive) {
    constexpr float rad=.0174532925199433f;
    const float demandSq=x*x+y*y;
    if(demandSq==0)return {std::max(azMin,std::min(azMax,current)),0,false};
    const float direction=std::atan2(y,x)/rad+zero;
    const float tolerance=std::max(1e-12f,1e-6f*demandSq);
    float bestError=2*demandSq,bestDistance=10000;
    TravelSolution best{current,0,true};
    for(int flip=0;flip<2;++flip) {
      for(int turn=-2;turn<=2;++turn) {
        const float az=std::max(azMin,std::min(azMax,direction+180*flip+360*turn));
        const float heading=(az-zero)*rad;
        const float bx=std::cos(heading),by=std::sin(heading);
        const float radial=std::max(negative,std::min(positive,x*bx+y*by));
        const float dx=x-radial*bx,dy=y-radial*by;
        const float error=dx*dx+dy*dy,distance=std::fabs(az-current);
        if(error<bestError-tolerance || (std::fabs(error-bestError)<=tolerance && distance<bestDistance)) {
          best={az,radial,error>tolerance};
          bestError=error;bestDistance=distance;
        }
      }
    }
    return best;
  }

  static bool correctionAngles(const GimbalConfig &c,float x,float y,float current,
                               float &azimuth,float &elevation) {
    if(!c.travel.calibrated) {
      vectorToAngles(x,y,current,c.azimuthZero,azimuth,elevation);
      return false;
    }
    if(std::hypot(x,y)<.15f)x=y=0;
    const auto solution=selectTravel(x,y,current,c.azimuthZero,
      usToAngle(c.travel.low[0],0),usToAngle(c.travel.high[0],0),
      std::max(-c.limit,usToAngle(c.travel.low[1],1)-ELEVATION_CENTER_DEG),
      std::min(c.limit,usToAngle(c.travel.high[1],1)-ELEVATION_CENTER_DEG));
    azimuth=solution.azimuth;elevation=ELEVATION_CENTER_DEG+solution.radial;
    return solution.limited;
  }

  Result update(uint64_t sampleUs,float leanX,float leanY,
                const int actualUs[2],int targetUs[2],bool allowIntegral=true) {
    if(!configurationValid() || !sampleUs || !std::isfinite(leanX) || !std::isfinite(leanY)) return INVALID;
    for(int k=0;k<2;++k)
      if(config_.travel.calibrated?actualUs[k]!=config_.travel.clamp(k,actualUs[k]):
                                  (actualUs[k]<500 || actualUs[k]>2500))return INVALID;
    if(sampleUs_ && sampleUs<sampleUs_) return INVALID;
    if(sampleUs==sampleUs_) return WAITING;
    if(!sampleUs_) { x_.prime(leanX);y_.prime(leanY);sampleUs_=sampleUs;return WAITING; }
    uint64_t elapsed=sampleUs-sampleUs_;
    if(elapsed<100 || elapsed>config_.maxGapUs) return INVALID;
    dt_=float(elapsed)/1e6f;
    const float control=controlLimit(config_);
    const bool canIntegrate=allowIntegral && (!config_.model.enabled || !limited_);
    if(!x_.update(0,leanX,dt_,-control,control,canIntegrate,residual_[0]) ||
       !y_.update(0,leanY,dt_,-control,control,canIntegrate,residual_[1])) return INVALID;
    float vx,vy;
    correctionVector(config_,leanX,leanY,x_.terms().output,y_.terms().output,vx,vy);
    if(!std::isfinite(vx)||!std::isfinite(vy))return INVALID;
    const float requested[2]={vx,vy};
    const float magnitude=std::hypot(vx,vy);
    const float limit=std::min(config_.limit,ELEVATION_MAX_DEFLECTION_DEG);
    bool saturated=magnitude>limit;
    if(saturated) { vx*=limit/magnitude;vy*=limit/magnitude; }
    float azimuth,elevation;
    float current=usToAngle(actualUs[0],0);
    // 小于舵机安定分辨率的二维向量沿当前方位投影，不驱动方位换向。
    const float correctionMagnitude=std::hypot(vx,vy);
    constexpr float RAD=.0174532925199433f;
    const float heading=(current-config_.azimuthZero)*RAD;
    if(correctionMagnitude<.15f) {
      azimuth=current;
      elevation=ELEVATION_CENTER_DEG+vx*std::cos(heading)+vy*std::sin(heading);
    } else {
      saturated=correctionAngles(config_,vx,vy,current,azimuth,elevation)||saturated;
    }
    // 转向过程中，只施加期望纠偏在当前已发送方位上的分量。
    // 方位来自上一拍 PWM 指令，实际舵机的位置和跟随速度需实物核对。
    const bool turning=std::fabs(azimuth-current)>5;
    if(turning)elevation=ELEVATION_CENTER_DEG+vx*std::cos(heading)+vy*std::sin(heading);
    int desired[2]={angleToUs(azimuth,0),angleToUs(elevation,1)};
    bool slew=false;
    for(int k=0;k<2;++k) {
      const int bounded=config_.travel.clamp(k,desired[k]);
      saturated |= bounded!=desired[k];
      int difference=bounded-actualUs[k];
      slew=slew || std::abs(difference)>config_.stepUs;
      targetUs[k]=actualUs[k]+std::max(-config_.stepUs,std::min(config_.stepUs,difference));
      targetUs[k]=config_.travel.clamp(k,targetUs[k]);
    }
    limited_=saturated || turning || slew;
    residual_[0]=residual_[1]=0;
    if(limited_ && !config_.model.enabled) {
      const float sentHeading=(usToAngle(targetUs[0],0)-config_.azimuthZero)*RAD;
      const float sentElevation=usToAngle(targetUs[1],1)-ELEVATION_CENTER_DEG;
      const float residualX=requested[0]-sentElevation*std::cos(sentHeading);
      const float residualY=requested[1]-sentElevation*std::sin(sentHeading);
      // 忽略不到半个俯仰 PWM 量化步长的残差，分别限制继续推大残差的积分。
      residual_[0]=std::fabs(residualX)>.0675f?config_.signX*residualX:0;
      residual_[1]=std::fabs(residualY)>.0675f?config_.signY*residualY:0;
    }
    sampleUs_=sampleUs;
    return UPDATED;
  }
 private:
  float residual_[2]={0,0};
  GimbalConfig config_;
  AnglePid x_,y_;
  uint64_t sampleUs_;
  float dt_;
  bool limited_;
};
