#pragma once
#include "Attitude.h"
#include <cstdint>

// 历史角速度按零阶保持，用真实经过的时间推进四元数。
// 每次积分步长不超过 10ms；历史加速度不能当成新的重力测量反复反馈。
inline bool advanceHeldGyro(Attitude &pose,const float gyro[3],uint64_t durationUs) {
  if(!pose.initialized() || durationUs>2000000) return false;
  while(durationUs) {
    uint32_t step=durationUs>10000?10000:uint32_t(durationUs);
    if(!pose.update(gyro[0],gyro[1],gyro[2],0,0,0,float(step)*1e-6f,false))return false;
    durationUs-=step;
  }
  return true;
}

// 基础滤波器只在真实新样本到来时推进。
// 缺帧区间用上一角速度，最后一个正常采样周期才使用新测量及重力校正。
inline bool acceptHistorySample(Attitude &pose,const float previousGyro[3],
                                const float gyro[3],const float accel[3],
                                uint32_t dtUs,bool gravityCorrection) {
  if(dtUs<100 || dtUs>2000000)return false;
  Attitude next=pose;
  if(dtUs>50000) {
    if(!advanceHeldGyro(next,previousGyro,dtUs-5000))return false;
    dtUs=5000;
  }
  if(!next.update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],
                  float(dtUs)*1e-6f,gravityCorrection))return false;
  pose=next;
  return true;
}

// 预测从真实姿态的副本出发，不写回基础状态，避免新样本到来时重复积分。
// 预测到上限后保持该姿态，不把旧数据无限外推。
inline bool projectHistory(const Attitude &measured,const float gyro[3],
                           uint64_t ageUs,uint32_t predictionLimitUs,Attitude &projected) {
  projected=measured;
  return advanceHeldGyro(projected,gyro,ageUs<predictionLimitUs?ageUs:predictionLimitUs);
}
