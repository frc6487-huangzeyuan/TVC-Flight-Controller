#pragma once
#include "PrimaryUartProtocol.h"
#include <cstdio>

// 完整一行日志可放入串口硬件 FIFO，避免多个任务的日志片段交错。
// 原始值换算：加速度除以 8192 得到 g；角速度除以 65.5 得到度/秒；温度除以 340 再加 36.53 得到摄氏度。
inline int formatPrimaryDebug(char *out,size_t capacity,const char *side,const char *result,const PrimaryUart::Response &r){
  return std::snprintf(out,capacity,"#IMU1 %s q=%lu s=%lu st=%u a=%d,%d,%d g=%d,%d,%d t=%d age=%lu r=%s\n",
    side,(unsigned long)r.id,(unsigned long)r.sequence,unsigned(r.status),
    int(r.raw[0]),int(r.raw[1]),int(r.raw[2]),int(r.raw[4]),int(r.raw[5]),int(r.raw[6]),int(r.raw[3]),(unsigned long)r.ageUs,result);
}
