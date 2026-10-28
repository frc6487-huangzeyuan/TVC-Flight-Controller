#include "../Attitude.h"
#include <cstdio>
#include <cstdlib>
#include <limits>

// 电脑端 C++11 及以上版本测试，预期结果来自物理向量和已知旋转，
// 不照抄姿态估计算法的四元数积分过程。
static int failures=0, checks=0;
static void expect(bool condition, const char *message) {
  ++checks;
  if (!condition) { ++failures; std::fprintf(stderr, "FAIL: %s\n", message); }
}
static bool close(double a, double b, double tolerance=1e-4) {
  return std::isfinite(a) && std::isfinite(b) && std::fabs(a-b)<=tolerance;
}
static void vectorIs(const AttitudeVec3 &v, double x, double y, double z,
                     const char *message, double tolerance=1e-4) {
  expect(close(v.x,x,tolerance)&&close(v.y,y,tolerance)&&close(v.z,z,tolerance),message);
}
static bool sameQuaternion(const AttitudeQuaternion &a,const AttitudeQuaternion &b) {
  return a.w==b.w&&a.x==b.x&&a.y==b.y&&a.z==b.z;
}
static double quaternionNorm(const AttitudeQuaternion &q) {
  return std::sqrt(double(q.w)*q.w+double(q.x)*q.x+double(q.y)*q.y+double(q.z)*q.z);
}
static void spin(Attitude &a,float gx,float gy,float gz,float ax,float ay,float az,
                 int samples,bool feedback=false) {
  for(int i=0;i<samples;++i)
    expect(a.update(gx,gy,gz,ax,ay,az,.005f,feedback),"normal 200 Hz update accepted");
}

int main() {
  Attitude level;
  expect(!level.initialized(),"constructor has no fabricated world frame");
  expect(!level.update(0,0,0,0,0,1,.005f,true),"update does not auto initialize");
  expect(!level.resetYaw(),"uninitialized heading cannot be reset");
  expect(std::isnan(level.worldVector(1,0,0).x),"uninitialized world vector is invalid");
  expect(level.initializeFromAccel(0,0,1),"level stationary initialization");
  vectorIs(level.worldVector(1,2,3),1,2,3,"level R_BW is identity");
  vectorIs(level.linearAcceleration(0,0,1),0,0,0,"stationary force +Z cancels gravity");
  vectorIs(level.linearAcceleration(.25f,-.5f,1.1f),2.4516625,-4.903325,.980665,
           "world linear acceleration uses correct sign and SI scale");
  expect(close(level.euler().roll,0)&&close(level.euler().pitch,0)&&
         close(level.euler().yaw,0),"explicit initial yaw is zero");

  Attitude roll90;
  expect(roll90.initializeFromAccel(0,1,0),"static roll +90 initialization");
  vectorIs(roll90.worldVector(0,1,0),0,0,1,"body +Y points up at roll +90");
  vectorIs(roll90.worldVector(0,0,1),0,-1,0,"body +Z points -worldY at roll +90");
  vectorIs(roll90.linearAcceleration(0,1,0),0,0,0,"roll90 gravity cancellation");
  expect(close(roll90.euler().roll,90),"positive roll follows right hand rule");

  Attitude pitch90;
  expect(pitch90.initializeFromAccel(-1,0,0),"static pitch +90 initialization");
  vectorIs(pitch90.worldVector(1,0,0),0,0,-1,"body +X points down at pitch +90");
  vectorIs(pitch90.worldVector(0,0,1),1,0,0,"body +Z points +worldX at pitch +90");
  vectorIs(pitch90.linearAcceleration(-1,0,0),0,0,0,"pitch90 gravity cancellation");
  const AttitudeQuaternion verticalBefore=pitch90.quaternion();
  expect(!pitch90.euler().headingDefined,"vertical body X has undefined heading");
  expect(!pitch90.resetYaw(),"vertical body X refuses resetYaw");
  expect(sameQuaternion(verticalBefore,pitch90.quaternion()),"refused yaw reset preserves q");
  Attitude nearlyVertical;
  const double pi=3.14159265358979323846;
  const double nearPitch=89.7*pi/180;
  expect(nearlyVertical.initializeFromAccel(float(-std::sin(nearPitch)),0,
                                           float(std::cos(nearPitch))),"near-vertical init");
  expect(!nearlyVertical.resetYaw(),"near-vertical horizontal projection is rejected");

  Attitude inverted;
  expect(inverted.initializeFromAccel(0,0,-1),"static inverted 180-degree initialization");
  vectorIs(inverted.worldVector(1,0,0),1,0,0,"inverted body X remains worldX");
  vectorIs(inverted.worldVector(0,1,0),0,-1,0,"inverted body Y has correct sign");
  vectorIs(inverted.worldVector(0,0,1),0,0,-1,"inverted body Z points down");
  vectorIs(inverted.linearAcceleration(0,0,-1),0,0,0,"inverted gravity cancellation");
  spin(inverted,0,0,0,0,0,-1,100,true);
  vectorIs(inverted.worldVector(0,0,1),0,0,-1,"inverted static pose remains stable");

  Attitude yaw;
  expect(yaw.initializeFromAccel(0,0,1),"yaw test initialization");
  spin(yaw,0,0,90,0,0,1,200,true);
  vectorIs(yaw.worldVector(1,0,0),0,1,0,"positive yaw rotates bodyX toward worldY");
  expect(close(yaw.euler().yaw,90,.002),"90 deg/s for one second yields +90 yaw");
  spin(yaw,0,0,0,0,0,1,200,true);
  expect(close(yaw.euler().yaw,90,.002),"gravity alone does not cancel yaw");
  spin(yaw,0,0,-90,0,0,1,400,false);
  vectorIs(yaw.worldVector(1,0,0),0,-1,0,"negative yaw rate gives -90 heading");

  Attitude rollRate,pitchRate;
  expect(rollRate.initializeFromAccel(0,0,1)&&pitchRate.initializeFromAccel(0,0,1),
         "gyro sign tests initialize");
  spin(rollRate,90,0,0,0,0,1,200);
  vectorIs(rollRate.worldVector(0,0,1),0,-1,0,"gyro +X uses right-handed roll");
  spin(pitchRate,0,90,0,0,0,1,200);
  vectorIs(pitchRate.worldVector(0,0,1),1,0,0,"gyro +Y uses right-handed pitch");

  Attitude tilted;
  const double roll=25*pi/180, pitch=-20*pi/180;
  const float ax=float(-std::sin(pitch)), ay=float(std::cos(pitch)*std::sin(roll)),
              az=float(std::cos(pitch)*std::cos(roll));
  expect(tilted.initializeFromAccel(ax,ay,az),"combined static tilt initialization");
  const AttitudeMatrix tiltRotation=tilted.rotation();
  // 沿 R 转置乘世界 Z 轴方向的机体角速度，只改变航向并保持实际倾角。
  spin(tilted,90*tiltRotation.m[2][0],90*tiltRotation.m[2][1],
       90*tiltRotation.m[2][2],ax,ay,az,200);
  const AttitudeEuler beforeReset=tilted.euler();
  const AttitudeMatrix beforeRotation=tilted.rotation();
  expect(close(beforeReset.yaw,90,.002),"tilted body rotates about worldZ");
  expect(tilted.resetYaw(),"defined horizontal heading can reset");
  const AttitudeEuler afterReset=tilted.euler();
  const AttitudeMatrix afterRotation=tilted.rotation();
  expect(close(afterReset.yaw,0,.002),"resetYaw chooses current body-X projection as worldX");
  expect(close(beforeReset.roll,afterReset.roll,.002)&&close(beforeReset.pitch,afterReset.pitch,.002),
         "resetYaw preserves roll and pitch");
  for(int i=0;i<3;++i)expect(close(beforeRotation.m[2][i],afterRotation.m[2][i]),
                          "resetYaw preserves world-vertical row");
  vectorIs(tilted.linearAcceleration(ax,ay,az),0,0,0,"resetYaw does not flatten tilt");

  Attitude corrective;
  expect(corrective.initializeFromAccel(0,float(std::sin(10*pi/180)),
                                       float(std::cos(10*pi/180))),"feedback test starts tilted");
  expect(corrective.update(0,0,0,0,0,1,.005f,true),"gravity feedback step accepted");
  expect(corrective.gravityCorrectionApplied()&&corrective.euler().roll<10,
         "gravity error has restoring rather than destabilizing sign");
  spin(corrective,0,0,0,0,0,1,1000,true);
  expect(std::fabs(corrective.euler().roll)<.01,"static tilt error converges");
  Attitude gated;
  expect(gated.initializeFromAccel(0,0,1),"gate test initialization");
  expect(gated.update(0,0,0,.6f,0,.8f,.005f,false),"caller can suppress gravity correction");
  expect(!gated.gravityCorrectionApplied()&&close(gated.euler().pitch,0),
         "disabled correction leaves static gyro orientation unchanged");
  expect(gated.update(0,0,0,1,0,1,.005f,true),"off-1g acceleration uses gyro-only update");
  expect(!gated.gravityCorrectionApplied()&&close(gated.euler().pitch,0),
         "non-gravity magnitude cannot force false tilt");
  expect(gated.update(0,0,0,0,0,0,.005f,true),"zero acceleration avoids normalization singularity");
  expect(!gated.gravityCorrectionApplied(),"zero acceleration has no gravity feedback");

  Attitude longRun;
  expect(longRun.initializeFromAccel(0,0,1),"normalization test initialization");
  spin(longRun,37,-19,11,0,0,0,20000,false);
  expect(close(quaternionNorm(longRun.quaternion()),1,2e-7),"quaternion stays unit length");
  const AttitudeMatrix r=longRun.rotation();
  for(int i=0;i<3;++i)for(int j=0;j<3;++j){
    double dot=0;for(int k=0;k<3;++k)dot+=double(r.m[k][i])*r.m[k][j];
    expect(close(dot,i==j?1:0,3e-7),"R_BW columns remain orthonormal");
  }
  const double det=double(r.m[0][0])*(double(r.m[1][1])*r.m[2][2]-double(r.m[1][2])*r.m[2][1])
                  -double(r.m[0][1])*(double(r.m[1][0])*r.m[2][2]-double(r.m[1][2])*r.m[2][0])
                  +double(r.m[0][2])*(double(r.m[1][0])*r.m[2][1]-double(r.m[1][1])*r.m[2][0]);
  expect(close(det,1,3e-7),"R_BW is proper right-handed rotation (det +1)");

  const float nan=std::numeric_limits<float>::quiet_NaN();
  const float inf=std::numeric_limits<float>::infinity();
  const AttitudeQuaternion validBefore=gated.quaternion();
  expect(!gated.update(nan,0,0,0,0,1,.005f,true),"NaN gyro is rejected");
  expect(!gated.update(0,0,inf,0,0,1,.005f,true),"infinite gyro is rejected");
  expect(!gated.update(0,0,0,0,nan,1,.005f,false),"NaN accel rejected even in gyro-only mode");
  expect(!gated.update(0,0,0,0,0,1,0,true),"zero dt rejected");
  expect(!gated.update(0,0,0,0,0,1,-.005f,true),"negative dt rejected");
  expect(!gated.update(0,0,0,0,0,1,nan,true),"NaN dt rejected");
  expect(!gated.initializeFromAccel(0,0,0),"zero-gravity initialization rejected");
  expect(!gated.initializeFromAccel(0,0,2),"dynamic initialization rejected");
  expect(!gated.initializeFromAccel(inf,0,1),"nonfinite initialization rejected");
  expect(sameQuaternion(validBefore,gated.quaternion()),"invalid operations preserve last q");
  expect(std::isnan(gated.worldVector(nan,0,0).x),"nonfinite world-vector input rejected");
  expect(std::isnan(gated.linearAcceleration(0,inf,0).z),"nonfinite SI acceleration rejected");
  expect(!gated.setGravityGain(-1)&&!gated.setGravityGain(nan),"invalid gain rejected");

  const AttitudeQuaternion yawBeforeGap=yaw.quaternion();
  expect(!yaw.update(0,0,90,0,0,1,.051f,true),"sampling gap rejected");
  expect(!yaw.initialized(),"gap invalidates tracking until explicit initialization");
  expect(sameQuaternion(yawBeforeGap,yaw.quaternion()),"gap does not reset world heading or q");
  expect(!yaw.update(0,0,0,0,0,1,.005f,true),"next sample cannot silently reinitialize");
  expect(sameQuaternion(yawBeforeGap,yaw.quaternion()),"post-gap update retains diagnostic q");
  expect(std::isnan(yaw.linearAcceleration(0,0,1).x),"gap makes world acceleration invalid");
  expect(yaw.initializeFromAccel(0,0,1)&&close(yaw.euler().yaw,0),
         "explicit recalibration may establish new heading zero");
  const AttitudeQuaternion qBeforeInvalidate=yaw.quaternion();
  yaw.invalidate();
  expect(!yaw.initialized()&&sameQuaternion(qBeforeInvalidate,yaw.quaternion()),
         "external stale-sample invalidation also preserves diagnostic orientation");

  std::printf("Attitude tests: %d checks, %d failures\n",checks,failures);
  return failures?EXIT_FAILURE:EXIT_SUCCESS;
}
