#pragma once
#include <cmath>
#include <limits>

// 独立的 C++11 数学计算，不依赖 Arduino、Wi-Fi 或传感器驱动。
// 坐标约定：右手世界坐标系，Z 轴向上；q=(w,x,y,z) 将机体坐标旋转到世界坐标。
// 旋转关系：R_BW = Rz(yaw) Ry(pitch) Rx(roll)，v_W = R_BW v_B，向量均为列向量。
// 输入数据已经由调用者按照固定的安装旋转矩阵，从传感器坐标转换到机体坐标，
// 并且已经由调用者扣除了陀螺仪零偏。
// 加速度计输入是以 g 为单位的比力，不是已经去除重力的线加速度。
// 设备静止、水平且机体 Z 轴朝上时，读数应接近 (0,0,+1)。
// 参考文献：Mahony、Hamel 和 Pflimlin，IEEE TAC，2008 年，附录 B，公式 (48)。
// DOI：10.1109/TAC.2008.923738。本实现只使用重力方向的比例反馈，
// 并按角速度恒定的假设精确更新四元数；不包含积分反馈，
// 也不包含磁力计校正。航向是相对值，可能随时间漂移。
// 本算法用于电子实验平台的姿态估计，不用于位置估计或飞行导航。

struct AttitudeVec3 { float x, y, z; };
struct AttitudeQuaternion { float w, x, y, z; };
struct AttitudeMatrix { float m[3][3]; };
struct AttitudeEuler {
  float roll, pitch, yaw;  // 单位为度，采用 ZYX 顺序的欧拉角。
  bool headingDefined;   // 机体 X 轴接近竖直时为 false，此时无法可靠定义航向。
};

class Attitude {
 public:
  Attitude() : q_{1.0f, 0.0f, 0.0f, 0.0f}, initialized_(false),
               gravityApplied_(false), gravityKp_(2.0f) {}

  bool initialized() const { return initialized_; }
  bool gravityCorrectionApplied() const { return gravityApplied_; }

  // 明确将姿态跟踪标记为无效，但不替换原来的四元数。
  // 调用者可保留 q/R 用于诊断，但必须把世界坐标数据标记为无效。
  void invalidate() { initialized_ = false; gravityApplied_ = false; }

  bool setGravityGain(float radiansPerSecond) {
    if (!finite(radiansPerSecond) || radiansPerSecond < 0.0f ||
        radiansPerSecond > 20.0f) return false;
    gravityKp_ = radiansPerSecond;
    return true;
  }

  // 只有调用者确认整个校准窗口静止且数据正常后，才能调用本函数。
  // 此操作明确建立新的相对世界航向，使 yaw=0。
  // 根据重力方向估计倾角，不会把实际倾斜的机体强行定义为水平。
  // 仅检查加速度模长，不能证明设备处于静止状态。
  bool initializeFromAccel(float ax, float ay, float az) {
    if (!finite3(ax, ay, az)) return false;
    const double norm = norm3(ax, ay, az);
    if (!gravityNorm(norm)) return false;
    const double roll = std::atan2(double(ay), double(az));
    const double pitch = std::atan2(-double(ax),
                                  std::sqrt(double(ay)*ay + double(az)*az));
    const double cr = std::cos(roll*0.5), sr = std::sin(roll*0.5);
    const double cp = std::cos(pitch*0.5), sp = std::sin(pitch*0.5);
    AttitudeQuaternion candidate = {
      float(cr*cp), float(sr*cp), float(cr*sp), float(-sr*sp)
    };
    if (!normalize(candidate)) return false;
    q_ = candidate;
    initialized_ = true;
    gravityApplied_ = false;
    return true;
  }

  // 机体角速度单位为度/秒，比力单位为 g，dt 单位为秒。
  // 调用者通过 allowCorrection，在运动或测量可信度较低时禁止重力反馈，
  // 加速度模长门限作为额外检查。
  // 输入无效时返回 false，并保持 q 不变；采样间隔超过 50 ms 时还会
  // 使姿态跟踪失效，必须显式重新初始化或校准，
  // 不会悄悄重置 yaw，也不会跨越未知运动区间继续积分。
  bool update(float gx, float gy, float gz, float ax, float ay, float az,
              float dt, bool allowCorrection) {
    gravityApplied_ = false;
    if (!initialized_) return false;
    if (!finite(dt) || dt <= 0.0f || !finite3(gx, gy, gz) ||
        !finite3(ax, ay, az)) return false;
    if (dt > 0.05f) { invalidate(); return false; }

    const double toRadians = pi()/180.0;
    double wx = double(gx)*toRadians, wy = double(gy)*toRadians,
           wz = double(gz)*toRadians;
    const double norm = norm3(ax, ay, az);
    bool applied = false;
    if (allowCorrection && gravityKp_ > 0.0f && gravityNorm(norm)) {
      const AttitudeMatrix r = rotation();
      // R_BW 的转置乘以世界向上向量，等于 R_BW 第 2 行，用机体坐标表示。
      const double ux = r.m[2][0], uy = r.m[2][1], uz = r.m[2][2];
      const double mx = ax/norm, my = ay/norm, mz = az/norm;
      // 用测得的向上方向叉乘估计的向上方向；正倾角误差通过
      // 相反方向的角速度来校正，该校正向量在世界 Z 轴上没有分量。
      wx += gravityKp_ * (my*uz - mz*uy);
      wy += gravityKp_ * (mz*ux - mx*uz);
      wz += gravityKp_ * (mx*uy - my*ux);
      applied = true;
    }

    const double speed = std::sqrt(wx*wx + wy*wy + wz*wz);
    if (!std::isfinite(speed)) return false;
    const double halfAngle = speed * double(dt) * 0.5;
    const double scale = speed > 1e-12 ? std::sin(halfAngle)/speed : double(dt)*0.5;
    const AttitudeQuaternion increment = {
      float(std::cos(halfAngle)), float(wx*scale), float(wy*scale), float(wz*scale)
    };
    AttitudeQuaternion candidate = multiply(q_, increment);
    if (!normalize(candidate)) return false;
    q_ = candidate;
    gravityApplied_ = applied;
    return true;
  }

  // 将机体 X 轴当前在水平面上的投影，选作新的世界 X 轴。
  // 只绕世界 Z 轴旋转，保留横滚、俯仰及重力对齐关系。
  // 机体 X 轴与竖直方向夹角小于约 0.57° 时拒绝操作，
  // 因为此时无法稳定定义其水平航向。
  bool resetYaw() {
    if (!initialized_) return false;
    const AttitudeMatrix r = rotation();
    const double horizontal = std::sqrt(double(r.m[0][0])*r.m[0][0] +
                                        double(r.m[1][0])*r.m[1][0]);
    if (!std::isfinite(horizontal) || horizontal < 0.01) return false;
    const double heading = std::atan2(double(r.m[1][0]), double(r.m[0][0]));
    const AttitudeQuaternion aboutWorldZ = {
      float(std::cos(-heading*0.5)), 0.0f, 0.0f, float(std::sin(-heading*0.5))
    };
    AttitudeQuaternion candidate = multiply(aboutWorldZ, q_);
    if (!normalize(candidate)) return false;
    q_ = candidate;
    return true;
  }

  // 调用 invalidate() 后仍保留 q/R，供诊断界面显示最后一次估计结果，
  // 但 initialized() 为 false 时，这些值不代表姿态跟踪仍然有效。
  AttitudeQuaternion quaternion() const { return q_; }
  AttitudeMatrix rotation() const {
    const double w=q_.w, x=q_.x, y=q_.y, z=q_.z;
    AttitudeMatrix r = {{{
      float(1-2*(y*y+z*z)), float(2*(x*y-w*z)), float(2*(x*z+w*y))
    }, {
      float(2*(x*y+w*z)), float(1-2*(x*x+z*z)), float(2*(y*z-w*x))
    }, {
      float(2*(x*z-w*y)), float(2*(y*z+w*x)), float(1-2*(x*x+y*y))
    }}};
    return r;
  }

  AttitudeEuler euler() const {
    const AttitudeMatrix r = rotation();
    double sinePitch = -double(r.m[2][0]);
    if (sinePitch > 1.0) sinePitch = 1.0;
    if (sinePitch < -1.0) sinePitch = -1.0;
    const double toDegrees = 180.0/pi();
    const double horizontal = std::sqrt(double(r.m[0][0])*r.m[0][0] +
                                        double(r.m[1][0])*r.m[1][0]);
    AttitudeEuler angles = {
      float(std::atan2(double(r.m[2][1]), double(r.m[2][2]))*toDegrees),
      float(std::asin(sinePitch)*toDegrees),
      float(std::atan2(double(r.m[1][0]), double(r.m[0][0]))*toDegrees),
      horizontal >= 0.01
    };
    // 万向锁位置的欧拉横滚角和偏航角不唯一，坐标变换应使用 q 或 R。
    return angles;
  }

  // 保持原向量单位；未初始化、数值非有限或溢出时，返回 NaN 向量。
  AttitudeVec3 worldVector(float bx, float by, float bz) const {
    if (!initialized_ || !finite3(bx, by, bz)) return invalidVector();
    const AttitudeMatrix r = rotation();
    AttitudeVec3 v = {
      float(double(r.m[0][0])*bx + double(r.m[0][1])*by + double(r.m[0][2])*bz),
      float(double(r.m[1][0])*bx + double(r.m[1][1])*by + double(r.m[1][2])*bz),
      float(double(r.m[2][0])*bx + double(r.m[2][1])*by + double(r.m[2][2])*bz)
    };
    return finite3(v.x, v.y, v.z) ? v : invalidVector();
  }

  // 线加速度：a_W = R_BW * f_B + g_W；f_B 输入单位为 g，g_W=(0,0,-9.80665)。
  // 这里只估计局部世界坐标系下的线加速度，不估计速度或位置。
  AttitudeVec3 linearAcceleration(float axG, float ayG, float azG) const {
    const AttitudeVec3 f = worldVector(axG, ayG, azG);
    const double gravity = 9.80665;
    AttitudeVec3 a = {
      float(double(f.x)*gravity), float(double(f.y)*gravity),
      float((double(f.z)-1.0)*gravity)
    };
    return finite3(a.x, a.y, a.z) ? a : invalidVector();
  }

 private:
  AttitudeQuaternion q_;
  bool initialized_, gravityApplied_;
  float gravityKp_;
  static double pi() { return 3.14159265358979323846; }
  static bool finite(float v) { return std::isfinite(double(v)); }
  static bool finite3(float x, float y, float z) {
    return finite(x) && finite(y) && finite(z);
  }
  static double norm3(float x, float y, float z) {
    return std::sqrt(double(x)*x + double(y)*y + double(z)*z);
  }
  static bool gravityNorm(double norm) {
    return std::isfinite(norm) && norm >= 0.85 && norm <= 1.15;
  }
  static AttitudeVec3 invalidVector() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    return {nan, nan, nan};
  }
  static AttitudeQuaternion multiply(const AttitudeQuaternion &a,
                                     const AttitudeQuaternion &b) {
    return {
      float(double(a.w)*b.w-double(a.x)*b.x-double(a.y)*b.y-double(a.z)*b.z),
      float(double(a.w)*b.x+double(a.x)*b.w+double(a.y)*b.z-double(a.z)*b.y),
      float(double(a.w)*b.y-double(a.x)*b.z+double(a.y)*b.w+double(a.z)*b.x),
      float(double(a.w)*b.z+double(a.x)*b.y-double(a.y)*b.x+double(a.z)*b.w)
    };
  }
  static bool normalize(AttitudeQuaternion &q) {
    const double norm = std::sqrt(double(q.w)*q.w+double(q.x)*q.x+
                                  double(q.y)*q.y+double(q.z)*q.z);
    if (!std::isfinite(norm) || norm < 1e-12) return false;
    q.w=float(q.w/norm);q.x=float(q.x/norm);q.y=float(q.y/norm);q.z=float(q.z/norm);
    return finite(q.w) && finite3(q.x,q.y,q.z);
  }
};
