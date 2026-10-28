#include "../AutoLevelPid.h"
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>

static int checks=0;
static void expect(bool ok, const char *name) {
  ++checks;
  if (!ok) { std::fprintf(stderr,"FAIL: %s\n",name); std::exit(1); }
}
static bool close(float a, float b, float tolerance=0.0001f) {
  return std::isfinite(a) && std::fabs(a-b) < tolerance;
}
static AutoLevelPidConfig config() {
  return {{1,0,0},{1,0,0},25,5,0.03f,500,2500,1500,10,-1,-1,50000};
}

int main() {
  // 使用独立手算结果验证并联形式 PID 的一次更新。
  AnglePid example({2,0.5f,0.1f},25,5,0);
  example.prime(3);
  expect(example.update(0,2,0.01f,-25,25),"worked sample accepted");
  expect(close(example.terms().p,-4) && close(example.terms().i,-0.01f) &&
         close(example.terms().d,10) && close(example.terms().output,5.99f),
         "P=-4, I=-0.01, D=10, u=5.99 with seconds");

  AnglePid i100({0,1,0},25,5,0),i200({0,1,0},25,5,0);
  for(int n=0;n<100;++n)expect(i100.update(2,0,.01f,-25,25),"100 Hz I step");
  for(int n=0;n<200;++n)expect(i200.update(2,0,.005f,-25,25),"200 Hz I step");
  expect(close(i100.terms().i,2) && close(i200.terms().i,2),
         "same one-second integral at different sample rates");
  for(int n=0;n<1000;++n)i100.update(2,0,.01f,-25,25);
  expect(close(i100.terms().i,5),"integral contribution has a separate hard limit");

  AnglePid saturation({2,1,0},5,2,0);
  for(int n=0;n<1000;++n)saturation.update(20,0,.01f,-5,5);
  expect(close(saturation.terms().i,0) && close(saturation.terms().output,5),
         "saturation cannot wind the integrator up");
  saturation.update(0,0,.01f,-5,5);
  expect(close(saturation.terms().output,0),"no stored kick after saturation clears");

  AnglePid unwind({0,1,0},5,5,0);
  for(int n=0;n<100;++n)unwind.update(2,0,.01f,-5,5);
  unwind.update(-2,0,.01f,-1,1);
  expect(unwind.terms().i<2 && close(unwind.terms().output,1),
         "opposite integration unwinds even while output is limited");

  AnglePid filtered({0,0,0.1f},25,5,.03f);
  filtered.prime(0);filtered.update(0,1,.01f,-25,25);
  expect(close(filtered.terms().d,-2.5f),"D uses a first-order 30 ms filter");
  filtered.reset();filtered.prime(1);filtered.update(10,1,.01f,-25,25);
  expect(close(filtered.terms().d,0),"reference step has no derivative kick");

  AnglePid wrapped({1,0,1},25,5,0);
  wrapped.prime(179);wrapped.update(179,-179,.01f,-25,25);
  expect(close(wrapped.terms().error,-2) && close(wrapped.terms().d,-200),
         "roll crossing 180 degrees follows the short path");

  const float nan=std::numeric_limits<float>::quiet_NaN();
  const float inf=std::numeric_limits<float>::infinity();
  const PidTerms before=example.terms();
  for(float dt : {0.f,-.01f,.00001f,.051f,nan,inf})
    expect(!example.update(0,2,dt,-25,25),"invalid dt rejected");
  expect(!example.update(nan,2,.01f,-25,25),"nonfinite reference rejected");
  expect(!example.update(0,inf,.01f,-25,25),"nonfinite measurement rejected");
  expect(!example.update(0,2,.01f,2,1),"impossible reachable range rejected");
  expect(close(example.terms().output,before.output) &&
         close(example.terms().i,before.i),"rejected samples do not change state");

  auto cfg=config();
  cfg.servoStepUs=2000; // 比较原有方向时，暂时取消单周期速度限制。
  AutoLevelPid legacy(cfg);
  int actual[2]={1500,1500},target[2]={1500,1500};
  expect(legacy.configurationValid(),"valid pin-independent servo configuration");
  expect(legacy.update(1000000,-4,10,0,0,actual,target)==AutoLevelPid::WAITING,
         "first automatic sample seeds D with no output jump");
  expect(target[0]==1500 && target[1]==1500,"first sample retains center");
  expect(legacy.update(1010000,-4,10,0,0,actual,target)==AutoLevelPid::UPDATED,
         "new IMU sample updates both axes");
  expect(target[0]==1426 && target[1]==1530,
         "pure P preserves original pitch-X and roll-Y signs");
  expect(close(legacy.dt(),.01f),"dt comes from the IMU timestamp");
  expect(legacy.update(1010000,15,15,0,0,actual,target)==AutoLevelPid::WAITING,
         "duplicate timestamp does not run PID again");
  expect(target[0]==1426 && target[1]==1530,"duplicate cannot change servo targets");
  expect(legacy.update(1009000,0,0,0,0,actual,target)==AutoLevelPid::INVALID,
         "backward sample clock rejected");
  expect(legacy.update(1060001,0,0,0,0,actual,target)==AutoLevelPid::INVALID,
         "over-50 ms sample gap rejected");

  cfg=config();cfg.pitch={1,.05f,.03f};cfg.roll=cfg.pitch;
  AutoLevelPid slew(cfg);
  slew.update(1000000,0,20,0,0,actual,target);
  slew.update(1010000,0,20,0,0,actual,target);
  expect(target[0]==1490 && target[1]==1500,"one tick observes existing 10 us slew");
  expect(close(slew.pitch().terms().i,0),"slew-limited correction freezes outward I");
  for(int n=2;n<300;++n) {
    actual[0]=target[0];actual[1]=target[1];
    expect(slew.update(1000000+10000*n,0,100,0,0,actual,target)==AutoLevelPid::UPDATED,
           "long large-error run accepted");
    expect(std::abs(target[0]-actual[0])<=10,"target respects per-tick slew");
    expect(target[0]>=1315 && target[0]<=1685,"correction stays within 25 degrees");
  }
  expect(close(slew.pitch().terms().i,0),"large held error never winds up");
  slew.reset();
  expect(close(slew.pitch().terms().i,0) && close(slew.pitch().terms().d,0) &&
         close(slew.pitch().terms().output,0) && close(slew.dt(),0),
         "stop reset clears I, D, output, and sample clock");
  actual[0]=actual[1]=target[0]=target[1]=1500;
  expect(slew.update(9000000,0,0,0,0,actual,target)==AutoLevelPid::WAITING,
         "restart accepts a fresh epoch with no old elapsed time");
  slew.update(9010000,0,0,0,0,actual,target);
  expect(target[0]==1500 && target[1]==1500,"restart has no stale integral correction");

  cfg=config();cfg.servoXSign=1;cfg.servoYSign=1;cfg.servoStepUs=2000;
  AutoLevelPid flipped(cfg);
  flipped.update(1000000,-4,10,0,0,actual,target);
  flipped.update(1010000,-4,10,0,0,actual,target);
  expect(target[0]==1574 && target[1]==1470,"each configurable direction can flip");
  cfg=config();cfg.pitch.kp=-1;
  expect(!AutoLevelPid(cfg).configurationValid(),"negative gain rejected");
  cfg=config();cfg.servoXSign=0;
  expect(!AutoLevelPid(cfg).configurationValid(),"zero actuator direction rejected");
  cfg=config();cfg.correctionLimitDeg=150;
  expect(!AutoLevelPid(cfg).configurationValid(),"correction beyond servo travel rejected");
  cfg=config();cfg.pitch.ki=inf;
  expect(!AutoLevelPid(cfg).configurationValid(),"nonfinite gain rejected");

  std::printf("PASS %d PID checks (no hardware connected).\n",checks);
  return 0;
}
