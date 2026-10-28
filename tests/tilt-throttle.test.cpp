#include "../TiltThrottle.h"
#include "../GimbalControl.h"
#include <cstdio>
#include <cstdlib>
static int checks=0;
void expect(bool ok,const char *message){++checks;if(!ok){std::fprintf(stderr,"FAIL %s\n",message);std::exit(1);}}
int main(){
  TiltThrottle t;expect(t.valid(),"default configuration valid");
  const auto z=[](float deg){return std::cos(deg*.0174532925199433f);};
  uint64_t now=1000000;t.update(now,z(12),true);
  expect(t.percent==0&&std::fabs(t.requested-10)<.001,"12-degree tilt requests 10 percent after 2-degree deadband; first update is stopped");
  for(int i=0;i<100;i++){now+=10000;expect(t.update(now,z(12),true),"fresh update succeeds");expect(t.percent<=10.001&&t.percent>=0,"output respects requested range");}
  expect(std::fabs(t.percent-10)<.001,"10 percent per second ramp reaches 10 after one second");
  const float old=t.percent;for(int i=0;i<20;i++){now+=10000;t.update(now,z(80),false);expect(t.percent<=old+.001,"historical sample cannot increase throttle");}
  now+=10000;t.update(now,z(180),true);expect(t.requested==30,"inverted body still respects maximum throttle");
  for(int i=0;i<200;i++){now+=10000;t.update(now,z(100),true);expect(t.pulse(1500,1700)>=1500&&t.pulse(1500,1700)<=1560,"PWM never exceeds 30 percent of forward span");}
  for(int i=0;i<400;i++){now+=10000;t.update(now,1,true);}expect(std::fabs(t.percent)<.001&&t.pulse(1500,1700)==1500,"upright body ramps back to base zero");
  t.config.base=5;now+=10000;t.update(now,z(1),true);expect(t.requested==5,"nonzero base is retained inside deadband");
  expect(!t.update(now+100000,z(10),true),"control scheduler gap rejected");
  t.reset();expect(t.percent==0&&t.requested==0,"reset clears all throttle state");
  expect(!t.update(now,NAN,true),"nonfinite orientation rejected");
  t.config.base=40;expect(!t.valid(),"base cannot exceed cap");t.config.base=0;t.config.ramp=0;expect(!t.valid(),"zero rate rejected");
  for(int us=-100;us<=3000;us+=7){int out=limitServoUs(1,us);expect(out>=1130&&out<=1870,"final elevation PWM clamp always respects mechanical travel");}
  expect(GimbalPid::usToAngle(1130,1)>=85&&GimbalPid::usToAngle(1870,1)<=185,"rounded PWM endpoints stay within signed 50-degree boundary");
  GimbalConfig c={{10,0,0},{10,0,0},50,3,.03,180,1,1,3,50000};
  GimbalPid pid(c);expect(pid.configurationValid(),"50-degree correction accepted");
  int actual[2]={1500,1500},target[2]={1500,1500};
  for(int n=0;n<500;n++){
    auto result=pid.update(1000000+10000*n,n<250?90:-90,30,actual,target);
    expect(result!=GimbalPid::INVALID,"valid control trace");
    expect(target[1]>=1130&&target[1]<=1870,"PID target never crosses elevation limit");
    for(int k=0;k<2;k++)actual[k]=target[k];
  }
  std::printf("PASS %d throttle, history hold, reset, PWM cap and signed elevation-limit checks; no hardware used.\n",checks);
}
