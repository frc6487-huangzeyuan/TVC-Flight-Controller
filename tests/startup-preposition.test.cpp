#include "../StartupPreposition.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main(){
  GimbalConfig cfg{{1,0,0},{.5f,0,0},15,3,.03f,180,1,1,3,50000};
  StartupPreposition p;int sent[2]={1500,1500};
  assert(p.begin(cfg,5,5,sent));
  float az=GimbalPid::usToAngle(p.goal(0),0),el=GimbalPid::usToAngle(p.goal(1),1);
  float angle=(az-180)*.0174532925199433f;
  assert(std::fabs((el-135)*std::cos(angle)+5)<.2f);
  assert(std::fabs((el-135)*std::sin(angle)+2.5f)<.2f);
  assert(!p.ready(sent,0,2000,true));
  sent[0]=p.goal(0);sent[1]=p.goal(1);
  assert(!p.ready(sent,100,2000,true));
  assert(!p.ready(sent,2099,2000,true));
  assert(p.ready(sent,2100,2000,true));
  assert(!p.ready(sent,2200,2000,false));
  assert(!p.ready(sent,2300,2000,true));
  assert(!p.moved(5.2f,5.2f,2));assert(p.moved(8,5,2));
  p.cancel();assert(!p.ready(sent,10000,2000,true));
  assert(!p.begin(cfg,NAN,0,sent));
  sent[0]=sent[1]=1500;
  assert(p.begin(cfg,-5,0,sent));const int firstGoal=p.goal(1);
  assert(p.begin(cfg,5,0,sent));assert(p.goal(1)!=firstGoal);
  cfg.signX=-1;assert(p.begin(cfg,5,0,sent));assert(p.goal(1)==firstGoal);
  cfg.signX=1;assert(p.begin(cfg,100,100,sent));
  assert(std::fabs(GimbalPid::usToAngle(p.goal(1),1)-135)<=15.1f);
  std::puts("PASS startup target, PWM dwell, stale measurement and cancellation");
}
