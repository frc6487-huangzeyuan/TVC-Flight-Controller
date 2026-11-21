#include "../ThrustTorqueControl.h"
#include "../ServoCalibration.h"
#include <cassert>
#include <cstdio>

static void checkRewind(const GimbalConfig &g,float startDeg,float nextDeg) {
  ThrustTorqueControl controller;
  ThrustTorqueConfig c;
  c.fullTorqueNm=3;c.basePct=50;c.maximumPct=60;
  c.gains[0]=c.gains[1]={.2f,.1f,.01f};
  const float angle=(nextDeg-g.azimuthZero)*.0174532925199433f;
  c.trimNm[0]=.05f*std::cos(angle)*g.signX;
  c.trimNm[1]=.05f*std::sin(angle)*g.signY;
  assert(controller.configure(c));
  AttitudeMatrix upright{};
  for(int k=0;k<3;++k)upright.m[k][k]=1;
  const float gyro[3]={};
  const int initial=GimbalPid::angleToUs(startDeg,0);
  const int home=GimbalPid::angleToUs(g.azimuthZero,0);
  int sent[2]={initial,ELEVATION_MAX_US},goal[2];
  int stoppedAtHome=0,maxStep=0;
  bool sawRewind=false;
  bool resumed=false;
  for(uint64_t t=10000;t<3000000;t+=10000) {
    assert(controller.update(g,t,upright,gyro,sent,goal));
    if(!controller.rewinding()) {
      assert(sawRewind);
      assert(stoppedAtHome>=15 && sent[0]==home);
      resumed=true;
      break;
    }
    sawRewind=true;
    assert(controller.throttlePct>=0 && controller.throttlePct<=c.basePct);
    const float alignment=controller.requestedNm[0]*controller.allocatedNm[0]+
                          controller.requestedNm[1]*controller.allocatedNm[1];
    assert(alignment>=-1e-5f);
    assert(controller.limited && controller.terms[0].i==0 && controller.terms[1].i==0);
    if(goal[0]!=sent[0]){
      maxStep=std::max(maxStep,std::abs(goal[0]-sent[0]));
      assert(maxStep<=g.stepUs);
      assert(std::abs(goal[0]-home)<std::abs(sent[0]-home));
    }
    if(sent[0]==home && goal[0]==home)++stoppedAtHome;
    sent[0]=goal[0];sent[1]=goal[1];
  }
  assert(resumed && maxStep==g.stepUs);
}

int main() {
  GimbalConfig g{{1,0,0},{1,0,0},25,3,.03f,180,1,1,25,50000};
  ThrustTorqueControl control;
  ThrustTorqueConfig c;
  assert(!control.configure(c));
  assert(!control.setTarget(NAN,0));
  assert(!control.setTarget(20.1f,0));
  c.fullTorqueNm=4;c.basePct=25;c.maximumPct=60;
  c.gains[0]=c.gains[1]={.2f,.1f,.01f};
  assert(control.configure(c));
  assert(control.setTarget(10,0));
  AttitudeMatrix upright{};
  for(int k=0;k<3;++k)upright.m[k][k]=1;
  int sent[2]={1500,1500},goal[2];
  float stopped[3]={},moving[3]={4,3,0};
  assert(control.update(g,10000,upright,stopped,sent,goal));
  assert(control.terms[0].p==2 && control.throttlePct==c.maximumPct && goal[1]>sent[1]);
  assert(control.update(g,20000,upright,stopped,sent,goal));
  assert(control.terms[0].p>0 && control.terms[0].d==0 && goal[1]>sent[1]);
  assert(control.update(g,30000,upright,moving,sent,goal));
  assert(control.terms[0].d<0 && control.terms[1].d>0);
  assert(!control.update(g,90000,upright,stopped,sent,goal));
  control.reset();
  assert(control.update(g,100000,upright,stopped,sent,goal));
  bool boosted=false;
  for(uint64_t tick=110000;tick<=3000000;tick+=10000) {
    assert(control.update(g,tick,upright,stopped,sent,goal));
    boosted|=control.throttlePct>c.basePct;
    assert(control.throttlePct<=c.maximumPct && std::abs(goal[1]-sent[1])<=g.stepUs);
    for(int k=0;k<2;++k)sent[k]=goal[k];
  }
  assert(boosted && control.limited && control.throttlePct==c.maximumPct);
  assert(sent[1]==ELEVATION_MAX_US);
  assert(control.requestedNm[0]>control.allocatedNm[0]);
  const float integral=control.terms[0].i;
  assert(control.update(g,3010000,upright,stopped,sent,goal));
  assert(control.terms[0].i==integral);
  assert(control.setTarget(0,0));
  assert(control.update(g,3020000,upright,stopped,sent,goal));
  assert(control.terms[0].d==0);
  auto geometry=g;
  geometry.limit=ELEVATION_MAX_DEFLECTION_DEG;
  checkRewind(geometry,359,2);
  geometry.signY=-1;
  checkRewind(geometry,1,358);
  const ServoCalibration servo{1868,175,0,1};
  const int elevationCenter=GimbalPid::angleToUs(servo.elevationDeg,1);
  assert(ELEVATION_MIN_US==1315 && ELEVATION_MAX_US==1685);
  assert(servo.outputUs(1,500,elevationCenter)==elevationCenter+185);
  assert(servo.outputUs(1,2500,elevationCenter)==elevationCenter-185);
  assert(servo.outputUs(1,2500,ELEVATION_PHYSICAL_MIN_US)==ELEVATION_PHYSICAL_MIN_US);
  // 电调尚未升速时，分配量反映 1505 µs，而不是尚未发出的目标值。
  auto request=c;
  request.fullTorqueNm=2;request.basePct=50;request.maximumPct=75;request.trimNm[0]=.4f;
  assert(control.configure(request) && control.setTarget(0,0));
  const int center[2]={1500,1500};
  const TorqueEscStep stoppedEsc{1500,1700,1500,5};
  assert(control.update(g,4000000,upright,stopped,center,goal,&stoppedEsc));
  assert(stoppedEsc.nextUs(control.throttlePct)==1505 && control.limited);
  assert(control.allocatedNm[0]>0 && control.allocatedNm[0]<.0001f);

  request.trimNm[0]=-.2f;
  assert(control.configure(request));
  const int positiveElevation[2]={1500,ELEVATION_MAX_US};
  const TorqueEscStep runningEsc{1500,1700,1600,5};
  assert(control.update(g,4010000,upright,stopped,positiveElevation,goal,&runningEsc));
  assert(control.throttlePct==0 && goal[1]<positiveElevation[1]);
  assert(runningEsc.nextUs(control.throttlePct)==1595 && control.limited);

  AttitudeMatrix inverted=upright;
  inverted.m[0][0]=inverted.m[2][2]=-1;
  assert(!control.update(g,4020000,inverted,stopped,center,goal));
  std::puts("PASS next issued ESC estimate, reverse demand ramp-down and inverted rejection");
  std::puts("PASS ±25 degree elevation, both unwind directions, projected correction, home dwell, and resume");
  std::puts("PASS target ramp, gyro damping, thrust allocation, slew, saturation, and integral guard");
}
