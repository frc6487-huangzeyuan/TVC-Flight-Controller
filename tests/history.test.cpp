#include "../HistoryAttitude.h"
#include "../GimbalControl.h"
#include <cstdio>
#include <cstdlib>
int checks=0;
void expect(bool ok,const char *message){++checks;if(!ok){std::fprintf(stderr,"FAIL %s\n",message);std::exit(1);}}
bool near(float a,float b,float tolerance=.001f){return std::fabs(a-b)<tolerance;}
int main(){
  Attitude measured,predicted;expect(measured.initializeFromAccel(0,0,1),"real stationary reference exists");
  float gyro[3]={0,0,90},accel[3]={0,0,1};
  expect(projectHistory(measured,gyro,100000,150000,predicted)&&near(predicted.euler().yaw,9),"100ms old angular rate predicts 9 degrees");
  expect(near(measured.euler().yaw,0),"forecast does not mutate the real-sample filter");
  expect(projectHistory(measured,gyro,100000,150000,predicted)&&near(predicted.euler().yaw,9),"repeated control ticks do not double integrate the history");
  expect(projectHistory(measured,gyro,1000000,150000,predicted)&&near(predicted.euler().yaw,13.5f),"longer missing interval holds capped 150ms forecast");
  expect(!predicted.gravityCorrectionApplied(),"cached acceleration is not fed back as repeated new gravity");
  expect(acceptHistorySample(measured,gyro,gyro,accel,100000,false)&&near(measured.euler().yaw,9),"new measurement commits the gap once, not twice");
  expect(projectHistory(measured,gyro,10000,150000,predicted)&&near(predicted.euler().yaw,9.9f),"next projection starts at the last real measurement");
  float stopped[3]={0,0,0};
  expect(acceptHistorySample(measured,gyro,stopped,accel,100000,false)&&near(measured.euler().yaw,17.55f),"missing span holds previous rate and last normal interval uses new rate");
  float before=measured.euler().yaw;
  expect(!acceptHistorySample(measured,gyro,gyro,accel,2000001,false)&&near(measured.euler().yaw,before),"gap beyond disconnect window does not commit an invented pose");
  Attitude empty;expect(!projectHistory(empty,gyro,100000,150000,predicted),"history cannot fabricate a first valid sample");
  GimbalConfig c={{.1f,1,0},{.1f,1,0},15,3,.03f,180,1,1,10,50000};
  GimbalPid controller(c);int actual[2]={1500,1500},target[2]={1500,1500};
  controller.update(1000000,.2f,0,actual,target,true);
  controller.update(1010000,.2f,0,actual,target,true);
  const float integral=controller.x().terms().i;
  for(int n=2;n<100;++n){
    expect(controller.update(1000000+10000*n,.2f,0,actual,target,false)==GimbalPid::UPDATED,"control clock keeps calculating during held measurements");
    for(int k=0;k<2;++k)actual[k]=target[k];
  }
  expect(near(controller.x().terms().i,integral),"historical measurements do not accumulate integral");
  std::printf("PASS %d history projection and sample-resume checks; no hardware used.\n",checks);
}
