#include "../StartupPreposition.h"
#include <cassert>
#include <cstdio>
static bool near(float a,float b){return std::fabs(a-b)<.002f;}
int main(){
  GimbalConfig c{{2,0,1},{2,0,1},15,3,.03f,180,1,1,3,50000};
  float vx,vy;GimbalPid::initialVector(c,2,-3,vx,vy);
  assert(near(vx,-4)&&near(vy,6)); // 旧配置的比例纠偏不变。
  c.model.enabled=true;c.model.bias[0]=-2;c.model.bias[1]=1;
  c.model.gravity[0]=3;c.model.gravity[3]=4;
  c.model.inverse[0]=.5f;c.model.inverse[3]=.25f;
  assert(GimbalPid(c).configurationValid());
  // 独立正向动力学检查：A*q+B*(v-bias) 应等于请求的角加速度。
  GimbalPid::correctionVector(c,1,-2,-5,6,vx,vy);
  assert(near(3+2*(vx+2),-5)&&near(-8+4*(vy-1),6));
  GimbalPid::initialVector(c,0,0,vx,vy);assert(near(vx,-2)&&near(vy,1));
  int sent[2]={1500,1500};StartupPreposition prepare;
  assert(prepare.begin(c,0,0,sent));
  float az=GimbalPid::usToAngle(prepare.goal(0),0),el=GimbalPid::usToAngle(prepare.goal(1),1);
  float a=(az-180)*.0174532925199433f;
  assert(std::fabs((el-135)*std::cos(a)+2)<.15f);
  assert(std::fabs((el-135)*std::sin(a)-1)<.15f);
  GimbalPid pid(c);int target[2]={1500,1500};
  for(int i=0;i<600;++i){
    auto result=pid.update(1000000+10000*i,i<300?20:-20,5,sent,target);
    assert(result!=GimbalPid::INVALID);
    for(int k=0;k<2;++k){assert(std::abs(target[k]-sent[k])<=3);sent[k]=target[k];}
    assert(sent[1]>=1130&&sent[1]<=1870);
  }
  c.model.inverse[3]=0;assert(!GimbalPid(c).configurationValid());
  c.model.inverse[3]=.25f;c.model.gravity[0]=NAN;assert(!GimbalPid(c).configurationValid());
  std::puts("PASS model inversion, legacy defaults, startup bias, actuator limits and invalid calibration");
}
