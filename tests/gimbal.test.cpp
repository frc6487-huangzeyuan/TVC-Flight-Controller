#include "../GimbalControl.h"
#include "../bench_config.h"
#include <cstdio>
#include <cstdlib>
static int checks=0;
void expect(bool ok,const char *message){++checks;if(!ok){std::fprintf(stderr,"FAIL %s\n",message);std::exit(1);}}
bool near(float a,float b,float tolerance=.001f){return std::fabs(a-b)<tolerance;}
GimbalConfig config(){return {{1,0,0},{1,0,0},15,3,.03f,180,1,1,10,50000};}
int main(){
  float mount[3][3],gravity[3]={0,1,0};
  expect(buildUprightMount(gravity,mount),"vertical PCB mounting can be calibrated");
  for(int row=0;row<3;++row){float v=0;for(int c=0;c<3;++c)v+=mount[row][c]*gravity[c];expect(near(v,row==2?1:0),"stationary upright gravity maps to body +Z");}
  expect(near(mount[0][2],1)&&near(mount[1][0],1),"outward PCB normal defines X and a right-handed Y");
  gravity[0]=0;gravity[1]=0;gravity[2]=1;
  expect(!buildUprightMount(gravity,mount),"horizontal PCB contradicts the required mounting reference");
  // 两块斜面上的 IMU 对同一真实横向向量必须给出相同机体分量。
  // 已知传感器重力为 +Y 时，朝外法线为传感器 +Z，水平切向为传感器 +X。
  gravity[0]=0;gravity[1]=1;gravity[2]=0;
  const float half=std::sqrt(.5f);
  const float golden[2][3][3]={{{half,0,-half},{-half,0,-half},{0,1,0}},
                              {{half,0,half},{half,0,-half},{0,1,0}}};
  expect(near(controlMountYawDeg(SOURCE_IMU1),-135)&&near(controlMountYawDeg(SOURCE_IMU2),-45),"IMU1 is clockwise 90 degrees from the IMU2 -45 degree face");
  for(int source=SOURCE_IMU1;source<=SOURCE_IMU2;++source){
    expect(buildUprightMount(gravity,mount,controlMountYawDeg(source)),"each source uses its own yaw mounting angle");
    for(int row=0;row<3;++row)for(int col=0;col<3;++col)
      expect(near(mount[row][col],golden[source-1][row][col]),"mounting rotation matches the independent octagon-face matrix");
    for(int row=0;row<3;++row)for(int other=0;other<3;++other){
      float dot=0;for(int col=0;col<3;++col)dot+=mount[row][col]*mount[other][col];
      expect(near(dot,row==other?1:0),"mounting remains orthonormal after yaw correction");
    }
    const float sensorPlusX[2][3]={{half,0,-half},{half,0,half}};
    const float sensorPlusY[2][3]={{-half,0,-half},{half,0,-half}};
    for(int row=0;row<3;++row){
      float bx=0,by=0,vertical=0;
      for(int col=0;col<3;++col){bx+=mount[row][col]*sensorPlusX[source-1][col];by+=mount[row][col]*sensorPlusY[source-1][col];vertical+=mount[row][col]*gravity[col];}
      expect(near(bx,row==0?1:0)&&near(by,row==1?1:0),"same physical X/Y vector maps identically for both sensor faces");
      expect(near(vertical,row==2?1:0),"yaw correction preserves upright gravity and total tilt");
    }
  }
  expect(!buildUprightMount(gravity,mount,NAN)&&!buildUprightMount(gravity,mount,181),"invalid mounting yaw is rejected");
  AttitudeMatrix r={{{0,-1,0},{1,0,0},{0,0,1}}};float x,y;
  expect(gimbalLean(r,x,y)&&near(x,0)&&near(y,0),"upright lean remains zero at arbitrary yaw");
  r={{{.8660254f,0,.5f},{0,1,0},{-.5f,0,.8660254f}}};
  expect(gimbalLean(r,x,y)&&near(x,30,.001f)&&near(y,0),"tilt uses body vertical vector");
  for(float angle : {-180.f,-120.f,-90.f,-60.1f,60.1f,90.f,120.f,180.f}){
    const float rad=angle*.0174532925199433f;
    r={{{std::cos(rad),0,std::sin(rad)},{0,1,0},{-std::sin(rad),0,std::cos(rad)}}};
    expect(gimbalLean(r,x,y)&&std::isfinite(x)&&std::isfinite(y),"finite attitude beyond 60 degrees no longer trips the tilt stop");
  }
  r.m[2][0]=NAN;expect(!gimbalLean(r,x,y),"nonfinite attitude is still rejected");
  r.m[2][0]=0;r.m[2][2]=INFINITY;expect(!gimbalLean(r,x,y),"infinite attitude is still rejected");
  for(int degrees=-180;degrees<=180;++degrees){
    float rad=degrees*.0174532925199433f,az,el;
    GimbalPid::vectorToAngles(10*std::cos(rad),10*std::sin(rad),359,180,az,el);
    float phi=(az-180)*.0174532925199433f;
    expect(az>=0&&az<=360&&el>=120&&el<=150,"all quadrants respect finite servo travel");
    expect(near((el-135)*std::cos(phi),10*std::cos(rad),.002f)&&near((el-135)*std::sin(phi),10*std::sin(rad),.002f),"equivalent azimuth and signed elevation preserve the requested vector");
  }
  float az,el;GimbalPid::vectorToAngles(0,0,287,180,az,el);
  expect(near(az,287)&&near(el,135),"zero correction centers elevation without rotating undefined azimuth");
  expect(GimbalPid::angleToUs(180,0)==1500&&GimbalPid::angleToUs(135,1)==1500,"different servo ranges share a 1500us center");
  expect(GimbalPid::angleToUs(360,0)==2500&&GimbalPid::angleToUs(270,1)==2500,"full scale maps correctly for each servo");
  GimbalPid pid(config());int actual[2]={1500,1500},target[2]={1500,1500};
  expect(pid.update(1000000,10,0,actual,target)==GimbalPid::WAITING,"first sample primes without a D kick");
  expect(pid.update(1010000,10,0,actual,target)==GimbalPid::UPDATED,"new primary sample drives control");
  expect(target[0]==1500&&target[1]==1490,"opposite tilt uses signed elevation rather than an unnecessary 180 degree turn");
  const auto terms=pid.x().terms();
  expect(pid.update(1010000,-20,20,actual,target)==GimbalPid::WAITING&&near(terms.output,pid.x().terms().output),"duplicate samples do not integrate or differentiate");
  expect(pid.update(1100000,0,0,actual,target)==GimbalPid::INVALID,"unknown sampling gaps are rejected");
  pid.reset();expect(pid.update(9000000,0,-10,actual,target)==GimbalPid::WAITING,"reset clears old sample time");
  expect(pid.update(9010000,0,-10,actual,target)==GimbalPid::UPDATED&&target[1]==1500,"large azimuth turn temporarily keeps elevation centered");
  auto c=config();c.x.ki=.5f;GimbalPid integral(c);actual[0]=actual[1]=1500;
  for(int i=0;i<1000;++i){integral.update(1000000+10000*i,50,50,actual,target);for(int k=0;k<2;++k)actual[k]=target[k];}
  expect(std::fabs(integral.x().terms().i)<.3f&&std::fabs(integral.y().terms().i)<.3f,"vector saturation and slew inhibit integral windup");
  c=config();c.signX=0;expect(!GimbalPid(c).configurationValid(),"invalid direction is rejected");
  c=config();c.limit=90;expect(!GimbalPid(c).configurationValid(),"unreviewed large deflection is rejected");
  std::printf("PASS %d gimbal geometry and PID checks; no physical hardware used.\n",checks);
}
