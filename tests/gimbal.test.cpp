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
  // 同一 PCB、同面同向的 IMU 必须把相同传感器向量映射到相同机体分量。
  // 已知传感器重力为 +Y 时，朝外法线为传感器 +Z，水平切向为传感器 +X。
  gravity[0]=0;gravity[1]=1;gravity[2]=0;
  const float golden[3][3]={{0,0,1},{1,0,0},{0,1,0}};
  const float sensorPlusX[3]={0,0,1};
  const float sensorPlusY[3]={1,0,0};
  expect(near(controlMountYawDeg(SOURCE_IMU1),0)&&near(controlMountYawDeg(SOURCE_IMU2),0),"both IMUs use zero mounting yaw");
  for(int source=SOURCE_IMU1;source<=SOURCE_IMU2;++source){
    expect(buildUprightMount(gravity,mount,controlMountYawDeg(source)),"each source uses its own yaw mounting angle");
    for(int row=0;row<3;++row)for(int col=0;col<3;++col)
      expect(near(mount[row][col],golden[row][col]),"mounting matrix preserves the unrotated horizontal axes");
    for(int row=0;row<3;++row)for(int other=0;other<3;++other){
      float dot=0;for(int col=0;col<3;++col)dot+=mount[row][col]*mount[other][col];
      expect(near(dot,row==other?1:0),"mounting remains orthonormal after yaw correction");
    }
    for(int row=0;row<3;++row){
      float bx=0,by=0,vertical=0;
      for(int col=0;col<3;++col){bx+=mount[row][col]*sensorPlusX[col];by+=mount[row][col]*sensorPlusY[col];vertical+=mount[row][col]*gravity[col];}
      expect(near(bx,row==0?1:0)&&near(by,row==1?1:0),"identical raw X/Y test vectors map identically for both IMUs");
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
  expect(pid.update(9010000,0,-10,actual,target)==GimbalPid::UPDATED&&target[0]>1500&&target[1]==1500,
         "correction perpendicular to the sent azimuth starts at zero elevation");
  actual[0]=GimbalPid::angleToUs(220,0);
  actual[1]=GimbalPid::angleToUs(135+10*std::sin(40*.0174532925199433f),1);
  expect(pid.update(9020000,0,-10,actual,target)==GimbalPid::UPDATED&&target[0]>actual[0]&&target[1]==actual[1]&&pid.limited(),
         "turning azimuth retains the reachable elevation component and inhibits integration");
  pid.reset();actual[1]=1500;
  pid.update(9000000,0,10,actual,target);
  expect(pid.update(9010000,0,10,actual,target)==GimbalPid::UPDATED&&target[0]>actual[0]&&target[1]<1500,
         "mirrored vector uses negative elevation while azimuth keeps rotating toward an equivalent solution");
  auto c=config();c.x.ki=.5f;GimbalPid integral(c);actual[0]=actual[1]=1500;
  for(int i=0;i<1000;++i){integral.update(1000000+10000*i,50,50,actual,target);for(int k=0;k<2;++k)actual[k]=target[k];}
  expect(std::fabs(integral.x().terms().i)<.3f&&std::fabs(integral.y().terms().i)<.3f,"vector saturation and slew inhibit integral windup");
  c=config();c.signX=0;expect(!GimbalPid(c).configurationValid(),"invalid direction is rejected");
  c=config();c.limit=90;expect(!GimbalPid(c).configurationValid(),"unreviewed large deflection is rejected");
  // Check the PCB defaults through both startup and the running control path.
  c=config();c.azimuthZero=GIMBAL_AZIMUTH_ZERO_DEG;c.signX=GIMBAL_VECTOR_X_SIGN;c.signY=GIMBAL_VECTOR_Y_SIGN;c.stepUs=SERVO_SLEW_US_PER_TICK;
  float vx,vy;GimbalPid::initialVector(c,2,3,vx,vy);
  expect(near(vx,-2)&&near(vy,3),"PCB Y correction reverses while X keeps its original direction");
  GimbalPid pcbPid(c);actual[0]=actual[1]=1500;
  for(int i=0;i<500;++i){expect(pcbPid.update(1000000+10000*i,2,3,actual,target)!=GimbalPid::INVALID,"PCB direction update remains valid");for(int k=0;k<2;++k)actual[k]=target[k];}
  az=GimbalPid::usToAngle(actual[0],0);el=GimbalPid::usToAngle(actual[1],1);
  float direction=(az-c.azimuthZero)*.0174532925199433f;
  expect(near((el-135)*std::cos(direction),-2,.2f)&&near((el-135)*std::sin(direction),3,.2f),"running PWM maps to the same corrected X/Y vector as startup");
  {
    auto cfg=config();
    cfg.limit=50;cfg.stepUs=25;cfg.x={1,.01f,0};
    GimbalPid limited(cfg);
    int sent[2]={1500,ELEVATION_MAX_US},goal[2]={sent[0],sent[1]};
    for(uint64_t t=10000;t<=3000000;t+=10000) {
      expect(limited.update(t,-26,0,sent,goal)!=GimbalPid::INVALID,"valid position-limited PID sample");
      sent[0]=goal[0];sent[1]=goal[1];
    }
    expect(limited.limited()&&sent[1]==ELEVATION_MAX_US,"physical elevation saturation reported");
    expect(limited.x().terms().i<.01f,"integration stops into physical saturation");

    cfg=config();cfg.x={.6f,0,0};cfg.trim[0]=2;
    GimbalPid trim(cfg);
    int atCenter[2]={1500,1500};
    trim.update(10000,0,0,atCenter,goal);
    expect(trim.update(20000,0,0,atCenter,goal)==GimbalPid::UPDATED&&goal[1]>1500,
           "measured trim works at zero attitude error");
  }
  std::printf("PASS %d gimbal geometry and PID checks; no physical hardware used.\n",checks);
}
