#include <ArduinoJson.h>
#include "../bench_config.h"
#include "../Imu2MonitorState.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

static int checks=0;
static void expect(bool ok,const char *name) {
  ++checks;
  if(!ok){std::fprintf(stderr,"FAIL %s\n",name);std::exit(1);}
}

// 模拟硬件与实时任务接口边界，并包含实际监测用的 ino 源文件。
#define IRAM_ATTR
enum { INPUT_PULLDOWN=1, RISING=2, portMAX_DELAY=0 };
int irqLock=0,stateLock=0;
uint8_t controlSource=DEFAULT_CONTROL_SOURCE;
volatile uint32_t irqCount=0;
uint32_t consumed=0;
struct {bool present=true;} imu;
static uint64_t fakeNow=1000000;
static int initCalls=0,readCalls=0,attachCalls=0,lastIntPin=-1;
static bool initGood=true,readGood=true,changeDuringRead=false;
extern volatile uint32_t imu2IrqCount;
uint64_t esp_timer_get_time(){return fakeNow;}
void portENTER_CRITICAL(int*){}
void portEXIT_CRITICAL(int*){}
void portENTER_CRITICAL_ISR(int*){}
void portEXIT_CRITICAL_ISR(int*){}
void xSemaphoreTake(int,int){}
void xSemaphoreGive(int){}
void pinMode(int pin,int mode){expect(pin==8&&mode==INPUT_PULLDOWN,"IMU2 pin input only");}
int digitalPinToInterrupt(int pin){return pin;}
void attachInterrupt(int pin,void(*)(),int edge){
  ++attachCalls;lastIntPin=pin;expect(edge==RISING,"rising data-ready interrupt");
}
bool initializeImu(uint8_t address,uint8_t divider){
  ++initCalls;expect(address==0x69,"only secondary address initialized");
  expect(divider==4&&1000/(1+divider)==200,"hardware divider requests 200 Hz");
  return initGood;
}
bool readRegisters(uint8_t address,uint8_t reg,uint8_t *bytes,size_t length){
  ++readCalls;
  expect(address==0x69&&reg==0x3b&&length==14,"14-byte secondary burst read");
  std::memset(bytes,0,length);bytes[0]=0xe0; // 传感器 -X 对应机体 +Z，静止时为 1 g。
  if(changeDuringRead)++imu2IrqCount;
  return readGood;
}

#include "../Imu2Monitor.ino"

int main(){
  expect(PRIMARY_UART_RX==16&&PRIMARY_UART_TX==17&&IMU_TIMEOUT_US==2000000,"primary UART pins and history protection preserved");
  expect(IMU2_SAMPLE_HZ==200&&IMU2_TIMEOUT_US==300000,"secondary configured for control sampling");
  beginImu2Monitor();
  expect(initCalls==1&&attachCalls==1&&lastIntPin==8&&imu2Monitor.present,
         "configured secondary attaches to GPIO8");
  imu.present=false;irqCount=1;imu2IrqCount=1;imu2IrqTime=fakeNow;
  sampleImu2Monitor();expect(readCalls==1,"missing primary cannot starve secondary");
  imu.present=true;consumed=irqCount;imu2Monitor=Imu2MonitorState();imu2Monitor.present=true;
  imu2IrqCount=imu2Consumed=0;imu2IrqTime=0;readCalls=0;
  JsonDocument doc;
  const auto telemetry=[&](){doc.clear();appendImu2Telemetry(doc.to<JsonObject>(),fakeNow);};
  telemetry();
  expect(!doc["valid"].as<bool>()&&std::strcmp(doc["status"],"WAIT_INT")==0,
         "no fabricated sample before first interrupt");
  expect(doc["sample_timeout_us"].as<uint32_t>()==300000&&doc["target_hz"].as<int>()==200,
         "telemetry advertises local sampling rate and freshness budget");
  for(int n=1;n<=200;++n){
    fakeNow=1000000+uint64_t(n)*5000;
    imu2Irq();sampleImu2Monitor();telemetry();
    expect(imu2Monitor.samples==uint32_t(n)&&doc["valid"].as<bool>(),"one sample per new secondary interrupt");
    expect(doc["tilt_valid"].as<bool>(),"static secondary gravity tilt remains valid");
    const int reads=readCalls;sampleImu2Monitor();
    expect(readCalls==reads,"task loop does not reread a consumed sample");
  }
  expect(std::fabs(doc["sample_hz"].as<float>()-200)<.01f,"one-second window reports 200 Hz");
  for(uint32_t age : {50000u,99999u,100000u,299999u}){
    fakeNow=2000000+age;telemetry();
    expect(doc["valid"].as<bool>(),"50 ms old secondary sample stays valid inside raw monitor budget");
  }
  fakeNow=2300000;telemetry();
  expect(!doc["valid"].as<bool>()&&std::strcmp(doc["status"],"STALE")==0,
         "secondary expires at exactly 300 ms");
  expect(doc["temp_c"].isNull()&&doc["roll_deg"].isNull(),"expired measurements are cleared");

  fakeNow=2400000;irqCount=2;consumed=1;imu2Irq();
  const int before=readCalls;sampleImu2Monitor();
  expect(readCalls==before+1,"remote primary UART cannot block local secondary I2C");
  consumed=irqCount;sampleImu2Monitor();
  expect(readCalls==before+1&&imu2Monitor.valid,"secondary can read after primary is serviced");
  expect(irqCount==2&&consumed==2,"secondary cannot alter primary interrupt accounting");

  fakeNow=2500000;readGood=false;imu2Irq();sampleImu2Monitor();telemetry();
  expect(std::strcmp(doc["status"],"READ_ERROR")==0,"I2C errors are explicit");
  fakeNow=2550000;readGood=true;imu2Irq();
  const int failedReads=readCalls;sampleImu2Monitor();
  expect(readCalls==failedReads,"failed secondary read backs off");
  fakeNow=2600000;sampleImu2Monitor();telemetry();
  expect(doc["valid"].as<bool>(),"secondary can recover on a new successful read");
  fakeNow=2700000;changeDuringRead=true;imu2Irq();sampleImu2Monitor();
  expect(!imu2Monitor.valid,"read spanning a new interrupt is still rejected");
  initGood=false;beginImu2Monitor();telemetry();
  expect(!imu2Monitor.present&&attachCalls==1&&std::strcmp(doc["status"],"NOT_FOUND")==0,
         "failed secondary detection never attaches another interrupt");
  std::printf("PASS %d IMU2 firmware sampling checks (fake hardware).\n",checks);
  return 0;
}
