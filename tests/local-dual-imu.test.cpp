#include <ArduinoJson.h>
#include "../bench_config.h"
#include "../Imu2MonitorState.h"
#include <cassert>
#include <cstring>
#include <cstdio>

#define IRAM_ATTR
enum { INPUT_PULLDOWN=1, RISING=2, portMAX_DELAY=0 };
int irqLock=0,stateLock=0;
uint8_t controlSource=DEFAULT_CONTROL_SOURCE;
static uint64_t nowUs=1000000;
static bool present1=true,present2=true,read1=true;
static int reads1=0,reads2=0,attached1=0,attached2=0;
uint64_t esp_timer_get_time(){return nowUs;}
void portENTER_CRITICAL(int*){}
void portEXIT_CRITICAL(int*){}
void portENTER_CRITICAL_ISR(int*){}
void portEXIT_CRITICAL_ISR(int*){}
void xSemaphoreTake(int,int){}
void xSemaphoreGive(int){}
void pinMode(int pin,int mode){assert((pin==8||pin==2)&&mode==INPUT_PULLDOWN);}
int digitalPinToInterrupt(int pin){return pin;}
void attachInterrupt(int pin,void(*)(),int edge){assert(edge==RISING);if(pin==8)++attached1;else if(pin==2)++attached2;else assert(false);}
bool initializeImu(uint8_t address,uint8_t divider){assert(divider==4);assert(address==0x68||address==0x69);return address==0x68?present1:present2;}
bool readRegisters(uint8_t address,uint8_t reg,uint8_t *bytes,size_t length){
  assert(reg==0x3b&&length==14);std::memset(bytes,0,length);
  if(address==0x68){++reads1;bytes[0]=0x20;return read1;}
  assert(address==0x69);++reads2;bytes[2]=0x20;return true;
}
#include "../Imu1Monitor.ino"
#include "../Imu2Monitor.ino"

int main(){
  assert(IMU1_LOCAL_I2C&&IMU1_INT_PIN==8&&IMU2_INT_PIN==2);
  assert(SDA_PIN==9&&SCL_PIN==10&&IMU_ADDRESS==0x68&&IMU2_ADDRESS==0x69);
  // IMU1 absence must not prevent healthy IMU2 acquisition.
  present1=false;beginImu1Monitor();beginImu2Monitor();
  assert(!imu1Monitor.present&&imu2Monitor.present&&attached1==0&&attached2==1);
  sampleImu1Monitor();sampleImu2Monitor();assert(reads1==0&&reads2==0);
  imu2Irq();sampleImu2Monitor();assert(reads2==1&&reads1==0);
  // Hot-plug IMU1 requires a fresh GPIO8 event before it can produce data.
  present1=true;nowUs+=IMU2_REINIT_INTERVAL_US;sampleImu1Monitor();
  assert(imu1Monitor.present&&attached1==1&&reads1==0);
  for(int i=0;i<200;++i){
    nowUs+=5000;imu1Irq();imu2Irq();sampleImu1Monitor();sampleImu2Monitor();
    const int a=reads1,b=reads2;sampleImu1Monitor();sampleImu2Monitor();
    assert(reads1==a&&reads2==b); // No fabricated or duplicate samples.
  }
  assert(imu1Monitor.samples==200&&imu2Monitor.samples==201);
  assert(imu1Monitor.accelG[0]==1&&imu1Monitor.accelG[1]==0);
  assert(imu2Monitor.accelG[0]==0&&imu2Monitor.accelG[1]==1);
  const uint32_t before=imu2Monitor.samples;
  read1=false;nowUs+=5000;imu1Irq();imu2Irq();sampleImu1Monitor();sampleImu2Monitor();
  assert(!imu1Monitor.valid&&imu1Monitor.errors==1&&imu2Monitor.valid&&imu2Monitor.samples==before+1);
  const int failed=reads1;nowUs+=1000;imu1Irq();sampleImu1Monitor();assert(reads1==failed);
  read1=true;nowUs+=IMU2_RETRY_DELAY_US;imu1Irq();sampleImu1Monitor();assert(imu1Monitor.valid);
  JsonDocument doc;appendImu2Telemetry(doc.to<JsonObject>(),nowUs);
  assert(doc["int_pin"].as<int>()==2&&doc["irq_count"].as<uint32_t>()==imu2IrqCount);
  std::puts("PASS dual local I2C addresses, GPIO8/GPIO2 routing, 200 Hz acquisition, independent faults, retry and telemetry");
}
