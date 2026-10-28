#include <cstdio>
#include <cstdlib>
#include <cstdint>
uint64_t hostNow=100000;
#include "../../C3_IMU_Bridge/C3_IMU_Bridge.ino"
static int checks=0;
void expect(bool v,const char *s){++checks;if(!v){std::fprintf(stderr,"FAIL %s\n",s);std::exit(1);}}
int main(){
  setup();expect(Serial.baud==115200&&imuLink.baud==460800&&imuLink.rxPin==7&&imuLink.txPin==6,"dedicated link does not reuse debug serial");
  pollImu();expect(present&&current.sequence==0&&current.status==PrimaryUart::WAIT_SAMPLE,"detected IMU still has no fabricated sample");
  expect(Wire.registers[0x19]==4&&Wire.registers[0x1b]==8&&Wire.registers[0x1c]==8,"200 Hz and original measurement scales configured");
  Wire.raw[0]=0xe0;Wire.ready=true;hostNow+=1000;pollImu();
  expect(current.sequence==1&&current.raw[0]==-8192,"polling DATA_RDY captures one signed raw sample without INT pin");
  const int reads=Wire.rawReads;hostNow+=1000;pollImu();
  expect(current.sequence==1&&Wire.rawReads==reads,"no new DATA_RDY means no repeated old sample");
  PrimaryUart::Receiver receiver;receiver.begin(99);uint8_t request[PrimaryUart::REQUEST_BYTES];
  receiver.makeRequest(1000000,request);for(uint8_t b:request)imuLink.rx.push_back(b);pollLink();
  expect(imuLink.tx.size()==PrimaryUart::RESPONSE_BYTES,"one request has one bounded reply");
  PrimaryUart::Parser rx;const uint8_t *packet=nullptr;PrimaryUart::Response response;
  for(uint8_t b:imuLink.tx)if(rx.push(b,2,packet))response=PrimaryUart::response(packet);
  PrimaryUart::Sample sample;
  expect(receiver.accept(response,1002000,sample)==PrimaryUart::ACCEPTED&&sample.frame.raw[0]==-8192,"real C3 response accepted by real S3 receiver");
  expect(response.ageUs>=5000,"polling uncertainty included in freshness age");
  loop();
  expect(Serial.output.find("#IMU1 C3_TX q=1 s=1")!=std::string::npos&&imuLink.tx.size()==PrimaryUart::RESPONSE_BYTES,"TX debug reaches computer Serial only, not the board link");
  Serial.freeBytes=0;debugPending=true;const auto printed=Serial.output;loop();
  expect(debugPending&&Serial.output==printed,"full debug serial buffer does not block or consume pending log");
  Serial.freeBytes=128;
  Wire.fail=true;hostNow+=1000;pollImu();
  expect(current.readErrors==1&&current.status==PrimaryUart::READ_ERROR&&current.sequence==1,"failed I2C read never fabricates a newer sample");
  Wire.fail=false;
  for(int n=0;n<600;++n){hostNow+=5000;Wire.ready=true;pollImu();}
  expect(current.sequence==601,"continued 200 Hz polling supplies calibration sample volume");
  std::printf("PASS %d actual C3 sampler checks using mocked I2C/UART.\n",checks);
}
