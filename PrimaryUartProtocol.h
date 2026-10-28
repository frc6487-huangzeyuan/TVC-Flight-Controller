#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>

// 明确使用小端字节序传输，不直接发送依赖编译器内存布局的结构体。
namespace PrimaryUart {
constexpr uint32_t BAUD=460800, PERIOD_US=5000, REPLY_TIMEOUT_US=10000, MAX_AGE_US=50000;
constexpr size_t REQUEST_BYTES=16, RESPONSE_BYTES=51;
enum Status : uint8_t { NOT_FOUND=0, WAIT_SAMPLE=1, SAMPLE_OK=2, READ_ERROR=3 };
struct Request {uint32_t session=0,id=0;};
struct Response {
  uint32_t session=0,id=0,boot=0,sequence=0,tickUs=0,ageUs=0,readErrors=0;
  Status status=NOT_FOUND;
  int16_t raw[7]={}; // 依次为 X/Y/Z 加速度、温度，以及 X/Y/Z 角速度原始值。
};
inline uint32_t crc(const uint8_t *p,size_t n){
  uint32_t c=0xffffffff;
  for(size_t i=0;i<n;++i){c^=p[i];for(int b=0;b<8;++b)c=(c>>1)^((c&1)?0xedb88320:0);}
  return c^0xffffffff;
}
inline void put32(uint8_t *p,uint32_t v){for(int i=0;i<4;++i)p[i]=uint8_t(v>>(8*i));}
inline uint32_t get32(const uint8_t *p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
inline void header(uint8_t *p,uint8_t type){p[0]=0xa5;p[1]=0x5a;p[2]=1;p[3]=type;}
inline void encode(const Request &r,uint8_t *p){header(p,1);put32(p+4,r.session);put32(p+8,r.id);put32(p+12,crc(p,12));}
inline void encode(const Response &r,uint8_t *p){
  header(p,2);put32(p+4,r.session);put32(p+8,r.id);put32(p+12,r.boot);put32(p+16,r.sequence);
  put32(p+20,r.tickUs);put32(p+24,r.ageUs);put32(p+28,r.readErrors);p[32]=uint8_t(r.status);
  for(int i=0;i<7;++i){const uint16_t v=uint16_t(r.raw[i]);p[33+2*i]=uint8_t(v);p[34+2*i]=uint8_t(v>>8);}
  put32(p+47,crc(p,47));
}
inline Request request(const uint8_t *p){Request r;r.session=get32(p+4);r.id=get32(p+8);return r;}
inline Response response(const uint8_t *p){
  Response r;r.session=get32(p+4);r.id=get32(p+8);r.boot=get32(p+12);r.sequence=get32(p+16);
  r.tickUs=get32(p+20);r.ageUs=get32(p+24);r.readErrors=get32(p+28);r.status=Status(p[32]);
  for(int i=0;i<7;++i)r.raw[i]=int16_t(uint16_t(p[33+2*i])|(uint16_t(p[34+2*i])<<8));
  return r;
}
class Parser {
  uint8_t buffer[RESPONSE_BYTES]={};size_t used=0;
  void drop(){if(used){--used;std::memmove(buffer,buffer+1,used);}}
public:
  uint32_t badFrames=0;
  // 每次返回的数据包应处理完后，再次调用 push 接收后续数据。
  bool push(uint8_t byte,uint8_t wantedType,const uint8_t *&packet){
    if(used==sizeof(buffer))drop();buffer[used++]=byte;
    for(;;){
      if(used<2)return false;
      if(buffer[0]!=0xa5||buffer[1]!=0x5a){drop();continue;}
      if(used<4)return false;
      if(buffer[2]!=1||buffer[3]!=wantedType){++badFrames;drop();continue;}
      const size_t n=wantedType==1?REQUEST_BYTES:RESPONSE_BYTES;
      if(used<n)return false;
      if(get32(buffer+n-4)!=crc(buffer,n-4)){++badFrames;drop();continue;}
      packet=buffer;used=0;return true;
    }
  }
};
struct Sample {
  Response frame;
  uint64_t localUs=0;
  uint32_t dtUs=5000,missed=0;
  bool restarted=false,gap=false;
};
enum Result { REJECTED, STATUS_ONLY, DUPLICATE, ACCEPTED };
class Receiver {
  uint32_t session=0,id=0,boot=0,sequence=0,tick=0;
  uint64_t sentAt=0;
  bool pending=false,haveBoot=false,haveSample=false,haveSent=false;
public:
  uint32_t rejected=0,timeouts=0,replies=0;
  void begin(uint32_t value){*this=Receiver();session=value?value:1;}
  bool makeRequest(uint64_t now,uint8_t *bytes){
    if(pending){if(now-sentAt<REPLY_TIMEOUT_US)return false;pending=false;++timeouts;}
    if(haveSent&&now-sentAt<PERIOD_US)return false;
    if(!++id)++id;Request r;r.session=session;r.id=id;encode(r,bytes);
    sentAt=now;haveSent=pending=true;return true;
  }
  Result accept(const Response &r,uint64_t now,Sample &s){
    s=Sample();
    if(!pending||r.session!=session||r.id!=id||now<sentAt||now-sentAt>=REPLY_TIMEOUT_US||!r.boot||r.status>READ_ERROR){++rejected;return REJECTED;}
    pending=false;++replies;s.frame=r;
    const uint64_t age=uint64_t(r.ageUs)+(now-sentAt);
    if(r.status==SAMPLE_OK&&(!r.sequence||age>=MAX_AGE_US||age>now)){++rejected;return REJECTED;}
    s.restarted=haveBoot&&boot!=r.boot;
    if(!haveBoot||s.restarted){boot=r.boot;haveBoot=true;haveSample=false;}
    if(r.status!=SAMPLE_OK)return STATUS_ONLY;
    if(haveSample){
      const uint32_t step=r.sequence-sequence;
      if(step==0)return DUPLICATE;
      if(step>0x7fffffffu){++rejected;return REJECTED;}
      s.dtUs=r.tickUs-tick;
      if(s.dtUs==0){++rejected;return REJECTED;}
      s.missed=step-1;s.gap=s.dtUs>MAX_AGE_US;
    }
    s.localUs=now-age;sequence=r.sequence;tick=r.tickUs;haveSample=true;return ACCEPTED;
  }
};
} // PrimaryUart 命名空间结束。
