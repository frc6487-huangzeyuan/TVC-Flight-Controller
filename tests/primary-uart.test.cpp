#include "../PrimaryUartProtocol.h"
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace PrimaryUart;
static int checks=0;
void expect(bool v,const char *s){++checks;if(!v){std::fprintf(stderr,"FAIL %s\n",s);std::exit(1);}}
Response frame(const uint8_t *requestBytes,uint32_t sequence=1,uint32_t tick=5000){
  const auto q=request(requestBytes);Response r;r.session=q.session;r.id=q.id;r.boot=42;
  r.sequence=sequence;r.tickUs=tick;r.ageUs=5000;r.status=SAMPLE_OK;r.raw[0]=-8192;r.raw[4]=-1234;return r;
}
int main(){
  expect(crc(reinterpret_cast<const uint8_t*>("123456789"),9)==0xcbf43926,"CRC known vector");
  uint8_t q[REQUEST_BYTES],bytes[RESPONSE_BYTES];Receiver receiver;receiver.begin(123);Sample s;
  expect(receiver.makeRequest(100000,q),"first request");const auto r=frame(q);encode(r,bytes);
  Parser parser;const uint8_t *packet=nullptr;int parsed=0;
  for(uint8_t c:bytes)if(parser.push(c,2,packet)){++parsed;auto decoded=response(packet);expect(decoded.raw[0]==-8192&&decoded.raw[4]==-1234&&decoded.boot==42,"signed raw round trip");}
  expect(parsed==1,"one exact response");
  expect(receiver.accept(r,102000,s)==ACCEPTED&&s.localUs==95000&&s.dtUs==5000,"fresh time is conservative using full round trip");
  expect(!receiver.makeRequest(104999,q)&&receiver.makeRequest(105000,q),"200 Hz request ceiling");
  auto duplicate=frame(q);expect(receiver.accept(duplicate,106000,s)==DUPLICATE,"duplicate sample never updates freshness");
  expect(receiver.makeRequest(110000,q),"next request");auto fresh=frame(q,3,15000);
  expect(receiver.accept(fresh,111000,s)==ACCEPTED&&s.missed==1&&s.dtUs==10000,"sequence loss counted with source-clock dt");
  expect(receiver.makeRequest(115000,q),"request before old packet");auto wrong=frame(q,4,20000);wrong.session++;
  expect(receiver.accept(wrong,116000,s)==REJECTED,"old session rejected");wrong=frame(q,4,20000);wrong.id--;
  expect(receiver.accept(wrong,116000,s)==REJECTED,"old request rejected");
  expect(receiver.accept(frame(q,4,20000),116000,s)==ACCEPTED,"unrelated old packets cannot consume current request");
  for(uint32_t age:{49000u,50000u,0xffffffffu}){
    expect(receiver.makeRequest(120000+uint64_t(age==49000?0:age==50000?1:2)*5000,q),"request for age test");
    auto old=frame(q,5,25000);old.ageUs=age;
    const uint64_t now=120000+uint64_t(age==49000?0:age==50000?1:2)*5000+1000;
    expect(receiver.accept(old,now,s)==REJECTED,"age including UART delay expires at 50 ms without overflow");
  }
  expect(receiver.makeRequest(135000,q),"late reply request");
  expect(receiver.accept(frame(q,5,25000),145000,s)==REJECTED,"10 ms round trip limit");
  expect(receiver.makeRequest(145000,q)&&receiver.timeouts==1,"timeout starts new request");
  expect(receiver.accept(frame(q,5,80000),146000,s)==ACCEPTED&&s.gap,"source sample gap detected");
  expect(receiver.makeRequest(150000,q),"reboot request");auto reboot=frame(q,1,100);reboot.boot=43;
  expect(receiver.accept(reboot,151000,s)==ACCEPTED&&s.restarted&&s.dtUs==5000,"C3 restart detected and sample history reset");
  expect(receiver.makeRequest(155000,q),"status request");auto missing=frame(q);missing.boot=44;missing.status=NOT_FOUND;
  expect(receiver.accept(missing,156000,s)==STATUS_ONLY&&s.restarted,"restart also detected on a status-only response");
  Receiver wrap;wrap.begin(55);wrap.makeRequest(100000,q);auto a=frame(q,0xfffffffeu,0xfffffff0u);
  expect(wrap.accept(a,101000,s)==ACCEPTED,"pre-wrap sample accepted");wrap.makeRequest(105000,q);a=frame(q,1,0x1378u);
  expect(wrap.accept(a,106000,s)==ACCEPTED&&s.dtUs==5000&&!s.gap,"source microsecond clock wraps safely");
  uint32_t rng=7;
  for(int trial=0;trial<200;++trial){
    Parser p;int count=0;
    for(int i=0;i<11;++i){rng=rng*1664525u+1013904223u;p.push(uint8_t(rng>>24),2,packet);}
    encode(r,bytes);bytes[10]^=0x40;
    for(uint8_t c:bytes)if(p.push(c,2,packet))++count;
    encode(r,bytes);
    for(uint8_t c:bytes)if(p.push(c,2,packet))++count;
    expect(count==1&&p.badFrames>=1,"noise/corruption resynchronizes to next complete valid packet");
    Parser dropped;count=0;
    for(size_t i=0;i<RESPONSE_BYTES;++i)if(i!=size_t(trial%RESPONSE_BYTES))dropped.push(bytes[i],2,packet);
    for(uint8_t c:bytes)if(dropped.push(c,2,packet))++count;
    expect(count==1,"one missing byte resynchronizes");
  }
  std::printf("PASS %d primary UART protocol/freshness checks.\n",checks);
}
