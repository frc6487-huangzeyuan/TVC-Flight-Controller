#include "../PrimaryDebug.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <initializer_list>
int main(){
  PrimaryUart::Response r;r.id=r.sequence=r.ageUs=0xffffffffu;r.status=PrimaryUart::READ_ERROR;
  for(auto &v:r.raw)v=-32768;
  char line[160];
  for(const char *side:{"C3_TX","C3_LOCAL","S3_RX"})for(const char *reason:{"TX","LOCAL","REJECT"}){
    int n=formatPrimaryDebug(line,sizeof(line),side,reason,r);
    assert(n>0&&n<128&&size_t(n)==std::strlen(line)&&line[n-1]=='\n');
    assert(std::strstr(line,"a=-32768,-32768,-32768 g=-32768,-32768,-32768 t=-32768"));
  }
  std::puts("PASS raw serial log values and worst-case single-line UART FIFO size.");
}
