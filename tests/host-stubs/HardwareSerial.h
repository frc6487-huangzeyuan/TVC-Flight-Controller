#pragma once
#include <vector>
#include <deque>
#include <cstdint>
#include <cstddef>
struct HardwareSerial {
  std::deque<uint8_t> rx;std::vector<uint8_t> tx;
  uint32_t baud=0;int rxPin=-1,txPin=-1;
  HardwareSerial(int){}
  void setRxBufferSize(size_t){}
  void begin(uint32_t b,int,int r,int t){baud=b;rxPin=r;txPin=t;}
  int available(){return int(rx.size());}
  int read(){auto b=rx.front();rx.pop_front();return b;}
  int availableForWrite(){return 128;}
  size_t write(const uint8_t *p,size_t n){tx.insert(tx.end(),p,p+n);return n;}
};
