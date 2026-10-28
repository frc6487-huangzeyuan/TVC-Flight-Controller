#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
struct HostWire {
  bool fail=false,ready=false;uint8_t reg=0;int pos=0,len=0;
  uint8_t registers[128]={},raw[14]={};int rawReads=0,statusReads=0;
  void begin(int sda,int scl,int hz){if(sda!=4||scl!=5||hz!=400000)std::abort();}
  void setTimeOut(int){}
  void beginTransmission(uint8_t address){if(address!=0x68)std::abort();pos=0;}
  void write(uint8_t byte){if(!pos)reg=byte;else registers[reg]=byte;++pos;}
  int endTransmission(bool=true){return fail?4:0;}
  int requestFrom(uint8_t,size_t n){pos=0;len=int(n);if(reg==0x3a)++statusReads;if(reg==0x3b)++rawReads;return int(n);}
  int available(){return len-pos;}
  int read(){
    ++pos;
    if(reg==0x75)return 0x68;
    if(reg==0x3a){const bool b=ready;ready=false;return b?1:0;}
    if(reg==0x3b)return raw[pos-1];
    return 0;
  }
};
inline HostWire Wire;
