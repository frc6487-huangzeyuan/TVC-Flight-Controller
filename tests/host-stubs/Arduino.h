#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <string>
#define SERIAL_8N1 0
extern uint64_t hostNow;
inline uint32_t millis(){return uint32_t(hostNow/1000);}
inline void delay(uint32_t ms){hostNow+=uint64_t(ms)*1000;}
struct DebugSerial {
  std::string output;int freeBytes=128;
  uint32_t baud=0;
  void begin(uint32_t value){baud=value;}
  int availableForWrite(){return freeBytes;}
  size_t write(const uint8_t *p,size_t n){output.append(reinterpret_cast<const char*>(p),n);return n;}
  explicit operator bool()const{return true;}
  template<class... T>void printf(const char*,T...){}
};
inline DebugSerial Serial;
