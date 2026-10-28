#pragma once
#include <cstdint>
extern uint64_t hostNow;
inline int64_t esp_timer_get_time(){return int64_t(hostNow);}
