#pragma once
#include <stdint.h>
extern uint32_t mockMicrosValue;
inline uint32_t micros() { return mockMicrosValue; }
