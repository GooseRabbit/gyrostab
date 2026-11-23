#pragma once

#include <stdint.h>
#include <string>

extern uint32_t mockMicrosValue;
inline uint32_t micros() { return mockMicrosValue; }

class MockSerialClass
{
public:
    std::string output;

    void print(uint32_t value);
    void print(int value);
    void print(char value);
    void print(float value, int digits);
    void println(float value, int digits);
    void clear();
};

extern MockSerialClass Serial;
