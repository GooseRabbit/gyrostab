#pragma once
#include <stdint.h>
class SPIClass
{
public:
    bool begin(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t ss = -1);
};
extern SPIClass SPI;
extern bool mockSpiBeginResult;
