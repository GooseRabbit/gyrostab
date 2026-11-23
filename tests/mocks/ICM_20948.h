#pragma once
#include <stdint.h>
#include "SPI.h"

enum ICM_20948_Status_e
{
    ICM_20948_Stat_Ok = 0,
    ICM_20948_Stat_Err,
    ICM_20948_Stat_NotImpl,
    ICM_20948_Stat_ParamErr,
    ICM_20948_Stat_WrongID,
    ICM_20948_Stat_InvalSensor,
    ICM_20948_Stat_NoData,
    ICM_20948_Stat_SensorNotSupported,
    ICM_20948_Stat_DMPNotSupported,
    ICM_20948_Stat_DMPVerifyFail,
    ICM_20948_Stat_FIFONoDataAvail,
    ICM_20948_Stat_FIFOIncompleteData,
    ICM_20948_Stat_FIFOMoreDataAvail,
    ICM_20948_Stat_UnrecognisedDMPHeader,
    ICM_20948_Stat_UnrecognisedDMPHeader2,
    ICM_20948_Stat_InvalDMPRegister,
    ICM_20948_Stat_NUM,
    ICM_20948_Stat_Unknown
};

enum ICM_20948_InternalSensorID_bm
{
    ICM_20948_Internal_Acc = 1,
    ICM_20948_Internal_Gyr = 2
};

enum ICM_20948_LP_CONFIG_CYCLE_e
{
    ICM_20948_Sample_Mode_Continuous = 0
};

struct ICM_20948_fss_t { uint8_t a; uint8_t g; };
struct ICM_20948_smplrt_t { uint16_t a; uint8_t g; };
struct ICM_20948_dlpcfg_t { uint8_t a; uint8_t g; };
struct ICM_20948_AGMT_t
{
    struct { int16_t x, y, z; } acc;
    struct { int16_t x, y, z; } gyr;
    struct { int16_t x, y, z; } mag;
    uint8_t magStat1;
    uint8_t magStat2;
};

extern ICM_20948_Status_e mockBeginStatus;
extern ICM_20948_Status_e mockReadStatus;
extern bool mockDataReady;
extern ICM_20948_AGMT_t mockAgmt;

class ICM_20948_SPI
{
public:
    ICM_20948_Status_e status = ICM_20948_Stat_Ok;
    ICM_20948_AGMT_t agmt = {};

    ICM_20948_Status_e begin(uint8_t, SPIClass&)
    {
        status = mockBeginStatus;
        return status;
    }
    ICM_20948_Status_e setSampleMode(uint8_t, uint8_t) { status = ICM_20948_Stat_Ok; return status; }
    ICM_20948_Status_e setFullScale(uint8_t, ICM_20948_fss_t) { status = ICM_20948_Stat_Ok; return status; }
    ICM_20948_Status_e setSampleRate(uint8_t, ICM_20948_smplrt_t) { status = ICM_20948_Stat_Ok; return status; }
    ICM_20948_Status_e setDLPFcfg(uint8_t, ICM_20948_dlpcfg_t) { status = ICM_20948_Stat_Ok; return status; }
    ICM_20948_Status_e enableDLPF(uint8_t, bool) { status = ICM_20948_Stat_Ok; return status; }
    bool dataReady()
    {
        const bool ready = mockDataReady;
        mockDataReady = false;
        status = ready ? ICM_20948_Stat_Ok : ICM_20948_Stat_NoData;
        return ready;
    }
    ICM_20948_AGMT_t getAGMT()
    {
        status = mockReadStatus;
        if (status == ICM_20948_Stat_Ok) agmt = mockAgmt;
        return agmt;
    }
    float accX() const { return static_cast<float>(agmt.acc.x); }
    float accY() const { return static_cast<float>(agmt.acc.y); }
    float accZ() const { return static_cast<float>(agmt.acc.z); }
    float gyrX() const { return static_cast<float>(agmt.gyr.x); }
    float gyrY() const { return static_cast<float>(agmt.gyr.y); }
    float gyrZ() const { return static_cast<float>(agmt.gyr.z); }
    float magX() const { return static_cast<float>(agmt.mag.x) * 0.15f; }
    float magY() const { return static_cast<float>(agmt.mag.y) * 0.15f; }
    float magZ() const { return static_cast<float>(agmt.mag.z) * 0.15f; }
};
