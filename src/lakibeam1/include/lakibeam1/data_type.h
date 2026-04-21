#ifndef LAKIBEAM1__DATA_TYPE_H_
#define LAKIBEAM1__DATA_TYPE_H_

#include <stdint.h>

#pragma pack(push, 1)

struct MeasuringResult
{
  uint16_t dist_1;
  uint8_t rssi_1;
  uint16_t dist_2;
  uint8_t rssi_2;
};

struct DataBlock
{
  uint16_t data_flag;
  uint16_t azimuth;
  MeasuringResult results[16];
};

struct MsopDataPacket
{
  DataBlock blocks[12];
  uint32_t timestamp;
  uint16_t factory;
};

#pragma pack(pop)

struct ScanPoint
{
  uint16_t angle;
  uint16_t distance_mm;
  uint8_t rssi;
  uint32_t timestamp;
};

#endif
