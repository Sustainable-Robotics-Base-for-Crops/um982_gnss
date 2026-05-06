// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__UTILS_HPP_
#define UM982_GNSS__UTILS_HPP_

#include <vector>
#include <string>
#include <stdint.h>

namespace um982_gnss
{
enum eASCIIState
{
  ASCII_SYNC,
  ASCII_TYPE,
  ASCII_DATA,
  ASCII_CRC
};

struct sASCII
{
  std::string sync;
  std::string type;
  std::vector<std::string> data;
  std::string crc;
  std::string msg;
  eASCIIState state = ASCII_SYNC;
};

enum eBinaryState
{
  BIN_SYNC,
  BIN_HEADER,
  BIN_DATA,
  BIN_CRC
};

struct sBinary
{
  uint16_t msg_id;
  uint16_t msg_length;
  std::vector<uint8_t> data;
  std::vector<uint8_t> crc;
  std::vector<uint8_t> msg;
  eBinaryState state = BIN_SYNC;
};

union uToFloat
{
  uint8_t bytes[4];
  float data;
};

union uToDouble
{
  uint8_t bytes[8];
  double data;
};
}  // namespace um982_gnss

#endif  // UM982_GNSS__UTILS_HPP_
