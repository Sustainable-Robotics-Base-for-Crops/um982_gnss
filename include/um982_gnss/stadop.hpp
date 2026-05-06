// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__STADOP_HPP_
#define UM982_GNSS__STADOP_HPP_

#include "um982_gnss/utils.hpp"

// This log contains dilution of precision (DOP) for all satellites used in the BESTNAV
// solution.

namespace um982_gnss
{
struct sStadop
{
  float gdop;
  float pdop;
  float tdop;
  float vdop;
  float hdop;
  float ndop;
  float edop;
  float cutoff;
  uint16_t prn_number;
  std::vector<uint16_t> pnr;
};

void parse_stadop(const sBinary& binary, sStadop& stadop)
{
  if (binary.data.size() < 42)
  {
    return;
  }

  uToFloat to_float;

  // reserved 4 bytes

  to_float.bytes[0] = binary.data[4];
  to_float.bytes[1] = binary.data[5];
  to_float.bytes[2] = binary.data[6];
  to_float.bytes[3] = binary.data[7];

  stadop.gdop = to_float.data;

  to_float.bytes[0] = binary.data[8];
  to_float.bytes[1] = binary.data[9];
  to_float.bytes[2] = binary.data[10];
  to_float.bytes[3] = binary.data[11];

  stadop.pdop = to_float.data;

  to_float.bytes[0] = binary.data[12];
  to_float.bytes[1] = binary.data[13];
  to_float.bytes[2] = binary.data[14];
  to_float.bytes[3] = binary.data[15];

  stadop.tdop = to_float.data;

  to_float.bytes[0] = binary.data[16];
  to_float.bytes[1] = binary.data[17];
  to_float.bytes[2] = binary.data[18];
  to_float.bytes[3] = binary.data[19];

  stadop.vdop = to_float.data;

  to_float.bytes[0] = binary.data[20];
  to_float.bytes[1] = binary.data[21];
  to_float.bytes[2] = binary.data[22];
  to_float.bytes[3] = binary.data[23];

  stadop.hdop = to_float.data;

  to_float.bytes[0] = binary.data[24];
  to_float.bytes[1] = binary.data[25];
  to_float.bytes[2] = binary.data[26];
  to_float.bytes[3] = binary.data[27];

  stadop.ndop = to_float.data;

  to_float.bytes[0] = binary.data[28];
  to_float.bytes[1] = binary.data[29];
  to_float.bytes[2] = binary.data[30];
  to_float.bytes[3] = binary.data[31];

  stadop.edop = to_float.data;

  to_float.bytes[0] = binary.data[32];
  to_float.bytes[1] = binary.data[33];
  to_float.bytes[2] = binary.data[34];
  to_float.bytes[3] = binary.data[35];

  stadop.cutoff = to_float.data;

  // reserved 4 bytes

  stadop.prn_number = uint16_t(binary.data[40]);
  stadop.prn_number |= uint16_t(binary.data[41] << 8);

  stadop.pnr.clear();

  size_t idx = 42;

  for (uint16_t i = 0; i < stadop.prn_number; i++)
  {
    if (binary.data.size() < idx + 2)
    {
      return;
    }

    uint16_t prn;
    prn = uint16_t(binary.data[idx]);
    prn |= uint16_t(binary.data[idx + 1] << 8);

    idx += 2;

    stadop.pnr.push_back(prn);
  }
}
}  // namespace um982_gnss

#endif  // UM982_GNSS__STADOP_HPP_
