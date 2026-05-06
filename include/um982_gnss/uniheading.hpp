// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__UNIHEADING_HPP_
#define UM982_GNSS__UNIHEADING_HPP_

#include "um982_gnss/utils.hpp"

// This log outputs the heading information of the receiver in motion. Heading refers to the
// clockwise angle between True North and the baseline vector from the master antenna to
// the slave antenna.

namespace um982_gnss
{
struct sUniheading
{
  uint32_t sol_stat;
  uint32_t pos_type;
  float length;
  float heading;
  float pitch;
  float hdg_std_dev;
  float ptch_std_dev;
  uint32_t stn_id;
  uint8_t sat_nb;
  uint8_t sol_sat_nb;
  uint8_t obs;
  uint8_t multi;
  uint8_t ext_sol_stat;
  uint8_t gal_bds3_sig_mask;
  uint8_t gps_glo_bds2_sig_mask;
};

void parse_uniheading(const sBinary& binary, sUniheading& uniheading)
{
  if (binary.data.size() < 44)
  {
    return;
  }

  uToFloat to_float;

  uniheading.sol_stat = uint32_t(binary.data[0]);
  uniheading.sol_stat |= uint32_t(binary.data[1] << 8);
  uniheading.sol_stat |= uint32_t(binary.data[2] << 16);
  uniheading.sol_stat |= uint32_t(binary.data[3] << 24);

  uniheading.pos_type = uint32_t(binary.data[4]);
  uniheading.pos_type |= uint32_t(binary.data[5] << 8);
  uniheading.pos_type |= uint32_t(binary.data[6] << 16);
  uniheading.pos_type |= uint32_t(binary.data[7] << 24);

  to_float.bytes[0] = binary.data[8];
  to_float.bytes[1] = binary.data[9];
  to_float.bytes[2] = binary.data[10];
  to_float.bytes[3] = binary.data[11];

  uniheading.length = to_float.data;

  to_float.bytes[0] = binary.data[12];
  to_float.bytes[1] = binary.data[13];
  to_float.bytes[2] = binary.data[14];
  to_float.bytes[3] = binary.data[15];

  uniheading.heading = to_float.data;

  to_float.bytes[0] = binary.data[16];
  to_float.bytes[1] = binary.data[17];
  to_float.bytes[2] = binary.data[18];
  to_float.bytes[3] = binary.data[19];

  uniheading.pitch = to_float.data;

  // reserved 4 bytes

  to_float.bytes[0] = binary.data[24];
  to_float.bytes[1] = binary.data[25];
  to_float.bytes[2] = binary.data[26];
  to_float.bytes[3] = binary.data[27];

  uniheading.hdg_std_dev = to_float.data;

  to_float.bytes[0] = binary.data[28];
  to_float.bytes[1] = binary.data[29];
  to_float.bytes[2] = binary.data[30];
  to_float.bytes[3] = binary.data[31];

  uniheading.ptch_std_dev = to_float.data;

  uniheading.stn_id = uint32_t(binary.data[32]);
  uniheading.stn_id |= uint32_t(binary.data[33] << 8);
  uniheading.stn_id |= uint32_t(binary.data[34] << 16);
  uniheading.stn_id |= uint32_t(binary.data[35] << 24);

  uniheading.sat_nb = binary.data[36];
  uniheading.sol_sat_nb = binary.data[37];
  uniheading.obs = binary.data[38];
  uniheading.multi = binary.data[39];
  // reserved 1 byte
  uniheading.ext_sol_stat = binary.data[41];
  uniheading.gal_bds3_sig_mask = binary.data[42];
  uniheading.gps_glo_bds2_sig_mask = binary.data[43];
}
}  // namespace um982_gnss

#endif  // UM982_GNSS__UNIHEADING_HPP_
