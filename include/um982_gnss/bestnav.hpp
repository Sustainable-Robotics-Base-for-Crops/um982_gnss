// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__BESTNAV_HPP_
#define UM982_GNSS__BESTNAV_HPP_

#include "um982_gnss/utils.hpp"

// This log contains the best GNSS and INS (if available) position and velocity computed
// by the receiver using the master antenna. It also contains several status indicators,
// including the differential age, which can be used to predict the abnormal operation
// caused by the interruption of the transmission of differential correction data. If the
// differential age is 0, it indicates that no differential correction is used.

namespace um982_gnss
{
enum ePositionType
{
  NONE = 0,
  FIXEDPOS = 1,
  FIXEHEIGHT = 2,
  DOPPLER_VELOCITY = 8,
  SINGLE = 16,
  PSRDIFF = 17,
  SBAS = 18,
  L1_FLOAT = 32,
  IONOFREE_FLOAT = 33,
  NARROW_FLOAT = 34,
  L1_INT = 48,
  WIDE_INT = 49,
  NARROW_INT = 50,
  INS = 52,
  INS_PSRSP = 53,
  INS_PSRDIFF = 54,
  INS_RTKFLOAT = 55,
  INS_RTK_FIXED = 56,
  PPP_CONVERGING = 68,
  PPP = 69
};

struct sBestnav
{
  uint32_t p_sol_status;
  uint32_t pos_type;
  double lat;
  double lon;
  double hgt;
  float undulation;
  uint32_t datum_id;
  float lat_std;
  float lon_std;
  float hgt_std;
  uint32_t stn_id;
  float diff_age;
  float sol_age;
  uint8_t sat_nb;
  uint8_t sol_sat_nb;
  uint8_t ext_sol_stat;
  uint8_t gal_bds3_sig_mask;
  uint8_t gps_glo_bds2_sig_mask;
  uint32_t v_sol_status;
  uint32_t vel_type;
  float latency;
  float age;
  double hor_spd;
  double trk_gnd;
  double vert_spd;
  float vert_spd_std;
  float hor_spd_std;
};

void parse_bestnav(const sBinary& binary, sBestnav& bestnav)
{
  if (binary.data.size() < 120)
  {
    return;
  }

  uToFloat to_float;
  uToDouble to_double;

  bestnav.p_sol_status = uint32_t(binary.data[0]);
  bestnav.p_sol_status |= uint32_t(binary.data[1] << 8);
  bestnav.p_sol_status |= uint32_t(binary.data[2] << 16);
  bestnav.p_sol_status |= uint32_t(binary.data[3] << 24);

  bestnav.pos_type = uint32_t(binary.data[4]);
  bestnav.pos_type |= uint32_t(binary.data[5] << 8);
  bestnav.pos_type |= uint32_t(binary.data[6] << 16);
  bestnav.pos_type |= uint32_t(binary.data[7] << 24);

  to_double.bytes[0] = binary.data[8];
  to_double.bytes[1] = binary.data[9];
  to_double.bytes[2] = binary.data[10];
  to_double.bytes[3] = binary.data[11];
  to_double.bytes[4] = binary.data[12];
  to_double.bytes[5] = binary.data[13];
  to_double.bytes[6] = binary.data[14];
  to_double.bytes[7] = binary.data[15];

  bestnav.lat = to_double.data;

  to_double.bytes[0] = binary.data[16];
  to_double.bytes[1] = binary.data[17];
  to_double.bytes[2] = binary.data[18];
  to_double.bytes[3] = binary.data[19];
  to_double.bytes[4] = binary.data[20];
  to_double.bytes[5] = binary.data[21];
  to_double.bytes[6] = binary.data[22];
  to_double.bytes[7] = binary.data[23];

  bestnav.lon = to_double.data;

  to_double.bytes[0] = binary.data[24];
  to_double.bytes[1] = binary.data[25];
  to_double.bytes[2] = binary.data[26];
  to_double.bytes[3] = binary.data[27];
  to_double.bytes[4] = binary.data[28];
  to_double.bytes[5] = binary.data[29];
  to_double.bytes[6] = binary.data[30];
  to_double.bytes[7] = binary.data[31];

  bestnav.hgt = to_double.data;

  to_float.bytes[0] = binary.data[32];
  to_float.bytes[1] = binary.data[33];
  to_float.bytes[2] = binary.data[34];
  to_float.bytes[3] = binary.data[35];

  bestnav.undulation = to_float.data;

  bestnav.datum_id = uint32_t(binary.data[36]);
  bestnav.datum_id |= uint32_t(binary.data[37] << 8);
  bestnav.datum_id |= uint32_t(binary.data[38] << 16);
  bestnav.datum_id |= uint32_t(binary.data[39] << 24);

  to_float.bytes[0] = binary.data[40];
  to_float.bytes[1] = binary.data[41];
  to_float.bytes[2] = binary.data[42];
  to_float.bytes[3] = binary.data[43];

  bestnav.lat_std = to_float.data;

  to_float.bytes[0] = binary.data[44];
  to_float.bytes[1] = binary.data[45];
  to_float.bytes[2] = binary.data[46];
  to_float.bytes[3] = binary.data[47];

  bestnav.lon_std = to_float.data;

  to_float.bytes[0] = binary.data[48];
  to_float.bytes[1] = binary.data[49];
  to_float.bytes[2] = binary.data[50];
  to_float.bytes[3] = binary.data[51];

  bestnav.hgt_std = to_float.data;

  bestnav.stn_id = uint32_t(binary.data[52]);
  bestnav.stn_id |= uint32_t(binary.data[53] << 8);
  bestnav.stn_id |= uint32_t(binary.data[54] << 16);
  bestnav.stn_id |= uint32_t(binary.data[55] << 24);

  to_float.bytes[0] = binary.data[56];
  to_float.bytes[1] = binary.data[57];
  to_float.bytes[2] = binary.data[58];
  to_float.bytes[3] = binary.data[59];

  bestnav.diff_age = to_float.data;

  to_float.bytes[0] = binary.data[60];
  to_float.bytes[1] = binary.data[61];
  to_float.bytes[2] = binary.data[62];
  to_float.bytes[3] = binary.data[63];

  bestnav.sol_age = to_float.data;

  bestnav.sat_nb = binary.data[64];
  bestnav.sol_sat_nb = binary.data[65];

  // reserved 1 byte
  // reserved 1 byte
  // reserved 1 byte

  bestnav.ext_sol_stat = binary.data[69];
  bestnav.gal_bds3_sig_mask = binary.data[70];
  bestnav.gps_glo_bds2_sig_mask = binary.data[71];

  bestnav.v_sol_status = uint32_t(binary.data[72]);
  bestnav.v_sol_status |= uint32_t(binary.data[73] << 8);
  bestnav.v_sol_status |= uint32_t(binary.data[74] << 16);
  bestnav.v_sol_status |= uint32_t(binary.data[75] << 24);

  bestnav.vel_type = uint32_t(binary.data[76]);
  bestnav.vel_type |= uint32_t(binary.data[77] << 8);
  bestnav.vel_type |= uint32_t(binary.data[78] << 16);
  bestnav.vel_type |= uint32_t(binary.data[79] << 24);

  to_float.bytes[0] = binary.data[80];
  to_float.bytes[1] = binary.data[81];
  to_float.bytes[2] = binary.data[82];
  to_float.bytes[3] = binary.data[83];

  bestnav.latency = to_float.data;

  to_float.bytes[0] = binary.data[84];
  to_float.bytes[1] = binary.data[85];
  to_float.bytes[2] = binary.data[86];
  to_float.bytes[3] = binary.data[87];

  bestnav.age = to_float.data;

  to_double.bytes[0] = binary.data[88];
  to_double.bytes[1] = binary.data[89];
  to_double.bytes[2] = binary.data[90];
  to_double.bytes[3] = binary.data[91];
  to_double.bytes[4] = binary.data[92];
  to_double.bytes[5] = binary.data[93];
  to_double.bytes[6] = binary.data[94];
  to_double.bytes[7] = binary.data[95];

  bestnav.hor_spd = to_double.data;

  to_double.bytes[0] = binary.data[96];
  to_double.bytes[1] = binary.data[97];
  to_double.bytes[2] = binary.data[98];
  to_double.bytes[3] = binary.data[99];
  to_double.bytes[4] = binary.data[100];
  to_double.bytes[5] = binary.data[101];
  to_double.bytes[6] = binary.data[102];
  to_double.bytes[7] = binary.data[103];

  bestnav.trk_gnd = to_double.data;

  to_double.bytes[0] = binary.data[104];
  to_double.bytes[1] = binary.data[105];
  to_double.bytes[2] = binary.data[106];
  to_double.bytes[3] = binary.data[107];
  to_double.bytes[4] = binary.data[108];
  to_double.bytes[5] = binary.data[109];
  to_double.bytes[6] = binary.data[110];
  to_double.bytes[7] = binary.data[111];

  bestnav.vert_spd = to_double.data;

  to_float.bytes[0] = binary.data[112];
  to_float.bytes[1] = binary.data[113];
  to_float.bytes[2] = binary.data[114];
  to_float.bytes[3] = binary.data[115];

  bestnav.vert_spd_std = to_float.data;

  to_float.bytes[0] = binary.data[116];
  to_float.bytes[1] = binary.data[117];
  to_float.bytes[2] = binary.data[118];
  to_float.bytes[3] = binary.data[119];

  bestnav.hor_spd_std = to_float.data;
}
}  // namespace um982_gnss

#endif  // UM982_GNSS__BESTNAV_HPP_
