// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__GGA_HPP_
#define UM982_GNSS__GGA_HPP_

#include "um982_gnss/utils.hpp"

// This log contains time, position, and fix related data.

namespace um982_gnss
{
void parse_gga(const sASCII& ascii, std::string& sentence)
{
  if (ascii.data.size() < 4)
  {
    return;
  }

  sentence.clear();
  sentence.append("$GNGGA");

  for (size_t i = 0; i < ascii.data.size(); i++)
  {
    sentence.append(",");

    if (i == 1 || i == 3)  // latitude / longitude
    {
      sentence.append(ascii.data[i].substr(0, ascii.data[i].size() - 4));
    }
    else if (i == 8 || i == 10)  // altitude / undulation
    {
      sentence.append(ascii.data[i].substr(0, ascii.data[i].size() - 3));
    }
    else
    {
      sentence.append(ascii.data[i]);
    }
  }

  uint8_t crc = 0;

  for (size_t i = 1; i < sentence.size(); i++)
  {
    crc ^= sentence[i];
  }

  std::stringstream ss;
  ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(crc);

  sentence.append("*");
  sentence.append(ss.str());
}
}  // namespace um982_gnss

#endif  // UM982_GNSS__GGA_HPP_
