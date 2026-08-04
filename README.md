# um982_gnss

ROS 2 package for Unicore UM982 dual-antenna GNSS receivers. It provides two lifecycle nodes: `um982_gnss` (serial driver) and `ntrip_client` (RTK corrections).

## um982_gnss

Lifecycle node that opens the serial port, configures the receiver, parses BESTNAV, STADOP, UNIHEADING and NMEA GGA, and publishes fixes for both antennas.

### Overview

On activate:

1. Open the serial device and send Unicore configuration (rover mode, heading baseline, message rates).
2. Parse the incoming stream in a background thread.
3. Publish `navsatfix/*` and `gpsfix/*` for main and auxiliary antennas.
4. Publish GNGGA sentences on `nmea` for NTRIP.
5. Forward incoming `rtcm` messages to the serial port.

**Stuck-receiver watchdog:** if the **main** antenna stays without an RTK/GBAS fix for **2 continuous minutes**, configuration is re-run (`RESET` + setup). The countdown **starts when main loses fix** and is **cancelled** as soon as main recovers. A late auxiliary fix alone does **not** trigger a RESET (dual-fix remains relevant for heading quality, not for this watchdog). This replaces the previous periodic “every 2 min, if not dual-fix then RESET” check, which could reset the receiver during a short outage or while it was already recovering.

### Node parameters

| Parameter              | Default (header) | Description                                           |
| ---------------------- | ---------------- | ----------------------------------------------------- |
| `device`               | `/dev/ttyUSB0`   | Serial device                                         |
| `baudrate`             | `115200`         | Serial baud rate                                      |
| `frame_main`           | `gnss_main`      | `frame_id` for main antenna messages                  |
| `frame_aux`            | `gnss_aux`       | `frame_id` for auxiliary antenna messages             |
| `heading.length`       | `100`            | Baseline length for heading (cm)                      |
| `heading.tolerance`    | `3`              | Baseline matching tolerance (cm)                      |
| `heading.offset`       | `0`              | Heading offset (deg), 1st `CONFIG HEADING OFFSET` arg |
| `heading.pitch_offset` | `0`              | Pitch offset (deg), 2nd `CONFIG HEADING OFFSET` arg   |

### Topics

| Topic            | Type                        | Direction | Description                                 |
| ---------------- | --------------------------- | --------- | ------------------------------------------- |
| `rtcm`           | `mavros_msgs/msg/RTCM`      | In        | RTCM3 corrections (e.g. from NTRIP)         |
| `navsatfix/main` | `sensor_msgs/msg/NavSatFix` | Out       | Main antenna WGS84 fix                      |
| `navsatfix/aux`  | `sensor_msgs/msg/NavSatFix` | Out       | Auxiliary antenna WGS84 fix                 |
| `gpsfix/main`    | `gps_msgs/msg/GPSFix`       | Out       | Main fix with speed, track, pitch, DOP      |
| `gpsfix/aux`     | `gps_msgs/msg/GPSFix`       | Out       | Auxiliary fix with speed, track, pitch, DOP |
| `nmea`           | `nmea_msgs/msg/Sentence`    | Out       | GNGGA sentence (`frame_id` = `frame_main`)  |

## ntrip_client

Lifecycle node that connects to an NTRIP caster, receives RTCM for the selected mountpoint, and republishes it on ROS. It forwards rover GGA from `nmea` to the caster.

### Overview

On activate:

1. Open a TCP connection to the NTRIP caster.
2. Subscribe to `nmea` and send GGA sentences to the caster.
3. Parse incoming RTCM3 and publish on `rtcm`.

### Node parameters

| Parameter      | Default (header) | Description                  |
| -------------- | ---------------- | ---------------------------- |
| `host`         | `127.0.0.1`      | NTRIP caster host            |
| `port`         | `2101`           | NTRIP caster port            |
| `authenticate` | `false`          | HTTP Basic authentication    |
| `mountpoint`   | `""`             | RTCM stream mountpoint       |
| `username`     | `""`             | Username when `authenticate` |
| `password`     | `""`             | Password when `authenticate` |
| `frame_id`     | `odom`           | `frame_id` on published RTCM |

### Topics

| Topic  | Type                     | Direction | Description                   |
| ------ | ------------------------ | --------- | ----------------------------- |
| `nmea` | `nmea_msgs/msg/Sentence` | In        | GGA from `um982_gnss`         |
| `rtcm` | `mavros_msgs/msg/RTCM`   | Out       | RTCM3 packets to the receiver |
