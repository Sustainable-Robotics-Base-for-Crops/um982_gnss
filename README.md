# um982_gnss

ROS 2 package for Unicore UM982 GNSS receivers in dual-antenna rover configuration: serial I/O (binary BESTNAV / STADOP, UNIHEADING, NMEA GGA), publishing fixes and speed/heading, and an optional NTRIP client for RTK corrections (RTCM).

---

## Node `um982_gnss`

On startup, the driver opens the serial port, sends a sequence of Unicore commands (rover mode, baseline length for heading, message output rates), then parses the stream and fills ROS messages.

### Parameters

| Parameter              | Type   | Default        | Description                                                           |
| ---------------------- | ------ | -------------- | --------------------------------------------------------------------- |
| `device`               | string | `/dev/ttyUSB0` | Serial device for the receiver                                        |
| `baudrate`             | int    | `115200`       | Serial baud rate                                                      |
| `frame_main`           | string | `gnss_main`    | `frame_id` for the main antenna (published messages)                  |
| `frame_aux`            | string | `gnss_aux`     | `frame_id` for the auxiliary antenna                                  |
| `heading.length`       | int    | `100`          | Baseline length for heading, in **cm** (`CONFIG HEADING LENGTH`)      |
| `heading.tolerance`    | int    | `3`            | Matching tolerance, in **cm**                                         |
| `heading.offset`       | int    | `0`            | Heading / pitch offset (deg), 1st argument to `CONFIG HEADING OFFSET` |
| `heading.pitch_offset` | int    | `0`            | 2nd pitch argument for `CONFIG HEADING OFFSET`                        |

### Publishers

| Topic            | Type                      | Description                                                                 |
| ---------------- | ------------------------- | --------------------------------------------------------------------------- |
| `navsatfix/main` | sensor_msgs/msg/NavSatFix | Main antenna position fix (WGS84, covariance, status)                       |
| `navsatfix/aux`  | sensor_msgs/msg/NavSatFix | Auxiliary antenna position fix                                              |
| `gpsfix/main`    | gps_msgs/msg/GPSFix       | Extended main fix: speed, heading (`track`), pitch, DOP, errors, etc.       |
| `gpsfix/aux`     | gps_msgs/msg/GPSFix       | Same for the auxiliary antenna                                              |
| `nmea`           | nmea_msgs/msg/Sentence    | GNGGA NMEA sentence, `header.frame_id` = `frame_main` — NTRIP / diagnostics |

### Subscribers

| Topic  | Type                 | Description                                                            |
| ------ | -------------------- | ---------------------------------------------------------------------- |
| `rtcm` | mavros_msgs/msg/RTCM | Received RTCM3 (e.g. from `ntrip_client`) forwarded to the serial port |

An internal timer resets configuration if RTK fix is not maintained on both expected chains (see `rtk_fix_` logic).

---

## Node `ntrip_client`

TCP connection to the caster, RTCM subscription for the chosen mountpoint, republication on ROS. GGA sentences on `nmea` are typically used to report rover position to the caster.

### Parameters

| Parameter      | Type   | Default     | Description                  |
| -------------- | ------ | ----------- | ---------------------------- |
| `host`         | string | `127.0.0.1` | NTRIP caster host            |
| `port`         | int    | `2101`      | Port (often 2101 for NTRIP)  |
| `authenticate` | bool   | `false`     | HTTP Basic authentication    |
| `mountpoint`   | string | `""`        | RTCM stream mountpoint       |
| `username`     | string | `""`        | Username when `authenticate` |
| `password`     | string | `""`        | Password when `authenticate` |
| `frame_id`     | string | `odom`      | `frame_id` on published RTCM |

### Publishers

| Topic  | Type                 | Description               |
| ------ | -------------------- | ------------------------- |
| `rtcm` | mavros_msgs/msg/RTCM | RTCM3 packets to receiver |

### Subscribers

| Topic  | Type                   | Description                             |
| ------ | ---------------------- | --------------------------------------- |
| `nmea` | nmea_msgs/msg/Sentence | GGA (and compatible stream) from driver |
