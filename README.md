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

A 2 min timer re-runs configuration if RTK fix is not maintained on both antenna chains (`rtk_fix_ == 3`).

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

Instrumentation parameters (see [Diagnosing heading dropouts](#diagnosing-heading-dropouts)):

| Parameter                          | Default | Description                                                            |
| ---------------------------------- | ------- | ---------------------------------------------------------------------- |
| `extra_logs`                       | `[]`    | Extra Unicore logs requested after configuration; a refusal only warns |
| `diagnostics.enable`               | `true`  | Publish `heading_diagnostics`                                          |
| `diagnostics.rate`                 | `10.0`  | Diagnostics rate (Hz), capped by the 20 Hz `BESTNAVH` rate             |
| `diagnostics.heading_std_warn`     | `2.0`   | Heading std dev (deg) above which the status is `WARN`                 |
| `raw_dump.enable`                  | `false` | Dump the raw receiver stream to a file for offline decoding            |
| `raw_dump.directory`               | `""`    | Destination directory, created if missing                              |
| `raw_dump.max_mb`                  | `256`   | Dump size cap; the dump stops (the driver does not) when reached       |

### Topics

| Topic            | Type                        | Direction | Description                                 |
| ---------------- | --------------------------- | --------- | ------------------------------------------- |
| `rtcm`           | `mavros_msgs/msg/RTCM`      | In        | RTCM3 corrections (e.g. from NTRIP)         |
| `navsatfix/main` | `sensor_msgs/msg/NavSatFix` | Out       | Main antenna WGS84 fix                      |
| `navsatfix/aux`  | `sensor_msgs/msg/NavSatFix` | Out       | Auxiliary antenna WGS84 fix                 |
| `gpsfix/main`    | `gps_msgs/msg/GPSFix`       | Out       | Main fix with speed, track, pitch, DOP      |
| `gpsfix/aux`     | `gps_msgs/msg/GPSFix`       | Out       | Auxiliary fix with speed, track, pitch, DOP |
| `nmea`           | `nmea_msgs/msg/Sentence`    | Out       | GNGGA sentence (`frame_id` = `frame_main`)  |
| `heading_diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Out | Heading / auxiliary antenna health, see below |

### Diagnosing heading dropouts

On a dual-antenna receiver the auxiliary solution is not a second independent GNSS: it is
the constrained solution of the baseline between both antennas (`CONFIG HEADING FIXLENGTH`
+ `CONFIG HEADING LENGTH`). It therefore fails for reasons the main antenna never sees —
differential multipath from the machine itself, antenna coupling, mount flex — and it fails
under motion rather than under a poor sky. Downstream, `gnss_to_odom` requires **both**
antennas at RTK fix, so every auxiliary dropout stops the odometry even though the absolute
position is still centimetre-accurate.

Field measurements that motivated this instrumentation (SRBC, 44 cm baseline, 20 min run):
main antenna fixed 100 % of the time, auxiliary fix lost 6 times (19 s total) over 24
tracking dips, each one producing an odometry gap of exactly the same length.

`heading_diagnostics` publishes what is needed to tell the failure modes apart:

| Key                                            | Reading                                                        |
| ---------------------------------------------- | -------------------------------------------------------------- |
| `heading.sat_tracked` vs `heading.sat_used`    | tracked high + used low = ambiguity resolution fails, **not** a radio-frequency problem. Both collapsing = antenna or cabling |
| `aux.sat_tracked` vs `main.sat_tracked`        | asymmetry between the two antenna chains                       |
| `heading.baseline_m`, `baseline_min/max_m`     | a baseline that moves points at mount flex or vibration; a stable one clears the mechanics |
| `heading.std_dev_deg`                          | quality of the receiver heading, the figure to gate on instead of a binary fix flag |
| `heading.pos_type`, `aux.pos_type`             | raw Unicore solution types (`NARROW_INT` = fixed, `NARROW_FLOAT` = float) |
| `aux.fix_loss_count`, `aux.fix_lost_s`         | cumulative cost of the phenomenon since activation             |
| `main.diff_age_s`, `aux.diff_age_s`            | rules corrections in or out                                    |

For per-satellite carrier-to-noise ratios, request the observation logs and decode them
offline from the raw dump:

```yaml
extra_logs: ["OBSVMB 1", "OBSVHB 1"]
raw_dump:
  enable: true
  directory: /home/srbc/expe/raw_gnss
```

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
