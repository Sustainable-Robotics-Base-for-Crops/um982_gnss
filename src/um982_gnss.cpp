// Copyright 2026 SABI AGRI

#include "um982_gnss/um982_gnss.hpp"
#include "um982_gnss/crc.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace um982_gnss
{
UM982Gnss::UM982Gnss(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("um982_gnss", options)
{
  this->declare_parameter("device", device_);
  this->declare_parameter("baudrate", baudrate_);
  this->declare_parameter("frame_main", frame_main_);
  this->declare_parameter("frame_aux", frame_aux_);
  this->declare_parameter("heading.length", heading_length_);
  this->declare_parameter("heading.tolerance", heading_tolerance_);
  this->declare_parameter("heading.offset", heading_offset_);
  this->declare_parameter("heading.pitch_offset", heading_pitch_offset_);
}

LNI::CallbackReturn UM982Gnss::on_configure(const rclcpp_lifecycle::State&)
{
  this->get_parameter("device", device_);
  this->get_parameter("baudrate", baudrate_);
  this->get_parameter("frame_main", frame_main_);
  this->get_parameter("frame_aux", frame_aux_);
  this->get_parameter("heading.length", heading_length_);
  this->get_parameter("heading.tolerance", heading_tolerance_);
  this->get_parameter("heading.offset", heading_offset_);
  this->get_parameter("heading.pitch_offset", heading_pitch_offset_);

  navsatfix_main_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix/main", 10);
  navsatfix_aux_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix/aux", 10);
  gpsfix_main_pub_ = this->create_publisher<gps_msgs::msg::GPSFix>("gpsfix/main", 10);
  gpsfix_aux_pub_ = this->create_publisher<gps_msgs::msg::GPSFix>("gpsfix/aux", 10);
  gga_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea", 10);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UM982Gnss::on_activate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_activate(state);

  std::string id = this->get_namespace();

  if (id != "/")
  {
    id += "/";
  }

  id += this->get_name();

  bond_ = std::make_unique<bond::Bond>("bond", id, shared_from_this());
  bond_->start();

  initialized_ = false;
  stop_thread_ = false;
  main_fix_lost_ = false;
  init_thread_ = std::thread(&UM982Gnss::init_thread_callback, this);

  // Poll often enough to start/cancel the 2 min countdown near the fix-loss edge.
  // The RESET itself is still gated by kMainFixLostTimeout (see timer_callback).
  timer_ = this->create_wall_timer(kWatchdogPollPeriod, std::bind(&UM982Gnss::timer_callback, this));

  rtcm_sub_ =
      this->create_subscription<mavros_msgs::msg::RTCM>("rtcm", 10, std::bind(&UM982Gnss::rtcm_callback, this, _1));

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UM982Gnss::on_deactivate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_deactivate(state);

  bond_->breakBond();
  close_serial();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UM982Gnss::on_cleanup(const rclcpp_lifecycle::State&)
{
  close_serial();

  timer_.reset();
  navsatfix_main_pub_.reset();
  navsatfix_aux_pub_.reset();
  gpsfix_main_pub_.reset();
  gpsfix_aux_pub_.reset();
  gga_pub_.reset();
  rtcm_sub_.reset();
  bond_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UM982Gnss::on_shutdown(const rclcpp_lifecycle::State&)
{
  close_serial();

  timer_.reset();
  navsatfix_main_pub_.reset();
  navsatfix_aux_pub_.reset();
  gpsfix_main_pub_.reset();
  gpsfix_aux_pub_.reset();
  gga_pub_.reset();
  rtcm_sub_.reset();
  bond_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

void UM982Gnss::close_serial()
{
  stop_thread_ = true;

  if (init_thread_.joinable())
  {
    init_thread_.join();
  }

  ser_.close();
}

void UM982Gnss::timer_callback()
{
  // Stuck-receiver watchdog (field intent)
  // ------------------------------------
  // Starting the robot indoors often yields no RTK fix. Once outdoors, the
  // receiver can remain stuck until a RESET (same effect as a power cycle).
  // This watchdog automates that remedy: if the main antenna stays without an
  // RTK/GBAS fix for kMainFixLostTimeout, re-run configuration (RESET + setup).
  //
  // Why not the previous "every 2 min, if rtk_fix_ != 3 then RESET"?
  // 1) Timing: a periodic snapshot can fire in the middle of a short outage
  //    (building, trees) and RESET a receiver that was already recovering.
  // 2) Criterion: requiring *both* antennas (rtk_fix_ == 3) is correct for
  //    heading quality, but a late aux fix can last minutes while main is already
  //    fixed — that must not trigger a RESET. Only a sustained main-fix loss does.
  //
  // Countdown starts at the loss edge and is cancelled as soon as main recovers.

  const bool main_has_fix = (rtk_fix_.load() & 0x01) != 0;
  const auto now = std::chrono::steady_clock::now();

  if (main_has_fix)
  {
    if (main_fix_lost_)
    {
      RCLCPP_INFO(this->get_logger(), "Main antenna RTK fix recovered; cancelling RESET watchdog timer");
      main_fix_lost_ = false;
    }
    return;
  }

  if (!main_fix_lost_)
  {
    main_fix_lost_ = true;
    main_fix_lost_since_ = now;
    RCLCPP_WARN(this->get_logger(),
                "Main antenna lost RTK fix; starting %ld min recovery timer before RESET",
                static_cast<long>(kMainFixLostTimeout.count()));
    return;
  }

  if (now - main_fix_lost_since_ < kMainFixLostTimeout)
  {
    return;
  }

  RCLCPP_WARN(this->get_logger(),
              "Main antenna without RTK fix for %ld min; restarting the receiver...",
              static_cast<long>(kMainFixLostTimeout.count()));
  initialized_ = false;
  // Arm a fresh countdown so hangar / stuck-outdoors cases can retry every timeout
  // until a main fix appears, without resetting on every 1 s poll.
  main_fix_lost_since_ = now;
}

void UM982Gnss::init_thread_callback()
{
  uint16_t sleep = 100;

  while (!stop_thread_)
  {
    if (sleep < 100)
    {
      sleep++;
      std::this_thread::sleep_for(10ms);
      continue;
    }

    if (initialized_)
    {
      sleep = 0;
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Configuring device...");

    if (!ser_.open(device_, baudrate_, &UM982Gnss::callback, this))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error opening serial port: " << device_);
      sleep = 0;
      continue;
    }

    bool err = false;

    // Clean command
    err |= !ser_.write("\r\n");

    // Restart the receiver
    command("RESET");

    // Sleep 5 seconds
    sleep = 0;
    while (!stop_thread_ && sleep < 500)
    {
      sleep++;
      std::this_thread::sleep_for(10ms);
    }

    // Clean command
    err |= !ser_.write("\r\n");

    // Stop message output
    err |= !command("UNLOG");

    // Sleep 1 second
    sleep = 0;
    while (!stop_thread_ && sleep < 100)
    {
      sleep++;
      std::this_thread::sleep_for(10ms);
    }

    // Reset the main antenna’s RTK algorithm
    err |= !command("CONFIG ALGRESET RTK1");

    // Reset the auxiliary antenna’s RTK algorithm
    err |= !command("CONFIG ALGRESET RTK2");

    // Rover configuration
    err |= !command("MODE ROVER");

    // Heading configuration
    err |= !command("CONFIG HEADING FIXLENGTH");

    std::string cmd = "CONFIG HEADING LENGTH";
    cmd.append(" ");
    cmd.append(std::to_string(heading_length_));
    cmd.append(" ");
    cmd.append(std::to_string(heading_tolerance_));

    err |= !command(cmd);

    cmd = "CONFIG HEADING OFFSET";
    cmd.append(" ");
    cmd.append(std::to_string(heading_offset_));
    cmd.append(" ");
    cmd.append(std::to_string(heading_pitch_offset_));

    err |= !command(cmd);

    // Output 20Hz BESTNAV message at the current port
    err |= !command("BESTNAVB 0.05");

    // Output 20Hz BESTNAVH message at the current port
    err |= !command("BESTNAVHB 0.05");

    // Output 1Hz GPGGA message at the current port
    err |= !command("GPGGA 1");

    // Output 1Hz STADOP message at the current port
    err |= !command("STADOPB 1");

    // Output 1Hz STADOPH message at the current port
    err |= !command("STADOPHB 1");

    // Output 10Hz UNIHEADING message at the current port
    err |= !command("UNIHEADINGB 0.1");

    if (err)
    {
      RCLCPP_ERROR(this->get_logger(), "Configuration failed");
      sleep = 0;
      continue;
    }

    RCLCPP_INFO(this->get_logger(), "Device configured");

    initialized_ = true;
  }
}

void UM982Gnss::parse_ascii(uint8_t data)
{
  if (ascii_.msg.size() > 1024)
  {
    ascii_.state = ASCII_SYNC;
  }

  if (data == '$' || data == '#')
  {
    ascii_.state = ASCII_SYNC;
  }

  switch (ascii_.state)
  {
    case ASCII_SYNC: {
      if (data == '$' || data == '#')
      {
        ascii_.msg.clear();
        ascii_.type.clear();
        ascii_.sync = data;
        ascii_.state = ASCII_TYPE;
      }

      break;
    }

    case ASCII_TYPE: {
      ascii_.msg.push_back(data);

      if (data == ',')
      {
        ascii_.data.clear();
        ascii_.data.push_back("");
        ascii_.state = ASCII_DATA;
      }
      else
      {
        ascii_.type.push_back(data);
      }

      break;
    }

    case ASCII_DATA: {
      if (data == '*')
      {
        ascii_.crc.clear();
        ascii_.state = ASCII_CRC;
      }
      else
      {
        ascii_.msg.push_back(data);

        if (data == ',')
        {
          ascii_.data.push_back("");
        }
        else
        {
          ascii_.data.back().push_back(data);
        }
      }

      break;
    }

    case ASCII_CRC: {
      ascii_.crc.push_back(data);

      if (ascii_.crc.size() == 2)
      {
        if (crc_ascii(ascii_))
        {
          process_ascii();
        }

        ascii_.state = ASCII_SYNC;
      }

      break;
    }

    default: {
      break;
    }
  }
}

void UM982Gnss::parse_binary(uint8_t data)
{
  switch (binary_.state)
  {
    case BIN_SYNC: {
      if (data == 0xAA)
      {
        binary_.msg.clear();
        binary_.msg.push_back(data);
        binary_.state = BIN_HEADER;
      }

      break;
    }

    case BIN_HEADER: {
      binary_.msg.push_back(data);

      if (binary_.msg.size() == 2)
      {
        if (data != 0x44)
        {
          binary_.state = BIN_SYNC;
        }
      }
      else if (binary_.msg.size() == 3)
      {
        if (data != 0xB5)
        {
          binary_.state = BIN_SYNC;
        }
      }
      else if (binary_.msg.size() == 6)
      {
        binary_.msg_id = binary_.msg[4];
        binary_.msg_id |= (binary_.msg[5] << 8);
      }
      else if (binary_.msg.size() == 8)
      {
        binary_.msg_length = binary_.msg[6];
        binary_.msg_length |= (binary_.msg[7] << 8);
      }
      else if (binary_.msg.size() == 24)
      {
        binary_.data.clear();

        if (binary_.msg_length == 0)
        {
          binary_.crc.clear();
          binary_.state = BIN_CRC;
        }
        else
        {
          binary_.state = BIN_DATA;
        }
      }

      break;
    }

    case BIN_DATA: {
      binary_.msg.push_back(data);
      binary_.data.push_back(data);

      if (binary_.data.size() == binary_.msg_length)
      {
        binary_.crc.clear();
        binary_.state = BIN_CRC;
      }

      break;
    }

    case BIN_CRC: {
      binary_.crc.push_back(data);

      if (binary_.crc.size() == 4)
      {
        if (crc_binary(binary_))
        {
          process_binary();
        }

        binary_.state = BIN_SYNC;
      }

      break;
    }

    default: {
      break;
    }
  }
}

void UM982Gnss::process_ascii()
{
  if (ascii_.type == "command")
  {
    if (ascii_.data.size() > 1 && ascii_.data[1] == "response: OK")
    {
      response_ = 1;
    }
    else
    {
      response_ = -1;
    }
  }
  else if (ascii_.type == "GNGGA")
  {
    std::string sentence;
    parse_gga(ascii_, sentence);

    nmea_msgs::msg::Sentence gga_msg;
    gga_msg.header.stamp = this->now();
    gga_msg.header.frame_id = frame_main_;
    gga_msg.sentence = sentence;

    gga_pub_->publish(gga_msg);
  }
}

void UM982Gnss::process_binary()
{
  if (binary_.msg_id == 2118)
  {
    parse_bestnav(binary_, bestnav_main_);

    sensor_msgs::msg::NavSatFix navsatfix_msg;
    gps_msgs::msg::GPSFix gpsfix_msg;

    rclcpp::Time t = this->now();
    navsatfix_msg.header.stamp = t;
    navsatfix_msg.header.frame_id = frame_main_;
    gpsfix_msg.header.stamp = t;
    gpsfix_msg.header.frame_id = frame_main_;

    handle_navsatfix(navsatfix_msg, bestnav_main_);
    handle_gpsfix(gpsfix_msg, bestnav_main_, stadop_main_);

    if (navsatfix_msg.status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
    {
      rtk_fix_ |= (1 << 0);
    }
    else
    {
      rtk_fix_ &= ~(1 << 0);
    }

    navsatfix_main_pub_->publish(navsatfix_msg);
    gpsfix_main_pub_->publish(gpsfix_msg);
  }
  else if (binary_.msg_id == 2119)
  {
    parse_bestnav(binary_, bestnav_aux_);

    sensor_msgs::msg::NavSatFix navsatfix_msg;
    gps_msgs::msg::GPSFix gpsfix_msg;

    rclcpp::Time t = this->now();
    navsatfix_msg.header.stamp = t;
    navsatfix_msg.header.frame_id = frame_aux_;
    gpsfix_msg.header.stamp = t;
    gpsfix_msg.header.frame_id = frame_aux_;

    handle_navsatfix(navsatfix_msg, bestnav_aux_);
    handle_gpsfix(gpsfix_msg, bestnav_aux_, stadop_aux_);

    if (navsatfix_msg.status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
    {
      rtk_fix_ |= (1 << 1);
    }
    else
    {
      rtk_fix_ &= ~(1 << 1);
    }

    navsatfix_aux_pub_->publish(navsatfix_msg);
    gpsfix_aux_pub_->publish(gpsfix_msg);
  }
  else if (binary_.msg_id == 954)
  {
    parse_stadop(binary_, stadop_main_);
  }
  else if (binary_.msg_id == 2122)
  {
    parse_stadop(binary_, stadop_aux_);
  }
  else if (binary_.msg_id == 972)
  {
    parse_uniheading(binary_, uniheading_);
  }
}

bool UM982Gnss::command(const std::string& cmd)
{
  if (!ser_.write(cmd + "\r\n"))
  {
    return false;
  }

  response_ = 0;
  uint16_t timeout = 0;

  while (!stop_thread_ && response_ == 0)
  {
    std::this_thread::sleep_for(1ms);

    timeout++;

    if (timeout > 1000)
    {
      return false;
    }
  }

  return (response_ > 0);
}

void UM982Gnss::callback(const std::vector<uint8_t>& data)
{
  for (uint8_t d : data)
  {
    parse_ascii(d);
    parse_binary(d);
  }
}

void UM982Gnss::rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr msg)
{
  if (!initialized_)
  {
    return;
  }

  ser_.write(msg->data);
}

void UM982Gnss::handle_navsatfix(sensor_msgs::msg::NavSatFix& msg, const sBestnav& bestnav)
{
  switch (bestnav.pos_type)
  {
    case (NONE): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      break;
    }
    case (SINGLE): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (PSRDIFF): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (SBAS): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (L1_FLOAT): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (IONOFREE_FLOAT): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (NARROW_FLOAT): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (L1_INT): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (WIDE_INT): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (NARROW_INT): {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    default: {
      msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      break;
    }
  }

  msg.status.service = 0;

  if (bestnav.gps_glo_bds2_sig_mask & 0x07)
  {
    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  }

  if (bestnav.gps_glo_bds2_sig_mask & 0x30)
  {
    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
  }

  if (bestnav.gal_bds3_sig_mask & 0x07)
  {
    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
  }

  if ((bestnav.gps_glo_bds2_sig_mask & 0xC0) || (bestnav.gal_bds3_sig_mask & 0xF0))
  {
    msg.status.service |= sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  }

  msg.latitude = bestnav.lat;
  msg.longitude = bestnav.lon;
  msg.altitude = bestnav.hgt;

  msg.position_covariance[0] = pow(bestnav.lon_std, 2);
  msg.position_covariance[4] = pow(bestnav.lat_std, 2);
  msg.position_covariance[8] = pow(bestnav.hgt_std, 2);
}

void UM982Gnss::handle_gpsfix(gps_msgs::msg::GPSFix& msg, const sBestnav& bestnav, const sStadop& stadop)
{
  msg.status.header = msg.header;

  std::vector<int32_t> prn(stadop.pnr.begin(), stadop.pnr.end());
  msg.status.satellites_used = stadop.prn_number;
  msg.status.satellite_used_prn = prn;

  switch (bestnav.pos_type)
  {
    case (NONE): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN;
      break;
    }
    case (SINGLE): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (PSRDIFF): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (SBAS): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (L1_FLOAT): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (IONOFREE_FLOAT): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (NARROW_FLOAT): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_SBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (L1_INT): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (WIDE_INT): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    case (NARROW_INT): {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_GBAS_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    }
    default: {
      msg.status.status = gps_msgs::msg::GPSStatus::STATUS_NO_FIX;
      msg.position_covariance_type = gps_msgs::msg::GPSFix::COVARIANCE_TYPE_UNKNOWN;
      break;
    }
  }

  if (bestnav.vel_type == NONE)
  {
    msg.status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_NONE;
  }
  else
  {
    msg.status.motion_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;
  }

  msg.status.orientation_source = gps_msgs::msg::GPSStatus::SOURCE_POINTS;
  msg.status.position_source = gps_msgs::msg::GPSStatus::SOURCE_GPS;

  msg.latitude = bestnav.lat;
  msg.longitude = bestnav.lon;
  msg.altitude = bestnav.hgt;
  msg.track = uniheading_.heading;
  msg.speed = bestnav.hor_spd;
  msg.climb = bestnav.vert_spd;
  msg.pitch = uniheading_.pitch;

  msg.gdop = stadop.gdop;
  msg.pdop = stadop.pdop;
  msg.hdop = stadop.hdop;
  msg.vdop = stadop.vdop;
  msg.tdop = stadop.tdop;

  msg.err = sqrt(pow(bestnav.lat_std, 2) + pow(bestnav.lon_std, 2) + pow(bestnav.hgt_std, 2));
  msg.err_horz = sqrt(pow(bestnav.lat_std, 2) + pow(bestnav.lon_std, 2));
  msg.err_vert = bestnav.hgt_std;
  msg.err_track = uniheading_.hdg_std_dev;
  msg.err_speed = bestnav.hor_spd_std;
  msg.err_climb = bestnav.vert_spd_std;
  msg.err_pitch = uniheading_.ptch_std_dev;

  msg.position_covariance[0] = pow(bestnav.lon_std, 2);
  msg.position_covariance[4] = pow(bestnav.lat_std, 2);
  msg.position_covariance[8] = pow(bestnav.hgt_std, 2);
}
}  // namespace um982_gnss

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(um982_gnss::UM982Gnss)
