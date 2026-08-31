// Copyright 2026 SABI AGRI

#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
#include <ctime>

#include "um982_gnss/um982_gnss.hpp"
#include "um982_gnss/crc.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace um982_gnss
{
namespace
{
/// \brief Human readable Unicore position type, kept verbatim in diagnostics so a field
/// log can be read without the protocol reference at hand.
const char* pos_type_label(uint32_t pos_type)
{
  switch (pos_type)
  {
    case NONE:
      return "NONE";
    case FIXEDPOS:
      return "FIXEDPOS";
    case FIXEHEIGHT:
      return "FIXEDHEIGHT";
    case DOPPLER_VELOCITY:
      return "DOPPLER_VELOCITY";
    case SINGLE:
      return "SINGLE";
    case PSRDIFF:
      return "PSRDIFF";
    case SBAS:
      return "SBAS";
    case L1_FLOAT:
      return "L1_FLOAT";
    case IONOFREE_FLOAT:
      return "IONOFREE_FLOAT";
    case NARROW_FLOAT:
      return "NARROW_FLOAT";
    case L1_INT:
      return "L1_INT";
    case WIDE_INT:
      return "WIDE_INT";
    case NARROW_INT:
      return "NARROW_INT";
    default:
      return "OTHER";
  }
}

bool is_rtk_fixed(uint32_t pos_type)
{
  return pos_type == L1_INT || pos_type == WIDE_INT || pos_type == NARROW_INT;
}

std::string to_str(double value, int digits)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.*f", digits, value);
  return std::string(buffer);
}

void add_kv(diagnostic_msgs::msg::DiagnosticStatus& status, const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

/// \brief mkdir -p, so a raw dump directory can be given as a nested path.
bool make_directories(const std::string& path)
{
  if (path.empty())
  {
    return false;
  }

  for (size_t i = 1; i <= path.size(); i++)
  {
    if (i != path.size() && path[i] != '/')
    {
      continue;
    }

    std::string part = path.substr(0, i);

    if (mkdir(part.c_str(), 0775) != 0 && errno != EEXIST)
    {
      return false;
    }
  }

  return true;
}
}  // namespace

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
  this->declare_parameter("extra_logs", extra_logs_);
  this->declare_parameter("diagnostics.enable", diagnostics_enable_);
  this->declare_parameter("diagnostics.rate", diagnostics_rate_);
  this->declare_parameter("diagnostics.heading_std_warn", heading_std_warn_);
  this->declare_parameter("raw_dump.enable", raw_dump_enable_);
  this->declare_parameter("raw_dump.directory", raw_dump_dir_);
  this->declare_parameter("raw_dump.max_mb", raw_dump_max_mb_);
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
  this->get_parameter("extra_logs", extra_logs_);
  this->get_parameter("diagnostics.enable", diagnostics_enable_);
  this->get_parameter("diagnostics.rate", diagnostics_rate_);
  this->get_parameter("diagnostics.heading_std_warn", heading_std_warn_);
  this->get_parameter("raw_dump.enable", raw_dump_enable_);
  this->get_parameter("raw_dump.directory", raw_dump_dir_);
  this->get_parameter("raw_dump.max_mb", raw_dump_max_mb_);

  navsatfix_main_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix/main", 10);
  navsatfix_aux_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsatfix/aux", 10);
  gpsfix_main_pub_ = this->create_publisher<gps_msgs::msg::GPSFix>("gpsfix/main", 10);
  gpsfix_aux_pub_ = this->create_publisher<gps_msgs::msg::GPSFix>("gpsfix/aux", 10);
  gga_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea", 10);
  heading_diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("heading_diagnostics", 10);

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

  aux_fix_seen_ = false;
  aux_fix_lost_ = false;
  aux_fix_loss_count_ = 0;
  aux_fix_lost_s_ = 0.0;
  heading_fix_seen_ = false;
  heading_fix_lost_ = false;
  heading_loss_count_ = 0;
  heading_lost_s_ = 0.0;
  heading_length_seen_ = false;
  uniheading_last_ = std::chrono::steady_clock::time_point{};
  diagnostics_last_ = std::chrono::steady_clock::time_point{};

  open_raw_dump();

  init_thread_ = std::thread(&UM982Gnss::init_thread_callback, this);

  timer_ = this->create_wall_timer(2min, std::bind(&UM982Gnss::timer_callback, this));

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
  heading_diag_pub_.reset();
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
  heading_diag_pub_.reset();
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

  // Safe here only: the serial thread is joined, so dump_raw() can no longer run.
  close_raw_dump();
}

void UM982Gnss::timer_callback()
{
  RCLCPP_WARN(this->get_logger(), "Restarting the receiver...");
  initialized_ = false;
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

    // Investigation logs, requested after navigation is already configured and never
    // gating on success: an unknown or unsupported log name must not keep the receiver
    // from being usable.
    for (const std::string& extra : extra_logs_)
    {
      if (command(extra))
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Extra log enabled: " << extra);
      }
      else
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "Extra log refused by the receiver: " << extra);
      }
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

    const bool aux_has_fix = navsatfix_msg.status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;

    if (aux_has_fix)
    {
      rtk_fix_ |= (1 << 1);
    }
    else
    {
      rtk_fix_ &= ~(1 << 1);
    }

    navsatfix_aux_pub_->publish(navsatfix_msg);
    gpsfix_aux_pub_->publish(gpsfix_msg);

    // BESTNAVH is the fastest message that always arrives (20 Hz), even while the heading
    // solution is lost, so instrumentation is driven from here.
    update_aux_fix_stats(aux_has_fix);
    publish_heading_diagnostics();
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
    update_heading_stats();
  }
}

void UM982Gnss::update_aux_fix_stats(bool aux_has_fix)
{
  const auto now = std::chrono::steady_clock::now();

  if (aux_has_fix)
  {
    if (aux_fix_lost_)
    {
      aux_fix_lost_ = false;
      aux_fix_lost_s_ += std::chrono::duration<double>(now - aux_fix_lost_since_).count();
    }

    aux_fix_seen_ = true;
    return;
  }

  // Only count losses after a first fix: the acquisition phase at startup is not a loss.
  if (aux_fix_seen_ && !aux_fix_lost_)
  {
    aux_fix_lost_ = true;
    aux_fix_lost_since_ = now;
    aux_fix_loss_count_++;
    RCLCPP_WARN(this->get_logger(),
                "Auxiliary antenna lost RTK fix (%s), heading solution %s; loss #%u. This stops "
                "odometry downstream while the main antenna is still fixed.",
                pos_type_label(bestnav_aux_.pos_type), pos_type_label(uniheading_.pos_type),
                static_cast<unsigned>(aux_fix_loss_count_));
  }
}

void UM982Gnss::update_heading_stats()
{
  const auto now = std::chrono::steady_clock::now();
  uniheading_last_ = now;

  const bool heading_fixed = is_rtk_fixed(uniheading_.pos_type);

  if (heading_fixed)
  {
    if (heading_fix_lost_)
    {
      heading_fix_lost_ = false;
      heading_lost_s_ += std::chrono::duration<double>(now - heading_lost_since_).count();
    }

    heading_fix_seen_ = true;

    // Baseline length is only meaningful on a fixed solution.
    if (!heading_length_seen_)
    {
      heading_length_seen_ = true;
      heading_length_min_ = uniheading_.length;
      heading_length_max_ = uniheading_.length;
    }
    else
    {
      heading_length_min_ = std::min(heading_length_min_, uniheading_.length);
      heading_length_max_ = std::max(heading_length_max_, uniheading_.length);
    }

    return;
  }

  if (heading_fix_seen_ && !heading_fix_lost_)
  {
    heading_fix_lost_ = true;
    heading_lost_since_ = now;
    heading_loss_count_++;
  }
}

void UM982Gnss::publish_heading_diagnostics()
{
  if (!diagnostics_enable_ || !heading_diag_pub_ || diagnostics_rate_ <= 0.0)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(1.0 / diagnostics_rate_);

  if (diagnostics_last_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - diagnostics_last_) < period)
  {
    return;
  }

  diagnostics_last_ = now;

  const bool heading_fixed = is_rtk_fixed(uniheading_.pos_type);
  const double uniheading_age =
      uniheading_last_.time_since_epoch().count() == 0
          ? -1.0
          : std::chrono::duration<double>(now - uniheading_last_).count();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "gnss dual antenna heading";
  status.hardware_id = device_;

  if (!heading_fixed)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "heading baseline not fixed";
  }
  else if (uniheading_.hdg_std_dev > heading_std_warn_)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "heading fixed but noisy";
  }
  else
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "heading fixed";
  }

  // Heading solution (baseline main -> aux, constrained by CONFIG HEADING LENGTH).
  add_kv(status, "heading.pos_type", pos_type_label(uniheading_.pos_type));
  add_kv(status, "heading.sol_stat", std::to_string(uniheading_.sol_stat));
  add_kv(status, "heading.deg", to_str(uniheading_.heading, 2));
  add_kv(status, "heading.pitch_deg", to_str(uniheading_.pitch, 2));
  add_kv(status, "heading.std_dev_deg", to_str(uniheading_.hdg_std_dev, 2));
  add_kv(status, "heading.baseline_m", to_str(uniheading_.length, 3));
  add_kv(status, "heading.age_s", to_str(uniheading_age, 2));

  // The pair that separates a radio-frequency problem from a solver problem: satellites
  // *tracked* by the antenna versus satellites *used* in the solution. Tracking high and
  // usage low means the antenna receives fine and the ambiguity resolution is what fails.
  add_kv(status, "heading.sat_tracked", std::to_string(uniheading_.sat_nb));
  add_kv(status, "heading.sat_used", std::to_string(uniheading_.sol_sat_nb));
  add_kv(status, "main.pos_type", pos_type_label(bestnav_main_.pos_type));
  add_kv(status, "main.sat_tracked", std::to_string(bestnav_main_.sat_nb));
  add_kv(status, "main.sat_used", std::to_string(bestnav_main_.sol_sat_nb));
  add_kv(status, "main.diff_age_s", to_str(bestnav_main_.diff_age, 1));
  add_kv(status, "aux.pos_type", pos_type_label(bestnav_aux_.pos_type));
  add_kv(status, "aux.sat_tracked", std::to_string(bestnav_aux_.sat_nb));
  add_kv(status, "aux.sat_used", std::to_string(bestnav_aux_.sol_sat_nb));
  add_kv(status, "aux.diff_age_s", to_str(bestnav_aux_.diff_age, 1));

  // Cumulative counters since activation, so a run can be judged without post-processing.
  add_kv(status, "aux.fix_loss_count", std::to_string(aux_fix_loss_count_));
  add_kv(status, "aux.fix_lost_s", to_str(aux_fix_lost_s_, 1));
  add_kv(status, "heading.loss_count", std::to_string(heading_loss_count_));
  add_kv(status, "heading.lost_s", to_str(heading_lost_s_, 1));
  add_kv(status, "heading.baseline_min_m", to_str(heading_length_seen_ ? heading_length_min_ : 0.f, 3));
  add_kv(status, "heading.baseline_max_m", to_str(heading_length_seen_ ? heading_length_max_ : 0.f, 3));
  add_kv(status, "config.baseline_cm", std::to_string(heading_length_));
  add_kv(status, "config.baseline_tolerance_cm", std::to_string(heading_tolerance_));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.header.frame_id = frame_main_;
  array.status.push_back(status);

  heading_diag_pub_->publish(array);
}

void UM982Gnss::open_raw_dump()
{
  if (!raw_dump_enable_ || raw_dump_file_ != nullptr)
  {
    return;
  }

  if (raw_dump_dir_.empty())
  {
    RCLCPP_WARN(this->get_logger(), "raw_dump.enable is set but raw_dump.directory is empty; dump disabled");
    return;
  }

  if (!make_directories(raw_dump_dir_))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot create raw dump directory: " << raw_dump_dir_);
    return;
  }

  std::time_t t = std::time(nullptr);
  char stamp[32] = { 0 };
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));

  const std::string path = raw_dump_dir_ + "/um982_raw_" + stamp + ".bin";
  raw_dump_file_ = fopen(path.c_str(), "wb");

  if (raw_dump_file_ == nullptr)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot open raw dump file: " << path);
    return;
  }

  raw_dump_bytes_ = 0;
  raw_dump_flushed_ = 0;
  raw_dump_full_ = false;
  RCLCPP_INFO_STREAM(this->get_logger(), "Raw receiver stream dumped to " << path);
}

void UM982Gnss::close_raw_dump()
{
  if (raw_dump_file_ == nullptr)
  {
    return;
  }

  fclose(raw_dump_file_);
  raw_dump_file_ = nullptr;
  RCLCPP_INFO(this->get_logger(), "Raw dump closed (%zu bytes)", raw_dump_bytes_);
}

void UM982Gnss::dump_raw(const std::vector<uint8_t>& data)
{
  if (raw_dump_file_ == nullptr || raw_dump_full_)
  {
    return;
  }

  const size_t written = fwrite(data.data(), 1, data.size(), raw_dump_file_);

  if (written != data.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Raw dump write failed, stopping the dump");
    raw_dump_full_ = true;
    return;
  }

  raw_dump_bytes_ += written;

  // Flush every 64 kB so a power loss keeps most of the capture without paying a syscall
  // on every serial chunk.
  if (raw_dump_bytes_ - raw_dump_flushed_ >= 65536u)
  {
    fflush(raw_dump_file_);
    raw_dump_flushed_ = raw_dump_bytes_;
  }

  if (raw_dump_max_mb_ > 0 && raw_dump_bytes_ >= static_cast<size_t>(raw_dump_max_mb_) * 1024u * 1024u)
  {
    raw_dump_full_ = true;
    fflush(raw_dump_file_);
    RCLCPP_WARN(this->get_logger(), "Raw dump size limit reached (%d MB), stopping the dump", raw_dump_max_mb_);
  }

  if (rtk_fix_ == 3)
  {
    timer_->reset();
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
  dump_raw(data);

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
