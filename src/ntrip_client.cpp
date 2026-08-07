// Copyright 2026 SABI AGRI

#include "um982_gnss/ntrip_client.hpp"

#include <chrono>

using std::placeholders::_1;

/*
RTCM3 transport layer bit format:
+-------+--------+--------+--------+----------------+--------+
| 0xd3  | 000000 | length |  type  |    content     |  crc   |
+-------+--------+--------+--------+----------------+--------+
|<- 8 ->|<- 6 -->|<- 10 ->|<- 12 ->|<-- variable -->|<- 24 ->|
|                         |<- payload; length x 8 ->|        |
*/

static const uint32_t crc24qtab[256] = {
  0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17, 0xA18139, 0x27CDC2, 0x2B5434,
  0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493,
  0xDC7D65, 0x5A319E, 0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7, 0x0CD1E9,
  0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE, 0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926,
  0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7, 0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C,
  0x56E077, 0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E, 0x19A3D2, 0x9FEF29,
  0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5, 0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A,
  0xAD88F1, 0xA11107, 0x275DFC, 0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
  0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375, 0x15723B, 0x933EC0, 0x9FA736,
  0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C, 0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918,
  0xADC0EE, 0x2B8C15, 0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5, 0x71BD8B,
  0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C, 0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52,
  0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571,
  0x0DB98A, 0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A, 0x578814, 0xD1C4EF,
  0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703, 0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC,
  0x2A3C57, 0x26A5A1, 0xA0E95A, 0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
  0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506, 0xD18CF0,
  0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C,
  0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58, 0xEFAAFF,
  0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8, 0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230,
  0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1, 0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673,
  0xB94A88, 0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED,
  0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401, 0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE,
  0x575035, 0x5BC9C3, 0xDD8538
};

namespace um982_gnss
{
NtripClient::NtripClient(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("ntrip_client", options)
{
  this->declare_parameter("host", host_);
  this->declare_parameter("port", port_);
  this->declare_parameter("authenticate", authenticate_);
  this->declare_parameter("mountpoint", mountpoint_);
  this->declare_parameter("username", username_);
  this->declare_parameter("password", password_);
  this->declare_parameter("frame_id", frame_id_);
  this->declare_parameter("rtcm_timeout", rtcm_timeout_);
  this->declare_parameter("reconnect_attempt_max", reconnect_attempt_max_);
  this->declare_parameter("reconnect_delay", reconnect_delay_);
  this->declare_parameter("reconnect_pause", reconnect_pause_);
}

LNI::CallbackReturn NtripClient::on_configure(const rclcpp_lifecycle::State&)
{
  this->get_parameter("host", host_);
  this->get_parameter("port", port_);
  this->get_parameter("authenticate", authenticate_);
  this->get_parameter("mountpoint", mountpoint_);
  this->get_parameter("username", username_);
  this->get_parameter("password", password_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("rtcm_timeout", rtcm_timeout_);
  this->get_parameter("reconnect_attempt_max", reconnect_attempt_max_);
  this->get_parameter("reconnect_delay", reconnect_delay_);
  this->get_parameter("reconnect_pause", reconnect_pause_);

  // Every timer period and the number of attempts must define a usable retry policy.
  if (rtcm_timeout_ <= 0.0 || reconnect_attempt_max_ <= 0 || reconnect_delay_ <= 0.0 || reconnect_pause_ <= 0.0)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameters 'rtcm_timeout', 'reconnect_attempt_max', 'reconnect_delay' and 'reconnect_pause' must be "
                 "greater than zero");
    return LNI::CallbackReturn::FAILURE;
  }

  rtcm_msg_.header.frame_id = frame_id_;
  rtcm_pub_ = this->create_publisher<mavros_msgs::msg::RTCM>("rtcm", 10);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn NtripClient::on_activate(const rclcpp_lifecycle::State& state)
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

  gga_sub_ =
      this->create_subscription<nmea_msgs::msg::Sentence>("nmea", 10, std::bind(&NtripClient::gga_callback, this, _1));

  // Each delay has its own wall timer; callbacks cancel them to provide one-shot behavior.
  rtcm_watchdog_timer_ = this->create_wall_timer(std::chrono::duration<double>(rtcm_timeout_),
                                                 std::bind(&NtripClient::rtcm_watchdog_callback, this));
  reconnect_timer_ = this->create_wall_timer(std::chrono::duration<double>(reconnect_delay_),
                                             std::bind(&NtripClient::reconnect_timer_callback, this));
  reconnect_pause_timer_ = this->create_wall_timer(std::chrono::duration<double>(reconnect_pause_),
                                                   std::bind(&NtripClient::reconnect_pause_timer_callback, this));
  rtcm_watchdog_timer_->cancel();
  reconnect_timer_->cancel();
  reconnect_pause_timer_->cancel();

  initialized_ = false;
  stop_thread_ = false;
  reconnect_attempts_ = 0;
  reconnect_requested_ = false;
  init_thread_ = std::thread(&NtripClient::init_thread_callback, this);
  request_connection_attempt();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn NtripClient::on_deactivate(const rclcpp_lifecycle::State& state)
{
  LifecycleNode::on_deactivate(state);

  // Stop network threads before destroying timers they can reset.
  close_tcp();
  rtcm_watchdog_timer_.reset();
  reconnect_timer_.reset();
  reconnect_pause_timer_.reset();
  bond_->breakBond();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn NtripClient::on_cleanup(const rclcpp_lifecycle::State&)
{
  close_tcp();
  rtcm_watchdog_timer_.reset();
  reconnect_timer_.reset();
  reconnect_pause_timer_.reset();

  rtcm_pub_.reset();
  gga_sub_.reset();
  bond_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn NtripClient::on_shutdown(const rclcpp_lifecycle::State&)
{
  close_tcp();
  rtcm_watchdog_timer_.reset();
  reconnect_timer_.reset();
  reconnect_pause_timer_.reset();

  rtcm_pub_.reset();
  gga_sub_.reset();
  bond_.reset();

  return LNI::CallbackReturn::SUCCESS;
}

void NtripClient::close_tcp()
{
  initialized_ = false;
  stop_thread_ = true;
  reconnect_condition_.notify_all();

  if (init_thread_.joinable())
  {
    init_thread_.join();
  }

  tcp_.close();
}

void NtripClient::init_thread_callback()
{
  while (!stop_thread_)
  {
    {
      // Wall-timer callbacks wake the worker only when an attempt is due.
      std::unique_lock<std::mutex> lock(reconnect_mutex_);
      reconnect_condition_.wait(lock, [this]() { return stop_thread_ || reconnect_requested_; });
    }

    if (stop_thread_)
    {
      break;
    }

    reconnect_requested_ = false;

    const int reconnect_attempt = ++reconnect_attempts_;
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "NTRIP connection attempt " << reconnect_attempt << "/" << reconnect_attempt_max_);

    // Joining the previous receiver prevents it from modifying the parser during its reset.
    tcp_.close();
    reset_rtcm_parser();
    if (!tcp_.open(port_, host_, &NtripClient::callback, this))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to connect socket to server at http://" << host_ << ":" << port_);
      schedule_reconnect();
      continue;
    }

    if (stop_thread_)
    {
      tcp_.close();
      break;
    }

    std::string request;

    if (authenticate_)
    {
      std::string auth = base64_encode(username_ + ":" + password_);
      request = "GET /" + mountpoint_ + " HTTP/1.0\r\n" + "User-Agent: NTRIP ROS2Client\r\n" + "Authorization: Basic " +
                auth + "\r\n" + "Connection: close\r\n\r\n";
    }
    else
    {
      request =
          "GET /" + mountpoint_ + " HTTP/1.0\r\n" + "User-Agent: NTRIP ROS2Client\r\n" + "Connection: close\r\n\r\n";
    }

    if (!tcp_.send(request))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to send request to server at http://" << host_ << ":" << port_);
      tcp_.close();
      schedule_reconnect();
      continue;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Connected to http://" << host_ << ":" << port_ << "/" << mountpoint_);

    // Start the watchdog from connection establishment to allow the first RTCM frame to arrive.
    initialized_ = true;
    rtcm_watchdog_timer_->reset();
  }
}

void NtripClient::request_connection_attempt()
{
  if (stop_thread_)
  {
    return;
  }

  // The atomic flag prevents concurrent timer callbacks from queuing duplicate attempts.
  if (!reconnect_requested_.exchange(true))
  {
    reconnect_condition_.notify_one();
  }
}

void NtripClient::schedule_reconnect()
{
  if (stop_thread_)
  {
    return;
  }

  rtcm_watchdog_timer_->cancel();
  const bool batch_exhausted = reconnect_attempts_ >= reconnect_attempt_max_;

  if (batch_exhausted)
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "No valid NTRIP connection after " << reconnect_attempt_max_ << " attempts; retrying in "
                                                          << reconnect_pause_ << " seconds");
    reconnect_timer_->cancel();
    reconnect_pause_timer_->reset();
  }
  else
  {
    // Retry once after the configured delay; the callback cancels this periodic timer.
    reconnect_timer_->reset();
  }
}

void NtripClient::reset_rtcm_parser()
{
  // Discard any partial frame left by the previous TCP stream.
  rtcm_msg_.data.clear();
  crc_.clear();
  state_ = PREAMBLE;
}

void NtripClient::rtcm_watchdog_callback()
{
  // create_wall_timer is periodic, so cancel it to implement a resettable one-shot watchdog.
  rtcm_watchdog_timer_->cancel();

  if (initialized_.exchange(false))
  {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "No valid RTCM message received for " << rtcm_timeout_ << " seconds; reconnecting to NTRIP");
    schedule_reconnect();
  }
}

void NtripClient::reconnect_timer_callback()
{
  // A retry timer represents exactly one attempt.
  reconnect_timer_->cancel();
  request_connection_attempt();
}

void NtripClient::reconnect_pause_timer_callback()
{
  // The cooldown opens a new batch of five consecutive attempts.
  reconnect_pause_timer_->cancel();

  if (stop_thread_)
  {
    return;
  }

  reconnect_attempts_ = 0;
  request_connection_attempt();
}

void NtripClient::parse_rtcm(uint8_t data)
{
  switch (state_)
  {
    case PREAMBLE: {
      if (data == 0xD3)
      {
        rtcm_msg_.header.stamp = this->now();
        rtcm_msg_.data.clear();
        rtcm_msg_.data.push_back(data);
        state_ = HEADER;
      }

      break;
    }

    case HEADER: {
      if ((data & ~0x03) == 0)
      {
        rtcm_msg_.data.push_back(data);
        state_ = PAYLOAD;
      }
      else
      {
        state_ = PREAMBLE;
      }

      break;
    }

    case PAYLOAD: {
      rtcm_msg_.data.push_back(data);

      uint16_t length = (rtcm_msg_.data[1] << 8) | rtcm_msg_.data[2];

      if (rtcm_msg_.data.size() == size_t(length + 3))
      {
        state_ = CRC;
      }

      break;
    }

    case CRC: {
      crc_.push_back(data);

      if (crc_.size() == 3)
      {
        uint32_t actual_crc = (crc_[0] << 16) | (crc_[1] << 8) | crc_[2];
        uint32_t expected_crc = crc24q(rtcm_msg_);

        if (actual_crc == expected_crc)
        {
          rtcm_msg_.data.push_back(crc_[0]);
          rtcm_msg_.data.push_back(crc_[1]);
          rtcm_msg_.data.push_back(crc_[2]);
          rtcm_pub_->publish(rtcm_msg_);
          // Only a complete frame with a valid CRC proves that the correction stream is healthy.
          reconnect_attempts_ = 0;
          rtcm_watchdog_timer_->reset();
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "RTCM3 message invalid - failed CRC");
        }

        crc_.clear();
        state_ = PREAMBLE;
      }

      break;
    }
  }
}

void NtripClient::callback(const std::vector<uint8_t>& data)
{
  if (!initialized_)
  {
    return;
  }

  if (data.size() == 0)
  {
    if (initialized_.exchange(false))
    {
      rtcm_msg_.data.clear();
      RCLCPP_WARN(this->get_logger(), "Connection closed by server");
      schedule_reconnect();
    }
    return;
  }

  for (uint8_t d : data)
  {
    parse_rtcm(d);
  }

  if (rtcm_msg_.data.empty())
  {
    std::string str(data.begin(), data.end());
    RCLCPP_INFO_STREAM(this->get_logger(), str);
  }
}

std::string NtripClient::base64_encode(const std::string& in)
{
  static const char lookup[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string out;
  int val = 0, valb = -6;

  for (unsigned char c : in)
  {
    val = (val << 8) + c;
    valb += 8;

    while (valb >= 0)
    {
      out.push_back(lookup[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }

  if (valb > -6)
  {
    out.push_back(lookup[((val << 8) >> (valb + 8)) & 0x3F]);
  }

  while (out.size() % 4)
  {
    out.push_back('=');
  }

  return out;
}

uint32_t NtripClient::crc24q(const mavros_msgs::msg::RTCM& msg)
{
  uint32_t crc = 0;

  for (uint8_t d : msg.data)
  {
    crc = ((crc << 8) & 0xFFFFFF) ^ crc24qtab[uint8_t(crc >> 16) ^ d];
  }

  return crc;
}

void NtripClient::gga_callback(const nmea_msgs::msg::Sentence::SharedPtr msg)
{
  if (!initialized_)
  {
    return;
  }

  if (!tcp_.send(msg->sentence + "\r\n") && initialized_.exchange(false))
  {
    // A failed GGA write is another reliable indication that the socket must be reopened.
    RCLCPP_WARN(this->get_logger(), "Unable to send GGA sentence; reconnecting to NTRIP");
    schedule_reconnect();
  }
}
}  // namespace um982_gnss

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(um982_gnss::NtripClient)
