// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__UM982_GNSS_HPP_
#define UM982_GNSS__UM982_GNSS_HPP_

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "um982_gnss/bestnav.hpp"
#include "um982_gnss/gga.hpp"
#include "um982_gnss/serial.hpp"
#include "um982_gnss/stadop.hpp"
#include "um982_gnss/uniheading.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "bondcpp/bond.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace um982_gnss
{
class UM982Gnss : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit UM982Gnss(const rclcpp::NodeOptions& options);

  ~UM982Gnss() override
  {
    close_serial();
  }

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

protected:
  void close_serial();
  void timer_callback();
  void init_thread_callback();
  void parse_ascii(uint8_t data);
  void parse_binary(uint8_t data);
  void process_ascii();
  void process_binary();
  bool command(const std::string& cmd);
  void callback(const std::vector<uint8_t>& data);
  void rtcm_callback(const mavros_msgs::msg::RTCM::SharedPtr msg);
  void handle_navsatfix(sensor_msgs::msg::NavSatFix& msg, const sBestnav& bestnav);
  void handle_gpsfix(gps_msgs::msg::GPSFix& msg, const sBestnav& bestnav, const sStadop& stadop);

  // Instrumentation. Everything below runs on the serial thread only (process_binary),
  // so the counters need no locking.
  void update_heading_stats();
  void update_aux_fix_stats(bool aux_has_fix);
  void publish_heading_diagnostics();
  void open_raw_dump();
  void close_raw_dump();
  void dump_raw(const std::vector<uint8_t>& data);

private:
  std::string device_{ "/dev/ttyUSB0" };
  int baudrate_{ 115200 };
  std::string frame_main_{ "gnss_main" };
  std::string frame_aux_{ "gnss_aux" };
  int heading_length_{ 100 };
  int heading_tolerance_{ 3 };
  int heading_offset_{ 0 };
  int heading_pitch_offset_{ 0 };

  // Extra Unicore logs requested after the mandatory ones, e.g. { "OBSVMB 1", "OBSVHB 1" }
  // to investigate per-antenna satellite tracking. A rejected log is only a warning:
  // navigation must never depend on an investigation log.
  std::vector<std::string> extra_logs_;
  bool diagnostics_enable_{ true };
  double diagnostics_rate_{ 10.0 };
  double heading_std_warn_{ 2.0 };
  bool raw_dump_enable_{ false };
  std::string raw_dump_dir_;
  int raw_dump_max_mb_{ 256 };

  Serial ser_;
  sASCII ascii_;
  sBinary binary_;
  std::atomic<int8_t> response_;

  sBestnav bestnav_main_;
  sBestnav bestnav_aux_;
  sStadop stadop_main_;
  sStadop stadop_aux_;
  sUniheading uniheading_;

  std::thread init_thread_;
  std::atomic<bool> initialized_;
  std::atomic<bool> stop_thread_;
  std::atomic<uint8_t> rtk_fix_{ 0 };
  std::unique_ptr<bond::Bond> bond_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Heading / auxiliary antenna instrumentation state (serial thread only).
  // Rationale: on this platform the *auxiliary* solution is what drops, never the main
  // one, and every drop stops /loc/odom downstream. These counters make the phenomenon
  // measurable in a bag without post-processing.
  bool aux_fix_seen_{ false };
  bool aux_fix_lost_{ false };
  uint32_t aux_fix_loss_count_{ 0 };
  double aux_fix_lost_s_{ 0.0 };
  std::chrono::steady_clock::time_point aux_fix_lost_since_{};

  bool heading_fix_seen_{ false };
  bool heading_fix_lost_{ false };
  uint32_t heading_loss_count_{ 0 };
  double heading_lost_s_{ 0.0 };
  std::chrono::steady_clock::time_point heading_lost_since_{};

  // Receiver-measured baseline length: tells a mechanical/flex problem (the length moves)
  // apart from an ambiguity-resolution problem (the length stays, the fix drops).
  bool heading_length_seen_{ false };
  float heading_length_min_{ 0.f };
  float heading_length_max_{ 0.f };
  std::chrono::steady_clock::time_point uniheading_last_{};
  std::chrono::steady_clock::time_point diagnostics_last_{};

  FILE* raw_dump_file_{ nullptr };
  size_t raw_dump_bytes_{ 0 };
  size_t raw_dump_flushed_{ 0 };
  bool raw_dump_full_{ false };

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_main_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_aux_pub_;
  rclcpp_lifecycle::LifecyclePublisher<gps_msgs::msg::GPSFix>::SharedPtr gpsfix_main_pub_;
  rclcpp_lifecycle::LifecyclePublisher<gps_msgs::msg::GPSFix>::SharedPtr gpsfix_aux_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nmea_msgs::msg::Sentence>::SharedPtr gga_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr heading_diag_pub_;
  rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr rtcm_sub_;
};
}  // namespace um982_gnss

#endif  // UM982_GNSS__UM982_GNSS_HPP_
