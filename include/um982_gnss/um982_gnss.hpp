// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__UM982_GNSS_HPP_
#define UM982_GNSS__UM982_GNSS_HPP_

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

private:
  std::string device_{ "/dev/ttyUSB0" };
  int baudrate_{ 115200 };
  std::string frame_main_{ "gnss_main" };
  std::string frame_aux_{ "gnss_aux" };
  int heading_length_{ 100 };
  int heading_tolerance_{ 3 };
  int heading_offset_{ 0 };
  int heading_pitch_offset_{ 0 };

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

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_main_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsatfix_aux_pub_;
  rclcpp_lifecycle::LifecyclePublisher<gps_msgs::msg::GPSFix>::SharedPtr gpsfix_main_pub_;
  rclcpp_lifecycle::LifecyclePublisher<gps_msgs::msg::GPSFix>::SharedPtr gpsfix_aux_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nmea_msgs::msg::Sentence>::SharedPtr gga_pub_;
  rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr rtcm_sub_;
};
}  // namespace um982_gnss

#endif  // UM982_GNSS__UM982_GNSS_HPP_
