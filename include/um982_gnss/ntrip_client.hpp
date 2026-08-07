// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__NTRIP_CLIENT_HPP_
#define UM982_GNSS__NTRIP_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "um982_gnss/tcp.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "bondcpp/bond.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <string>

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace um982_gnss
{
class NtripClient : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit NtripClient(const rclcpp::NodeOptions& options);

  ~NtripClient() override
  {
    close_tcp();
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
  void close_tcp();
  void init_thread_callback();
  void request_connection_attempt();
  void schedule_reconnect();
  void reset_rtcm_parser();
  void rtcm_watchdog_callback();
  void reconnect_timer_callback();
  void reconnect_pause_timer_callback();
  void parse_rtcm(uint8_t data);
  void callback(const std::vector<uint8_t>& data);
  std::string base64_encode(const std::string& in);
  uint32_t crc24q(const mavros_msgs::msg::RTCM& msg);
  void gga_callback(const nmea_msgs::msg::Sentence::SharedPtr msg);

private:
  enum eRTCM
  {
    PREAMBLE,
    HEADER,
    PAYLOAD,
    CRC
  } state_{ PREAMBLE };

  std::string host_{ "127.0.0.1" };
  int port_{ 2101 };
  bool authenticate_{ false };
  std::string mountpoint_{ "" };
  std::string username_{ "" };
  std::string password_{ "" };
  std::string frame_id_{ "odom" };
  int reconnect_attempt_max_{ 5 };
  double rtcm_timeout_{ 15.0 };
  double reconnect_delay_{ 5.0 };
  double reconnect_pause_{ 120.0 };

  TCP tcp_;
  std::thread init_thread_;
  std::atomic<bool> initialized_{ false };
  std::atomic<bool> stop_thread_{ true };
  // The counter is cleared only after a complete RTCM frame proves the connection is usable.
  std::atomic<int> reconnect_attempts_{ 0 };
  std::atomic<bool> reconnect_requested_{ false };
  std::condition_variable reconnect_condition_;
  std::mutex reconnect_mutex_;
  std::vector<uint8_t> crc_;
  std::unique_ptr<bond::Bond> bond_;

  mavros_msgs::msg::RTCM rtcm_msg_;
  rclcpp_lifecycle::LifecyclePublisher<mavros_msgs::msg::RTCM>::SharedPtr rtcm_pub_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr gga_sub_;
  // Dedicated wall timers make watchdog, retry delay and cooldown independent.
  rclcpp::TimerBase::SharedPtr rtcm_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_pause_timer_;
};
}  // namespace um982_gnss

#endif  // UM982_GNSS__NTRIP_CLIENT_HPP_
