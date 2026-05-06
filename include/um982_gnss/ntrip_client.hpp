// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__NTRIP_CLIENT_HPP_
#define UM982_GNSS__NTRIP_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "um982_gnss/tcp.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "mavros_msgs/msg/rtcm.hpp"
#include "bondcpp/bond.hpp"

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
  } state_;

  std::string host_{ "127.0.0.1" };
  int port_{ 2101 };
  bool authenticate_{ false };
  std::string mountpoint_{ "" };
  std::string username_{ "" };
  std::string password_{ "" };
  std::string frame_id_{ "odom" };

  TCP tcp_;
  std::thread init_thread_;
  std::atomic<bool> initialized_;
  std::atomic<bool> stop_thread_;
  std::vector<uint8_t> crc_;
  std::unique_ptr<bond::Bond> bond_;

  mavros_msgs::msg::RTCM rtcm_msg_;
  rclcpp_lifecycle::LifecyclePublisher<mavros_msgs::msg::RTCM>::SharedPtr rtcm_pub_;
  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr gga_sub_;
};
}  // namespace um982_gnss

#endif  // UM982_GNSS__NTRIP_CLIENT_HPP_
