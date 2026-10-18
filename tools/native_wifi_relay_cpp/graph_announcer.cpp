#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/topic_statistics_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_api/msg/request.hpp>
#include <libstatistics_collector/topic_statistics_collector/received_message_age.hpp>

#include <chrono>
#include <memory>

namespace libstatistics_collector::topic_statistics_collector
{
template<>
struct HasHeader<unitree_api::msg::Request_<std::allocator<void>>, void> : std::false_type {};
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("go2_wifi_relay");
  auto qos = rclcpp::SensorDataQoS();
  // These publishers announce the ROS graph endpoints. The native relay owns
  // the actual samples, avoiding a second serialization/copy path.
  auto cloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/utlidar/cloud", qos);
  auto imu = node->create_publisher<sensor_msgs::msg::Imu>("/utlidar/imu", qos);
  auto sport = node->create_publisher<unitree_go::msg::SportModeState>("/sportmodestate", qos);
  auto low = node->create_publisher<unitree_go::msg::LowState>("/lowstate", qos);
  rclcpp::SubscriptionOptions request_options;
  request_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;
  auto request_qos = rclcpp::QoS(rclcpp::KeepLast(4)).reliable();
  auto request_in = node->create_subscription<unitree_api::msg::Request>(
    "/go2/wifi/sport/request", request_qos,
    [](unitree_api::msg::Request::ConstSharedPtr) {}, request_options);
  auto request_out = node->create_publisher<unitree_api::msg::Request>(
    "/api/sport/request", request_qos);
  (void)cloud;
  (void)imu;
  (void)sport;
  (void)low;
  (void)request_in;
  (void)request_out;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
