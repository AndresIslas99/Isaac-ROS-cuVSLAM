#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>

namespace agv_slam
{

class PipelineWatchdogNode : public rclcpp::Node
{
public:
  explicit PipelineWatchdogNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PipelineWatchdogNode();

private:
  // Monitor callbacks
  void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // Watchdog timer callback
  void watchdog_check();

  // Publish heartbeat
  void publish_heartbeat();

  // Log performance metrics to disk
  void log_metrics();

  // SIGINT handler setup
  void setup_signal_handler();

  // Subscribers
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr heartbeat_pub_;

  // Timers
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  // Node health tracking
  struct NodeHealth {
    std::chrono::steady_clock::time_point last_seen;
    bool alive = false;
    uint64_t message_count = 0;
    double last_rate_hz = 0.0;
  };

  std::unordered_map<std::string, NodeHealth> node_health_;
  std::mutex health_mutex_;

  // Parameters
  std::string log_directory_;
  double heartbeat_rate_;
  double node_timeout_sec_;

  // Metrics log file
  std::ofstream metrics_log_;

  // Pipeline state
  bool pipeline_healthy_ = false;
  uint64_t restart_count_ = 0;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace agv_slam
