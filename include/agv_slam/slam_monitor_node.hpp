#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <mutex>

namespace agv_slam
{

class SlamMonitorNode : public rclcpp::Node
{
public:
  explicit SlamMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void publish_diagnostics();

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Rate tracking
  struct RateTracker {
    std::chrono::steady_clock::time_point last_time;
    uint64_t count = 0;
    double hz = 0.0;
    std::mutex mtx;

    void tick() {
      std::lock_guard<std::mutex> lock(mtx);
      auto now = std::chrono::steady_clock::now();
      count++;
      if (count > 1) {
        double dt = std::chrono::duration<double>(now - last_time).count();
        hz = 0.9 * hz + 0.1 * (1.0 / dt);  // Exponential moving average
      }
      last_time = now;
    }

    double get_hz() {
      std::lock_guard<std::mutex> lock(mtx);
      return hz;
    }
  };

  RateTracker rgb_rate_;
  RateTracker depth_rate_;
  RateTracker imu_rate_;
  RateTracker odom_rate_;

  // Odom tracking
  double total_distance_ = 0.0;
  double last_x_ = 0.0, last_y_ = 0.0;
  bool first_odom_ = true;
};

}  // namespace agv_slam
