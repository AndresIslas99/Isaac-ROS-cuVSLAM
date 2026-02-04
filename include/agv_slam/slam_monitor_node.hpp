#pragma once

#include "agv_slam/rate_tracker.hpp"
#include "agv_slam/tegrastats_parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>

namespace agv_slam
{

class SlamMonitorNode : public rclcpp::Node
{
public:
  explicit SlamMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SlamMonitorNode();

private:
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void publish_diagnostics();

  // tegrastats
  void tegrastats_thread_func();
  void parse_tegrastats_line(const std::string & line);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Rate tracking (see agv_slam/rate_tracker.hpp)
  RateTracker rgb_rate_;
  RateTracker depth_rate_;
  RateTracker imu_rate_;
  RateTracker odom_rate_;

  // Odom tracking
  double total_distance_ = 0.0;
  double last_x_ = 0.0, last_y_ = 0.0;
  bool first_odom_ = true;

  // tegrastats data (protected by mutex)
  struct TegraStats {
    std::mutex mtx;
    double gpu_freq_pct = 0.0;     // GR3D_FREQ percentage
    double gpu_temp_c = 0.0;       // gpu@XX.XC
    double cpu_temp_c = 0.0;       // cpu@XX.XC
    double tj_temp_c = 0.0;        // tj@XX.XC (junction max)
    double ram_used_mb = 0.0;
    double ram_total_mb = 0.0;
    double vdd_gpu_soc_mw = 0.0;   // VDD_GPU_SOC
    double vdd_cpu_cv_mw = 0.0;    // VDD_CPU_CV
    double vin_sys_mw = 0.0;       // VIN_SYS_5V0 (total)
    double cpu_avg_pct = 0.0;      // Average CPU utilization
    bool valid = false;
  } tegra_;

  std::thread tegra_thread_;
  std::atomic<bool> tegra_running_{false};
  FILE * tegra_pipe_ = nullptr;
};

}  // namespace agv_slam
