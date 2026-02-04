#pragma once

#include "agv_slam/rate_tracker.hpp"
#include "agv_slam/tegrastats_parser.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <atomic>
#include <string>
#include <memory>
#include <sys/statvfs.h>

namespace agv_slam
{

/// Threshold configuration for 3-tier alerting.
struct ThresholdConfig
{
  double expected;
  double yellow;
  double red;

  /// For rates: higher is better
  uint8_t classify(double value) const
  {
    if (value >= yellow) return 0;  // OK
    if (value >= red) return 1;     // WARN
    return 2;                       // ERROR
  }

  /// For upper-bound metrics: lower is better
  uint8_t classify_upper(double value) const
  {
    if (value <= expected) return 0;
    if (value <= yellow) return 1;
    return 2;
  }

  static std::string level_to_color(uint8_t level)
  {
    switch (level) {
      case 0: return "green";
      case 1: return "yellow";
      case 2: return "red";
      default: return "unknown";
    }
  }
};

/// Data quality metrics aggregated from cuVSLAM status and depth filter.
struct DataQuality
{
  std::mutex mtx;
  double cuvslam_confidence = 0.0;
  int vo_state = 0;                  // 0=unknown, 1=success, 2=failed
  double track_latency_ms = 0.0;
  double depth_valid_ratio = 0.0;
  double depth_mean = 0.0;
  double depth_filter_latency_ms = 0.0;
  double velocity_mps = 0.0;
  double rotation_dps = 0.0;
};

/// Disk I/O metrics.
struct DiskStats
{
  double free_gb = 0.0;
  double total_gb = 0.0;
  double write_rate_mbps = 0.0;
  double estimated_minutes_remaining = 9999.0;
  bool is_critical = false;
};

class SlamMonitorNode : public rclcpp::Node
{
public:
  explicit SlamMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SlamMonitorNode();

private:
  // Topic callbacks
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void nvblox_mesh_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr & msg);
  void coverage_grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void zed_raw_rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void zed_raw_depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // New monitoring callbacks
  void depth_filter_latency_callback(const std_msgs::msg::Float32::ConstSharedPtr & msg);
  void depth_filter_quality_callback(const std_msgs::msg::String::ConstSharedPtr & msg);

  void publish_diagnostics();

  // tegrastats
  void tegrastats_thread_func();
  void parse_tegrastats_line(const std::string & line);

  // Disk monitoring
  void update_disk_stats();

  // ── Subscribers ──
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr nvblox_mesh_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr coverage_grid_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr zed_raw_rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr zed_raw_depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr depth_filter_latency_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr depth_filter_quality_sub_;

  // ── Publishers ──
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr quality_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // ── TF ──
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Rate trackers ──
  RateTracker rgb_rate_{15.0};
  RateTracker depth_rate_{15.0};
  RateTracker imu_rate_{0.0};  // IMU publishes in bursts, disable drop detection
  RateTracker odom_rate_{15.0};
  RateTracker nvblox_mesh_rate_{2.0};
  RateTracker coverage_grid_rate_{0.5};
  RateTracker zed_raw_rgb_rate_{15.0};
  RateTracker zed_raw_depth_rate_{15.0};

  // ── Thresholds ──
  std::map<std::string, ThresholdConfig> rate_thresholds_;
  ThresholdConfig frame_drop_threshold_;
  std::map<std::string, ThresholdConfig> system_thresholds_;
  ThresholdConfig storage_threshold_;
  ThresholdConfig depth_valid_ratio_threshold_;
  ThresholdConfig tracking_confidence_threshold_;
  ThresholdConfig depth_filter_latency_threshold_;
  ThresholdConfig cuvslam_latency_threshold_;
  double tf_max_age_ms_ = 100.0;

  // ── Data quality ──
  DataQuality data_quality_;

  // ── Odom tracking ──
  double total_distance_ = 0.0;
  double last_x_ = 0.0, last_y_ = 0.0;
  bool first_odom_ = true;
  std::mutex odom_mtx_;
  std::optional<geometry_msgs::msg::Pose> prev_odom_pose_;
  rclcpp::Time prev_odom_stamp_;

  // ── tegrastats (background pipe) ──
  struct TegraData {
    std::mutex mtx;
    double gpu_pct = 0.0;
    double gpu_temp_c = 0.0;
    double cpu_temp_c = 0.0;
    double tj_temp_c = 0.0;
    double ram_used_mb = 0.0;
    double ram_total_mb = 0.0;
    double swap_used_mb = 0.0;
    double swap_total_mb = 0.0;
    double cpu_avg_pct = 0.0;
    double cpu_max_core_pct = 0.0;
    double vdd_gpu_soc_mw = 0.0;
    double vdd_cpu_cv_mw = 0.0;
    double vin_sys_mw = 0.0;
    double gpu_clock_mhz = 0.0;
    std::vector<double> per_core_pct;
    bool valid = false;
  } tegra_;

  std::thread tegra_thread_;
  std::atomic<bool> tegra_running_{false};
  FILE * tegra_pipe_ = nullptr;

  // ── Disk I/O ──
  DiskStats disk_stats_;
  std::mutex disk_mtx_;
  uint64_t last_sectors_written_ = 0;
  std::chrono::steady_clock::time_point last_disk_check_;
};

}  // namespace agv_slam
