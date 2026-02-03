#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <chrono>
#include <mutex>
#include <cmath>

namespace agv_slam
{

class MapQualityNode : public rclcpp::Node
{
public:
  explicit MapQualityNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("map_quality", options)
  {
    // ── Subscribers ──
    auto qos = rclcpp::SensorDataQoS();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rtabmap/cloud_map", qos,
      std::bind(&MapQualityNode::cloud_callback, this, std::placeholders::_1));

    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/rtabmap/grid_map", qos,
      std::bind(&MapQualityNode::grid_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/visual_slam/tracking/odometry", qos,
      std::bind(&MapQualityNode::odom_callback, this, std::placeholders::_1));

    // ── Publishers ──
    quality_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/slam/quality", 10);

    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/slam/quality_diagnostics", 10);

    // ── Timer: compute quality at 0.2 Hz ──
    quality_timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&MapQualityNode::compute_quality, this));

    RCLCPP_INFO(this->get_logger(), "Map quality node initialized");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cloud_point_count_ = msg->width * msg->height;
    last_cloud_time_ = std::chrono::steady_clock::now();
    cloud_received_ = true;

    // Compute point density from cloud bounds
    if (cloud_point_count_ > 0) {
      float min_x = 1e9f, max_x = -1e9f;
      float min_y = 1e9f, max_y = -1e9f;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

      uint32_t sample_count = 0;
      uint32_t step = std::max(1u, cloud_point_count_ / 1000u);  // Sample ~1000 points

      for (uint32_t i = 0; iter_x != iter_x.end() && i < cloud_point_count_;
           ++iter_x, ++iter_y, ++i)
      {
        if (i % step != 0) continue;
        float x = *iter_x;
        float y = *iter_y;
        if (std::isfinite(x) && std::isfinite(y)) {
          min_x = std::min(min_x, x);
          max_x = std::max(max_x, x);
          min_y = std::min(min_y, y);
          max_y = std::max(max_y, y);
          sample_count++;
        }
      }

      if (sample_count > 10) {
        float area = (max_x - min_x) * (max_y - min_y);
        if (area > 0.01f) {
          point_density_ = static_cast<float>(cloud_point_count_) / area;
        }
      }
    }
  }

  void grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    grid_received_ = true;
    last_grid_time_ = std::chrono::steady_clock::now();

    // Compute grid coverage: ratio of explored cells vs total
    uint32_t total = msg->info.width * msg->info.height;
    uint32_t explored = 0;
    uint32_t obstacles = 0;

    for (const auto & cell : msg->data) {
      if (cell >= 0) explored++;  // -1 = unknown
      if (cell > 50) obstacles++;  // >50% occupied
    }

    if (total > 0) {
      grid_coverage_ = static_cast<float>(explored) / static_cast<float>(total);
      grid_obstacle_ratio_ = static_cast<float>(obstacles) / static_cast<float>(total);
    }
    grid_resolution_ = msg->info.resolution;
  }

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Track odometry covariance as quality indicator
    // Lower covariance = more confident tracking
    double cov_x = msg->pose.covariance[0];   // xx
    double cov_y = msg->pose.covariance[7];   // yy
    double cov_yaw = msg->pose.covariance[35]; // yaw-yaw

    if (std::isfinite(cov_x) && std::isfinite(cov_y) && cov_x > 0 && cov_y > 0) {
      odom_uncertainty_ = std::sqrt(cov_x + cov_y);
    }
    odom_yaw_uncertainty_ = std::isfinite(cov_yaw) && cov_yaw > 0 ? std::sqrt(cov_yaw) : 0.0;
    odom_received_ = true;
  }

  void compute_quality()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Quality score: 0.0 (bad) to 1.0 (excellent)
    float quality = 0.0f;
    int components = 0;

    // ── Component 1: Point cloud density (0-0.3) ──
    if (cloud_received_) {
      // Good density: >500 points/m^2
      float density_score = std::min(1.0f, point_density_ / 500.0f);
      quality += density_score * 0.3f;
      components++;
    }

    // ── Component 2: Grid coverage (0-0.3) ──
    if (grid_received_) {
      // Good coverage: >30% explored
      float coverage_score = std::min(1.0f, grid_coverage_ / 0.3f);
      quality += coverage_score * 0.3f;
      components++;
    }

    // ── Component 3: Odometry confidence (0-0.2) ──
    if (odom_received_) {
      // Good: uncertainty < 0.05m (±5cm target)
      float odom_score = 1.0f;
      if (odom_uncertainty_ > 0.001) {
        odom_score = std::max(0.0f, 1.0f - static_cast<float>(odom_uncertainty_) / 0.1f);
      }
      quality += odom_score * 0.2f;
      components++;
    }

    // ── Component 4: Data freshness (0-0.2) ──
    auto now = std::chrono::steady_clock::now();
    float freshness = 0.0f;
    if (cloud_received_) {
      double cloud_age = std::chrono::duration<double>(now - last_cloud_time_).count();
      freshness += (cloud_age < 10.0) ? 0.1f : 0.0f;
    }
    if (grid_received_) {
      double grid_age = std::chrono::duration<double>(now - last_grid_time_).count();
      freshness += (grid_age < 10.0) ? 0.1f : 0.0f;
    }
    quality += freshness;

    // ── Publish quality score ──
    std_msgs::msg::Float32 quality_msg;
    quality_msg.data = quality;
    quality_pub_->publish(quality_msg);

    // ── Publish diagnostics ──
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = this->now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "SLAM/MapQuality";

    if (quality > 0.7f) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Map quality excellent";
    } else if (quality > 0.4f) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "Map quality moderate";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Map quality poor";
    }

    auto add_kv = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };

    add_kv("Quality Score", std::to_string(quality));
    add_kv("Point Count", std::to_string(cloud_point_count_));
    add_kv("Point Density (pts/m2)", std::to_string(point_density_));
    add_kv("Grid Coverage", std::to_string(grid_coverage_));
    add_kv("Grid Resolution (m)", std::to_string(grid_resolution_));
    add_kv("Odom Uncertainty (m)", std::to_string(odom_uncertainty_));
    add_kv("Odom Yaw Uncertainty (rad)", std::to_string(odom_yaw_uncertainty_));

    diag_array.status.push_back(status);
    diag_pub_->publish(diag_array);

    // Console summary
    static int counter = 0;
    if (++counter % 6 == 0) {  // Every 30s
      RCLCPP_INFO(this->get_logger(),
        "Map Quality: %.2f | pts=%u density=%.0f/m2 | grid_cov=%.1f%% | odom_unc=%.4fm",
        quality, cloud_point_count_, point_density_,
        grid_coverage_ * 100.0f, odom_uncertainty_);
    }
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr quality_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr quality_timer_;

  // Data mutex
  std::mutex data_mutex_;

  // Cloud metrics
  bool cloud_received_ = false;
  uint32_t cloud_point_count_ = 0;
  float point_density_ = 0.0f;
  std::chrono::steady_clock::time_point last_cloud_time_;

  // Grid metrics
  bool grid_received_ = false;
  float grid_coverage_ = 0.0f;
  float grid_obstacle_ratio_ = 0.0f;
  float grid_resolution_ = 0.05f;
  std::chrono::steady_clock::time_point last_grid_time_;

  // Odometry metrics
  bool odom_received_ = false;
  double odom_uncertainty_ = 0.0;
  double odom_yaw_uncertainty_ = 0.0;
};

}  // namespace agv_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_slam::MapQualityNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
