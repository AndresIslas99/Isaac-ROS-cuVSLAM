#include "agv_slam/slam_monitor_node.hpp"
#include <cmath>

namespace agv_slam
{

SlamMonitorNode::SlamMonitorNode(const rclcpp::NodeOptions & options)
: Node("slam_monitor", options)
{
  // ── Subscribers ──
  auto qos = rclcpp::SensorDataQoS();

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/filtered/rgb", qos,
    std::bind(&SlamMonitorNode::rgb_callback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/filtered/depth", qos,
    std::bind(&SlamMonitorNode::depth_callback, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/zed/zed_node/imu/data", qos,
    std::bind(&SlamMonitorNode::imu_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/visual_slam/tracking/odometry", qos,
    std::bind(&SlamMonitorNode::odom_callback, this, std::placeholders::_1));

  // ── Publisher ──
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10);

  // ── Timer: publish diagnostics at 1 Hz ──
  diag_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SlamMonitorNode::publish_diagnostics, this));

  // ── TF listener (for checking map→odom→base_link chain) ──
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "SLAM monitor initialized");
}

void SlamMonitorNode::rgb_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr &) { rgb_rate_.tick(); }

void SlamMonitorNode::depth_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr &) { depth_rate_.tick(); }

void SlamMonitorNode::imu_callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr &) { imu_rate_.tick(); }

void SlamMonitorNode::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  odom_rate_.tick();

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  if (!first_odom_) {
    double dx = x - last_x_;
    double dy = y - last_y_;
    total_distance_ += std::sqrt(dx * dx + dy * dy);
  }
  first_odom_ = false;
  last_x_ = x;
  last_y_ = y;
}

void SlamMonitorNode::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = this->now();

  auto make_status = [](const std::string & name, const std::string & message,
                        uint8_t level) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.message = message;
    status.level = level;
    return status;
  };

  auto add_kv = [](diagnostic_msgs::msg::DiagnosticStatus & status,
                    const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };

  // ── Sensor rates ──
  double rgb_hz = rgb_rate_.get_hz();
  double depth_hz = depth_rate_.get_hz();
  double imu_hz = imu_rate_.get_hz();
  double odom_hz = odom_rate_.get_hz();

  auto sensor_status = make_status("SLAM/Sensors", "", 0);
  add_kv(sensor_status, "RGB Hz", std::to_string(rgb_hz));
  add_kv(sensor_status, "Depth Hz", std::to_string(depth_hz));
  add_kv(sensor_status, "IMU Hz", std::to_string(imu_hz));
  add_kv(sensor_status, "cuVSLAM Odom Hz", std::to_string(odom_hz));
  add_kv(sensor_status, "Distance Traveled (m)", std::to_string(total_distance_));

  // Determine health
  bool healthy = true;
  std::string issues;

  if (rgb_hz < 10.0) { healthy = false; issues += "RGB low; "; }
  if (depth_hz < 10.0) { healthy = false; issues += "Depth low; "; }
  if (imu_hz < 10.0) { healthy = false; issues += "IMU low; "; }
  if (odom_hz < 10.0) { healthy = false; issues += "cuVSLAM odom low; "; }

  sensor_status.level = healthy ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  sensor_status.message = healthy ? "All sensors nominal" : issues;

  diag_array.status.push_back(sensor_status);

  // ── TF chain check ──
  auto tf_status = make_status("SLAM/TF", "", 0);
  try {
    auto transform = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
    add_kv(tf_status, "map→base_link", "OK");
    add_kv(tf_status, "x", std::to_string(transform.transform.translation.x));
    add_kv(tf_status, "y", std::to_string(transform.transform.translation.y));
    tf_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    tf_status.message = "TF chain healthy";
  } catch (tf2::TransformException & ex) {
    add_kv(tf_status, "map→base_link", "MISSING");
    tf_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    tf_status.message = std::string("TF error: ") + ex.what();
  }

  diag_array.status.push_back(tf_status);

  // ── Publish ──
  diag_pub_->publish(diag_array);

  // Console summary every 10 seconds
  static int counter = 0;
  if (++counter % 10 == 0) {
    RCLCPP_INFO(this->get_logger(),
      "SLAM Status: RGB=%.1fHz Depth=%.1fHz IMU=%.0fHz Odom=%.1fHz Dist=%.2fm %s",
      rgb_hz, depth_hz, imu_hz, odom_hz, total_distance_,
      healthy ? "[OK]" : "[WARN]");
  }
}

}  // namespace agv_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_slam::SlamMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
