#include "agv_slam/slam_monitor_node.hpp"
#include <cmath>
#include <regex>
#include <sstream>
#include <cstdio>
#include <csignal>

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

  // ── Publishers ──
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10);

  quality_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/slam/quality", 10);

  // ── Timer: publish diagnostics at 1 Hz ──
  diag_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SlamMonitorNode::publish_diagnostics, this));

  // ── TF listener (for checking odom→base_link chain) ──
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Start tegrastats reader thread ──
  tegra_running_ = true;
  tegra_thread_ = std::thread(&SlamMonitorNode::tegrastats_thread_func, this);

  RCLCPP_INFO(this->get_logger(), "SLAM monitor initialized (with tegrastats)");
}

SlamMonitorNode::~SlamMonitorNode()
{
  tegra_running_ = false;
  if (tegra_pipe_) {
    pclose(tegra_pipe_);
    tegra_pipe_ = nullptr;
  }
  if (tegra_thread_.joinable()) {
    tegra_thread_.join();
  }
}

// ── tegrastats background reader ──

void SlamMonitorNode::tegrastats_thread_func()
{
  // Run tegrastats at 2-second interval (low overhead)
  tegra_pipe_ = popen("tegrastats --interval 2000 2>/dev/null", "r");
  if (!tegra_pipe_) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to start tegrastats (not running as root?). "
      "GPU/thermal monitoring disabled.");
    return;
  }

  char buf[2048];
  while (tegra_running_ && fgets(buf, sizeof(buf), tegra_pipe_)) {
    parse_tegrastats_line(std::string(buf));
  }

  if (tegra_pipe_) {
    pclose(tegra_pipe_);
    tegra_pipe_ = nullptr;
  }
}

void SlamMonitorNode::parse_tegrastats_line(const std::string & line)
{
  std::lock_guard<std::mutex> lock(tegra_.mtx);

  // RAM 8302/62841MB
  {
    std::regex re(R"(RAM\s+(\d+)/(\d+)MB)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.ram_used_mb = std::stod(m[1]);
      tegra_.ram_total_mb = std::stod(m[2]);
    }
  }

  // CPU [64%@2201,53%@2201,...] — extract average utilization
  {
    std::regex re(R"(CPU\s+\[([^\]]+)\])");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      std::string cpu_str = m[1];
      std::regex core_re(R"((\d+)%@)");
      auto begin = std::sregex_iterator(cpu_str.begin(), cpu_str.end(), core_re);
      auto end = std::sregex_iterator();
      double sum = 0.0;
      int count = 0;
      for (auto it = begin; it != end; ++it) {
        sum += std::stod((*it)[1]);
        count++;
      }
      if (count > 0) {
        tegra_.cpu_avg_pct = sum / count;
      }
    }
  }

  // GR3D_FREQ 19%@[1299,1289] or GR3D_FREQ 0%@0
  {
    std::regex re(R"(GR3D_FREQ\s+(\d+)%)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.gpu_freq_pct = std::stod(m[1]);
    }
  }

  // Temperatures: cpu@64.875C gpu@56.906C tj@64.687C
  {
    std::regex re(R"(cpu@([\d.]+)C)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.cpu_temp_c = std::stod(m[1]);
    }
  }
  {
    std::regex re(R"(gpu@([\d.]+)C)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.gpu_temp_c = std::stod(m[1]);
    }
  }
  {
    std::regex re(R"(tj@([\d.]+)C)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.tj_temp_c = std::stod(m[1]);
    }
  }

  // Power: VDD_GPU_SOC 8771mW/8771mW (instant/average)
  {
    std::regex re(R"(VDD_GPU_SOC\s+(\d+)mW)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.vdd_gpu_soc_mw = std::stod(m[1]);
    }
  }
  {
    std::regex re(R"(VDD_CPU_CV\s+(\d+)mW)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.vdd_cpu_cv_mw = std::stod(m[1]);
    }
  }
  {
    std::regex re(R"(VIN_SYS_5V0\s+(\d+)mW)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.vin_sys_mw = std::stod(m[1]);
    }
  }

  tegra_.valid = true;
}

// ── Sensor callbacks ──

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

// ── Diagnostics publisher ──

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

  auto fmt = [](double val, int precision = 1) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << val;
    return oss.str();
  };

  // ── Sensor rates ──
  double rgb_hz = rgb_rate_.get_hz();
  double depth_hz = depth_rate_.get_hz();
  double imu_hz = imu_rate_.get_hz();
  double odom_hz = odom_rate_.get_hz();

  auto sensor_status = make_status("SLAM/Sensors", "", 0);
  add_kv(sensor_status, "RGB Hz", fmt(rgb_hz));
  add_kv(sensor_status, "Depth Hz", fmt(depth_hz));
  add_kv(sensor_status, "IMU Hz", fmt(imu_hz, 0));
  add_kv(sensor_status, "cuVSLAM Odom Hz", fmt(odom_hz));
  add_kv(sensor_status, "Distance Traveled (m)", fmt(total_distance_, 2));

  bool sensors_healthy = true;
  std::string issues;

  if (rgb_hz < 5.0) { sensors_healthy = false; issues += "RGB low; "; }
  if (depth_hz < 5.0) { sensors_healthy = false; issues += "Depth low; "; }
  if (imu_hz < 10.0) { sensors_healthy = false; issues += "IMU low; "; }
  if (odom_hz < 5.0) { sensors_healthy = false; issues += "cuVSLAM odom low; "; }

  sensor_status.level = sensors_healthy ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  sensor_status.message = sensors_healthy ? "All sensors nominal" : issues;

  diag_array.status.push_back(sensor_status);

  // ── TF chain check ──
  auto tf_status = make_status("SLAM/TF", "", 0);
  bool tf_ok = false;
  try {
    auto transform = tf_buffer_->lookupTransform(
      "odom", "base_link", tf2::TimePointZero);
    add_kv(tf_status, "odom->base_link", "OK");
    add_kv(tf_status, "x", fmt(transform.transform.translation.x, 3));
    add_kv(tf_status, "y", fmt(transform.transform.translation.y, 3));
    tf_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    tf_status.message = "TF chain healthy";
    tf_ok = true;
  } catch (tf2::TransformException & ex) {
    add_kv(tf_status, "odom->base_link", "MISSING");
    tf_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    tf_status.message = std::string("TF error: ") + ex.what();
  }

  diag_array.status.push_back(tf_status);

  // ── Jetson Hardware (tegrastats) ──
  bool thermal_ok = true;
  {
    std::lock_guard<std::mutex> lock(tegra_.mtx);
    if (tegra_.valid) {
      auto hw_status = make_status("SLAM/Jetson", "", 0);

      add_kv(hw_status, "GPU Utilization (%)", fmt(tegra_.gpu_freq_pct, 0));
      add_kv(hw_status, "CPU Utilization (%)", fmt(tegra_.cpu_avg_pct, 0));
      add_kv(hw_status, "GPU Temp (C)", fmt(tegra_.gpu_temp_c));
      add_kv(hw_status, "CPU Temp (C)", fmt(tegra_.cpu_temp_c));
      add_kv(hw_status, "Tj Max (C)", fmt(tegra_.tj_temp_c));
      add_kv(hw_status, "RAM Used (MB)", fmt(tegra_.ram_used_mb, 0));
      add_kv(hw_status, "RAM Total (MB)", fmt(tegra_.ram_total_mb, 0));
      add_kv(hw_status, "RAM Usage (%)",
        fmt(tegra_.ram_total_mb > 0 ? (tegra_.ram_used_mb / tegra_.ram_total_mb * 100.0) : 0.0));
      add_kv(hw_status, "GPU+SoC Power (W)", fmt(tegra_.vdd_gpu_soc_mw / 1000.0));
      add_kv(hw_status, "CPU Power (W)", fmt(tegra_.vdd_cpu_cv_mw / 1000.0));
      add_kv(hw_status, "System Power (W)", fmt(tegra_.vin_sys_mw / 1000.0));

      // Thermal thresholds for Orin AGX: throttle at 97C, shutdown at 105C
      if (tegra_.tj_temp_c > 90.0) {
        hw_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        hw_status.message = "CRITICAL: Junction temp > 90C — thermal throttling imminent";
        thermal_ok = false;
      } else if (tegra_.tj_temp_c > 80.0) {
        hw_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        hw_status.message = "Junction temp > 80C — consider improving cooling";
        thermal_ok = false;
      } else {
        hw_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        hw_status.message = "Thermal and power nominal";
      }

      diag_array.status.push_back(hw_status);
    }
  }

  // ── Publish diagnostics ──
  diag_pub_->publish(diag_array);

  // ── Publish quality summary as JSON string ──
  {
    std_msgs::msg::String quality_msg;
    std::lock_guard<std::mutex> lock(tegra_.mtx);
    std::ostringstream json;
    json << std::fixed << std::setprecision(1);
    json << "{";
    json << "\"rgb_hz\":" << rgb_hz;
    json << ",\"depth_hz\":" << depth_hz;
    json << ",\"imu_hz\":" << imu_hz;
    json << ",\"odom_hz\":" << odom_hz;
    json << ",\"distance_m\":" << total_distance_;
    json << ",\"sensors_ok\":" << (sensors_healthy ? "true" : "false");
    json << ",\"tf_ok\":" << (tf_ok ? "true" : "false");
    if (tegra_.valid) {
      json << ",\"gpu_pct\":" << tegra_.gpu_freq_pct;
      json << ",\"cpu_pct\":" << tegra_.cpu_avg_pct;
      json << ",\"gpu_temp\":" << tegra_.gpu_temp_c;
      json << ",\"cpu_temp\":" << tegra_.cpu_temp_c;
      json << ",\"tj_temp\":" << tegra_.tj_temp_c;
      json << ",\"ram_used_mb\":" << tegra_.ram_used_mb;
      json << ",\"ram_total_mb\":" << tegra_.ram_total_mb;
      json << ",\"gpu_power_w\":" << (tegra_.vdd_gpu_soc_mw / 1000.0);
      json << ",\"cpu_power_w\":" << (tegra_.vdd_cpu_cv_mw / 1000.0);
      json << ",\"sys_power_w\":" << (tegra_.vin_sys_mw / 1000.0);
      json << ",\"thermal_ok\":" << (thermal_ok ? "true" : "false");
    }
    json << "}";
    quality_msg.data = json.str();
    quality_pub_->publish(quality_msg);
  }

  // Console summary every 10 seconds
  static int counter = 0;
  if (++counter % 10 == 0) {
    std::lock_guard<std::mutex> lock(tegra_.mtx);
    if (tegra_.valid) {
      RCLCPP_INFO(this->get_logger(),
        "SLAM: RGB=%.1fHz Depth=%.1fHz Odom=%.1fHz Dist=%.2fm | "
        "GPU=%0.f%% %.1fC CPU=%0.f%% %.1fC Tj=%.1fC | "
        "RAM=%.0f/%.0fMB | Power: GPU=%.1fW CPU=%.1fW Sys=%.1fW %s",
        rgb_hz, depth_hz, odom_hz, total_distance_,
        tegra_.gpu_freq_pct, tegra_.gpu_temp_c,
        tegra_.cpu_avg_pct, tegra_.cpu_temp_c, tegra_.tj_temp_c,
        tegra_.ram_used_mb, tegra_.ram_total_mb,
        tegra_.vdd_gpu_soc_mw / 1000.0,
        tegra_.vdd_cpu_cv_mw / 1000.0,
        tegra_.vin_sys_mw / 1000.0,
        (sensors_healthy && tf_ok && thermal_ok) ? "[OK]" : "[WARN]");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "SLAM: RGB=%.1fHz Depth=%.1fHz Odom=%.1fHz Dist=%.2fm %s (no tegrastats)",
        rgb_hz, depth_hz, odom_hz, total_distance_,
        sensors_healthy ? "[OK]" : "[WARN]");
    }
  }
}

}  // namespace agv_slam

#ifndef AGV_SLAM_BUILDING_TESTS
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_slam::SlamMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
