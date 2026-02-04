#include "agv_slam/slam_monitor_node.hpp"
#include <cmath>
#include <regex>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstdio>
#include <csignal>

namespace agv_slam
{

SlamMonitorNode::SlamMonitorNode(const rclcpp::NodeOptions & options)
: Node("slam_monitor", options)
{
  // ── Hardcoded production thresholds ──
  rate_thresholds_["filtered_rgb"]  = {15.0, 13.0, 10.0};
  rate_thresholds_["filtered_depth"] = {15.0, 13.0, 10.0};
  rate_thresholds_["imu"]           = {400.0, 360.0, 200.0};
  rate_thresholds_["cuvslam_odom"]  = {15.0, 13.0, 10.0};
  rate_thresholds_["nvblox_mesh"]   = {2.0, 1.0, 0.0};
  rate_thresholds_["coverage_grid"] = {0.5, 0.2, 0.05};
  rate_thresholds_["zed_raw_rgb"]   = {15.0, 13.0, 10.0};
  rate_thresholds_["zed_raw_depth"] = {15.0, 13.0, 10.0};

  frame_drop_threshold_ = {0.0, 3.0, 10.0};

  system_thresholds_["gpu_pct"]          = {80.0, 90.0, 98.0};
  system_thresholds_["cpu_max_core_pct"] = {80.0, 90.0, 95.0};
  system_thresholds_["temp_c"]           = {80.0, 90.0, 95.0};
  system_thresholds_["ram_used_pct"]     = {75.0, 87.0, 95.0};

  storage_threshold_ = {50.0, 20.0, 5.0};
  depth_valid_ratio_threshold_ = {0.7, 0.5, 0.3};
  tracking_confidence_threshold_ = {0.8, 0.5, 0.2};
  depth_filter_latency_threshold_ = {30.0, 60.0, 100.0};
  cuvslam_latency_threshold_ = {5.0, 10.0, 30.0};
  tf_max_age_ms_ = 150.0;

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

  nvblox_mesh_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/nvblox_node/mesh_marker", qos,
    std::bind(&SlamMonitorNode::nvblox_mesh_callback, this, std::placeholders::_1));

  coverage_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/coverage/grid", qos,
    std::bind(&SlamMonitorNode::coverage_grid_callback, this, std::placeholders::_1));

  zed_raw_rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed/zed_node/rgb/color/rect/image", qos,
    std::bind(&SlamMonitorNode::zed_raw_rgb_callback, this, std::placeholders::_1));

  zed_raw_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/zed/zed_node/depth/depth_registered", qos,
    std::bind(&SlamMonitorNode::zed_raw_depth_callback, this, std::placeholders::_1));

  // New monitoring subscribers
  depth_filter_latency_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/depth_filter/latency", 10,
    std::bind(&SlamMonitorNode::depth_filter_latency_callback, this, std::placeholders::_1));

  depth_filter_quality_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/depth_filter/quality", 10,
    std::bind(&SlamMonitorNode::depth_filter_quality_callback, this, std::placeholders::_1));

  // ── Publishers ──
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10);
  quality_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/slam/quality", 10);

  // ── Timer: diagnostics at 1 Hz ──
  diag_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&SlamMonitorNode::publish_diagnostics, this));

  // ── TF listener ──
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Start tegrastats reader thread ──
  tegra_running_ = true;
  tegra_thread_ = std::thread(&SlamMonitorNode::tegrastats_thread_func, this);

  // ── Init disk monitoring ──
  last_disk_check_ = std::chrono::steady_clock::now();
  last_sectors_written_ = 0;

  prev_odom_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  RCLCPP_INFO(this->get_logger(),
    "SLAM monitor initialized (production: 7 diagnostic groups, drop detection, disk I/O)");
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
  tegra_pipe_ = popen("tegrastats --interval 2000 2>/dev/null", "r");
  if (!tegra_pipe_) {
    RCLCPP_WARN(this->get_logger(),
      "Failed to start tegrastats. GPU/thermal monitoring disabled.");
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

  // RAM
  {
    std::regex re(R"(RAM\s+(\d+)/(\d+)MB)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.ram_used_mb = std::stod(m[1]);
      tegra_.ram_total_mb = std::stod(m[2]);
    }
  }

  // SWAP
  {
    std::regex re(R"(SWAP\s+(\d+)/(\d+)MB)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.swap_used_mb = std::stod(m[1]);
      tegra_.swap_total_mb = std::stod(m[2]);
    }
  }

  // CPU per-core: [12%@2201,10%@2201,...,off,...]
  {
    std::regex re(R"(CPU\s*\[([^\]]+)\])");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      std::string cores_str = m[1];
      std::regex core_re(R"((\d+)%@\d+|off)");
      auto begin = std::sregex_iterator(cores_str.begin(), cores_str.end(), core_re);
      auto end = std::sregex_iterator();

      double sum = 0.0;
      int active_count = 0;
      double max_core = 0.0;
      tegra_.per_core_pct.clear();

      for (auto it = begin; it != end; ++it) {
        std::string match_str = (*it)[0].str();
        if (match_str == "off") continue;
        double pct = std::stod((*it)[1].str());
        tegra_.per_core_pct.push_back(pct);
        sum += pct;
        active_count++;
        if (pct > max_core) max_core = pct;
      }

      tegra_.cpu_avg_pct = (active_count > 0) ? (sum / active_count) : 0.0;
      tegra_.cpu_max_core_pct = max_core;
    }
  }

  // GPU: GR3D_FREQ XX%@YYYY
  {
    std::regex re(R"(GR3D_FREQ\s+(\d+)%(?:@(\d+))?)");
    std::smatch m;
    if (std::regex_search(line, m, re)) {
      tegra_.gpu_pct = std::stod(m[1]);
      if (m[2].matched) {
        tegra_.gpu_clock_mhz = std::stod(m[2]);
      }
    }
  }

  // Temperatures
  {
    std::regex re(R"(cpu@([\d.]+)C)");
    std::smatch m;
    if (std::regex_search(line, m, re)) tegra_.cpu_temp_c = std::stod(m[1]);
  }
  {
    std::regex re(R"(gpu@([\d.]+)C)");
    std::smatch m;
    if (std::regex_search(line, m, re)) tegra_.gpu_temp_c = std::stod(m[1]);
  }
  {
    std::regex re(R"(tj@([\d.]+)C)");
    std::smatch m;
    if (std::regex_search(line, m, re)) tegra_.tj_temp_c = std::stod(m[1]);
  }

  // Power
  {
    std::regex re(R"(VDD_GPU_SOC\s+(\d+)mW)");
    std::smatch m;
    if (std::regex_search(line, m, re)) tegra_.vdd_gpu_soc_mw = std::stod(m[1]);
  }
  {
    std::regex re(R"(VDD_CPU_CV\s+(\d+)mW)");
    std::smatch m;
    if (std::regex_search(line, m, re)) tegra_.vdd_cpu_cv_mw = std::stod(m[1]);
  }
  {
    std::regex re(R"(VIN_SYS_5V0\s+(\d+)mW)");
    std::smatch m;
    if (std::regex_search(line, m, re)) tegra_.vin_sys_mw = std::stod(m[1]);
  }

  tegra_.valid = true;
}

// ── Disk monitoring ──

void SlamMonitorNode::update_disk_stats()
{
  DiskStats ds;

  // Free space via statvfs
  struct statvfs stat;
  if (statvfs("/", &stat) == 0) {
    double block_size = static_cast<double>(stat.f_frsize);
    ds.free_gb = static_cast<double>(stat.f_bavail) * block_size / (1024.0 * 1024.0 * 1024.0);
    ds.total_gb = static_cast<double>(stat.f_blocks) * block_size / (1024.0 * 1024.0 * 1024.0);
  }

  // Write rate from /proc/diskstats
  auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_disk_check_).count();

  uint64_t sectors = 0;
  std::ifstream proc_file("/proc/diskstats");
  if (proc_file.is_open()) {
    std::string line;
    while (std::getline(proc_file, line)) {
      std::istringstream iss(line);
      int major, minor;
      std::string dev;
      iss >> major >> minor >> dev;
      if (dev == "nvme0n1" || dev == "sda" || dev == "mmcblk0") {
        uint64_t val;
        for (int i = 0; i < 6; ++i) iss >> val;
        iss >> sectors;
        break;
      }
    }
  }

  if (dt > 0.0 && last_sectors_written_ > 0 && sectors >= last_sectors_written_) {
    double bytes_written = static_cast<double>(sectors - last_sectors_written_) * 512.0;
    ds.write_rate_mbps = (bytes_written / (1024.0 * 1024.0)) / dt;
  }

  last_disk_check_ = now;
  last_sectors_written_ = sectors;

  // Estimate remaining time
  if (ds.write_rate_mbps > 0.0) {
    ds.estimated_minutes_remaining = (ds.free_gb * 1024.0) / ds.write_rate_mbps / 60.0;
  }

  ds.is_critical = (ds.free_gb < 5.0);

  std::lock_guard<std::mutex> lock(disk_mtx_);
  disk_stats_ = ds;
}

// ── Topic callbacks ──

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

  // Velocity / rotation tracking
  {
    std::lock_guard<std::mutex> lock(odom_mtx_);
    rclcpp::Time stamp(msg->header.stamp.sec,
      msg->header.stamp.nanosec, this->get_clock()->get_clock_type());
    if (prev_odom_pose_.has_value()) {
      double dt = 0.0;
      try { dt = (stamp - prev_odom_stamp_).seconds(); }
      catch (const std::exception &) { dt = 0.0; }
      if (dt > 0.0 && dt < 1.0) {
        auto & prev = prev_odom_pose_.value();
        auto & curr = msg->pose.pose;
        double dx_v = curr.position.x - prev.position.x;
        double dy_v = curr.position.y - prev.position.y;
        double dz_v = curr.position.z - prev.position.z;
        double dist = std::sqrt(dx_v * dx_v + dy_v * dy_v + dz_v * dz_v);

        auto quat_to_yaw = [](const geometry_msgs::msg::Quaternion & q) {
          return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        };
        double dyaw = quat_to_yaw(curr.orientation) - quat_to_yaw(prev.orientation);
        while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0 * M_PI;

        std::lock_guard<std::mutex> qlock(data_quality_.mtx);
        data_quality_.velocity_mps = 0.9 * data_quality_.velocity_mps + 0.1 * (dist / dt);
        data_quality_.rotation_dps = 0.9 * data_quality_.rotation_dps +
          0.1 * (std::abs(dyaw) * 180.0 / M_PI / dt);
      }
    }
    prev_odom_pose_ = msg->pose.pose;
    prev_odom_stamp_ = stamp;
  }

  // Tracking confidence from covariance
  double cov_sum = 0.0;
  for (int i = 0; i < 6; ++i) cov_sum += std::abs(msg->pose.covariance[i * 7]);
  double confidence = 1.0 / (1.0 + cov_sum);
  {
    std::lock_guard<std::mutex> lock(data_quality_.mtx);
    data_quality_.cuvslam_confidence = 0.9 * data_quality_.cuvslam_confidence + 0.1 * confidence;
  }
}

void SlamMonitorNode::nvblox_mesh_callback(
  const visualization_msgs::msg::MarkerArray::ConstSharedPtr &) { nvblox_mesh_rate_.tick(); }
void SlamMonitorNode::coverage_grid_callback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &) { coverage_grid_rate_.tick(); }
void SlamMonitorNode::zed_raw_rgb_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr &) { zed_raw_rgb_rate_.tick(); }
void SlamMonitorNode::zed_raw_depth_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr &) { zed_raw_depth_rate_.tick(); }

void SlamMonitorNode::depth_filter_latency_callback(
  const std_msgs::msg::Float32::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(data_quality_.mtx);
  data_quality_.depth_filter_latency_ms =
    0.9 * data_quality_.depth_filter_latency_ms + 0.1 * msg->data;
}

void SlamMonitorNode::depth_filter_quality_callback(
  const std_msgs::msg::String::ConstSharedPtr & msg)
{
  const auto & data = msg->data;
  auto extract = [&data](const std::string & key) -> double {
    auto pos = data.find("\"" + key + "\":");
    if (pos == std::string::npos) return 0.0;
    pos += key.size() + 3;
    try { return std::stod(data.substr(pos)); } catch (...) { return 0.0; }
  };

  std::lock_guard<std::mutex> lock(data_quality_.mtx);
  data_quality_.depth_valid_ratio = extract("valid_ratio");
  data_quality_.depth_mean = extract("mean_depth");
}

// ── Diagnostics publisher (7 groups) ──

void SlamMonitorNode::publish_diagnostics()
{
  // Update disk stats
  update_disk_stats();

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

  // ── 1. SLAM/Sensors — rates + drop rates ──
  struct RateInfo {
    std::string name;
    std::string key;
    RateTracker * tracker;
  };

  std::vector<RateInfo> rates = {
    {"Filtered RGB Hz", "filtered_rgb", &rgb_rate_},
    {"Filtered Depth Hz", "filtered_depth", &depth_rate_},
    {"IMU Hz", "imu", &imu_rate_},
    {"cuVSLAM Odom Hz", "cuvslam_odom", &odom_rate_},
    {"Nvblox Mesh Hz", "nvblox_mesh", &nvblox_mesh_rate_},
    {"Coverage Grid Hz", "coverage_grid", &coverage_grid_rate_},
    {"ZED Raw RGB Hz", "zed_raw_rgb", &zed_raw_rgb_rate_},
    {"ZED Raw Depth Hz", "zed_raw_depth", &zed_raw_depth_rate_},
  };

  auto sensor_status = make_status("SLAM/Sensors", "", 0);
  uint8_t worst_sensor = 0;

  for (auto & ri : rates) {
    double hz = ri.tracker->get_hz();
    double drop = ri.tracker->drop_percent();
    add_kv(sensor_status, ri.name, fmt(hz));
    add_kv(sensor_status, ri.name + " drop%", fmt(drop));

    auto it = rate_thresholds_.find(ri.key);
    if (it != rate_thresholds_.end()) {
      uint8_t level = it->second.classify(hz);
      add_kv(sensor_status, ri.name + " status", ThresholdConfig::level_to_color(level));
      if (level > worst_sensor) worst_sensor = level;
    }
    // Drop level
    uint8_t dl = (drop <= frame_drop_threshold_.yellow) ? 0 :
                 (drop <= frame_drop_threshold_.red) ? 1 : 2;
    if (dl > worst_sensor) worst_sensor = dl;
  }

  add_kv(sensor_status, "Distance (m)", fmt(total_distance_, 2));
  sensor_status.level = worst_sensor;
  sensor_status.message = (worst_sensor == 0) ? "All sensors nominal" : "Sensor rate degraded";
  diag_array.status.push_back(sensor_status);

  // ── 2. SLAM/TF — age validation + camera TF existence ──
  auto tf_status = make_status("SLAM/TF", "", 0);
  uint8_t worst_tf = 0;

  try {
    auto transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    add_kv(tf_status, "odom->base_link", "OK");
    add_kv(tf_status, "x", fmt(transform.transform.translation.x, 3));
    add_kv(tf_status, "y", fmt(transform.transform.translation.y, 3));

    double age_ms = 0.0;
    try {
      rclcpp::Time tf_time(transform.header.stamp.sec,
        transform.header.stamp.nanosec, this->get_clock()->get_clock_type());
      age_ms = (this->now() - tf_time).seconds() * 1000.0;
    } catch (const std::exception &) {
      age_ms = 0.0;  // Cannot compute age, assume OK
    }
    add_kv(tf_status, "odom_base_age_ms", fmt(age_ms));
    if (age_ms > tf_max_age_ms_) {
      worst_tf = 2;
      tf_status.message = "TF stale — cuVSLAM may have lost tracking";
    }
  } catch (tf2::TransformException & ex) {
    add_kv(tf_status, "odom->base_link", "MISSING");
    worst_tf = 2;
    tf_status.message = std::string("TF error: ") + ex.what();
  }

  // Static camera TF
  try {
    tf_buffer_->lookupTransform("base_link", "zed_camera_link", tf2::TimePointZero);
    add_kv(tf_status, "base_link->camera", "OK");
  } catch (tf2::TransformException &) {
    add_kv(tf_status, "base_link->camera", "MISSING");
    if (worst_tf < 1) worst_tf = 1;
  }

  if (worst_tf == 0) tf_status.message = "TF chain healthy";
  tf_status.level = worst_tf;
  diag_array.status.push_back(tf_status);

  // ── 3. SLAM/Jetson — GPU/CPU/thermal/RAM/swap/power/per-core ──
  (void)0; // thermal checked via worst_hw
  {
    std::lock_guard<std::mutex> lock(tegra_.mtx);
    if (tegra_.valid) {
      auto hw = make_status("SLAM/Jetson", "", 0);
      add_kv(hw, "gpu_pct", fmt(tegra_.gpu_pct, 0));
      add_kv(hw, "gpu_clock_mhz", fmt(tegra_.gpu_clock_mhz, 0));
      add_kv(hw, "cpu_avg_pct", fmt(tegra_.cpu_avg_pct, 0));
      add_kv(hw, "cpu_max_core_pct", fmt(tegra_.cpu_max_core_pct, 0));
      add_kv(hw, "gpu_temp_c", fmt(tegra_.gpu_temp_c));
      add_kv(hw, "cpu_temp_c", fmt(tegra_.cpu_temp_c));
      add_kv(hw, "tj_temp_c", fmt(tegra_.tj_temp_c));
      add_kv(hw, "ram_used_mb", fmt(tegra_.ram_used_mb, 0));
      add_kv(hw, "ram_total_mb", fmt(tegra_.ram_total_mb, 0));
      add_kv(hw, "swap_used_mb", fmt(tegra_.swap_used_mb, 0));
      add_kv(hw, "swap_total_mb", fmt(tegra_.swap_total_mb, 0));
      add_kv(hw, "gpu_power_w", fmt(tegra_.vdd_gpu_soc_mw / 1000.0));
      add_kv(hw, "cpu_power_w", fmt(tegra_.vdd_cpu_cv_mw / 1000.0));
      add_kv(hw, "sys_power_w", fmt(tegra_.vin_sys_mw / 1000.0));

      // Per-core
      {
        std::ostringstream cs;
        bool any_hot = false;
        for (size_t i = 0; i < tegra_.per_core_pct.size(); ++i) {
          if (i > 0) cs << ",";
          cs << static_cast<int>(tegra_.per_core_pct[i]) << "%";
          if (tegra_.per_core_pct[i] > 95.0) any_hot = true;
        }
        add_kv(hw, "per_core_pct", cs.str());
        add_kv(hw, "any_core_above_95", any_hot ? "true" : "false");
      }

      uint8_t worst_hw = 0;
      auto check = [&worst_hw, this](const std::string & k, double v) {
        auto it = system_thresholds_.find(k);
        if (it != system_thresholds_.end()) {
          uint8_t l = it->second.classify_upper(v);
          if (l > worst_hw) worst_hw = l;
        }
      };
      check("gpu_pct", tegra_.gpu_pct);
      check("temp_c", tegra_.tj_temp_c);
      check("cpu_max_core_pct", tegra_.cpu_max_core_pct);
      if (tegra_.ram_total_mb > 0) {
        double ram_pct = 100.0 * tegra_.ram_used_mb / tegra_.ram_total_mb;
        add_kv(hw, "ram_used_pct", fmt(ram_pct));
        check("ram_used_pct", ram_pct);
      }

      hw.level = worst_hw;
      hw.message = (worst_hw == 0) ? "System resources nominal" : "System resources under stress";
      diag_array.status.push_back(hw);
    } else {
      auto hw = make_status("SLAM/Jetson", "No tegrastats data", 1);
      diag_array.status.push_back(hw);
    }
  }

  // ── 4. SLAM/Latency ──
  double filter_lat, cuvslam_lat;
  {
    std::lock_guard<std::mutex> lock(data_quality_.mtx);
    filter_lat = data_quality_.depth_filter_latency_ms;
    cuvslam_lat = data_quality_.track_latency_ms;
  }

  auto lat_status = make_status("SLAM/Latency", "", 0);
  add_kv(lat_status, "depth_filter_ms", fmt(filter_lat));
  add_kv(lat_status, "cuvslam_ms", fmt(cuvslam_lat));
  uint8_t worst_lat = 0;
  {
    uint8_t l = depth_filter_latency_threshold_.classify_upper(filter_lat);
    if (l > worst_lat) worst_lat = l;
  }
  {
    uint8_t l = cuvslam_latency_threshold_.classify_upper(cuvslam_lat);
    if (l > worst_lat) worst_lat = l;
  }
  lat_status.level = worst_lat;
  lat_status.message = (worst_lat == 0) ? "Latency nominal" : "Elevated latency";
  diag_array.status.push_back(lat_status);

  // ── 5. SLAM/DataQuality ──
  double dq_conf, dq_ratio, dq_mean, dq_vel, dq_rot;
  int dq_vo;
  {
    std::lock_guard<std::mutex> lock(data_quality_.mtx);
    dq_conf = data_quality_.cuvslam_confidence;
    dq_vo = data_quality_.vo_state;
    dq_ratio = data_quality_.depth_valid_ratio;
    dq_mean = data_quality_.depth_mean;
    dq_vel = data_quality_.velocity_mps;
    dq_rot = data_quality_.rotation_dps;
  }

  auto dq_status = make_status("SLAM/DataQuality", "", 0);
  add_kv(dq_status, "tracking_confidence", fmt(dq_conf, 2));
  add_kv(dq_status, "vo_state", std::to_string(dq_vo));
  add_kv(dq_status, "depth_valid_ratio", fmt(dq_ratio, 2));
  add_kv(dq_status, "mean_depth_m", fmt(dq_mean, 2));
  add_kv(dq_status, "velocity_mps", fmt(dq_vel, 2));
  add_kv(dq_status, "rotation_dps", fmt(dq_rot, 1));

  uint8_t worst_dq = 0;
  {
    uint8_t l = tracking_confidence_threshold_.classify(dq_conf);
    if (l > worst_dq) worst_dq = l;
  }
  {
    uint8_t l = depth_valid_ratio_threshold_.classify(dq_ratio);
    if (l > worst_dq) worst_dq = l;
  }
  if (dq_vo == 2) worst_dq = 2;

  dq_status.level = worst_dq;
  dq_status.message = (dq_vo == 2) ? "CRITICAL: cuVSLAM tracking FAILED" :
    (worst_dq == 0) ? "Data quality nominal" : "Degraded data quality";
  diag_array.status.push_back(dq_status);

  // ── 6. SLAM/DiskIO ──
  DiskStats disk;
  {
    std::lock_guard<std::mutex> lock(disk_mtx_);
    disk = disk_stats_;
  }

  auto disk_status = make_status("SLAM/DiskIO", "", 0);
  add_kv(disk_status, "free_gb", fmt(disk.free_gb));
  add_kv(disk_status, "total_gb", fmt(disk.total_gb));
  add_kv(disk_status, "write_rate_mbps", fmt(disk.write_rate_mbps));
  add_kv(disk_status, "est_minutes", fmt(disk.estimated_minutes_remaining, 0));

  uint8_t disk_level = storage_threshold_.classify(disk.free_gb);
  disk_status.level = disk_level;
  disk_status.message = disk.is_critical ? "CRITICAL: Low disk space" :
    (disk_level == 0) ? "Disk I/O OK" : "Disk space getting low";
  diag_array.status.push_back(disk_status);

  // ── 7. SLAM/Pipeline — overall health ──
  uint8_t overall = 0;
  for (auto & s : diag_array.status) {
    if (s.level > overall) overall = s.level;
  }

  auto pipe_status = make_status("SLAM/Pipeline", "", overall);
  add_kv(pipe_status, "overall_health", ThresholdConfig::level_to_color(overall));
  add_kv(pipe_status, "nvblox_mesh_hz", fmt(nvblox_mesh_rate_.get_hz()));
  add_kv(pipe_status, "coverage_grid_hz", fmt(coverage_grid_rate_.get_hz()));
  add_kv(pipe_status, "distance_m", fmt(total_distance_, 2));
  pipe_status.message = (overall == 0) ? "Pipeline fully operational" :
    (overall == 1) ? "Pipeline degraded" : "Pipeline critical";
  diag_array.status.push_back(pipe_status);

  // ── Publish diagnostics ──
  diag_pub_->publish(diag_array);

  // ── Publish quality JSON ──
  {
    std::ostringstream json;
    json << std::fixed << std::setprecision(1);
    json << "{";

    // Pipeline rates + drops
    json << "\"pipeline\":{";
    bool first = true;
    for (auto & ri : rates) {
      double hz = ri.tracker->get_hz();
      double drop = ri.tracker->drop_percent();
      auto it = rate_thresholds_.find(ri.key);
      std::string color = "green";
      if (it != rate_thresholds_.end()) {
        color = ThresholdConfig::level_to_color(it->second.classify(hz));
      }
      if (!first) json << ",";
      json << "\"" << ri.key << "_hz\":[" << hz << ",\"" << color << "\"]";
      json << ",\"" << ri.key << "_drop_pct\":" << drop;
      first = false;
    }
    json << "}";

    // System
    {
      std::lock_guard<std::mutex> lock(tegra_.mtx);
      json << ",\"system\":{";
      json << "\"gpu_pct\":" << tegra_.gpu_pct;
      json << ",\"cpu_avg_pct\":" << tegra_.cpu_avg_pct;
      json << ",\"cpu_max_pct\":" << tegra_.cpu_max_core_pct;
      json << ",\"gpu_temp_c\":" << tegra_.gpu_temp_c;
      json << ",\"cpu_temp_c\":" << tegra_.cpu_temp_c;
      json << ",\"tj_temp_c\":" << tegra_.tj_temp_c;
      json << ",\"ram_used_mb\":" << tegra_.ram_used_mb;
      json << ",\"ram_total_mb\":" << tegra_.ram_total_mb;
      json << ",\"swap_used_mb\":" << tegra_.swap_used_mb;
      json << ",\"swap_total_mb\":" << tegra_.swap_total_mb;
      json << ",\"gpu_clock_mhz\":" << tegra_.gpu_clock_mhz;
      json << ",\"gpu_power_w\":" << (tegra_.vdd_gpu_soc_mw / 1000.0);
      json << ",\"cpu_power_w\":" << (tegra_.vdd_cpu_cv_mw / 1000.0);
      json << ",\"sys_power_w\":" << (tegra_.vin_sys_mw / 1000.0);
      json << ",\"per_core_pct\":[";
      for (size_t i = 0; i < tegra_.per_core_pct.size(); ++i) {
        if (i > 0) json << ",";
        json << tegra_.per_core_pct[i];
      }
      json << "]}";
    }

    // Latency
    json << ",\"latency\":{\"depth_filter_ms\":" << filter_lat
         << ",\"cuvslam_ms\":" << cuvslam_lat << "}";

    // Data quality
    json.precision(2);
    json << ",\"data_quality\":{\"tracking_confidence\":" << dq_conf
         << ",\"vo_state\":" << dq_vo
         << ",\"depth_valid_ratio\":" << dq_ratio
         << ",\"mean_depth_m\":" << dq_mean
         << ",\"velocity_mps\":" << dq_vel
         << ",\"rotation_dps\":" << dq_rot << "}";

    // Storage
    json.precision(1);
    json << ",\"storage\":{\"free_gb\":" << disk.free_gb
         << ",\"total_gb\":" << disk.total_gb
         << ",\"write_mbps\":" << disk.write_rate_mbps
         << ",\"est_minutes\":" << disk.estimated_minutes_remaining << "}";

    json << ",\"distance_m\":" << total_distance_;
    json << ",\"overall_status\":\"" << ThresholdConfig::level_to_color(overall) << "\"";
    json << "}";

    std_msgs::msg::String msg;
    msg.data = json.str();
    quality_pub_->publish(msg);
  }

  // ── Console summary every 10 seconds ──
  static int counter = 0;
  if (++counter % 10 == 0) {
    std::lock_guard<std::mutex> lock(tegra_.mtx);
    if (tegra_.valid) {
      RCLCPP_INFO(this->get_logger(),
        "SLAM: RGB=%.1fHz Depth=%.1fHz IMU=%.0fHz Odom=%.1fHz Dist=%.1fm | "
        "GPU=%.0f%% Tj=%.0fC RAM=%.0f/%.0fMB Disk=%.0fGB | "
        "Conf=%.2f Drop=%.1f%% [%s]",
        rgb_rate_.get_hz(), depth_rate_.get_hz(), imu_rate_.get_hz(),
        odom_rate_.get_hz(), total_distance_,
        tegra_.gpu_pct, tegra_.tj_temp_c,
        tegra_.ram_used_mb, tegra_.ram_total_mb, disk.free_gb,
        dq_conf, rgb_rate_.drop_percent(),
        ThresholdConfig::level_to_color(overall).c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
        "SLAM: RGB=%.1fHz Depth=%.1fHz Odom=%.1fHz Dist=%.1fm Conf=%.2f [%s] (no tegrastats)",
        rgb_rate_.get_hz(), depth_rate_.get_hz(),
        odom_rate_.get_hz(), total_distance_, dq_conf,
        ThresholdConfig::level_to_color(overall).c_str());
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
