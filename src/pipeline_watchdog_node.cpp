#include "agv_slam/pipeline_watchdog_node.hpp"
#include <filesystem>
#include <csignal>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace agv_slam
{

// Global pointer for signal handler
static PipelineWatchdogNode * g_watchdog_instance = nullptr;

static void signal_handler(int signum)
{
  if (g_watchdog_instance) {
    RCLCPP_WARN(rclcpp::get_logger("pipeline_watchdog"),
      "Received signal %d, saving state and shutting down...", signum);
  }
  rclcpp::shutdown();
}

PipelineWatchdogNode::PipelineWatchdogNode(const rclcpp::NodeOptions & options)
: Node("pipeline_watchdog", options)
{
  start_time_ = std::chrono::steady_clock::now();
  g_watchdog_instance = this;

  // ── Parameters ──
  this->declare_parameter("log_directory", "/mnt/ssd/slam_logs/");
  this->declare_parameter("heartbeat_rate", 1.0);
  this->declare_parameter("node_timeout_sec", 10.0);

  log_directory_ = this->get_parameter("log_directory").as_string();
  heartbeat_rate_ = this->get_parameter("heartbeat_rate").as_double();
  node_timeout_sec_ = this->get_parameter("node_timeout_sec").as_double();

  // ── Create log directory ──
  std::filesystem::create_directories(log_directory_);

  // ── Open metrics log ──
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << log_directory_ << "/slam_metrics_"
     << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";

  metrics_log_.open(ss.str());
  if (metrics_log_.is_open()) {
    metrics_log_ << "timestamp,uptime_sec,pipeline_healthy,rgb_hz,depth_hz,odom_hz,"
                 << "restart_count,rgb_msgs,depth_msgs,odom_msgs" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Metrics log: %s", ss.str().c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Could not open metrics log at %s", ss.str().c_str());
  }

  // ── Initialize node health tracking ──
  auto init_time = std::chrono::steady_clock::now();
  node_health_["rgb"] = {init_time, false, 0, 0.0};
  node_health_["depth"] = {init_time, false, 0, 0.0};
  node_health_["odom"] = {init_time, false, 0, 0.0};
  node_health_["diagnostics"] = {init_time, false, 0, 0.0};

  // ── Subscribers ──
  auto qos = rclcpp::SensorDataQoS();

  diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10,
    std::bind(&PipelineWatchdogNode::diagnostics_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/visual_slam/tracking/odometry", qos,
    std::bind(&PipelineWatchdogNode::odom_callback, this, std::placeholders::_1));

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/filtered/rgb", qos,
    std::bind(&PipelineWatchdogNode::rgb_callback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/filtered/depth", qos,
    std::bind(&PipelineWatchdogNode::depth_callback, this, std::placeholders::_1));

  // ── Publisher: heartbeat ──
  heartbeat_pub_ = this->create_publisher<std_msgs::msg::Header>(
    "/watchdog/heartbeat", 10);

  // ── Timers ──
  auto watchdog_period = std::chrono::milliseconds(
    static_cast<int>(1000.0 / heartbeat_rate_));

  watchdog_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&PipelineWatchdogNode::watchdog_check, this));

  heartbeat_timer_ = this->create_wall_timer(
    watchdog_period,
    std::bind(&PipelineWatchdogNode::publish_heartbeat, this));

  log_timer_ = this->create_wall_timer(
    std::chrono::seconds(10),
    std::bind(&PipelineWatchdogNode::log_metrics, this));

  // ── Signal handling ──
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  RCLCPP_INFO(this->get_logger(),
    "Pipeline watchdog initialized: timeout=%.1fs, log=%s",
    node_timeout_sec_, log_directory_.c_str());
}

PipelineWatchdogNode::~PipelineWatchdogNode()
{
  if (metrics_log_.is_open()) {
    metrics_log_.flush();
    metrics_log_.close();
  }
  g_watchdog_instance = nullptr;
  RCLCPP_INFO(this->get_logger(), "Watchdog shutdown complete. Metrics saved.");
}

void PipelineWatchdogNode::diagnostics_callback(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr &)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  auto & health = node_health_["diagnostics"];
  health.last_seen = std::chrono::steady_clock::now();
  health.alive = true;
  health.message_count++;
}

void PipelineWatchdogNode::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr &)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  auto & health = node_health_["odom"];
  auto now = std::chrono::steady_clock::now();
  if (health.message_count > 0) {
    double dt = std::chrono::duration<double>(now - health.last_seen).count();
    if (dt > 0.001) {
      health.last_rate_hz = 0.9 * health.last_rate_hz + 0.1 * (1.0 / dt);
    }
  }
  health.last_seen = now;
  health.alive = true;
  health.message_count++;
}

void PipelineWatchdogNode::rgb_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr &)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  auto & health = node_health_["rgb"];
  auto now = std::chrono::steady_clock::now();
  if (health.message_count > 0) {
    double dt = std::chrono::duration<double>(now - health.last_seen).count();
    if (dt > 0.001) {
      health.last_rate_hz = 0.9 * health.last_rate_hz + 0.1 * (1.0 / dt);
    }
  }
  health.last_seen = now;
  health.alive = true;
  health.message_count++;
}

void PipelineWatchdogNode::depth_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr &)
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  auto & health = node_health_["depth"];
  auto now = std::chrono::steady_clock::now();
  if (health.message_count > 0) {
    double dt = std::chrono::duration<double>(now - health.last_seen).count();
    if (dt > 0.001) {
      health.last_rate_hz = 0.9 * health.last_rate_hz + 0.1 * (1.0 / dt);
    }
  }
  health.last_seen = now;
  health.alive = true;
  health.message_count++;
}

void PipelineWatchdogNode::watchdog_check()
{
  std::lock_guard<std::mutex> lock(health_mutex_);
  auto now = std::chrono::steady_clock::now();
  bool all_healthy = true;
  std::string issues;

  for (auto & [name, health] : node_health_) {
    double age_sec = std::chrono::duration<double>(now - health.last_seen).count();

    if (age_sec > node_timeout_sec_ && health.message_count > 0) {
      health.alive = false;
      all_healthy = false;
      issues += name + "(dead " + std::to_string(static_cast<int>(age_sec)) + "s) ";
    } else if (health.message_count == 0) {
      // Never received a message — might still be starting up
      double uptime = std::chrono::duration<double>(now - start_time_).count();
      if (uptime > 30.0) {
        // After 30s startup window, flag as issue
        health.alive = false;
        all_healthy = false;
        issues += name + "(never seen) ";
      }
    }
  }

  pipeline_healthy_ = all_healthy;

  if (!all_healthy) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "Pipeline issues: %s", issues.c_str());
  }
}

void PipelineWatchdogNode::publish_heartbeat()
{
  std_msgs::msg::Header heartbeat;
  heartbeat.stamp = this->now();
  heartbeat.frame_id = pipeline_healthy_ ? "healthy" : "degraded";
  heartbeat_pub_->publish(heartbeat);
}

void PipelineWatchdogNode::log_metrics()
{
  if (!metrics_log_.is_open()) return;

  std::lock_guard<std::mutex> lock(health_mutex_);
  auto now = std::chrono::steady_clock::now();
  double uptime = std::chrono::duration<double>(now - start_time_).count();

  auto time_now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(time_now);

  std::stringstream ts;
  ts << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");

  double rgb_hz = node_health_.count("rgb") ? node_health_["rgb"].last_rate_hz : 0.0;
  double depth_hz = node_health_.count("depth") ? node_health_["depth"].last_rate_hz : 0.0;
  double odom_hz = node_health_.count("odom") ? node_health_["odom"].last_rate_hz : 0.0;

  uint64_t rgb_msgs = node_health_.count("rgb") ? node_health_["rgb"].message_count : 0;
  uint64_t depth_msgs = node_health_.count("depth") ? node_health_["depth"].message_count : 0;
  uint64_t odom_msgs = node_health_.count("odom") ? node_health_["odom"].message_count : 0;

  metrics_log_ << ts.str() << ","
               << std::fixed << std::setprecision(1) << uptime << ","
               << (pipeline_healthy_ ? "true" : "false") << ","
               << std::setprecision(1) << rgb_hz << ","
               << depth_hz << ","
               << odom_hz << ","
               << restart_count_ << ","
               << rgb_msgs << ","
               << depth_msgs << ","
               << odom_msgs << std::endl;

  // Console summary every 60s
  static int log_counter = 0;
  if (++log_counter % 6 == 0) {
    RCLCPP_INFO(this->get_logger(),
      "Watchdog: uptime=%.0fs healthy=%s RGB=%.1fHz Depth=%.1fHz Odom=%.1fHz msgs=%lu/%lu/%lu",
      uptime, pipeline_healthy_ ? "YES" : "NO",
      rgb_hz, depth_hz, odom_hz, rgb_msgs, depth_msgs, odom_msgs);
  }
}

}  // namespace agv_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_slam::PipelineWatchdogNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
