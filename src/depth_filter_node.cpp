#include "agv_slam/depth_filter_node.hpp"
#include <chrono>
#include <algorithm>
#include <sstream>
#include <cmath>

namespace agv_slam
{

DepthFilterNode::DepthFilterNode(const rclcpp::NodeOptions & options)
: Node("depth_filter", options)
{
  // ── Declare parameters ──
  this->declare_parameter("min_depth", 0.3);
  this->declare_parameter("max_depth", 10.0);
  this->declare_parameter("temporal_window", 3);
  this->declare_parameter("enable_bilateral", true);
  this->declare_parameter("bilateral_d", 5);
  this->declare_parameter("bilateral_sigma_color", 50.0);
  this->declare_parameter("bilateral_sigma_space", 50.0);
  this->declare_parameter("enable_hole_filling", true);
  this->declare_parameter("hole_kernel_size", 3);
  this->declare_parameter("enable_temporal", true);
  this->declare_parameter("input_depth_topic", "/zed/zed_node/depth/depth_registered");
  this->declare_parameter("input_rgb_topic", "/zed/zed_node/rgb/image_rect_color");
  this->declare_parameter("output_depth_topic", "/filtered/depth");
  this->declare_parameter("output_rgb_topic", "/filtered/rgb");

  // ── Read parameters ──
  min_depth_ = this->get_parameter("min_depth").as_double();
  max_depth_ = this->get_parameter("max_depth").as_double();
  temporal_window_ = this->get_parameter("temporal_window").as_int();
  enable_bilateral_ = this->get_parameter("enable_bilateral").as_bool();
  bilateral_d_ = this->get_parameter("bilateral_d").as_int();
  bilateral_sigma_color_ = this->get_parameter("bilateral_sigma_color").as_double();
  bilateral_sigma_space_ = this->get_parameter("bilateral_sigma_space").as_double();
  enable_hole_filling_ = this->get_parameter("enable_hole_filling").as_bool();
  hole_kernel_size_ = this->get_parameter("hole_kernel_size").as_int();
  enable_temporal_ = this->get_parameter("enable_temporal").as_bool();
  input_depth_topic_ = this->get_parameter("input_depth_topic").as_string();
  input_rgb_topic_ = this->get_parameter("input_rgb_topic").as_string();
  output_depth_topic_ = this->get_parameter("output_depth_topic").as_string();
  output_rgb_topic_ = this->get_parameter("output_rgb_topic").as_string();

  // ── Setup synchronized subscribers ──
  depth_sub_.subscribe(this, input_depth_topic_);
  rgb_sub_.subscribe(this, input_rgb_topic_);

  sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), depth_sub_, rgb_sub_);
  sync_->registerCallback(
    std::bind(&DepthFilterNode::depth_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  // ── Publishers ──
  filtered_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_depth_topic_, rclcpp::SensorDataQoS());
  filtered_rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    output_rgb_topic_, rclcpp::SensorDataQoS());

  // ── Camera info passthrough ──
  cam_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/filtered/camera_info", rclcpp::SensorDataQoS());
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/zed/zed_node/rgb/color/rect/camera_info", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      cam_info_pub_->publish(*msg);
    });

  // ── Monitoring publishers ──
  latency_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/depth_filter/latency", 10);
  quality_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/depth_filter/quality", 10);

  RCLCPP_INFO(this->get_logger(),
    "Depth filter initialized: range=[%.1f, %.1f]m, temporal=%d, bilateral=%s",
    min_depth_, max_depth_, temporal_window_,
    enable_bilateral_ ? "ON" : "OFF");
}

void DepthFilterNode::depth_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
{
  auto t_start = std::chrono::steady_clock::now();

  // Convert to OpenCV — use toCvCopy since we modify in-place
  cv_bridge::CvImagePtr depth_cv;
  try {
    depth_cv = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "cv_bridge depth error: %s", e.what());
    return;
  }

  cv::Mat & depth = depth_cv->image;
  int total_pixels = depth.rows * depth.cols;

  // ── Stage 1: PassThrough + NaN removal + quality metrics (single pass) ──
  // Combined: apply range filter AND compute valid count + depth sum in one pass
  int valid_pixels = 0;
  double depth_sum = 0.0;
  uint64_t removed_this_frame = 0;
  {
    const float min_d = min_depth_;
    const float max_d = max_depth_;
    const float nan_val = std::numeric_limits<float>::quiet_NaN();

    for (int r = 0; r < depth.rows; ++r) {
      float * row = depth.ptr<float>(r);
      for (int c = 0; c < depth.cols; ++c) {
        float d = row[c];
        // NaN check: d != d is true for NaN
        if (d != d || d < min_d || d > max_d) {
          row[c] = nan_val;
          removed_this_frame++;
        } else {
          valid_pixels++;
          depth_sum += static_cast<double>(d);
        }
      }
    }
  }

  // ── Stage 2: Temporal averaging ──
  if (enable_temporal_ && temporal_window_ > 1) {
    apply_temporal_filter(depth);
  }

  // ── Stage 3: Bilateral smoothing ──
  if (enable_bilateral_) {
    cv::Mat smoothed;
    apply_bilateral_filter(depth, smoothed);
    depth = smoothed;
  }

  // ── Stage 4: Morphological hole filling ──
  if (enable_hole_filling_) {
    apply_hole_filling(depth);
  }

  // ── Publish filtered depth ──
  filtered_depth_pub_->publish(*depth_cv->toImageMsg());

  // ── Publish RGB passthrough ──
  filtered_rgb_pub_->publish(*rgb_msg);

  // ── Timing ──
  auto t_end = std::chrono::steady_clock::now();
  double latency_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  // ── Publish latency ──
  std_msgs::msg::Float32 latency_msg;
  latency_msg.data = static_cast<float>(latency_ms);
  latency_pub_->publish(latency_msg);

  // ── Publish quality JSON (metrics already computed in Stage 1) ──
  double valid_ratio = (total_pixels > 0) ?
    static_cast<double>(valid_pixels) / static_cast<double>(total_pixels) : 0.0;
  double mean_depth = (valid_pixels > 0) ? depth_sum / valid_pixels : 0.0;

  {
    std::ostringstream json;
    json << std::fixed;
    json.precision(3);
    json << "{\"valid_ratio\":" << valid_ratio
         << ",\"mean_depth\":" << mean_depth
         << ",\"latency_ms\":" << latency_ms << "}";

    std_msgs::msg::String quality_msg;
    quality_msg.data = json.str();
    quality_pub_->publish(quality_msg);
  }

  // ── Diagnostics ──
  frames_processed_++;
  points_removed_ += removed_this_frame;

  if (frames_processed_ % 100 == 0) {
    double removal_pct = (static_cast<double>(removed_this_frame) / total_pixels) * 100.0;
    RCLCPP_INFO(this->get_logger(),
      "Frame %lu: %.1fms, removed %.1f%% invalid pixels, %d valid remaining",
      frames_processed_, latency_ms, removal_pct, valid_pixels);
  }
}

void DepthFilterNode::apply_confidence_filter(
  cv::Mat & depth, float min_val, float max_val)
{
  // Note: This function is kept for API compatibility but the main callback
  // now uses an optimized single-pass implementation instead.
  cv::Mat mask_nan = (depth != depth);
  cv::Mat mask_range = (depth < min_val) | (depth > max_val);
  cv::Mat invalid = mask_nan | mask_range;
  depth.setTo(std::numeric_limits<float>::quiet_NaN(), invalid);
}

void DepthFilterNode::apply_temporal_filter(cv::Mat & depth)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  temporal_buffer_.push_back(depth.clone());
  while (static_cast<int>(temporal_buffer_.size()) > temporal_window_) {
    temporal_buffer_.pop_front();
  }
  if (temporal_buffer_.size() < 2) return;

  cv::Mat accumulator = cv::Mat::zeros(depth.size(), CV_32FC1);
  cv::Mat count = cv::Mat::zeros(depth.size(), CV_32FC1);
  for (const auto & frame : temporal_buffer_) {
    cv::Mat valid_mask = (frame == frame);
    cv::Mat valid_float;
    valid_mask.convertTo(valid_float, CV_32FC1, 1.0 / 255.0);
    cv::Mat clean_frame = frame.clone();
    clean_frame.setTo(0, ~valid_mask);
    accumulator += clean_frame;
    count += valid_float;
  }
  cv::Mat safe_count = cv::max(count, 1.0f);
  cv::Mat averaged = accumulator / safe_count;
  averaged.setTo(std::numeric_limits<float>::quiet_NaN(), (count < 0.5f));
  depth = averaged;
}

void DepthFilterNode::apply_bilateral_filter(
  const cv::Mat & input, cv::Mat & output)
{
  cv::Mat clean_input = input.clone();
  cv::Mat nan_mask = (input != input);
  clean_input.setTo(0, nan_mask);
  cv::bilateralFilter(clean_input, output, bilateral_d_,
    bilateral_sigma_color_, bilateral_sigma_space_);
  output.setTo(std::numeric_limits<float>::quiet_NaN(), nan_mask);
}

void DepthFilterNode::apply_hole_filling(cv::Mat & depth)
{
  cv::Mat valid_mask = (depth == depth);
  cv::Mat binary;
  valid_mask.convertTo(binary, CV_8UC1);
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(hole_kernel_size_, hole_kernel_size_));
  cv::Mat closed;
  cv::morphologyEx(binary, closed, cv::MORPH_CLOSE, kernel);
  cv::Mat new_pixels = closed & ~binary;
  if (cv::countNonZero(new_pixels) == 0) return;

  cv::Mat depth_8u;
  double min_val, max_val;
  cv::minMaxLoc(depth, &min_val, &max_val, nullptr, nullptr, valid_mask);
  if (max_val <= min_val) return;
  cv::Mat normalized = (depth - min_val) / (max_val - min_val) * 255.0;
  normalized.setTo(0, ~valid_mask);
  normalized.convertTo(depth_8u, CV_8UC1);
  cv::Mat inpaint_mask;
  new_pixels.convertTo(inpaint_mask, CV_8UC1);
  cv::Mat inpainted;
  cv::inpaint(depth_8u, inpaint_mask, inpainted, 3, cv::INPAINT_TELEA);
  cv::Mat inpainted_float;
  inpainted.convertTo(inpainted_float, CV_32FC1);
  inpainted_float = inpainted_float / 255.0 * (max_val - min_val) + min_val;
  inpainted_float.copyTo(depth, new_pixels);
}

}  // namespace agv_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_slam::DepthFilterNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
