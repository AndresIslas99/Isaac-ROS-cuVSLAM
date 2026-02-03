#include "agv_slam/depth_filter_node.hpp"
#include <chrono>
#include <algorithm>

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
    "/zed/zed_node/rgb/camera_info", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      cam_info_pub_->publish(*msg);
    });

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

  // Convert to OpenCV
  cv_bridge::CvImagePtr depth_cv;
  try {
    depth_cv = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "cv_bridge depth error: %s", e.what());
    return;
  }

  cv::Mat & depth = depth_cv->image;

  // ── Stage 1: PassThrough + NaN removal ──
  uint64_t removed_this_frame = 0;
  apply_confidence_filter(depth, min_depth_, max_depth_);

  // Count removed points for diagnostics
  int total_pixels = depth.rows * depth.cols;
  int valid_pixels = cv::countNonZero(depth == depth);  // NaN != NaN
  removed_this_frame = total_pixels - valid_pixels;

  // ── Stage 2: Temporal averaging (reduces flickering noise) ──
  if (enable_temporal_ && temporal_window_ > 1) {
    apply_temporal_filter(depth);
  }

  // ── Stage 3: Bilateral smoothing (edge-preserving) ──
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

  // ── Publish RGB passthrough (same timestamp for sync) ──
  filtered_rgb_pub_->publish(*rgb_msg);

  // ── Diagnostics ──
  frames_processed_++;
  points_removed_ += removed_this_frame;

  if (frames_processed_ % 100 == 0) {
    auto t_end = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    double removal_pct = (static_cast<double>(removed_this_frame) / total_pixels) * 100.0;

    RCLCPP_INFO(this->get_logger(),
      "Frame %lu: %.1fms, removed %.1f%% invalid pixels, %d valid remaining",
      frames_processed_, ms, removal_pct, valid_pixels);
  }
}

void DepthFilterNode::apply_confidence_filter(
  cv::Mat & depth, float min_val, float max_val)
{
  // Set out-of-range and NaN pixels to 0 (invalid)
  // This is faster than per-pixel if/else using OpenCV parallel ops
  cv::Mat mask_nan;
  cv::Mat mask_range;

  // NaN mask: NaN != NaN is true
  mask_nan = (depth != depth);

  // Range mask
  mask_range = (depth < min_val) | (depth > max_val);

  // Combined invalid mask
  cv::Mat invalid = mask_nan | mask_range;

  // Set invalid pixels to NaN (RTAB-Map expects NaN for missing depth)
  depth.setTo(std::numeric_limits<float>::quiet_NaN(), invalid);
}

void DepthFilterNode::apply_temporal_filter(cv::Mat & depth)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  // Add current frame to buffer
  temporal_buffer_.push_back(depth.clone());

  // Keep only N frames
  while (static_cast<int>(temporal_buffer_.size()) > temporal_window_) {
    temporal_buffer_.pop_front();
  }

  if (temporal_buffer_.size() < 2) {
    return;
  }

  // Compute median of valid values per pixel
  // For efficiency, use running average instead of true median
  cv::Mat accumulator = cv::Mat::zeros(depth.size(), CV_32FC1);
  cv::Mat count = cv::Mat::zeros(depth.size(), CV_32FC1);

  for (const auto & frame : temporal_buffer_) {
    cv::Mat valid_mask = (frame == frame);  // Not NaN
    cv::Mat valid_float;
    valid_mask.convertTo(valid_float, CV_32FC1, 1.0 / 255.0);

    cv::Mat clean_frame = frame.clone();
    clean_frame.setTo(0, ~valid_mask);

    accumulator += clean_frame;
    count += valid_float;
  }

  // Avoid division by zero
  cv::Mat safe_count = cv::max(count, 1.0f);
  cv::Mat averaged = accumulator / safe_count;

  // Restore NaN where no frames had valid data
  cv::Mat no_data = (count < 0.5f);
  averaged.setTo(std::numeric_limits<float>::quiet_NaN(), no_data);

  depth = averaged;
}

void DepthFilterNode::apply_bilateral_filter(
  const cv::Mat & input, cv::Mat & output)
{
  // Bilateral filter doesn't handle NaN — replace with 0 temporarily
  cv::Mat clean_input = input.clone();
  cv::Mat nan_mask = (input != input);
  clean_input.setTo(0, nan_mask);

  cv::bilateralFilter(clean_input, output, bilateral_d_,
    bilateral_sigma_color_, bilateral_sigma_space_);

  // Restore NaN positions
  output.setTo(std::numeric_limits<float>::quiet_NaN(), nan_mask);
}

void DepthFilterNode::apply_hole_filling(cv::Mat & depth)
{
  // Morphological closing fills small holes
  cv::Mat valid_mask = (depth == depth);  // Not NaN
  cv::Mat binary;
  valid_mask.convertTo(binary, CV_8UC1);

  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(hole_kernel_size_, hole_kernel_size_));

  cv::Mat closed;
  cv::morphologyEx(binary, closed, cv::MORPH_CLOSE, kernel);

  // For newly filled pixels, interpolate from neighbors
  cv::Mat new_pixels = closed & ~binary;
  if (cv::countNonZero(new_pixels) == 0) return;

  // Use inpainting for the small holes (Telea method, fast)
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

  // Convert back to float depth
  cv::Mat inpainted_float;
  inpainted.convertTo(inpainted_float, CV_32FC1);
  inpainted_float = inpainted_float / 255.0 * (max_val - min_val) + min_val;

  // Only fill the new pixels
  inpainted_float.copyTo(depth, new_pixels);
}

}  // namespace agv_slam

// ── Main ──
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<agv_slam::DepthFilterNode>();

  // Use multi-threaded executor for parallel callback processing
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
