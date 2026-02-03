#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <mutex>

namespace agv_slam
{

class DepthFilterNode : public rclcpp::Node
{
public:
  explicit DepthFilterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback for synchronized RGB + Depth
  void depth_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg);

  // Apply confidence-based filtering
  void apply_confidence_filter(cv::Mat & depth, float min_val, float max_val);

  // Apply temporal averaging over N frames
  void apply_temporal_filter(cv::Mat & depth);

  // Apply bilateral edge-preserving smoothing
  void apply_bilateral_filter(const cv::Mat & input, cv::Mat & output);

  // Apply morphological closing to fill small holes
  void apply_hole_filling(cv::Mat & depth);

  // Subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;
  std::shared_ptr<Synchronizer> sync_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_rgb_pub_;

  // Camera info passthrough
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;

  // Temporal filter buffer
  std::deque<cv::Mat> temporal_buffer_;
  std::mutex buffer_mutex_;

  // Parameters
  float min_depth_;
  float max_depth_;
  int temporal_window_;
  bool enable_bilateral_;
  int bilateral_d_;
  double bilateral_sigma_color_;
  double bilateral_sigma_space_;
  bool enable_hole_filling_;
  int hole_kernel_size_;
  bool enable_temporal_;
  std::string input_depth_topic_;
  std::string input_rgb_topic_;
  std::string output_depth_topic_;
  std::string output_rgb_topic_;

  // Stats
  uint64_t frames_processed_ = 0;
  uint64_t points_removed_ = 0;
};

}  // namespace agv_slam
