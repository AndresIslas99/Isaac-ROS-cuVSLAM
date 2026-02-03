#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

#include <QApplication>
#include <QMainWindow>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QTimer>
#include <QLabel>
#include <QGridLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QProgressBar>
#include <QStatusBar>
#include <QMenuBar>
#include <QAction>
#include <QFileDialog>

#include <mutex>
#include <vector>
#include <deque>
#include <atomic>

namespace agv_slam
{

// ── 3D Point Cloud Viewer (OpenGL) ──
class PointCloudGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
  Q_OBJECT

public:
  explicit PointCloudGLWidget(QWidget * parent = nullptr);

  void updatePointCloud(const std::vector<float> & points,
                        const std::vector<float> & colors);
  void updateTrajectory(const std::vector<float> & trajectory);
  void updateRobotPose(float x, float y, float z, float yaw);

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  // Point cloud data
  std::vector<float> points_;     // x,y,z interleaved
  std::vector<float> colors_;     // r,g,b interleaved
  std::vector<float> trajectory_; // x,y,z interleaved
  std::mutex data_mutex_;

  // Robot pose
  float robot_x_ = 0.0f, robot_y_ = 0.0f, robot_z_ = 0.0f;
  float robot_yaw_ = 0.0f;

  // Camera controls
  float cam_distance_ = 10.0f;
  float cam_azimuth_ = 45.0f;
  float cam_elevation_ = 30.0f;
  float cam_target_x_ = 0.0f;
  float cam_target_y_ = 0.0f;
  QPoint last_mouse_pos_;

  // Rendering state
  uint32_t point_count_ = 0;
  uint32_t trajectory_count_ = 0;
};

// ── Main GUI Window ──
class SlamGuiWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit SlamGuiWindow(QWidget * parent = nullptr);

  // Update methods called from ROS thread
  void updatePointCloud(const std::vector<float> & points,
                        const std::vector<float> & colors);
  void updateTrajectory(const std::vector<float> & trajectory);
  void updateRobotPose(float x, float y, float z, float yaw);
  void updateRGBImage(const cv::Mat & image);
  void updateDepthImage(const cv::Mat & image);
  void updateOccupancyGrid(const nav_msgs::msg::OccupancyGrid & grid);
  void updateDiagnostics(const std::string & status_text,
                         double rgb_hz, double depth_hz,
                         double imu_hz, double odom_hz,
                         double distance, double quality);
  void updateLoopClosures(int count);

public slots:
  void onSaveMap();
  void onLoadMap();
  void onResetMap();

private:
  void setupUI();
  void refreshDisplay();

  // 3D viewer
  PointCloudGLWidget * gl_widget_;

  // Camera feeds
  QLabel * rgb_label_;
  QLabel * depth_label_;

  // 2D grid
  QLabel * grid_label_;

  // Status dashboard
  QLabel * rgb_hz_label_;
  QLabel * depth_hz_label_;
  QLabel * imu_hz_label_;
  QLabel * odom_hz_label_;
  QLabel * distance_label_;
  QLabel * nodes_label_;
  QLabel * loops_label_;
  QLabel * quality_label_;
  QLabel * status_label_;
  QProgressBar * quality_bar_;

  // Data mutex
  std::mutex gui_mutex_;
  cv::Mat latest_rgb_;
  cv::Mat latest_depth_;

  // Refresh timer
  QTimer * refresh_timer_;
};

// ── ROS2 Node that bridges topics to Qt GUI ──
class SlamGuiNode : public rclcpp::Node
{
public:
  explicit SlamGuiNode(SlamGuiWindow * gui,
                       const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  void grid_callback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void path_callback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void diag_callback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg);
  void quality_callback(const std_msgs::msg::Float32::ConstSharedPtr & msg);

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr quality_sub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // GUI reference
  SlamGuiWindow * gui_;

  // Rate tracking
  double quality_score_ = 0.0;
  int loop_closure_count_ = 0;
};

}  // namespace agv_slam
