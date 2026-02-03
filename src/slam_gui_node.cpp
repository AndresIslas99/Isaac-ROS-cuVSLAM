#include "agv_slam/slam_gui_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <thread>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPixmap>
#include <QImage>

namespace agv_slam
{

// =============================================================================
// PointCloudGLWidget — OpenGL 3D Renderer
// =============================================================================

PointCloudGLWidget::PointCloudGLWidget(QWidget * parent)
: QOpenGLWidget(parent)
{
  setMinimumSize(640, 480);
}

void PointCloudGLWidget::updatePointCloud(
  const std::vector<float> & points, const std::vector<float> & colors)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  points_ = points;
  colors_ = colors;
  point_count_ = points.size() / 3;
  update();
}

void PointCloudGLWidget::updateTrajectory(const std::vector<float> & trajectory)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  trajectory_ = trajectory;
  trajectory_count_ = trajectory.size() / 3;
  update();
}

void PointCloudGLWidget::updateRobotPose(float x, float y, float z, float yaw)
{
  robot_x_ = x;
  robot_y_ = y;
  robot_z_ = z;
  robot_yaw_ = yaw;
  update();
}

void PointCloudGLWidget::initializeGL()
{
  initializeOpenGLFunctions();
  glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POINT_SMOOTH);
  glPointSize(2.0f);
}

void PointCloudGLWidget::resizeGL(int w, int h)
{
  glViewport(0, 0, w, h);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  float aspect = static_cast<float>(w) / static_cast<float>(h);
  float fovy = 60.0f;
  float near_plane = 0.1f;
  float far_plane = 100.0f;

  float top = near_plane * tanf(fovy * 0.5f * M_PI / 180.0f);
  float right = top * aspect;
  glFrustum(-right, right, -top, top, near_plane, far_plane);

  glMatrixMode(GL_MODELVIEW);
}

void PointCloudGLWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Camera position from orbit controls
  float az_rad = cam_azimuth_ * M_PI / 180.0f;
  float el_rad = cam_elevation_ * M_PI / 180.0f;

  float cam_x = cam_target_x_ + cam_distance_ * cosf(el_rad) * cosf(az_rad);
  float cam_y = cam_target_y_ + cam_distance_ * cosf(el_rad) * sinf(az_rad);
  float cam_z = cam_distance_ * sinf(el_rad);

  // gluLookAt equivalent
  float fx = cam_target_x_ - cam_x;
  float fy = cam_target_y_ - cam_y;
  float fz = -cam_z;
  float len = sqrtf(fx * fx + fy * fy + fz * fz);
  if (len > 0.001f) { fx /= len; fy /= len; fz /= len; }

  float ux = 0.0f, uy = 0.0f, uz = 1.0f;  // Up vector
  float sx = fy * uz - fz * uy;
  float sy = fz * ux - fx * uz;
  float sz = fx * uy - fy * ux;
  float uux = sy * fz - sz * fy;
  float uuy = sz * fx - sx * fz;
  float uuz = sx * fy - sy * fx;

  float m[16] = {
    sx, uux, -fx, 0,
    sy, uuy, -fy, 0,
    sz, uuz, -fz, 0,
    0,  0,   0,   1
  };
  glMultMatrixf(m);
  glTranslatef(-cam_x, -cam_y, -cam_z);

  std::lock_guard<std::mutex> lock(data_mutex_);

  // ── Draw grid floor ──
  glColor4f(0.3f, 0.3f, 0.3f, 0.5f);
  glBegin(GL_LINES);
  for (int i = -10; i <= 10; i++) {
    glVertex3f(static_cast<float>(i), -10.0f, 0.0f);
    glVertex3f(static_cast<float>(i), 10.0f, 0.0f);
    glVertex3f(-10.0f, static_cast<float>(i), 0.0f);
    glVertex3f(10.0f, static_cast<float>(i), 0.0f);
  }
  glEnd();

  // ── Draw point cloud ──
  if (point_count_ > 0) {
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, points_.data());

    if (colors_.size() == points_.size()) {
      glEnableClientState(GL_COLOR_ARRAY);
      glColorPointer(3, GL_FLOAT, 0, colors_.data());
    } else {
      glColor3f(0.4f, 0.8f, 0.4f);  // Default green
    }

    glDrawArrays(GL_POINTS, 0, point_count_);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);
  }

  // ── Draw trajectory ──
  if (trajectory_count_ > 1) {
    glColor3f(0.2f, 1.0f, 0.2f);
    glLineWidth(2.0f);
    glBegin(GL_LINE_STRIP);
    for (uint32_t i = 0; i < trajectory_count_; i++) {
      glVertex3f(trajectory_[i * 3], trajectory_[i * 3 + 1], trajectory_[i * 3 + 2]);
    }
    glEnd();
  }

  // ── Draw robot position ──
  glColor3f(1.0f, 0.3f, 0.3f);
  glPointSize(8.0f);
  glBegin(GL_POINTS);
  glVertex3f(robot_x_, robot_y_, robot_z_);
  glEnd();

  // Draw heading arrow
  float arrow_len = 0.5f;
  float ax = robot_x_ + arrow_len * cosf(robot_yaw_);
  float ay = robot_y_ + arrow_len * sinf(robot_yaw_);
  glColor3f(1.0f, 0.5f, 0.0f);
  glLineWidth(3.0f);
  glBegin(GL_LINES);
  glVertex3f(robot_x_, robot_y_, robot_z_ + 0.1f);
  glVertex3f(ax, ay, robot_z_ + 0.1f);
  glEnd();

  glPointSize(2.0f);  // Reset
  glLineWidth(1.0f);
}

void PointCloudGLWidget::mousePressEvent(QMouseEvent * event)
{
  last_mouse_pos_ = event->pos();
}

void PointCloudGLWidget::mouseMoveEvent(QMouseEvent * event)
{
  int dx = event->pos().x() - last_mouse_pos_.x();
  int dy = event->pos().y() - last_mouse_pos_.y();

  if (event->buttons() & Qt::LeftButton) {
    cam_azimuth_ += dx * 0.3f;
    cam_elevation_ += dy * 0.3f;
    cam_elevation_ = std::max(-89.0f, std::min(89.0f, cam_elevation_));
  } else if (event->buttons() & Qt::RightButton) {
    cam_target_x_ -= dx * 0.01f * cam_distance_;
    cam_target_y_ += dy * 0.01f * cam_distance_;
  }

  last_mouse_pos_ = event->pos();
  update();
}

void PointCloudGLWidget::wheelEvent(QWheelEvent * event)
{
  float delta = event->angleDelta().y() / 120.0f;
  cam_distance_ *= (1.0f - delta * 0.1f);
  cam_distance_ = std::max(0.5f, std::min(100.0f, cam_distance_));
  update();
}

// =============================================================================
// SlamGuiWindow — Main Application Window
// =============================================================================

SlamGuiWindow::SlamGuiWindow(QWidget * parent)
: QMainWindow(parent)
{
  setupUI();

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &SlamGuiWindow::refreshDisplay);
  refresh_timer_->start(33);  // ~30fps
}

void SlamGuiWindow::setupUI()
{
  setWindowTitle("AGV SLAM — Industrial Visualization");
  resize(1600, 900);

  auto * central = new QWidget(this);
  setCentralWidget(central);
  auto * main_layout = new QGridLayout(central);

  // ── 3D Point Cloud Viewer (top-left, spans 2 rows) ──
  gl_widget_ = new PointCloudGLWidget();
  main_layout->addWidget(gl_widget_, 0, 0, 2, 1);

  // ── Camera feeds (top-right) ──
  auto * camera_group = new QGroupBox("Camera");
  auto * camera_layout = new QGridLayout(camera_group);
  rgb_label_ = new QLabel("RGB: waiting...");
  rgb_label_->setMinimumSize(320, 180);
  rgb_label_->setAlignment(Qt::AlignCenter);
  rgb_label_->setStyleSheet("background-color: #1a1a2e; color: white;");
  depth_label_ = new QLabel("Depth: waiting...");
  depth_label_->setMinimumSize(320, 180);
  depth_label_->setAlignment(Qt::AlignCenter);
  depth_label_->setStyleSheet("background-color: #1a1a2e; color: white;");
  camera_layout->addWidget(rgb_label_, 0, 0);
  camera_layout->addWidget(depth_label_, 0, 1);
  main_layout->addWidget(camera_group, 0, 1);

  // ── Status Dashboard (bottom-right) ──
  auto * status_group = new QGroupBox("SLAM Status");
  auto * status_layout = new QGridLayout(status_group);

  auto make_label = [](const QString & text) {
    auto * label = new QLabel(text);
    label->setStyleSheet("font-family: monospace; font-size: 12pt;");
    return label;
  };

  rgb_hz_label_ = make_label("RGB:     -- Hz");
  depth_hz_label_ = make_label("Depth:   -- Hz");
  imu_hz_label_ = make_label("IMU:     -- Hz");
  odom_hz_label_ = make_label("Odom:    -- Hz");
  distance_label_ = make_label("Distance: -- m");
  nodes_label_ = make_label("Map Nodes: --");
  loops_label_ = make_label("Loop Closures: --");
  quality_label_ = make_label("Quality: --");
  status_label_ = make_label("Status: Initializing...");
  status_label_->setStyleSheet("font-family: monospace; font-size: 14pt; font-weight: bold;");

  quality_bar_ = new QProgressBar();
  quality_bar_->setRange(0, 100);
  quality_bar_->setValue(0);
  quality_bar_->setFormat("Quality: %p%");

  status_layout->addWidget(rgb_hz_label_, 0, 0);
  status_layout->addWidget(depth_hz_label_, 1, 0);
  status_layout->addWidget(imu_hz_label_, 2, 0);
  status_layout->addWidget(odom_hz_label_, 3, 0);
  status_layout->addWidget(distance_label_, 0, 1);
  status_layout->addWidget(nodes_label_, 1, 1);
  status_layout->addWidget(loops_label_, 2, 1);
  status_layout->addWidget(quality_label_, 3, 1);
  status_layout->addWidget(quality_bar_, 4, 0, 1, 2);
  status_layout->addWidget(status_label_, 5, 0, 1, 2);

  // ── Buttons ──
  auto * btn_layout = new QGridLayout();
  auto * save_btn = new QPushButton("Save Map");
  auto * load_btn = new QPushButton("Load Map");
  auto * reset_btn = new QPushButton("Reset Map");
  connect(save_btn, &QPushButton::clicked, this, &SlamGuiWindow::onSaveMap);
  connect(load_btn, &QPushButton::clicked, this, &SlamGuiWindow::onLoadMap);
  connect(reset_btn, &QPushButton::clicked, this, &SlamGuiWindow::onResetMap);
  btn_layout->addWidget(save_btn, 0, 0);
  btn_layout->addWidget(load_btn, 0, 1);
  btn_layout->addWidget(reset_btn, 0, 2);
  status_layout->addLayout(btn_layout, 6, 0, 1, 2);

  main_layout->addWidget(status_group, 1, 1);

  // ── 2D Grid (bottom, full width) ──
  grid_label_ = new QLabel("Occupancy Grid: waiting...");
  grid_label_->setMinimumHeight(150);
  grid_label_->setAlignment(Qt::AlignCenter);
  grid_label_->setStyleSheet("background-color: #1a1a2e; color: white;");
  main_layout->addWidget(grid_label_, 2, 0, 1, 2);

  // Layout ratios
  main_layout->setColumnStretch(0, 3);
  main_layout->setColumnStretch(1, 2);
  main_layout->setRowStretch(0, 2);
  main_layout->setRowStretch(1, 2);
  main_layout->setRowStretch(2, 1);

  // Status bar
  statusBar()->showMessage("AGV SLAM — Connecting to pipeline...");
}

void SlamGuiWindow::updateRGBImage(const cv::Mat & image)
{
  std::lock_guard<std::mutex> lock(gui_mutex_);
  latest_rgb_ = image.clone();
}

void SlamGuiWindow::updateDepthImage(const cv::Mat & image)
{
  std::lock_guard<std::mutex> lock(gui_mutex_);
  latest_depth_ = image.clone();
}

void SlamGuiWindow::updatePointCloud(
  const std::vector<float> & points, const std::vector<float> & colors)
{
  gl_widget_->updatePointCloud(points, colors);
}

void SlamGuiWindow::updateTrajectory(const std::vector<float> & trajectory)
{
  gl_widget_->updateTrajectory(trajectory);
}

void SlamGuiWindow::updateRobotPose(float x, float y, float z, float yaw)
{
  gl_widget_->updateRobotPose(x, y, z, yaw);
}

void SlamGuiWindow::updateOccupancyGrid(const nav_msgs::msg::OccupancyGrid & grid)
{
  int w = grid.info.width;
  int h = grid.info.height;
  if (w == 0 || h == 0) return;

  QImage qimg(w, h, QImage::Format_RGB888);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      int8_t cell = grid.data[y * w + x];
      QRgb color;
      if (cell < 0) {
        color = qRgb(40, 40, 60);      // Unknown: dark blue-gray
      } else if (cell > 50) {
        color = qRgb(20, 20, 30);      // Occupied: near black
      } else {
        color = qRgb(200, 200, 220);   // Free: light gray
      }
      qimg.setPixel(x, h - 1 - y, color);  // Flip Y
    }
  }

  QPixmap pixmap = QPixmap::fromImage(qimg).scaled(
    grid_label_->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
  grid_label_->setPixmap(pixmap);
}

void SlamGuiWindow::updateDiagnostics(
  const std::string & status_text,
  double rgb_hz, double depth_hz,
  double imu_hz, double odom_hz,
  double distance, double quality)
{
  rgb_hz_label_->setText(QString("RGB:     %1 Hz").arg(rgb_hz, 0, 'f', 1));
  depth_hz_label_->setText(QString("Depth:   %1 Hz").arg(depth_hz, 0, 'f', 1));
  imu_hz_label_->setText(QString("IMU:     %1 Hz").arg(imu_hz, 0, 'f', 0));
  odom_hz_label_->setText(QString("Odom:    %1 Hz").arg(odom_hz, 0, 'f', 1));
  distance_label_->setText(QString("Distance: %1 m").arg(distance, 0, 'f', 2));
  quality_label_->setText(QString("Quality: %1").arg(quality, 0, 'f', 2));
  status_label_->setText(QString::fromStdString(status_text));

  int quality_pct = static_cast<int>(quality * 100.0);
  quality_bar_->setValue(quality_pct);

  if (quality_pct > 70) {
    quality_bar_->setStyleSheet("QProgressBar::chunk { background-color: #27ae60; }");
    status_label_->setStyleSheet(
      "font-family: monospace; font-size: 14pt; font-weight: bold; color: #27ae60;");
  } else if (quality_pct > 40) {
    quality_bar_->setStyleSheet("QProgressBar::chunk { background-color: #f39c12; }");
    status_label_->setStyleSheet(
      "font-family: monospace; font-size: 14pt; font-weight: bold; color: #f39c12;");
  } else {
    quality_bar_->setStyleSheet("QProgressBar::chunk { background-color: #e74c3c; }");
    status_label_->setStyleSheet(
      "font-family: monospace; font-size: 14pt; font-weight: bold; color: #e74c3c;");
  }
}

void SlamGuiWindow::updateLoopClosures(int count)
{
  loops_label_->setText(QString("Loop Closures: %1").arg(count));
}

void SlamGuiWindow::refreshDisplay()
{
  std::lock_guard<std::mutex> lock(gui_mutex_);

  if (!latest_rgb_.empty()) {
    cv::Mat rgb_display;
    cv::resize(latest_rgb_, rgb_display, cv::Size(320, 180));
    QImage qimg(rgb_display.data, rgb_display.cols, rgb_display.rows,
                rgb_display.step, QImage::Format_RGB888);
    rgb_label_->setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
  }

  if (!latest_depth_.empty()) {
    cv::Mat depth_vis;
    cv::Mat depth_norm;
    double min_val, max_val;
    cv::Mat valid = (latest_depth_ == latest_depth_);  // Not NaN
    cv::minMaxLoc(latest_depth_, &min_val, &max_val, nullptr, nullptr, valid);
    if (max_val > min_val) {
      latest_depth_.convertTo(depth_norm, CV_8UC1, 255.0 / (max_val - min_val),
                              -min_val * 255.0 / (max_val - min_val));
    } else {
      depth_norm = cv::Mat::zeros(latest_depth_.size(), CV_8UC1);
    }
    cv::applyColorMap(depth_norm, depth_vis, cv::COLORMAP_JET);
    cv::resize(depth_vis, depth_vis, cv::Size(320, 180));
    QImage qimg(depth_vis.data, depth_vis.cols, depth_vis.rows,
                depth_vis.step, QImage::Format_RGB888);
    depth_label_->setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
  }
}

void SlamGuiWindow::onSaveMap()
{
  statusBar()->showMessage("Saving map... (call ros2 service)");
}

void SlamGuiWindow::onLoadMap()
{
  QString path = QFileDialog::getOpenFileName(
    this, "Load SLAM Map", "/mnt/ssd/slam_maps/", "Database (*.db)");
  if (!path.isEmpty()) {
    statusBar()->showMessage("Loading map: " + path);
  }
}

void SlamGuiWindow::onResetMap()
{
  statusBar()->showMessage("Map reset requested");
}

// =============================================================================
// SlamGuiNode — ROS2 Node bridging topics to Qt GUI
// =============================================================================

SlamGuiNode::SlamGuiNode(SlamGuiWindow * gui, const rclcpp::NodeOptions & options)
: Node("slam_gui", options), gui_(gui)
{
  auto qos = rclcpp::SensorDataQoS();

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/filtered/rgb", qos,
    std::bind(&SlamGuiNode::rgb_callback, this, std::placeholders::_1));

  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/filtered/depth", qos,
    std::bind(&SlamGuiNode::depth_callback, this, std::placeholders::_1));

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/rtabmap/cloud_map", qos,
    std::bind(&SlamGuiNode::cloud_callback, this, std::placeholders::_1));

  grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/rtabmap/grid_map", qos,
    std::bind(&SlamGuiNode::grid_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/visual_slam/tracking/odometry", qos,
    std::bind(&SlamGuiNode::odom_callback, this, std::placeholders::_1));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/visual_slam/tracking/slam_path", qos,
    std::bind(&SlamGuiNode::path_callback, this, std::placeholders::_1));

  diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10,
    std::bind(&SlamGuiNode::diag_callback, this, std::placeholders::_1));

  quality_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/slam/quality", 10,
    std::bind(&SlamGuiNode::quality_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "SLAM GUI node initialized");
}

void SlamGuiNode::rgb_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    auto cv_img = cv_bridge::toCvShare(msg, "bgr8");
    gui_->updateRGBImage(cv_img->image);
  } catch (...) {}
}

void SlamGuiNode::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    auto cv_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    gui_->updateDepthImage(cv_img->image);
  } catch (...) {}
}

void SlamGuiNode::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::vector<float> points;
  std::vector<float> colors;

  uint32_t count = msg->width * msg->height;
  uint32_t step = std::max(1u, count / 50000u);  // Limit to ~50k points for rendering

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // Note: RGB field in point cloud not used — height-based coloring instead

  points.reserve(count / step * 3);
  colors.reserve(count / step * 3);

  for (uint32_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    if (i % step != 0) continue;
    float x = *iter_x, y = *iter_y, z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    points.push_back(x);
    points.push_back(y);
    points.push_back(z);

    // Height-based coloring if no RGB
    float h = std::max(0.0f, std::min(2.0f, z)) / 2.0f;
    colors.push_back(h);
    colors.push_back(1.0f - h);
    colors.push_back(0.3f);
  }

  gui_->updatePointCloud(points, colors);
}

void SlamGuiNode::grid_callback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  gui_->updateOccupancyGrid(*msg);
}

void SlamGuiNode::odom_callback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  float z = msg->pose.pose.position.z;

  // Extract yaw from quaternion
  double siny_cosp = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                             msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
  float yaw = std::atan2(siny_cosp, cosy_cosp);

  gui_->updateRobotPose(x, y, z, yaw);
}

void SlamGuiNode::path_callback(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
  std::vector<float> trajectory;
  trajectory.reserve(msg->poses.size() * 3);

  for (const auto & pose : msg->poses) {
    trajectory.push_back(pose.pose.position.x);
    trajectory.push_back(pose.pose.position.y);
    trajectory.push_back(pose.pose.position.z);
  }

  gui_->updateTrajectory(trajectory);
}

void SlamGuiNode::diag_callback(
  const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & msg)
{
  double rgb_hz = 0, depth_hz = 0, imu_hz = 0, odom_hz = 0, distance = 0;
  std::string status_text = "OK";

  for (const auto & status : msg->status) {
    if (status.name == "SLAM/Sensors") {
      for (const auto & kv : status.values) {
        if (kv.key == "RGB Hz") rgb_hz = std::stod(kv.value);
        else if (kv.key == "Depth Hz") depth_hz = std::stod(kv.value);
        else if (kv.key == "IMU Hz") imu_hz = std::stod(kv.value);
        else if (kv.key == "cuVSLAM Odom Hz") odom_hz = std::stod(kv.value);
        else if (kv.key == "Distance Traveled (m)") distance = std::stod(kv.value);
      }
      status_text = status.message;
    }
  }

  gui_->updateDiagnostics(status_text, rgb_hz, depth_hz, imu_hz, odom_hz,
                           distance, quality_score_);
}

void SlamGuiNode::quality_callback(const std_msgs::msg::Float32::ConstSharedPtr & msg)
{
  quality_score_ = msg->data;
}

}  // namespace agv_slam

// =============================================================================
// Main — Qt + ROS2 dual event loop
// =============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // Apply dark theme
  app.setStyleSheet(
    "QMainWindow { background-color: #2d2d2d; }"
    "QGroupBox { color: #ecf0f1; border: 1px solid #555; border-radius: 4px; "
    "  margin-top: 8px; padding-top: 12px; font-weight: bold; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; }"
    "QLabel { color: #ecf0f1; }"
    "QPushButton { background-color: #34495e; color: white; border: 1px solid #555; "
    "  border-radius: 4px; padding: 6px 16px; }"
    "QPushButton:hover { background-color: #3d566e; }"
    "QProgressBar { border: 1px solid #555; border-radius: 3px; text-align: center; "
    "  color: white; background-color: #34495e; }"
    "QStatusBar { color: #bdc3c7; }"
  );

  auto gui = new agv_slam::SlamGuiWindow();
  gui->show();

  auto node = std::make_shared<agv_slam::SlamGuiNode>(gui);

  // Run ROS2 in a separate thread
  std::thread ros_thread([&node]() {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  });

  int result = app.exec();

  rclcpp::shutdown();
  if (ros_thread.joinable()) {
    ros_thread.join();
  }

  delete gui;
  return result;
}
