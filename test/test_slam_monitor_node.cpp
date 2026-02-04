#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "agv_slam/slam_monitor_node.hpp"
#include <chrono>
#include <memory>
#include <atomic>

class SlamMonitorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    monitor_node_ = std::make_shared<agv_slam::SlamMonitorNode>();
    helper_node_ = rclcpp::Node::make_shared("test_helper");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(monitor_node_);
    executor_->add_node(helper_node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_->remove_node(helper_node_);
    executor_->remove_node(monitor_node_);
    helper_node_.reset();
    monitor_node_.reset();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<agv_slam::SlamMonitorNode> monitor_node_;
  std::shared_ptr<rclcpp::Node> helper_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(SlamMonitorTest, NodeCreation)
{
  EXPECT_EQ(std::string(monitor_node_->get_name()), "slam_monitor");
}

TEST_F(SlamMonitorTest, DiagnosticsPublished)
{
  std::atomic<bool> received{false};
  auto sub = helper_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10,
    [&received](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      if (msg->status.size() >= 2) {
        received = true;
      }
    });

  // Diagnostics timer fires at 1Hz
  spin_for(std::chrono::milliseconds(2500));
  EXPECT_TRUE(received.load()) << "No diagnostics message received within 2.5 seconds";
}

TEST_F(SlamMonitorTest, OdomDistanceTracking)
{
  auto odom_pub = helper_node_->create_publisher<nav_msgs::msg::Odometry>(
    "/visual_slam/tracking/odometry", rclcpp::SensorDataQoS());

  // First odom at origin
  nav_msgs::msg::Odometry msg1;
  msg1.pose.pose.position.x = 0.0;
  msg1.pose.pose.position.y = 0.0;
  odom_pub->publish(msg1);
  spin_for(std::chrono::milliseconds(200));

  // Second odom at (3, 4) — distance should be 5.0
  nav_msgs::msg::Odometry msg2;
  msg2.pose.pose.position.x = 3.0;
  msg2.pose.pose.position.y = 4.0;
  odom_pub->publish(msg2);
  spin_for(std::chrono::milliseconds(200));

  // Wait for next diagnostics publish cycle
  std::atomic<bool> found_distance{false};
  auto sub = helper_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10,
    [&found_distance](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      for (const auto & status : msg->status) {
        for (const auto & kv : status.values) {
          if (kv.key == "Distance Traveled (m)") {
            double dist = std::stod(kv.value);
            if (dist >= 4.9 && dist <= 5.1) {
              found_distance = true;
            }
          }
        }
      }
    });

  spin_for(std::chrono::milliseconds(2000));
  EXPECT_TRUE(found_distance.load()) << "Expected distance ~5.0m in diagnostics";
}

TEST_F(SlamMonitorTest, LowRateWarning)
{
  std::atomic<bool> found_warn{false};
  auto sub = helper_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "/slam/diagnostics", 10,
    [&found_warn](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
      for (const auto & status : msg->status) {
        if (status.name == "SLAM/Sensors" &&
            status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
          found_warn = true;
        }
      }
    });

  spin_for(std::chrono::milliseconds(2500));
  EXPECT_TRUE(found_warn.load()) << "Expected WARN level when no sensor data flowing";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
