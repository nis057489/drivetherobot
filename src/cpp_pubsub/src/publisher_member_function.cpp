// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

using namespace std::chrono_literals;

enum RobotState { MOVING_FORWARD, TURNING, RECOVERING };

class AutonomousCarterDriver : public rclcpp::Node
{
public:
  AutonomousCarterDriver()
  : Node("autonomous_carter_driver"), state_(MOVING_FORWARD), stable_clear_path_count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/front_3d_lidar/lidar_points", 10, 
      std::bind(&AutonomousCarterDriver::pointcloud_callback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      200ms, std::bind(&AutonomousCarterDriver::timer_callback, this));
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    float min_distance_ahead = std::numeric_limits<float>::infinity();
    float min_distance_left = std::numeric_limits<float>::infinity();
    float min_distance_right = std::numeric_limits<float>::infinity();

    const float forward_width = 0.5;
    const float min_distance_threshold = 0.2;  // Ignore extremely close noise
    const float side_detection_threshold = 1.0;

    for (const auto& point : cloud) {
      float distance = std::sqrt(point.x * point.x + point.y * point.y);
      if (distance < min_distance_threshold) {
        continue;  // Ignore noisy or irrelevant points
      }

      if (point.x > 0.0 && std::abs(point.y) < forward_width && point.z > -0.3 && point.z < 0.3) {
        if (distance < min_distance_ahead) {
          min_distance_ahead = distance;
        }
      } else if (point.x > 0.0 && point.y > 0.3) {
        if (distance < min_distance_right) {
          min_distance_right = distance;
        }
      } else if (point.x > 0.0 && point.y < -0.3) {
        if (distance < min_distance_left) {
          min_distance_left = distance;
        }
      }
    }

    obstacle_detected_ = (min_distance_ahead < 0.5);
    clear_direction_ = (min_distance_right > min_distance_left && min_distance_right > side_detection_threshold)
                         ? "right" : "left";

    RCLCPP_INFO(this->get_logger(), 
                "Ahead: %.2f m, Left: %.2f m, Right: %.2f m, Turn to: %s",
                min_distance_ahead, min_distance_left, min_distance_right, clear_direction_.c_str());
  }

  void timer_callback()
  {
    auto twist_msg = geometry_msgs::msg::Twist();

    switch (state_) {
      case MOVING_FORWARD:
        if (obstacle_detected_) {
          RCLCPP_INFO(this->get_logger(), "Obstacle detected! Switching to TURNING state.");
          state_ = TURNING;
          turn_timer_ = 5;
          stable_clear_path_count_ = 0;
        } else {
          twist_msg.linear.x = 0.5;
          twist_msg.angular.z = 0.0;
          forward_motion_timer_++;  // Track forward progress
          RCLCPP_INFO(this->get_logger(), "Path is clear, moving forward...");
        }

        // If the robot is stuck moving forward for too long without progress, switch to recovery
        if (forward_motion_timer_ > 10) {
          RCLCPP_WARN(this->get_logger(), "No progress detected, switching to RECOVERING state.");
          state_ = RECOVERING;
          recovery_timer_ = 5;
        }
        break;

      case TURNING:
        if (turn_timer_ > 0) {
          twist_msg.linear.x = 0.0;
          twist_msg.angular.z = (clear_direction_ == "right") ? -0.5 : 0.5;
          turn_timer_--;
          RCLCPP_INFO(this->get_logger(), "Turning %s... Remaining turns: %d", clear_direction_.c_str(), turn_timer_);
        } else {
          if (!obstacle_detected_) {
            stable_clear_path_count_++;
            if (stable_clear_path_count_ >= 3) {
              RCLCPP_INFO(this->get_logger(), "Turn complete. Stable path detected, switching to MOVING_FORWARD.");
              state_ = MOVING_FORWARD;
              forward_motion_timer_ = 0;  // Reset forward progress timer
            }
          } else {
            stable_clear_path_count_ = 0;
          }
        }
        break;

      case RECOVERING:
        if (recovery_timer_ > 0) {
          // Reverse and turn slightly
          twist_msg.linear.x = -0.2;
          twist_msg.angular.z = (clear_direction_ == "right") ? 0.3 : -0.3;
          recovery_timer_--;
          RCLCPP_INFO(this->get_logger(), "Recovering... Remaining recovery cycles: %d", recovery_timer_);
        } else {
          RCLCPP_INFO(this->get_logger(), "Recovery complete, switching to MOVING_FORWARD.");
          state_ = MOVING_FORWARD;
          forward_motion_timer_ = 0;
        }
        break;
    }

    publisher_->publish(twist_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  RobotState state_;
  bool obstacle_detected_;
  int turn_timer_;
  int stable_clear_path_count_;
  int forward_motion_timer_ = 0;
  int recovery_timer_ = 0;
  std::string clear_direction_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonomousCarterDriver>());
  rclcpp::shutdown();
  return 0;
}
