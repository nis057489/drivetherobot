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
#include <cmath>
#include <vector>
#include <functional>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

using namespace std::chrono_literals;

class VFHNavigator : public rclcpp::Node
{
public:
  VFHNavigator()
  : Node("vfh_navigator")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/front_3d_lidar/lidar_points", 10, 
      std::bind(&VFHNavigator::pointcloud_callback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      200ms, std::bind(&VFHNavigator::navigate, this));
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    min_distance_ahead_ = std::numeric_limits<float>::infinity();  // Reset forward obstacle distance

    for (const auto& point : cloud) {
        if (point.x > 0.2 && point.x < 3.0 && point.z > -1.0 && point.z < 2.0) {
            float distance = std::sqrt(point.x * point.x + point.y * point.y);

            float angle = std::atan2(point.y, point.x) * 180.0 / M_PI;
            if (std::abs(angle - 180) < 30) {  // Within 30 degrees of forward direction
                if (distance < min_distance_ahead_) {
                    min_distance_ahead_ = distance;
                }
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Min distance ahead after processing: %.2f meters", min_distance_ahead_);
  }

  void navigate()
  {
    auto twist_msg = geometry_msgs::msg::Twist();

    if (min_distance_ahead_ < 1.0) {
      RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping and turning.");
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 1.0;  // Turn to avoid obstacle
    } else {
      twist_msg.linear.x = 0.3;  // Move forward
      twist_msg.angular.z = 0.0;
    }

    publisher_->publish(twist_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  float min_distance_ahead_;  // Closest obstacle directly ahead
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFHNavigator>());
  rclcpp::shutdown();
  return 0;
}
