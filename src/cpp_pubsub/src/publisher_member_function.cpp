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

    // Clear and resize the histogram for this scan
    histogram_.assign(36, 0.0);  // Coarser resolution: 10 degrees per bin

    // Build the histogram by processing each point
    for (const auto& point : cloud) {
      float distance = std::sqrt(point.x * point.x + point.y * point.y);
      if (distance < 0.2 || distance > 2.5) {
        continue;  // Ignore very close or far readings
      }

      // Determine the angle of the point relative to the robot
      int angle = static_cast<int>((std::atan2(point.y, point.x) * 180.0 / M_PI + 360)) % 360;
      int bin_index = angle / 10;  // Group into 10-degree bins

      // Amplify obstacle weight for nearby obstacles
      float obstacle_weight = std::min(1.0 / (distance * distance), 10.0);
      histogram_[bin_index] += obstacle_weight;
    }
  }

  void navigate()
  {
    // Normalize histogram values
    float max_density = *std::max_element(histogram_.begin(), histogram_.end());
    if (max_density > 0) {
      for (float &density : histogram_) {
        density /= max_density;
      }
    }

    // Find the clearest direction by minimizing obstacle and goal cost
    int best_bin = -1;
    float min_cost = std::numeric_limits<float>::infinity();

    for (int bin = 0; bin < 36; ++bin) {
      int angle = bin * 10;
      float obstacle_cost = histogram_[bin];
      float goal_cost = std::abs(angle - target_direction_) * 0.02;  // Favor directions toward the goal
      float total_cost = obstacle_cost + goal_cost;

      if (total_cost < min_cost) {
        min_cost = total_cost;
        best_bin = bin;
      }
    }

    // Generate velocity command based on the best direction
    auto twist_msg = geometry_msgs::msg::Twist();
    if (best_bin == -1 || min_cost > 0.8) {
      // No clear path found
      RCLCPP_WARN(this->get_logger(), "No clear path, stopping.");
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.0;
    } else {
      // Move in the best direction
      int best_direction = best_bin * 10;
      float angular_speed = (best_direction - 180) * 0.01;  // Convert to angular velocity
      RCLCPP_INFO(this->get_logger(), "Best direction: %d degrees, Angular Speed: %.2f", best_direction, angular_speed);

      twist_msg.linear.x = 0.4;
      twist_msg.angular.z = angular_speed;
    }

    publisher_->publish(twist_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  std::vector<float> histogram_;  // Polar histogram with 10-degree bins
  int target_direction_ = 180;    // Default to moving forward
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFHNavigator>());
  rclcpp::shutdown();
  return 0;
}
