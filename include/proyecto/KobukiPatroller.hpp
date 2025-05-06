// Copyright 2024 Intelligent Robotics Lab
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

#ifndef KOBUKI_PATROLLER_HPP_
#define KOBUKI_PATROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>
#include <utility>

namespace proyecto
{

enum class State { GO_TO_ZONE, PATROL, SCAN_AT_POINT, STOPPED };

class KobukiPatroller : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    KobukiPatroller();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time rotate_start_time_;
    State state_;

    bool anomaly_detected_;
    bool goal_in_progress_;

    std::vector<std::pair<double, double>> patrol_polygon_;
    
    geometry_msgs::msg::PoseStamped create_pose(double x, double y);
    geometry_msgs::msg::PoseStamped generate_random_pose();
    bool point_in_polygon(double x, double y, const std::vector<std::pair<double, double>>& polygon);
    void state_machine();
    bool send_goal(const geometry_msgs::msg::PoseStamped &goal_pose);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

}  //namespace proyecto

#endif
