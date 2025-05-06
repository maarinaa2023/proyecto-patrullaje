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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <random>
#include <limits>

#include "proyecto/KobukiPatroller.hpp"

namespace proyecto
{

using namespace std::chrono_literals;

KobukiPatroller::KobukiPatroller()
    : Node("kobuki_patroller"), 
      state_(State::GO_TO_ZONE), 
      anomaly_detected_(false), 
      goal_in_progress_(false) {

    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&KobukiPatroller::image_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        500ms, std::bind(&KobukiPatroller::state_machine, this));

    // Puntos para patrullar la zona A
    patrol_polygon_ = {
        {-8.467, -0.545},
        {-7.641, 2.054},
        {-4.615, 1.902},
        {-3.254, -1.070}
    };
}

geometry_msgs::msg::PoseStamped KobukiPatroller::create_pose(double x, double y) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.w = 1.0; // Sin rotación
    return pose;
}

void KobukiPatroller::state_machine() {
    switch (state_) {
        case State::GO_TO_ZONE:
            if (!goal_in_progress_) {
                RCLCPP_INFO(this->get_logger(), "Yendo a Zona A...");
                send_goal(generate_random_pose());
                state_ = State::PATROL;
            }
            break;

        case State::PATROL:
            if (anomaly_detected_) {
                RCLCPP_WARN(this->get_logger(), "¡Anomalía detectada! Deteniéndose.");
                state_ = State::STOPPED;
                break;
            }

            if (!goal_in_progress_) {
                auto random_goal = generate_random_pose();
                RCLCPP_INFO(this->get_logger(), "Patrullando.");
                send_goal(random_goal );
            }
            break;

        case State::SCAN_AT_POINT: {
            if (anomaly_detected_) {
                RCLCPP_WARN(this->get_logger(), "¡Anomalía detectada! Deteniéndose.");
                state_ = State::STOPPED;
                break;
            }

            geometry_msgs::msg::Twist vel;
            vel.linear.x = 0.0;
            vel.angular.z = 0.5;
            vel_pub_->publish(vel);

            auto elapsed = (now() - rotate_start_time_).seconds();
            if (elapsed > 20.0) {  // ~una vuelta completa
                RCLCPP_INFO(this->get_logger(), "No se detectó nada. Continuando patrullaje.");
                vel.angular.z = 0.0;
                vel_pub_->publish(vel);
                state_ = State::PATROL;
            }
            break;
        }

        case State::STOPPED:
            RCLCPP_INFO(this->get_logger(), "Robot detenido por evento. Detección!");
            break;
    }
}

bool KobukiPatroller::send_goal(const geometry_msgs::msg::PoseStamped &goal_pose) {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "Servidor de navegación no disponible.");
        return false;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    goal_in_progress_ = true;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.result_callback = [this](const auto & result) {

        goal_in_progress_ = false;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Objetivo alcanzado con éxito.");
            rotate_start_time_ = now();
            state_ = State::SCAN_AT_POINT;
        } else {
            RCLCPP_WARN(this->get_logger(), "Fallo al alcanzar el objetivo.");
            state_ = State::PATROL;
        }

    };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
}

geometry_msgs::msg::PoseStamped KobukiPatroller::generate_random_pose() {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto& p : patrol_polygon_) {
        min_x = std::min(min_x, p.first);
        max_x = std::max(max_x, p.first);
        min_y = std::min(min_y, p.second);
        max_y = std::max(max_y, p.second);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(min_x, max_x);
    std::uniform_real_distribution<> y_dist(min_y, max_y);

    double rand_x, rand_y;
    do {
        rand_x = x_dist(gen);
        rand_y = y_dist(gen);
    } while (!point_in_polygon(rand_x, rand_y, patrol_polygon_));

    return create_pose(rand_x, rand_y);
}

bool KobukiPatroller::point_in_polygon(double x, double y, const std::vector<std::pair<double, double>>& polygon) {
    int n = polygon.size();
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon[i].first, yi = polygon[i].second;
        double xj = polygon[j].first, yj = polygon[j].second;

        bool intersect = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-9) + xi);
        if (intersect)
            inside = !inside;
    }
    return inside;
}

void KobukiPatroller::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        auto cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;

        cv::Mat hsv;
        cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask);

        int red_pixels = cv::countNonZero(mask);
        if (red_pixels > 5000) {
            anomaly_detected_ = true;
        }

    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
    }
}

} // namespace proyecto