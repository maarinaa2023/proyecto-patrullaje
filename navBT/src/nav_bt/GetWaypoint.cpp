// Copyright 2021 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>
#include <random>
#include <limits>

#include "nav_bt/GetWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

namespace nav_bt
{

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);

  // Puntos para patrullar la zona A
  declare_parameter("active_zone", active_zone);
  get_parameter("active_zone", active_zone);

  polygon.clear();
  for (int i = 1; i <= 4; ++i) {
    std::string x_key = active_zone + ".pt" + std::to_string(i) + ".x";
    std::string y_key = active_zone + ".pt" + std::to_string(i) + ".y";

    double x, y;
    declare_parameter(x_key, rclcpp::ParameterValue(0.0));
    declare_parameter(y_key, rclcpp::ParameterValue(0.0));
    get_parameter(x_key, x);
    get_parameter(y_key, y);

    polygon.emplace_back(x, y);
  }
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
  wp_ = generate_random_pose();
  setOutput("waypoint", wp_);
  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped MoveInSection::generate_random_pose()
{
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & p : polygon) {
    min_x = std::min(min_x, p.first);
    max_x = std::max(max_x, p.first);
    min_y = std::min(min_y, p.second);
    max_y = std::max(max_y, p.second);
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> x_dist(min_x, max_x);
  std::uniform_real_distribution<> y_dist(min_y, max_y);

  // Cargar el mapa
  cv::Mat map_image = cv::imread("/src/kobuki/maps/aws_lab.pgm", cv::IMREAD_GRAYSCALE);
  double map_resolution = 0.05;  // cambiar según tu mapa.yaml
  double origin_x = -41.2;       // cambiar según tu mapa.yaml
  double origin_y = -44.8;

  if (map_image.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "No se pudo cargar el mapa para validación.");
    return create_pose(min_x, min_y);  // fallback
  }

  geometry_msgs::msg::PoseStamped pose;
  int max_attempts = 100;
  int attempts = 0;

  double rand_x, rand_y;
  while (attempts < max_attempts)
  {
    rand_x = x_dist(gen);
    rand_y = y_dist(gen);
    attempts++;

    if (!point_in_polygon(rand_x, rand_y, polygon)) continue;

    // Convertir coordenada mundial a pixel
    int pixel_x = static_cast<int>((rand_x - origin_x) / map_resolution);
    int pixel_y = map_image.rows - static_cast<int>((rand_y - origin_y) / map_resolution);

    if (pixel_x < 0 || pixel_x >= map_image.cols || pixel_y < 0 || pixel_y >= map_image.rows)
        continue;

    // Validar que el píxel no sea obstáculo (normalmente 0 es negro = obstáculo)
    if (map_image_.at<uchar>(py, px) > 200) {
      pose.header.frame_id = "map";
      pose.header.stamp = node_->now();
      pose.pose.position.x = rand_x;
      pose.pose.position.y = rand_y;
      pose.pose.orientation.w = 1.0;
      RCLCPP_INFO(node_->get_logger(), "[GetWaypoint] Punto válido encontrado.");
      return pose;
    }
  }

  RCLCPP_WARN(node_->get_logger(), "[GetWaypoint] No se encontró un punto válido. Usando default.");
  pose.header.frame_id = "map";
  pose.pose.position.x = min_x;
  pose.pose.position.y = min_y;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool MoveInSection::point_in_polygon(double x, double y, const std::vector<std::pair<double, double>> & polygon)
{
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


}  // namespace nav_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav_bt::GetWaypoint>("GetWaypoint");
}
