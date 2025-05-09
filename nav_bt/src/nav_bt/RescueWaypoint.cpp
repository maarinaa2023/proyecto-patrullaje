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

#include "nav_bt/RescueWaypoint.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

namespace nav_bt
{

RescueWaypoint::RescueWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  has_robot2_pose_(false)
{
  config().blackboard->get("node", node_);

  // Puntos para asistir en la zona indicada por otro kobuki
  zone_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/active_zone", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      active_zone = msg->data;
      RCLCPP_INFO(node_->get_logger(), "Zona activa actualizada desde el topic: %s", active_zone.c_str());
      this->load_zone_points();
    });

  robot2_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/robot/pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      robot2_pose_ = *msg;
      has_robot2_pose_ = true;
      RCLCPP_INFO(node_->get_logger(), "Pose de robot2 actualizada: x = %.3f, y = %.3f", robot2_pose_.pose.position.x, robot2_pose_.pose.position.y);
    });
  
}

void
RescueWaypoint::halt()
{
}

BT::NodeStatus
RescueWaypoint::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[RescueWaypoint] Ejecutando tick.");
  
  wp_ = generate_random_pose();
  setOutput("waypoint", wp_);
  return BT::NodeStatus::SUCCESS;
}

void RescueWaypoint::load_zone_points() {
  // Limpiar el polígono
  polygon.clear();
  for (int i = 1; i <= 4; ++i) {
    std::string x_key = active_zone + ".pt" + std::to_string(i) + ".x";
    std::string y_key = active_zone + ".pt" + std::to_string(i) + ".y";

    double x, y;
    node_->declare_parameter(x_key, rclcpp::ParameterValue(0.0));
    node_->declare_parameter(y_key, rclcpp::ParameterValue(0.0));
    node_->get_parameter(x_key, x);
    node_->get_parameter(y_key, y);

    RCLCPP_INFO(node_->get_logger(), "Punto %d: (%.3f, %.3f)", i, x, y);
    polygon.emplace_back(x, y);
  }
}

geometry_msgs::msg::PoseStamped RescueWaypoint::generate_random_pose()
{
  if (has_robot2_pose_) {
    std::uniform_real_distribution<> angle_dist(0, 2 * M_PI);
    std::random_device rd;
    std::mt19937 gen(rd());

    double angle = angle_dist(gen);
    double distance = 1.0;  // 1 metro
    double new_x = robot2_pose_.pose.position.x + distance * cos(angle);
    double new_y = robot2_pose_.pose.position.y + distance * sin(angle);

    // Verificar que esté dentro del polígono
    if (point_in_polygon(new_x, new_y, polygon)) {
      return create_pose(new_x, new_y);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Punto cerca de robot2 fuera del polígono. Usando fallback aleatorio.");
    }
  }

  // Si no se tiene la pose o es inválida, usar una posición aleatoria de la zona
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

  // Obtener la ruta del paquete que contiene el mapa
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("kobuki");

  // Construir la ruta completa al archivo del mapa

  std::string map_path = package_share_directory + "/maps/aws_lab.pgm";

  // Leer el mapa
  cv::Mat map_image = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    double map_resolution = 0.05;  // cambiar según tu mapa.yaml
    double origin_x = -9.62;       // cambiar según tu mapa.yaml
    double origin_y = -5.83;

    if (map_image.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "No se pudo cargar el mapa para validación.");
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

      // Imprimir los valores aleatorios generados
      RCLCPP_INFO(node_->get_logger(), "Intento %d: rand_x = %.3f, rand_y = %.3f", attempts, rand_x, rand_y);

      if (!point_in_polygon(rand_x, rand_y, polygon)) continue;

      // Convertir coordenada mundial a pixel
      int pixel_x = static_cast<int>((rand_x - origin_x) / map_resolution);
      int pixel_y = map_image.rows - static_cast<int>((rand_y - origin_y) / map_resolution);

      // Imprimir las coordenadas de píxeles
      RCLCPP_INFO(node_->get_logger(), "Pixel coords: (%d, %d)", pixel_x, pixel_y);

      if (pixel_x < 0 || pixel_x >= map_image.cols || pixel_y < 0 || pixel_y >= map_image.rows)
          continue;

      // Validar que el píxel no sea obstáculo (normalmente 0 es negro = obstáculo)
      int pixel_value = static_cast<int>(map_image.at<uchar>(pixel_y, pixel_x));
      RCLCPP_INFO(node_->get_logger(), "Valor del píxel: %d", pixel_value);

      // Validar que el píxel no sea obstáculo (normalmente 0 es negro = obstáculo)
      if (map_image.at<uchar>(pixel_y, pixel_x) > 100) {
        pose.header.frame_id = "map";
        pose.header.stamp = node_->now();
        pose.pose.position.x = rand_x;
        pose.pose.position.y = rand_y;
        pose.pose.orientation.w = 1.0;
        RCLCPP_INFO(node_->get_logger(), "[RescueWaypoint] Punto válido encontrado.");
        return pose;
      }
    }

    RCLCPP_WARN(node_->get_logger(), "[RescueWaypoint] No se encontró un punto válido. Usando default.");
    pose.header.frame_id = "map";
    pose.pose.position.x = min_x;
    pose.pose.position.y = min_y;
    pose.pose.orientation.w = 1.0;
    return pose;
}

bool RescueWaypoint::point_in_polygon(double x, double y, const std::vector<std::pair<double, double>> & polygon)
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

geometry_msgs::msg::PoseStamped RescueWaypoint::create_pose(double x, double y) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.w = 1.0; // Sin rotación
  return pose;
}


}  // namespace nav_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav_bt::RescueWaypoint>("RescueWaypoint");
}
