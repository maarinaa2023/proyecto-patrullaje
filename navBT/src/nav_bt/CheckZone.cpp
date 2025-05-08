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

#include "nav_bt/CheckZone.hpp"

namespace nav_bt
{

CheckZone::CheckZone(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  rclcpp::Node::SharedPtr node;
  config().blackboard->get("node", node);
}

void
CheckZone::halt()
{
}

BT::NodeStatus
CheckZone::tick()
{
  geometry_msgs::msg::PoseStamped current_pose;

  // Define los límites de tu zona asignada
  double min_x = 1.0, max_x = 5.0;
  double min_y = -2.0, max_y = 2.0;

  double x = current_pose.pose.position.x;
  double y = current_pose.pose.position.y;

  bool in_zone = (x >= min_x && x <= max_x && y >= min_y && y <= max_y);

  RCLCPP_INFO(node_->get_logger(), "[ChackZone] Pose actual: (%.2f, %.2f), ¿en zona?: %s", x, y, in_zone ? "sí" : "no");

  return in_zone ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList 
CheckZone::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose") };
}


}  // namespace nav_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav_bt::CheckZone>("CheckZone");
}
