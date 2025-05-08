// Copyright 2019 Intelligent Robotics Lab
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

#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_bt/msg/patrol_message.hpp"

#include "nav_bt/UpdateLocation.hpp"

namespace nav_bt
{

UpdateLocation::UpdateLocation(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  pub_ = node_->create_publisher<nav_bt::msg::PatrolMessage>("patrol_message", 10);
}

void
UpdateLocation::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);

  patrol_msgs::msg::PatrolMessage msg;

  msg.victim_report = true;  // o lo que corresponda según tu lógica

  msg.victim_location = {
    static_cast<int32_t>(std::round(goal.pose.position.x)),
    static_cast<int32_t>(std::round(goal.pose.position.y))
  };
}

BT::NodeStatus
UpdateLocation::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "[UpdateLocation] Publicado victim_location: (%.2f, %.2f)",
              goal.pose.position.x, goal.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}


}  // namespace nav_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav_bt::UpdateLocation>("UpdateLocation");
}

