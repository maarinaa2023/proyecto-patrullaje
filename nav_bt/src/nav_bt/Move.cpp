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

#include "nav_bt/Move.hpp"

namespace nav_bt
{

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: nav_bt::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
Move::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("goal", goal);

  // Validación adicional si quieres asegurarte de que el goal no sea cero
  if (std::isnan(goal.pose.position.x) || std::isnan(goal.pose.position.y)) {
    throw BT::RuntimeError("Goal position contains NaN");
  }

  goal_.pose = goal;

  RCLCPP_INFO(node_->get_logger(), "Moving to goal: [%.2f, %.2f]",
            goal.pose.position.x, goal.pose.position.y);
}

BT::NodeStatus
Move::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "** NAVIGATION SUCCEEDED **");

  return BT::NodeStatus::SUCCESS;
}


}  // namespace nav_bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav_bt::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<nav_bt::Move>(
    "Move", builder);
}
