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

#ifndef NAV_BT__GETWAYPOINT_HPP_
#define NAV_BT__GETWAYPOINT_HPP_

#include <utility>
#include <string>
#include <iostream>
#include <vector>
#include <random>
#include <limits>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav_bt
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
  explicit GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint")
      });
  }

private:
  geometry_msgs::msg::PoseStamped wp_;

  std::string active_zone;
  std::vector<std::pair<double, double>> polygon;

  rclcpp::Node::SharedPtr node_;

  geometry_msgs::msg::PoseStamped generate_random_pose();
  bool point_in_polygon(double x, double y, const std::vector<std::pair<double, double>> & polygon);
  geometry_msgs::msg::PoseStamped create_pose(double x, double y);
};

}  // namespace nav_bt

#endif  // NAV_BT__GETWAYPOINT_HPP_
