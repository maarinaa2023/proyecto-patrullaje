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

#ifndef NAV_BT__CHECKZONE_HPP_
#define NAV_BT__CHECKZONE_HPP_

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_bt/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav_bt
{

class CheckZone : public BT::ConditionNode
{
public:
  explicit CheckZone(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("current_pose")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
};


}  // namespace nav_bt

#endif  // NAV_BT__CHECKZONE_HPP_
