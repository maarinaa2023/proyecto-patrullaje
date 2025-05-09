#include <string>
#include <iostream>

#include "bt_patrol/ResetVictim.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

using namespace std::chrono_literals;

ResetVictim::ResetVictim(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  reset_pub_ = node_->create_publisher<std_msgs::msg::String>("/detections/zone", 100);
}

BT::NodeStatus
ResetVictim::tick()
{
  std_msgs::msg::String reset_msgs;
  reset_msgs.data = NULL;
  reset_pub_->publish(reset_msgs);

  RCLCPP_INFO(node_->get_logger(), "Victim report status changed to false");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_patrol

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_patrol::ResetVictim>("ResetVictim");
}
