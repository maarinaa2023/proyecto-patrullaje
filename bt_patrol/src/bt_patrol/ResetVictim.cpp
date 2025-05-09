#include <string>
#include <iostream>

#include "bt_patrol/ResetVictim.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "patrol_msgs/msg/patrol_msgs.hpp"
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

  reset_pub_ = node_->create_publisher<patrol_msgs::msg::PatrolMsgs>("/topic", 100);
}

BT::NodeStatus
ResetVictim::tick()
{
  patrol_msgs::msg::PatrolMsgs reset_msgs;
  reset_msgs.victim_report = false;
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
