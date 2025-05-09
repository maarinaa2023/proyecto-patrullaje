#include <string>
#include <iostream>

#include "bt_patrol/ReportVictim.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "patrol_msgs/msg/patrol_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

using namespace std::chrono_literals;

ReportVictim::ReportVictim(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  report_pub_ = node_->create_publisher<patrol_msgs::msg::PatrolMsgs>("/topic", 100);
}

BT::NodeStatus
ReportVictim::tick()
{
  patrol_msgs::msg::PatrolMsgs report_msgs;
  report_msgs.victim_report = true;
  report_pub_->publish(report_msgs);

  wp_ = generate_random_pose();
  setOutput("waypoint", wp_);

  RCLCPP_INFO(node_->get_logger(), "Victim report status changed");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_patrol

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_patrol::ReportVictim>("ReportVictim");
}
