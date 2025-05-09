#include <string>
#include <utility>

#include "bt_patrol/IsEveryoneHere.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "patrol_msgs/msg/patrol_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsEveryoneHere::IsEveryoneHere(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  report_sub_ = node_->create_subscription<patrol_msgs::msg::PatrolMsgs>(
    "/topic", 100, std::bind(&IsEveryoneHere::report_callback, this, _1));

}

void
IsEveryoneHere::report_callback(patrol_msgs::msg::PatrolMsgs::UniquePtr msg)
{
  last_report_ = msg->robot_in_location;
}

BT::NodeStatus
IsEveryoneHere::tick()
{
  if (last_report_) {
    RCLCPP_INFO(node_->get_logger(), "All robots in location!");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_patrol

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_patrol::IsEveryoneHere>("IsEveryoneHere");
}
