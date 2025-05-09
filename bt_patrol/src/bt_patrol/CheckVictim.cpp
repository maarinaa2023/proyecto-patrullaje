#include <string>
#include <utility>

#include "bt_patrol/CheckVictim.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

using namespace std::chrono_literals;
using namespace std::placeholders;

CheckVictim::CheckVictim(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  report_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/detections/zone", 100, std::bind(&CheckVictim::report_callback, this, _1));

  last_reading_time_ = node_->now();
}

void
CheckVictim::report_callback(std_msgs::msg::String::UniquePtr msg)
{
  last_report_ = msg->data;
}

BT::NodeStatus
CheckVictim::tick()
{
  if (last_report_ != NULL) {
    RCLCPP_INFO(node_->get_logger(), "Victim reported!");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_patrol

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_patrol::CheckVictim>("CheckVictim");
}
