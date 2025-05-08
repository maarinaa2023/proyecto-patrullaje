
#include <string>
#include <utility>

#include "bt_patrol/checkVictimReport.hpp"
#include "patrol_msgs/msg/patrol_msgs.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

using namespace std::chrono_literals;
using namespace std::placeholders;

checkVictimReport::(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // TO DO: get topic info from victim_report_state
  
  comms_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/topic", 100, std::bind(&checkVictimReport::report_state_callback, this, _1));

  last_reading_time_ = node_->now();
  
}

void
checkVictimReport::report_state_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

BT::NodeStatus
IsObstacle::tick()
{
  if (last_scan_ == nullptr) {
    return BT::NodeStatus::FAILURE;
  }
  double distance = 1.0;
  getInput("distance", distance);
  for (size_t pos = last_scan_->ranges.size() / 2 - 50;
    pos < last_scan_->ranges.size() / 2 + 50;
    pos++)
  {
    if (last_scan_->ranges[pos] < distance) {
      RCLCPP_INFO(
        node_->get_logger(), "Obstacle detected at %.2f meters (dist=%.2f, pos=%zu)",
        last_scan_->ranges[pos], distance, pos);
      return BT::NodeStatus::SUCCESS;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "No obstacle detected");
  return BT::NodeStatus::FAILURE;

}

}  // namespace bt_patrol

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_patrol::checkVictimReport>("checkVictimReport");
}

