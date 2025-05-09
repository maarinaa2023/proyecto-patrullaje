#ifndef BT_PATROL__CHECK_REPORT_HPP_
#define BT_PATROL__CHECK_REPORT_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "patrol_msgs/msg/patrol_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

class CheckReport : public BT::ConditionNode
{
public:
  explicit CheckReport(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        // BT::InputPort<double>("distance")
      });
  }

  void report_callback(patrol_msgs::msg::PatrolMsgs::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time last_reading_time_;
  rclcpp::Subscription<patrol_msgs::msg::PatrolMsgs>::SharedPtr report_sub_;
  bool last_report_;
};

}  // namespace bt_patrol

#endif  // BT_PATROL__CHECK_REPORT_HPP_
