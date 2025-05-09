#ifndef BT_PATROL__RESET_VICTIM_HPP_
#define BT_PATROL__RESET_VICTIM_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "patrol_msgs/msg/patrol_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

class ResetVictim : public BT::ActionNodeBase
{
public:
  explicit ResetVictim(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<patrol_msgs::msg::PatrolMsgs>::SharedPtr reset_pub_;
};

}  // namespace bt_patrol

#endif  // BT_PATROL__REPORT_VICTIM_HPP_
