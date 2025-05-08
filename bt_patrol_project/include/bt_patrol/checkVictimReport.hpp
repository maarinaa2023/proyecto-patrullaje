#ifndef CHECK_VICTIM_REPORT_HPP
#define CHECK_VICTIM_REPORT_HPP

#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "patrol_msgs/msg/patrol_msgs.hpp" 

namespace my_bt_patrol
{

class checkVictimReport : public BT::ConditionNode
{
public:
  checkVictimReport(const std::string& name, 
                     const BT::NodeConfiguration& config, 
                     rclcpp::Node::SharedPtr node_ptr,
                     const std::string& topic_name,
                     std::function<bool(const patrol_msgs::msg::PatrolMsgs::SharedPtr)> condition_function);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void topic_callback(const patrol_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Subscription<patrol_msgs::msg::String>::SharedPtr subscription_;
  std::string topic_name_;
  std::function<bool(const patrol_msgs::msg::String::SharedPtr)> condition_function_;
  patrol_msgs::msg::String::SharedPtr last_message_;
  bool message_received_ = false;
  bool condition_result_ = false;
};

}  // namespace my_bt_patrol

#endif  // CHECK_VICTIM_REPORT_HPP