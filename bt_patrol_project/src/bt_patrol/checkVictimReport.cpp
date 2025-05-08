
#include <string>
#include <utility>

#include "bt_patrol/checkVictimReport.hpp"
#include "patrol_msgs/msg/patrol_msgs.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

// topic_name = "/patrol/victim_report_state"

using namespace std::chrono_literals;
using namespace std::placeholders;

checkVictimReport::(
  const std::string& name, 
    const BT::NodeConfiguration& config, 
    rclcpp::Node::SharedPtr node_ptr,
    const std::string& topic_name,
    std::function<bool(const patrol_msgs::msg::PatrolMsgs::SharedPtr)> condition_function)
  : BT::ConditionNode(name, config), 
    node_ptr_(node_ptr),
    topic_name_(topic_name),
    condition_function_(condition_function)
{
  // Create subscription to the topic
  subscription_ = node_ptr_->create_subscription<patrol_msgs::msg::PatrolMsgs>(
    topic_name_, 10, 
    std::bind(&checkVictimNode::topic_callback, this, std::placeholders::_1));
    
  RCLCPP_INFO(node_ptr_->get_logger(), "checkVictimNode subscribing to %s", topic_name_.c_str());
}

BT::PortsList checkVictimReport::providedPorts()
{
  // No ports required
  return {};
}

BT::NodeStatus checkVictimNode::tick()
{
  // Check if we've received any messages yet
  if (!message_received_) {
    RCLCPP_DEBUG(node_ptr_->get_logger(), "Waiting for message on topic %s", topic_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Return the result based on the last evaluation
  return condition_result_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void checkVictimNode::topic_callback(const patrol_msgs::msg::PatrolMsgs::SharedPtr msg)
{
  last_message_ = msg;
  message_received_ = true;
  
  // Evaluate the condition function with the received message
  condition_result_ = condition_function_(msg.victim_report);
  
  RCLCPP_DEBUG(node_ptr_->get_logger(), 
               "Received message on %s, condition evaluated to: %s", 
               topic_name_.c_str(), 
               condition_result_ ? "true" : "false");
}

}  // namespace bt_patrol
