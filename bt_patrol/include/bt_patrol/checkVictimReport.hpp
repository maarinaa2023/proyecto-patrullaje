#ifndef BT_PATROL__CHECKVICTIMREPORT_HPP_
#define BT_PATROL__CHECKVICTIMREPORT_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "bt_patrullaje/msg/patrol_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrol
{

class checkVictimReport : public BT::ConditionNode
{
public:
  explicit checkVictimReport(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        // no input needed
        //BT::InputPort<double>("distance")
      });
  }

  // void laser_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);

private:
  rclcpp::Node::SharedPtr node_;
  //TO DO: add server_topics suscriber.... for the victim_report_state
  //rclcpp::Time last_reading_time_;
  //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  bt_patrullaje::msg::patrol_msgs::UniquePtr last_msg_; // Ultimo mensaje recibido

};

}  // namespace bt_patrol

#endif  // BT_PATROL__CHECKVICTIMREPORT_HPP_
