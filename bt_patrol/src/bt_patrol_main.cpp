#include <string>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("patrol_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("check_report_node"));
  factory.registerFromPlugin(loader.getOSName("everyone_here_node"));
  factory.registerFromPlugin(loader.getOSName("report_victim_node"));
  factory.registerFromPlugin(loader.getOSName("reset_victim_node"));
  factory.registerFromPlugin(loader.getOSName("get_waypoint_node"));
  factory.registerFromPlugin(loader.getOSName("move_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_patrol");
  std::string xml_file = pkgpath + "/behaviour_tree_xml/patrol.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
