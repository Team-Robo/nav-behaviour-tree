#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "nav_behaviour_tree/nav_to_pose.hpp"

using namespace std::chrono_literals;

// Helper: register a leaf that needs the rclcpp node in its constructor.
template<typename T>
static void registerRosLeaf(
  BT::BehaviorTreeFactory & factory, const std::string & tag,
  const rclcpp::Node::SharedPtr & node)
{
  BT::NodeBuilder builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, node);
    };
  factory.registerBuilder<T>(tag, builder);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("nav_behaviour_tree");

  const std::string share = ament_index_cpp::get_package_share_directory("nav_behaviour_tree");
  const std::string tree_id = node->declare_parameter<std::string>("tree_id", "MainTree");
  const double tick_hz = node->declare_parameter<double>("tick_hz", 20.0);

  BT::BehaviorTreeFactory factory;

  // --- (A) register leaves: XML tag string -> C++ class -------------------
  // Add new leaves here as you write them (see include/nav_behaviour_tree/).
  registerRosLeaf<nav_behaviour_tree::NavToPose>(factory, "NavToPose", node);

  // --- (B) register every tree file (subtrees first, then tasks) ----------
  // Add more registerBehaviorTreeFromFile() calls here as you split the tree
  // into subtrees / tasks under bt_xml/.
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/main_tree.xml");

  // --- (C) build the requested tree (resolves <SubTree> refs by ID) -------
  RCLCPP_INFO(node->get_logger(), "Building tree '%s'", tree_id.c_str());
  auto tree = factory.createTree(tree_id);

  rclcpp::Rate rate(tick_hz);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(
    node->get_logger(), "Tree '%s' finished: %s", tree_id.c_str(),
    status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

  rclcpp::shutdown();
  return 0;
}
