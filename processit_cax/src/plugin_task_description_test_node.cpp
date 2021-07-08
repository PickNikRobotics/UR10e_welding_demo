#include "rclcpp/rclcpp.hpp"

// C++
#include <thread>
#include <fstream>
#include <string>

// ROS
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;
const rclcpp::Logger LOGGER = rclcpp::get_logger("plugin_task_description_test_node");

class PluginTaskDescriptionTestNode
{
public:
  PluginTaskDescriptionTestNode(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
  }

  void run()
  {
    namespace rvt = rviz_visual_tools;
    rviz_visual_tools::RvizVisualTools visual_tools("world", "plugin_task_description", node_);
    visual_tools.deleteAllMarkers();

    geometry_msgs::msg::Pose workpiece_pose;
    std::string filename;
    node_->get_parameter("plugin_task_description_test_node.scene", filename);
    RCLCPP_INFO(LOGGER, "Workpiece filename" + filename);
    // visual_tools.publishMesh(workpiece_pose

    // );
    visual_tools.trigger();
    // TODO Add hardcoded test case
    // - call service and load exemplary task description
    // - load workpiece to planning scene and publish workpiece tf
  }

private:
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("plugin_task_description_test_node");

  PluginTaskDescriptionTestNode plugin_task_description_test_node(node);
  std::thread run_plugin_task_description_test([&plugin_task_description_test_node]() {
    // rclcpp::sleep_for(5s);
    plugin_task_description_test_node.run();
  });

  rclcpp::spin(node);
  run_plugin_task_description_test.join();

  return 0;
}
