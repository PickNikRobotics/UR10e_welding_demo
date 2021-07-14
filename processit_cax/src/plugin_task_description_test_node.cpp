#include "rclcpp/rclcpp.hpp"

// C++
#include <thread>
#include <fstream>
#include <string>

// ROS
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
// #include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/pose.hpp>
#include <processit_msgs/srv/load_task_description.hpp>

// tmp
#include <ament_index_cpp/get_package_share_directory.hpp>

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
    // TODO Add hardcoded test case
    // - call service and load exemplary task description
    // - load workpiece to planning scene and publish workpiece tf

    std::string workpiece_path;
    node_->get_parameter("workpiece_path", workpiece_path);

    // ******************* Step 1 load workpiece
    // Initialize and setup moveit visual tools
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "world", "/moveit_visual_tools"));
    visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->loadMarkerPub(true);
    visual_tools_->setManualSceneUpdating();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->triggerPlanningSceneUpdate();

    Eigen::Isometry3d workpiece_pose = Eigen::Isometry3d::Identity();
    workpiece_pose.translation().x() = 0.1;
    workpiece_pose.translation().y() = -0.2;
    workpiece_pose.translation().z() = 0.71;
    std::string stl_file = "file://" + workpiece_path + ".STL";
    RCLCPP_INFO(LOGGER, "Loading mesh " + stl_file);
    visual_tools_->publishCollisionMesh(visual_tools_->convertPose(workpiece_pose), "Workpiece", stl_file,
                                        rviz_visual_tools::Colors::GREEN);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    visual_tools_->triggerPlanningSceneUpdate();

    // ******************* Step 2 load task description
    // Service call to load a task description
    rclcpp::Client<processit_msgs::srv::LoadTaskDescription>::SharedPtr client =
        node_->create_client<processit_msgs::srv::LoadTaskDescription>("plugin_task_description/load_task_description");

    auto request = std::make_shared<processit_msgs::srv::LoadTaskDescription::Request>();
    std::string task_file = workpiece_path + ".xml";
    RCLCPP_INFO(LOGGER, "Loading task " + task_file);
    request->task_description_file = task_file;

    while (!client->wait_for_service(2s))
    {
      RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    }

    processit_msgs::srv::LoadTaskDescription::Response::SharedPtr response;
    auto result = client->async_send_request(request);
    response = result.get();

    RCLCPP_INFO_STREAM(LOGGER, "success" << response->success);
  }

private:
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("plugin_task_description_test_node", "", node_options);

  PluginTaskDescriptionTestNode plugin_task_description_test_node(node);
  std::thread run_plugin_task_description_test([&plugin_task_description_test_node]() {
    rclcpp::sleep_for(5s);
    // Get parameter containing task description file name and run test node
    plugin_task_description_test_node.run();
  });

  rclcpp::spin(node);
  run_plugin_task_description_test.join();

  return 0;
}
