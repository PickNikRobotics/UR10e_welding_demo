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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <processit_msgs/srv/load_task_description.hpp>
#include <processit_msgs/srv/add_pose_marker.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>

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
    addintmarker_client = node_->create_client<processit_msgs::srv::AddPoseMarker>("pose_marker/add_pose_marker");
  }

  std::string addPoseMarker(geometry_msgs::msg::Pose pose)
  {
    auto request = std::make_shared<processit_msgs::srv::AddPoseMarker::Request>();
    request->pose = pose;
    request->frame_id = "world";
    request->add_controls = false;
    request->scale = 0.1;
    request->marker_type = 2;
    request->marker_name = "pose_marker_";

    processit_msgs::srv::AddPoseMarker::Response::SharedPtr response;
    auto result = addintmarker_client->async_send_request(request);
    response = result.get();
    return response->int_marker_id;
  }

  std::string addLineMarker(int id, double length, geometry_msgs::msg::Pose pose)
  {
    // Add Pose Marker add start and end point
    auto request = std::make_shared<processit_msgs::srv::AddPoseMarker::Request>();
    request->pose = pose;
    request->frame_id = "world";
    request->add_controls = false;
    request->scale = length;
    request->marker_type = 3;
    request->marker_name = "seam_marker_";

    processit_msgs::srv::AddPoseMarker::Response::SharedPtr response;
    auto result = addintmarker_client->async_send_request(request);
    response = result.get();
    return response->int_marker_id;
  }

  geometry_msgs::msg::Pose getPose(Eigen::Vector3d& positionVector, Eigen::Quaternion<double>& q)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = positionVector[0];
    pose.position.y = positionVector[1];
    pose.position.z = positionVector[2];
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  void run()
  {
    // TODO Add hardcoded test case
    // - call service and load exemplary task description
    // - load workpiece to planning scene and publish workpiece tf

    std::string workpiece_path;
    node_->get_parameter("workpiece_path", workpiece_path);

    // ******************* Step 1 load workpiece
    // // Initialize and setup moveit visual tools
    // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    // visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(node_, "world", "/planning_scene"));
    // visual_tools_->loadPlanningSceneMonitor();
    // visual_tools_->loadMarkerPub(true);
    // visual_tools_->setManualSceneUpdating();
    // visual_tools_->deleteAllMarkers();
    // visual_tools_->removeAllCollisionObjects();
    // visual_tools_->triggerPlanningSceneUpdate();

    // TODO currently only hardcoded, get pose from planning scene instead
    Eigen::Isometry3d workpiece_pose = Eigen::Isometry3d::Identity();
    workpiece_pose.translation().x() = 0.2;
    workpiece_pose.translation().y() = -0.2;
    workpiece_pose.translation().z() = 0.71;
    Eigen::Quaternion<double> q(-0.7071068, 0, 0, 0.7071068);
    workpiece_pose = workpiece_pose.rotate(q);

    // std::string stl_file = "file://" + workpiece_path + ".STL";
    // RCLCPP_INFO(LOGGER, "Loading mesh '%s'", stl_file.c_str());
    // visual_tools_->publishCollisionMesh(visual_tools_->convertPose(workpiece_pose), "Workpiece", stl_file,
    //                                     rviz_visual_tools::Colors::GREEN);
    // rclcpp::sleep_for(std::chrono::milliseconds(1000));
    // visual_tools_->triggerPlanningSceneUpdate();

    // ******************* Step 2 load task description
    // Service call to load a task description and visualize them in RViz
    rclcpp::Client<processit_msgs::srv::LoadTaskDescription>::SharedPtr client =
        node_->create_client<processit_msgs::srv::LoadTaskDescription>("plugin_task_description/load_task_description");
    auto request = std::make_shared<processit_msgs::srv::LoadTaskDescription::Request>();

    // Set task description filename
    std::string task_file = workpiece_path + ".xml";
    RCLCPP_INFO(LOGGER, "Loading task '%s'", task_file.c_str());
    request->task_description_file = task_file;

    // Set workpiece pose (task description is relative to workpiece frame)
    geometry_msgs::msg::PoseStamped workpiece_pose_stamped;
    workpiece_pose_stamped.header.frame_id = "world";
    workpiece_pose_stamped.pose = tf2::toMsg(workpiece_pose);
    request->workpiece_pose = workpiece_pose_stamped;

    while (!client->wait_for_service(1s))
    {
      RCLCPP_INFO(LOGGER, "service not available, waiting again...");
    }

    processit_msgs::srv::LoadTaskDescription::Response::SharedPtr response;
    auto result = client->async_send_request(request);
    response = result.get();

    for (auto const& weld_seam : response->weld_seams)
    {
      for (auto const& pose : weld_seam.poses)
      {
        RCLCPP_INFO(LOGGER, "Weld seam position [x,y,z]: %f, %f, %f", pose.position.x, pose.position.y, pose.position.z);
        addPoseMarker(pose);
      }
    }

    RCLCPP_INFO_STREAM(LOGGER, "success" << response->success);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedPtr addintmarker_client;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("plugin_task_description_test_node", "", node_options);

  PluginTaskDescriptionTestNode plugin_task_description_test_node(node);
  std::thread run_plugin_task_description_test([&plugin_task_description_test_node]() {
    rclcpp::sleep_for(10s);
    // Get parameter containing task description file name and run test node
    plugin_task_description_test_node.run();
  });

  rclcpp::spin(node);
  run_plugin_task_description_test.join();

  return 0;
}
