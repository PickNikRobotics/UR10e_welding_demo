#ifndef POSE_MARKER_H_
#define POSE_MARKER_H_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <string>

#include <processit_msgs/srv/add_pose_marker.hpp>
#include <processit_msgs/srv/edit_pose_marker.hpp>
#include <std_srvs/srv/empty.hpp>

namespace processit_program
{
class PoseMarker
{
public:
  PoseMarker(rclcpp::Node::SharedPtr& node);
  //~PoseMarker();

  void addPoseMarkerService(const std::shared_ptr<processit_msgs::srv::AddPoseMarker::Request> request,
                            std::shared_ptr<processit_msgs::srv::AddPoseMarker::Response> response);

  void editPoseMarkerService(const std::shared_ptr<processit_msgs::srv::EditPoseMarker::Request> request,
                             std::shared_ptr<processit_msgs::srv::EditPoseMarker::Response> response);

  void deleteAllPoseMarkersService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> response);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedpoint_sub;
  // rclcpp::Subscription<>::SharedPtr planningscene_sub;
  rclcpp::Service<processit_msgs::srv::AddPoseMarker>::SharedPtr addintmarker_service;
  rclcpp::Service<processit_msgs::srv::EditPoseMarker>::SharedPtr editintmarker_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr deleteallposemarkers_service;
  rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedPtr addintmarker_service_client;

  // MoveIt variables
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Variables for interactive markers
  interactive_markers::InteractiveMarkerServer int_marker_server;
  // interactive_markers::MenuHandler menu_handler;
  unsigned int pose_marker_counter = 0;
  // enum
  // {
  //     MENU_JOINT = 1u,
  //     SUB_MENU_PLAN_JOINT = 2u,
  //     SUB_MENU_EXECUTE_JOINT = 3u,
  //     SUB_MENU_PLAN_EXECUTE_JOINT = 4u,
  //     MENU_CARTESIAN = 5u,
  //     SUB_MENU_PLAN_CARTESIAN = 6u,
  //     SUB_MENU_EXECUTE_CARTESIAN = 7u,
  //     SUB_MENU_PLAN_EXECUTE_CARTESIAN = 8u,
  //     MENU_COPY_PASTE_POINT = 9u,
  //     MENU_REMOVE_POINT = 10u,
  // };

  // Initializers
  void initializeSubscribers();
  void initializePublishers();
  void initializeServices();
  // void initializeMenuHandler();

  // Helper functions for GUI
  visualization_msgs::msg::Marker makeBox(visualization_msgs::msg::InteractiveMarker& msg, int marker_type);
  visualization_msgs::msg::Marker makeArrow(visualization_msgs::msg::InteractiveMarker& msg, char direction);
  visualization_msgs::msg::InteractiveMarkerControl makeBoxControl(visualization_msgs::msg::InteractiveMarker& msg,
                                                                   int marker_type, bool add_controls = true);
  visualization_msgs::msg::InteractiveMarkerControl makeControl(unsigned int interaction_mode, double orientation[4],
                                                                std::string name);

  // Methods to handle the interactive markers
  // void copyPoseMarker(const std::string &marker_name);
  std::string addPoseMarker(const geometry_msgs::msg::Pose& pose, const std::string frame_id, bool add_controls = true,
                            double scale = 0.1, int marker_type = visualization_msgs::msg::Marker::SPHERE,
                            std::string marker_name = "pose_marker_");

  // Helper functions (TODO move to other files(?))
  void convertTF(geometry_msgs::msg::PoseStamped& poseStamped, const std::string& source_frame,
                 std::string& target_frame);

  // Callbacks and Feedback
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void clickedPointFeedback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
};

}  // end namespace processit_program

#endif
