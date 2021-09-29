#ifndef PLUGIN_TASK_DESCRIPTION_H_
#define PLUGIN_TASK_DESCRIPTION_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ROS
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <processit_msgs/srv/load_task_description.hpp>
#include <processit_msgs/srv/add_pose_marker.hpp>
#include <processit_msgs/msg/weld_seam.hpp>
#include <geometry_msgs/msg/pose.hpp>

// C++
#include <Eigen/Eigen>
#include <memory>

namespace processit_cax
{
class PluginTaskDescription
{
public:
  PluginTaskDescription(rclcpp::Node::SharedPtr& nh);

  geometry_msgs::msg::Pose getPose(Eigen::Vector3d& positionVector, Eigen::Quaternion<double>& q);
  void loadTaskDescription(const std::shared_ptr<processit_msgs::srv::LoadTaskDescription::Request> request,
                           std::shared_ptr<processit_msgs::srv::LoadTaskDescription::Response> response);

private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Service<processit_msgs::srv::LoadTaskDescription>::SharedPtr load_task_description_service;
  rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedPtr addintmarker_client;

  void initializePublishers();
  void initializeServices();
  void initializeSubscribers();

  rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedFutureAndRequestId
  addPoseMarker(Eigen::Vector3d& positionVector, Eigen::Quaternion<double>& q);
  rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedFutureAndRequestId
  addLineMarker(int id, double length, Eigen::Vector3d& positionVectorCenter, Eigen::Quaternion<double>& q);
};

};  // namespace processit_cax

#endif
