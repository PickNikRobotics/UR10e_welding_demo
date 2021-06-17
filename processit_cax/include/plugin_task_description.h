#ifndef PLUGIN_TASK_DESCRIPTION_H_
#define PLUGIN_TASK_DESCRIPTION_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

#include <processit_msgs/LoadTaskDescription.h>

namespace processit_cax
{
class PluginTaskDescription
{
public:
  PluginTaskDescription(ros::NodeHandle* nodehandle);

private:
  ros::NodeHandle nh;
  ros::ServiceServer loadtaskdescription_service;
  ros::ServiceClient addintmarker_client;
  ros::Publisher linemarker_pub;
  ros::Subscriber intmarker_sub;

  std::multimap<std::string, std::pair<int, std::string>> weld_seam_list;

  void initializePublishers();
  void initializeServices();
  void initializeSubscribers();

  bool loadTaskDescription(processit_msgs::LoadTaskDescription::Request& req,
                           processit_msgs::LoadTaskDescription::Response& res);

  std::string setPositionVector(Eigen::Vector3d& positionVector, Eigen::Quaternion<double>& q);

  void publishFrameTF(geometry_msgs::Transform& newFrame, std::string referenceFrameID, std::string newFrameID);

  std::string addLine(int id, double length, Eigen::Vector3d& positionVectorCenter, Eigen::Quaternion<double>& q);

  void poseMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback feedback);
  void poseMarkerUpdate(visualization_msgs::InteractiveMarkerUpdate feedback);
};

};  // namespace processit_cax

#endif
