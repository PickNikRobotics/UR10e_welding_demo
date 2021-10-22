#include "plugin_task_description.h"
#include "TaskDescription/TaskDefinition.h"

// C++
#include <string>
#include <exception>
#include <yaml-cpp/yaml.h>

// ROS
#include <visualization_msgs/msg/marker.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace processit_cax
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("plugin_task_description");

//----------------------------------------------------------------------
// CONSTRUCTORS
//----------------------------------------------------------------------

PluginTaskDescription::PluginTaskDescription(rclcpp::Node::SharedPtr& nh)
{
  nh_ = nh;
  initializeServices();
  initializePublishers();
  initializeSubscribers();
}

//----------------------------------------------------------------------
// INITIALIZING
//----------------------------------------------------------------------

void PluginTaskDescription::initializeServices()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  RCLCPP_INFO(LOGGER, "Create service to load task description ");
  load_task_description_service = nh_->create_service<processit_msgs::srv::LoadTaskDescription>(
      "~/load_task_description", std::bind(&PluginTaskDescription::loadTaskDescription, this, _1, _2));
  addintmarker_client = nh_->create_client<processit_msgs::srv::AddPoseMarker>("pose_marker/add_pose_marker");
}

void PluginTaskDescription::initializePublishers()
{
}

void PluginTaskDescription::initializeSubscribers()
{
}

//----------------------------------------------------------------------
// LOADING TASK DESCRIPTION PLUGIN
//----------------------------------------------------------------------

/**
 * @brief Set an pose marker at the start and end position of the weld seam
 *
 * @param pose geometry_msgs/Pose the pose
 *
 * @return std::string the pose marker id
 */
rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedFutureAndRequestId
PluginTaskDescription::addPoseMarker(Eigen::Vector3d& positionVector, Eigen::Quaternion<double>& q)
{
  // Create pose (assuming mm convention)
  geometry_msgs::msg::Pose pose = getPose(positionVector, q);

  auto request = std::make_shared<processit_msgs::srv::AddPoseMarker::Request>();
  request->pose = pose;
  request->frame_id = "world";
  request->add_controls = false;
  request->scale = 0.1;
  request->marker_type = 2;
  request->marker_name = "pose_marker_";

  using ServiceResponseFuture = rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO_STREAM(LOGGER, "Created marker " << result->int_marker_id);
  };
  auto future_result = addintmarker_client->async_send_request(request, response_received_callback);
  return future_result;
}

/**
 * @brief Add a line representing the weld seam
 *
 * At the moment, the line is represented by a cylindric marker.
 * Its position is located in the middle between start and end point.
 * Its orientation is needed to point in direction of the seam.
 *
 * @param id integer
 * @param length double
 * @param positionVectorStart  an Eigen:Vector3d, in mm convention
 * @param q the quaternion of the manufacturingFrame in the start pose
 */
rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedFutureAndRequestId
PluginTaskDescription::addLineMarker(int id, double length, Eigen::Vector3d& positionVectorCenter,
                                     Eigen::Quaternion<double>& q)
{
  // Create pose (assuming mm convention)
  geometry_msgs::msg::Pose pose = getPose(positionVectorCenter, q);

  // Add Pose Marker add start and end point
  auto request = std::make_shared<processit_msgs::srv::AddPoseMarker::Request>();
  request->pose = pose;
  request->frame_id = "world";
  request->add_controls = false;
  request->scale = length;
  request->marker_type = 3;
  request->marker_name = "seam_marker_";

  using ServiceResponseFuture = rclcpp::Client<processit_msgs::srv::AddPoseMarker>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO_STREAM(LOGGER, "Created line " << result->int_marker_id);
  };
  auto future_result = addintmarker_client->async_send_request(request, response_received_callback);
  return future_result;
}

/**
 * @brief Convert an Eigen position vector and a corresponding rotation to a geometry_msgs::msg::Pose
 *
 * @param positionVector Eigen::Vector3d
 * @param q Eigen::Quaternion<double>
 * @return std::string
 */
geometry_msgs::msg::Pose PluginTaskDescription::getPose(Eigen::Vector3d& positionVector, Eigen::Quaternion<double>& q)
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

/**
 * @brief Service to load a given xml task definition and add it to the GUI
 * *
 * @param req LoadTaskDescription Request which contains path to xml
 * @param res LoadTaskDescription Response, currently empty
 *
 * @return true if successful, otherwise false
 */
void PluginTaskDescription::loadTaskDescription(
    const std::shared_ptr<processit_msgs::srv::LoadTaskDescription::Request> request,
    std::shared_ptr<processit_msgs::srv::LoadTaskDescription::Response> response)
{
  RCLCPP_INFO_STREAM(LOGGER, "Loading file " << request->task_description_file << ".");

  TaskList tasklist;
  int seam_count = 0;
  std::string int_marker_id_start, int_marker_id_end, int_marker_id_center, int_marker_id_line;
  tasklist.ReadXML(request->task_description_file);
  std::string manufacturingSubFrameID;
  Eigen::Isometry3d workpiece_pose;
  tf2::fromMsg(request->workpiece_pose.pose, workpiece_pose);

  // Scaling the translation (given in [mm], should be in [m]
  double unit_scaling = 0.001;
  workpiece_pose.translation().x() = 1 / unit_scaling * workpiece_pose.translation().x();
  workpiece_pose.translation().y() = 1 / unit_scaling * workpiece_pose.translation().y();
  workpiece_pose.translation().z() = 1 / unit_scaling * workpiece_pose.translation().z();

  RCLCPP_INFO_STREAM(LOGGER, "Found " << tasklist.GetNumberOfWeldTasks() << " seams.");

  for (int i = 0; i < tasklist.GetNumberOfWeldTasks(); i++)
  {
    int no_weld_segments = tasklist.GetNumberOfWeldSegments(i);
    double task_length = 0;
    double segment_length = 0;

    // RCLCPP_INFO_STREAM(LOGGER, "Seam " << i << " has " << no_weld_segments << " segments.");

    for (int j = 0; j < no_weld_segments; j++)
    {
      std::string ContourType = tasklist.GetContourTypeOfPositionSegment(i, j);

      if (ContourType == "LinearContour")
      {
        segment_length = tasklist.GetWeldSegmentLength(i, j);

        // Get Manufacturing Coordinate system for start and end point and rotation
        Eigen::Matrix4d manufacturingFrameStart = tasklist.GetManufacturingCoordinateSystem(i, task_length);
        Eigen::Matrix4d manufacturingFrameCenter =
            tasklist.GetManufacturingCoordinateSystem(i, task_length + segment_length / 2);
        Eigen::Matrix4d manufacturingFrameEnd =
            tasklist.GetManufacturingCoordinateSystem(i, task_length + segment_length);

        // Transform to world frame
        manufacturingFrameStart = workpiece_pose.matrix() * manufacturingFrameStart;
        manufacturingFrameCenter = workpiece_pose.matrix() * manufacturingFrameCenter;
        manufacturingFrameEnd = workpiece_pose.matrix() * manufacturingFrameEnd;

        // Rotation of the point
        Eigen::Matrix3d rot_mat = manufacturingFrameStart.block<3, 3>(0, 0);
        Eigen::Quaternion<double> q(rot_mat);

        // Translation (given in mm in the task description)
        // Scaling the position that should be in [m]
        Eigen::Vector3d positionVectorStart = unit_scaling * manufacturingFrameStart.col(3).head(3);
        Eigen::Vector3d positionVectorCenter = unit_scaling * manufacturingFrameCenter.col(3).head(3);
        Eigen::Vector3d positionVectorEnd = unit_scaling * manufacturingFrameEnd.col(3).head(3);

        // Set start and end pose and get pose marker id;
        auto future_start = addPoseMarker(positionVectorStart, q);
        auto future_end = addPoseMarker(positionVectorEnd, q);

        // Add a line (cube) connecting both points in direction of y axis (along the seam)
        // // Add a line (cube) connecting both points
        // // In direction of y axis (along the seam)
        double a = M_PI * 0.5;
        Eigen::Quaternion<double> factor(cos(a / 2), sin(a / 2), 0, 0);
        Eigen::Quaternion<double> q_line = q * factor;
        auto future_line = addLineMarker(seam_count, unit_scaling * segment_length, positionVectorCenter, q_line);

        // TODO Transform to workpiece frame

        geometry_msgs::msg::Pose start = getPose(positionVectorStart, q);
        geometry_msgs::msg::Pose end = getPose(positionVectorEnd, q);
        processit_msgs::msg::WeldSeam weld_seam;
        weld_seam.poses.push_back(start);
        weld_seam.poses.push_back(end);
        response->weld_seams.push_back(weld_seam);

        // RCLCPP_INFO_STREAM(LOGGER, "Found seam segment " << i << " of length " << segment_length);
      }

      task_length += segment_length;
    }
  }
  response->success = true;
}

}  // namespace processit_cax

//----------------------------------------------------------------------
// MAIN FUNCTION
//----------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("plugin_task_description");

  processit_cax::PluginTaskDescription plugin_task_description_node(node);
  // using rclcpp::executors::MultiThreadedExecutor;
  // MultiThreadedExecutor executor;
  // executor.add_node(node);
  // executor.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
