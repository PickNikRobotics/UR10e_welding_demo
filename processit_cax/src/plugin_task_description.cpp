#include "plugin_task_description.h"
#include "TaskDescription/TaskDefinition.h"

// C++
#include <string>
#include <exception>
#include <yaml-cpp/yaml.h>

// ROS
#include <visualization_msgs/msg/marker.h>

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

  load_task_description_service = nh_->create_service<processit_msgs::srv::LoadTaskDescription>(
      "~/load_task_description", std::bind(&PluginTaskDescription::loadTaskDescription, this, _1, _2));
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
  TaskList tasklist;
  int seam_count = 0;
  std::string int_marker_id_start, int_marker_id_end, int_marker_id_center, int_marker_id_line;
  tasklist.ReadXML(request->task_description_file);
  std::string manufacturingSubFrameID;
  // Translation (given in mm in the task description)
  // Scaling the position that should be in [m]
  double unit_scaling = 0.001;

  for (int i = 0; i < tasklist.GetNumberOfWeldTasks(); i++)
  {
    int NumberOfWeldSegments = tasklist.GetNumberOfWeldSegments(i);
    double task_length = 0;
    double segment_length = 0;

    for (int j = 0; j < NumberOfWeldSegments; j++)
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

        // Rotation of the point
        Eigen::Matrix3d rot_mat = manufacturingFrameStart.block<3, 3>(0, 0);
        Eigen::Quaternion<double> q(rot_mat);

        // Translation (given in mm in the task description)
        // Scaling the position that should be in [m]
        Eigen::Vector3d positionVectorStart = unit_scaling * manufacturingFrameStart.col(3).head(3);
        Eigen::Vector3d positionVectorCenter = unit_scaling * manufacturingFrameCenter.col(3).head(3);
        Eigen::Vector3d positionVectorEnd = unit_scaling * manufacturingFrameEnd.col(3).head(3);

        // Set start and end pose and get pose marker id;
        // int_marker_id_start = setPositionVector(positionVectorStart, q);
        // int_marker_id_end = setPositionVector(positionVectorEnd, q);

        // Publish the manufacturing frames as TF transforms to describe the weld seam

        // Add a line (cube) connecting both points in direction of y axis (along the seam)
        double a = M_PI * 0.5;
        Eigen::Quaternion<double> factor(cos(a / 2), sin(a / 2), 0, 0);
        q = q * factor;
        // int_marker_id_line = addLine(seam_count, unit_scaling * segment_length, positionVectorCenter, q);
      }

      task_length += segment_length;
    }
  }
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
  rclcpp::spin(node);
  return 0;
}