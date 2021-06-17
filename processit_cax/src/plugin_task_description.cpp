#include "plugin_task_description.h"

#include "TaskDescription/TaskDefinition.h"
#include <processit_msgs/AddPoseMarker.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <exception>

#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>

namespace processit_cax
{
//----------------------------------------------------------------------
// CONSTRUCTORS
//----------------------------------------------------------------------

PluginTaskDescription::PluginTaskDescription(ros::NodeHandle* nodehandle) : nh(*nodehandle)
{
  initializeServices();
  initializePublishers();
  initializeSubscribers();
}

//----------------------------------------------------------------------
// INITIALIZING
//----------------------------------------------------------------------

void PluginTaskDescription::initializeServices()
{
  loadtaskdescription_service = nh.advertiseService("plugin_task_description/load_task_description_server",
                                                    &PluginTaskDescription::loadTaskDescription, this);
  addintmarker_client = nh.serviceClient<processit_msgs::AddPoseMarker>("processit_program/add_pose_marker_server");
}

void PluginTaskDescription::initializePublishers()
{
  linemarker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void PluginTaskDescription::initializeSubscribers()
{
  // intmarker_sub = nh.subscribe("pose_marker/feedback", 1,
  //     &PluginTaskDescription::poseMarkerFeedback, this);
  intmarker_sub = nh.subscribe("pose_marker/update", 1, &PluginTaskDescription::poseMarkerUpdate, this);
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
std::string PluginTaskDescription::setPositionVector(Eigen::Vector3d& positionVector, Eigen::Quaternion<double>& q)
{
  // Create pose (assuming m convention)
  geometry_msgs::Pose pose;
  pose.position.x = positionVector[0];
  pose.position.y = positionVector[1];
  pose.position.z = positionVector[2];
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  processit_msgs::AddPoseMarker srv;
  srv.request.pose = pose;
  srv.request.frame_id = "workpiece";
  srv.request.add_controls = false;
  srv.request.scale = 0.1;
  srv.request.marker_type = 2;
  srv.request.marker_name = "pose_marker_";
  bool success = addintmarker_client.call(srv);
  return srv.response.int_marker_id;
}

/**
 * @brief Publish a frame as TF transform with respect to a reference frame
 *
 * @param transformStamped geometry_msgs/TransformStamped the transformation between reference and new frame
 */
void PluginTaskDescription::publishFrameTF(geometry_msgs::Transform& newFrame, std::string referenceFrameID,
                                           std::string newFrameID)
{
  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = referenceFrameID;
  transformStamped.child_frame_id = newFrameID;
  transformStamped.transform = newFrame;
  br.sendTransform(transformStamped);
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
std::string PluginTaskDescription::addLine(int id, double length, Eigen::Vector3d& positionVectorCenter,
                                           Eigen::Quaternion<double>& q)
{
  // Create pose (assuming mm convention)
  geometry_msgs::Pose pose;
  pose.position.x = positionVectorCenter[0];
  pose.position.y = positionVectorCenter[1];
  pose.position.z = positionVectorCenter[2];
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  // Add Pose Marker add start and end point
  processit_msgs::AddPoseMarker srv;
  srv.request.pose = pose;
  srv.request.frame_id = "workpiece";
  srv.request.add_controls = false;
  srv.request.scale = length;
  srv.request.marker_type = 3;
  srv.request.marker_name = "seam_";
  bool success = addintmarker_client.call(srv);
  return srv.response.int_marker_id;
}

/**
 * @brief Service to load a given xml task definition and add it to the GUI
 *
 * Using the Pose Marker Service to add points and
 * connected by lines to visualize the "weld seam"
 *
 * @param req LoadTaskDescription Request which contains path to xml
 * @param res LoadTaskDescription Response, currently empty
 *
 * @return true if successful, otherwise false
 */
bool PluginTaskDescription::loadTaskDescription(processit_msgs::LoadTaskDescription::Request& req,
                                                processit_msgs::LoadTaskDescription::Response& res)
{
  TaskList tasklist;
  int seam_count = 0;
  std::string int_marker_id_start, int_marker_id_end, int_marker_id_center, int_marker_id_line;
  tasklist.ReadXML(req.task_description_file);
  geometry_msgs::Transform task_transform;
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
        Eigen::Matrix4d manufacturingFrameStart = tasklist.GetManufacturingCoordinateSystem(
            i, 1.001 * task_length);  // HACK to enable planning with non-continuous task description
        // Eigen::Matrix4d manufacturingFrameStart = tasklist.GetManufacturingCoordinateSystem(i, task_length);
        Eigen::Matrix4d manufacturingFrameCenter =
            tasklist.GetManufacturingCoordinateSystem(i, task_length + segment_length / 2);
        Eigen::Matrix4d manufacturingFrameEnd =
            tasklist.GetManufacturingCoordinateSystem(i, task_length + segment_length);

        // Rotation of the point
        Eigen::Matrix3d rot_mat = manufacturingFrameStart.block<3, 3>(0, 0);
        Eigen::Quaternion<double> q(rot_mat);
        geometry_msgs::Quaternion q_msg;
        tf2::convert(q, q_msg);

        // Translation (given in mm in the task description)
        // Scaling the position that should be in [m]

        Eigen::Vector3d positionVectorStart = unit_scaling * manufacturingFrameStart.col(3).head(3);
        Eigen::Vector3d positionVectorCenter = unit_scaling * manufacturingFrameCenter.col(3).head(3);
        Eigen::Vector3d positionVectorEnd = unit_scaling * manufacturingFrameEnd.col(3).head(3);
        geometry_msgs::Vector3 positionVectorStart_msg;
        tf2::Vector3 positionVectorStart_tf;
        tf2::convert(positionVectorStart, positionVectorStart_tf);
        tf2::convert(positionVectorStart_tf, positionVectorStart_msg);
        geometry_msgs::Vector3 positionVectorEnd_msg;
        tf2::Vector3 positionVectorEnd_tf;
        tf2::convert(positionVectorEnd, positionVectorEnd_tf);
        tf2::convert(positionVectorEnd_tf, positionVectorEnd_msg);

        // TODO Apply task frame offsets defined by user in task sequence

        // Set start and end pose and get pose marker id;
        int_marker_id_start = setPositionVector(positionVectorStart, q);
        int_marker_id_end = setPositionVector(positionVectorEnd, q);

        // Publish the manufacturing frames as TF transforms to describe the weld seam
        // if (j == 0)
        // {
        manufacturingSubFrameID = "task_" + to_string(i) + "_segment_" + to_string(j) + "_start";
        task_transform.translation = positionVectorStart_msg;
        task_transform.rotation = q_msg;
        publishFrameTF(task_transform, "workpiece", manufacturingSubFrameID);
        // }
        manufacturingSubFrameID = "task_" + to_string(i) + "_segment_" + to_string(j) + "_end";
        task_transform.translation = positionVectorEnd_msg;
        task_transform.rotation = q_msg;
        publishFrameTF(task_transform, "workpiece", manufacturingSubFrameID);

        // // TODO Even better, the frames are defined as subframes in the workpiece
        // moveit_msgs::CollisionObject
        // box.subframe_names.resize(5);
        // box.subframe_poses.resize(5);

        // box.subframe_names[0] = "bottom";
        // box.subframe_poses[0].position.y = -.05;
        // box.subframe_poses[0].position.z = 0.0 + z_offset_box;

        // tf2::Quaternion orientation;
        // orientation.setRPY(90.0 / 180.0 * M_PI, 0, 0);
        // box.subframe_poses[0].orientation = tf2::toMsg(orientation);

        // Add a line (cube) connecting both points
        // In direction of y axis (along the seam)
        double a = M_PI * 0.5;
        Eigen::Quaternion<double> factor(cos(a / 2), sin(a / 2), 0, 0);
        q = q * factor;
        int_marker_id_line = addLine(seam_count, unit_scaling * segment_length, positionVectorCenter, q);

        // Register new "weld seam" (the line) to handle feedback
        // (e.g. position update/removal) from
        weld_seam_list.insert(std::make_pair(int_marker_id_start, std::make_pair(seam_count, int_marker_id_end)));
        weld_seam_list.insert(std::make_pair(int_marker_id_end, std::make_pair(seam_count, int_marker_id_start)));
        seam_count++;
      }
      else if ((ContourType == "CircleContour") or (ContourType == "SplineContour"))
      {
        segment_length = tasklist.GetWeldSegmentLength(i, j);

        // Set start and end point
        Eigen::Matrix4d manufacturingFrameStart = tasklist.GetManufacturingCoordinateSystem(i, task_length);
        Eigen::Matrix4d manufacturingFrameCenter =
            tasklist.GetManufacturingCoordinateSystem(i, task_length + segment_length / 2);
        Eigen::Matrix4d manufacturingFrameEnd =
            tasklist.GetManufacturingCoordinateSystem(i, task_length + segment_length);

        // Rotation of the point
        Eigen::Matrix3d rot_mat_start = manufacturingFrameStart.block<3, 3>(0, 0);
        Eigen::Quaternion<double> q_start(rot_mat_start);
        geometry_msgs::Quaternion q_start_msg;
        tf2::convert(q_start, q_start_msg);
        Eigen::Matrix3d rot_mat_center = manufacturingFrameCenter.block<3, 3>(0, 0);
        Eigen::Quaternion<double> q_center(rot_mat_center);
        geometry_msgs::Quaternion q_center_msg;
        tf2::convert(q_center, q_center_msg);
        Eigen::Matrix3d rot_mat_end = manufacturingFrameEnd.block<3, 3>(0, 0);
        Eigen::Quaternion<double> q_end(rot_mat_end);
        geometry_msgs::Quaternion q_end_msg;
        tf2::convert(q_end, q_end_msg);

        // Translation (given in mm in the task description)
        // Scaling the position that should be in [m]

        Eigen::Vector3d positionVectorStart = unit_scaling * manufacturingFrameStart.col(3).head(3);
        Eigen::Vector3d positionVectorCenter = unit_scaling * manufacturingFrameCenter.col(3).head(3);
        Eigen::Vector3d positionVectorEnd = unit_scaling * manufacturingFrameEnd.col(3).head(3);
        geometry_msgs::Vector3 positionVectorStart_msg;
        tf2::Vector3 positionVectorStart_tf;
        tf2::convert(positionVectorStart, positionVectorStart_tf);
        tf2::convert(positionVectorStart_tf, positionVectorStart_msg);
        geometry_msgs::Vector3 positionVectorCenter_msg;
        tf2::Vector3 positionVectorCenter_tf;
        tf2::convert(positionVectorCenter, positionVectorCenter_tf);
        tf2::convert(positionVectorCenter_tf, positionVectorCenter_msg);
        geometry_msgs::Vector3 positionVectorEnd_msg;
        tf2::Vector3 positionVectorEnd_tf;
        tf2::convert(positionVectorEnd, positionVectorEnd_tf);
        tf2::convert(positionVectorEnd_tf, positionVectorEnd_msg);

        // Set start and end pose and get interactive marker id;
        int_marker_id_start = setPositionVector(positionVectorStart, q_start);
        int_marker_id_center = setPositionVector(positionVectorCenter, q_center);
        int_marker_id_end = setPositionVector(positionVectorEnd, q_end);

        // Publish the manufacturing frames as TF transforms to describe the weld seam
        // if (j == 0)
        // {
        manufacturingSubFrameID = "task_" + to_string(i) + "_segment_" + to_string(j) + "_start";
        task_transform.translation = positionVectorStart_msg;
        task_transform.rotation = q_start_msg;
        publishFrameTF(task_transform, "workpiece", manufacturingSubFrameID);
        // }
        manufacturingSubFrameID = "task_" + to_string(i) + "_segment_" + to_string(j) + "_center";
        task_transform.translation = positionVectorCenter_msg;
        task_transform.rotation = q_center_msg;
        publishFrameTF(task_transform, "workpiece", manufacturingSubFrameID);

        manufacturingSubFrameID = "task_" + to_string(i) + "_segment_" + to_string(j) + "_end";
        task_transform.translation = positionVectorEnd_msg;
        task_transform.rotation = q_end_msg;
        publishFrameTF(task_transform, "workpiece", manufacturingSubFrameID);

        double incr = 5;
        double temp = ceil(segment_length / incr);
        double incrNew = segment_length / temp;
        double iteration_length = 0;

        for (double k = 0; k < temp; k++)
        {
          // Get Manufacturing Coordinate system for discretized parts
          Eigen::Matrix4d manufacturingFrameStart =
              tasklist.GetManufacturingCoordinateSystem(i, task_length + iteration_length);
          Eigen::Matrix4d manufacturingFrameCenter =
              tasklist.GetManufacturingCoordinateSystem(i, task_length + iteration_length + incr / 2);

          // Rotation of the point
          Eigen::Matrix3d rot_mat_center = manufacturingFrameCenter.block<3, 3>(0, 0);
          Eigen::Quaternion<double> q_center(rot_mat_center);

          // Translation (given in mm in the task description)
          // Scaling the position that should be in [m]
          Eigen::Vector3d positionVectorStart = unit_scaling * manufacturingFrameStart.col(3).head(3);

          // Add a line (cube) connecting both points
          // In direction of y axis (along the seam)
          double a = M_PI * 0.5;
          Eigen::Quaternion<double> factor(cos(a / 2), sin(a / 2), 0, 0);
          q_center = q_center * factor;
          int_marker_id_line = addLine(seam_count, unit_scaling * incr, positionVectorStart, q_center);

          // Register new "weld seam" (the line) to handle feedback
          // (e.g. position update/removal) from
          weld_seam_list.insert(std::make_pair(int_marker_id_start, std::make_pair(seam_count, "")));
          seam_count++;

          iteration_length += incrNew;
        }
      }

      task_length += segment_length;
    }
  }
  return true;
}

//----------------------------------------------------------------------
// HANDLE FEEDBACK
//----------------------------------------------------------------------

void PluginTaskDescription::poseMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback feedback)
{
}

/**
 * @brief Handle feedback of /pose_marker/update
 *
 * If one or multiple interactive markers are erased,
 * the corresponding lines to this point should be removed as well.
 *
 * @param update a message of type visualization_msgs::InteractiveMarkerUpdate
 */
void PluginTaskDescription::poseMarkerUpdate(visualization_msgs::InteractiveMarkerUpdate update)
{
  if (update.server_id == "/pose_marker")
  {
    // if not empty, get all deleted markers
    if (!update.erases.empty())
    {
      for (int i = 0; i < update.erases.size(); i++)
      {
        if (weld_seam_list.find(update.erases[i]) != weld_seam_list.end())
        {
          visualization_msgs::Marker weldseam_marker;
          // remove all associated weld seam markers
          for (auto itr = weld_seam_list.find(update.erases[i]); itr != weld_seam_list.end(); itr++)
          {
            // auto itr = weld_seam_list.find(update.erases[i]);
            if (itr->first == update.erases[i])
            {
              weldseam_marker.id = itr->second.first;
              weldseam_marker.ns = "plugin_taskdescription";
              weldseam_marker.action = visualization_msgs::Marker::DELETE;
              linemarker_pub.publish(weldseam_marker);

              weld_seam_list.erase(itr->second.second);
            }
          }
          weld_seam_list.erase(update.erases[i]);
        }
      }
    }
  }
}

}  // namespace processit_cax

//----------------------------------------------------------------------
// MAIN FUNCTION
//----------------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plugin_task_description");  // node name
  ros::NodeHandle nh;

  processit_cax::PluginTaskDescription plugintaskdescription(&nh);

  ros::spin();
  return 0;
}
