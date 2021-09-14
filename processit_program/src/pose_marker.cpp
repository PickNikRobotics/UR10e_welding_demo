#include <pose_marker.h>

#include <boost/bind.hpp>
#include <iostream>
#include <fstream>
#include <memory>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace processit_program
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("pose_marker");

//----------------------------------------------------------------------
// CONSTRUCTORS
//----------------------------------------------------------------------

/** Construct a new Pose Marker object */
PoseMarker::PoseMarker(rclcpp::Node::SharedPtr& node) : int_marker_server("pose_marker", node)
{
  node_ = node;
  // int_marker_server = new interactive_markers::InteractiveMarkerServer("pose_marker", node_);
  initializeSubscribers();
  initializeServices();
  // initializeMenuHandler();
}

//----------------------------------------------------------------------
// INITIALIZINIG
//----------------------------------------------------------------------

/** Initialize subscribers */
void PoseMarker::initializeSubscribers()
{
  RCLCPP_INFO(LOGGER, "Initializing Subscribers");
  using std::placeholders::_1;

  clickedpoint_sub = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, std::bind(&PoseMarker::clickedPointFeedback, this, _1));
}

/** Initialize Services */
void PoseMarker::initializeServices()
{
  RCLCPP_INFO(LOGGER, "Initializing Services");

  using std::placeholders::_1;
  using std::placeholders::_2;

  addintmarker_service = node_->create_service<processit_msgs::srv::AddPoseMarker>(
      "~/add_pose_marker", std::bind(&PoseMarker::addPoseMarkerService, this, _1, _2));
  editintmarker_service = node_->create_service<processit_msgs::srv::EditPoseMarker>(
      "~/edit_pose_marker", std::bind(&PoseMarker::editPoseMarkerService, this, _1, _2));
  deleteallposemarkers_service = node_->create_service<std_srvs::srv::Empty>(
      "~/delete_pose_marker", std::bind(&PoseMarker::deleteAllPoseMarkersService, this, _1, _2));

  addintmarker_service_client = node_->create_client<processit_msgs::srv::AddPoseMarker>("~/add_pose_marker");
}

//----------------------------------------------------------------------
// HELPER FUNCTIONS GUI
//----------------------------------------------------------------------

/**
 * @brief Create a box (sphere shape) for the interactie marker in grey.
 *
 * @param msg A pointer to the pose marker
 * @return visualization_msgs::msg::Marker
 */
visualization_msgs::msg::Marker PoseMarker::makeBox(visualization_msgs::msg::InteractiveMarker& msg, int marker_type)
{
  visualization_msgs::msg::Marker marker;
  marker.type = marker_type;
  if (marker_type == visualization_msgs::msg::Marker::SPHERE)
  {
    marker.scale.x = msg.scale * 0.1;
    marker.scale.y = msg.scale * 0.1;
    marker.scale.z = msg.scale * 0.1;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
  }
  else if (marker_type == visualization_msgs::msg::Marker::CYLINDER)
  {
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = msg.scale;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
  }

  return marker;
}

/**
 * @brief Create an Arrow in the specified direction (x,y,z) and corresponding rgb colors
 *
 * @param msg A pointer to the pose marker
 * @param direction Choose which direction the arrow is pointing add. Either x (default),y or z
 * @return visualization_msgs::msg::Marker
 */
visualization_msgs::msg::Marker PoseMarker::makeArrow(visualization_msgs::msg::InteractiveMarker& msg, char direction)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale.x = msg.scale * 0.5;
  marker.scale.y = msg.scale * 0.025;
  marker.scale.z = msg.scale * 0.025;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  switch (direction)
  {
    case 'x':
      marker.color.r = 1.0;
      break;
    case 'y':
      marker.pose.orientation.z = 1;
      marker.color.g = 1.0;
      break;
    case 'z':
      marker.pose.orientation.y = -1;
      marker.color.b = 1.0;
      break;
    default:
      break;
  }
  return marker;
}

/** create an interactive control which contains the box */
visualization_msgs::msg::InteractiveMarkerControl
PoseMarker::makeBoxControl(visualization_msgs::msg::InteractiveMarker& msg, int marker_type, bool add_controls)
{
  visualization_msgs::msg::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  if (add_controls)
  {
    box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  }
  else if (marker_type != visualization_msgs::msg::Marker::CYLINDER)
  {
    box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
  }
  else
  {
    box_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
  }
  box_control.markers.push_back(makeBox(msg, marker_type));
  if (marker_type != visualization_msgs::msg::Marker::CYLINDER)
  {
    box_control.markers.push_back(makeArrow(msg, 'x'));
    box_control.markers.push_back(makeArrow(msg, 'y'));
    box_control.markers.push_back(makeArrow(msg, 'z'));
  }
  return box_control;
}

/** create a control for translation or rotation axes */
visualization_msgs::msg::InteractiveMarkerControl PoseMarker::makeControl(unsigned int interaction_mode,
                                                                          double orientation[4], std::string name)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = orientation[0];
  control.orientation.x = orientation[1];
  control.orientation.y = orientation[2];
  control.orientation.z = orientation[3];
  control.name = name;
  control.interaction_mode = interaction_mode;

  return control;
}

//----------------------------------------------------------------------
// METHODS TO HANDLE THE POSE MARKERS
//----------------------------------------------------------------------

/**
 * @brief Provides a service for adding pose markers
 *
 * @param req  The request, processit_msgs::srv::AddPoseMarker::Request
 * @param res  The response, processit_msgs::srv::AddPoseMarker::Response
 * @return true
 */

void PoseMarker::addPoseMarkerService(const std::shared_ptr<processit_msgs::srv::AddPoseMarker::Request> request,
                                      std::shared_ptr<processit_msgs::srv::AddPoseMarker::Response> response)
{
  response->int_marker_id =
      addPoseMarker((const geometry_msgs::msg::Pose)request->pose, (const std::string)request->frame_id,
                    request->add_controls, request->scale, request->marker_type, request->marker_name);
}

/**
 * @brief Callback function to edit the pose of a pose marker
 *
 * @param req Server request
 * @param res Server response
 * @return true
 * @return false
 */
void PoseMarker::editPoseMarkerService(const std::shared_ptr<processit_msgs::srv::EditPoseMarker::Request> request,
                                       std::shared_ptr<processit_msgs::srv::EditPoseMarker::Response> response)
{
  std::string marker_name = request->marker_name;
  geometry_msgs::msg::Pose new_pose;
  new_pose.position.x = request->new_pose.position.x;
  new_pose.position.y = request->new_pose.position.y;
  new_pose.position.z = request->new_pose.position.z;
  new_pose.orientation.x = request->new_pose.orientation.x;
  new_pose.orientation.y = request->new_pose.orientation.y;
  new_pose.orientation.z = request->new_pose.orientation.z;
  new_pose.orientation.w = request->new_pose.orientation.w;

  int_marker_server.setPose(request->marker_name, new_pose);
  int_marker_server.applyChanges();
}

/**
 * @brief Service to delete a marker by its id
 *
 * @param req
 * @param res
 * @return true
 * @return false
 */
void PoseMarker::deleteAllPoseMarkersService(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                             std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  RCLCPP_INFO_STREAM(LOGGER, "Deleting all Pose Markers");
  int_marker_server.clear();
  int_marker_server.applyChanges();
}

/**
 * @brief Add an pose marker to the server and to the internal list.
 *
 * @param pose
 * @param frame_id
 * @param add_controls Whether to have interactive controls or not
 * @param scale The size of the marker (between 0 and 1)
 * @param marker_type 1 for sphere and 2 for cylinder, see enum in visualisation_msgs::Marker (isn't it 2 and 3 ?)
 * @param marker_name The name of the marker (default = "pose_marker_")
 *
 */
std::string PoseMarker::addPoseMarker(const geometry_msgs::msg::Pose& pose, const std::string frame_id,
                                      bool add_controls, double scale, int marker_type, std::string marker_name)
{
  // create a pose marker
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  // int_marker.header.stamp=ros::Time::now();
  if (int_marker_server.size() > pose_marker_counter)
  {
    pose_marker_counter = int_marker_server.size() + 1;
  }
  int_marker.name = marker_name + std::to_string(pose_marker_counter + 1);
  pose_marker_counter++;
  int_marker.scale = scale;  // default 0.1
  int_marker.pose.position.x = pose.position.x;
  int_marker.pose.position.y = pose.position.y;
  int_marker.pose.position.z = pose.position.z;
  int_marker.pose.orientation.x = pose.orientation.x;
  int_marker.pose.orientation.y = pose.orientation.y;
  int_marker.pose.orientation.z = pose.orientation.z;
  int_marker.pose.orientation.w = pose.orientation.w;

  // add a box with an interactive control to the pose marker
  int_marker.controls.push_back(makeBoxControl(int_marker, marker_type, add_controls));

  if (add_controls)
  {
    // create a control for each translation and rotation axis
    int_marker.controls.push_back(makeControl(visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
                                              new double[4]{ 1, 1, 0, 0 }, "rotate_x"));
    int_marker.controls.push_back(makeControl(visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS,
                                              new double[4]{ 1, 1, 0, 0 }, "move_x"));
    int_marker.controls.push_back(makeControl(visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
                                              new double[4]{ 1, 0, 1, 0 }, "rotate_y"));
    int_marker.controls.push_back(makeControl(visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS,
                                              new double[4]{ 1, 0, 1, 0 }, "move_y"));
    int_marker.controls.push_back(makeControl(visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS,
                                              new double[4]{ 1, 0, 0, 1 }, "rotate_z"));
    int_marker.controls.push_back(makeControl(visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS,
                                              new double[4]{ 1, 0, 0, 1 }, "move_z"));
  }

  // Add right-click menu
  int_marker.controls.push_back(
      makeControl(visualization_msgs::msg::InteractiveMarkerControl::MENU, new double[4]{ 0, 0, 0, 0 }, "menu"));

  // add the pose marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  int_marker_server.insert(int_marker, boost::bind(&PoseMarker::processFeedback, this, _1));

  // 'commit' changes and send to all clients
  // menu_handler.apply(server, int_marker.name);
  int_marker_server.applyChanges();

  return int_marker.name;
}

//----------------------------------------------------------------------
// CALLBACKS AND FEEDBACK
//----------------------------------------------------------------------

/**
 * @brief Process all feedback of the Pose Marker
 *
 * @param feedback A constant pointer to the Pose Marker Feedback message
 */
void PoseMarker::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  // tf2::Quaternion quaternion(feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
  //                            feedback->pose.orientation.w);

  // RCLCPP_INFO_STREAM(LOGGER,  feedback->marker_name << " is now at "
  //     << feedback->pose.position.x << ", " << feedback->pose.position.y
  //     << ", " << feedback->pose.position.z
  //     << ", "<< quaternion.getAxis().getX()
  //     << ", "<< quaternion.getAxis().getY()
  //     << ", "<< quaternion.getAxis().getZ());

  // geometry_msgs::msg::PoseStamped poseStamped;
  // poseStamped.pose = feedback->pose;
  // poseStamped.header.frame_id = feedback->header.frame_id;
  // std::string target_frame = "world";

  // processit_msgs::srv::PlanExecuteMoveItPoseGoal goal;

  // switch (feedback->event_type)
  // {
  // case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
  //     switch (feedback->menu_entry_id)
  //     {
  //     case SUB_MENU_PLAN_JOINT:
  //         convertTF(poseStamped, feedback->header.frame_id, target_frame);
  //         goal.pose = poseStamped.pose;
  //         goal.plan_cartesian = false;
  //         goal.do_plan = true;
  //         goal.do_execute = false;
  //         planExecuteMoveItPose_action_client.sendGoal(goal);
  //         break;
  //     case SUB_MENU_EXECUTE_JOINT:
  //         convertTF(poseStamped, feedback->header.frame_id, target_frame);
  //         goal.pose = poseStamped.pose;
  //         goal.plan_cartesian = false;
  //         goal.do_plan = false;
  //         goal.do_execute = true;
  //         planExecuteMoveItPose_action_client.sendGoal(goal);
  //         break;
  //     case SUB_MENU_PLAN_EXECUTE_JOINT:
  //         convertTF(poseStamped, feedback->header.frame_id, target_frame);
  //         goal.pose = poseStamped.pose;
  //         goal.plan_cartesian = false;
  //         goal.do_plan = true;
  //         goal.do_execute = true;
  //         planExecuteMoveItPose_action_client.sendGoal(goal);
  //         break;
  //     case SUB_MENU_PLAN_CARTESIAN:
  //         convertTF(poseStamped, feedback->header.frame_id, target_frame);
  //         goal.pose = poseStamped.pose;
  //         goal.plan_cartesian = true;
  //         goal.do_plan = true;
  //         goal.do_execute = false;
  //         planExecuteMoveItPose_action_client.sendGoal(goal);
  //         break;
  //     case SUB_MENU_EXECUTE_CARTESIAN:
  //         convertTF(poseStamped, feedback->header.frame_id, target_frame);
  //         goal.pose = poseStamped.pose;
  //         goal.plan_cartesian = true;
  //         goal.do_plan = false;
  //         goal.do_execute = true;
  //         planExecuteMoveItPose_action_client.sendGoal(goal);
  //         break;
  //     case SUB_MENU_PLAN_EXECUTE_CARTESIAN:
  //         convertTF(poseStamped, feedback->header.frame_id, target_frame);
  //         goal.pose = poseStamped.pose;
  //         goal.plan_cartesian = true;
  //         goal.do_plan = true;
  //         goal.do_execute = true;
  //         planExecuteMoveItPose_action_client.sendGoal(goal);
  //         break;

  //     default:
  //         break;
  //     }
  //     break;
  // }

  // int_marker_server.applyChanges();
}

/**
 * @brief Add an pose Marker at the clicked point and connect it to a workpiece
 *
 * @param msg the coordinates (x,y,z) of the clicked point
 */
void PoseMarker::clickedPointFeedback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // Get translation and set default orientation
  geometry_msgs::msg::PoseStamped poseStamped;
  poseStamped.header.frame_id = msg->header.frame_id;

  poseStamped.pose.position = msg->point;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0);
  poseStamped.pose.orientation.x = quaternion.getX();
  poseStamped.pose.orientation.y = quaternion.getY();
  poseStamped.pose.orientation.z = quaternion.getZ();
  poseStamped.pose.orientation.w = quaternion.getW();

  std::string target_frame = "workpiece";
  convertTF(poseStamped, msg->header.frame_id, target_frame);

  // add the pose marker
  auto request = std::make_shared<processit_msgs::srv::AddPoseMarker::Request>();
  request->pose = poseStamped.pose;
  request->frame_id = target_frame;
  request->add_controls = true;
  request->scale = 0.1;
  request->marker_name = "pose_marker_";
  request->marker_type = visualization_msgs::msg::Marker::SPHERE;
  auto result = addintmarker_service_client->async_send_request(request);
}

//----------------------------------------------------------------------
// FUNCTIONS TO BE MOVED OR REMOVED
//----------------------------------------------------------------------

// TODO: Move to WorkpieceHandler (Service)
void PoseMarker::convertTF(geometry_msgs::msg::PoseStamped& poseStamped, const std::string& source_frame,
                           std::string& target_frame)
{
  try
  {
    auto tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    auto transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

    tf_buffer_->transform(poseStamped, poseStamped, target_frame);
  }
  catch (tf2::TransformException& ex)
  {
    // If no workpiece frame is available, do not change frames.
    target_frame = source_frame;
  }
}

}  // end namespace processit_program

//----------------------------------------------------------------------
// MAIN FUNCTION
//----------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pose_marker");

  processit_program::PoseMarker pose_marker(node);
  //   using rclcpp::executors::MultiThreadedExecutor;
  //   MultiThreadedExecutor executor;
  //   executor.add_node(node);
  //   executor.spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
