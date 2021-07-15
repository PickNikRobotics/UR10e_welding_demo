#ifndef WELDING_H_
#define WELDING_H_

#include <string>
#include <exception>

// YAML
#include <yaml-cpp/yaml.h>

// ROS
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

// MTC Welding IPA
#include <processit_tasks/cartesian_task.h>

namespace processit_tasks
{
using namespace moveit::task_constructor;

class Welding
{
public:
  Welding(const int task_id, const int segment_id, const ros::NodeHandle& nh);
  ~Welding() = default;

  // Welding of Fillet weld seam
  void weldFillet(int task_id);

  // Welding of all Fillets
  void weldAllFillets(int last_task_id);

  // Welding a specific pass of a multi-layer task
  void weldSinglePass(int pass_id);

  // Welding all passes of a multi-layer task
  void weldAllPasses();

private:
  // Resets the CartesianTask instance
  void resetTask();

  // Sets parameters according to the welding layout
  void setPass(int pass_id);

  // Approaches the starting point of the pass
  void approach();

  // Performs the welding motion
  void weld();

  // Retrieves from the welding end section
  void retrieve();

  // Invokes the planning of the Cartesian task
  bool plan();

  // Invokes the execution of the planned Cartesian task
  bool execute();

  // Loads the relevant parameters from the parameter server
  void loadParameters();

  // Loads the welding layout from a YAML file
  void loadWeldingLayout();

  ros::NodeHandle nh_;

  // Service client for welding
  ros::ServiceClient welding_src_client_;
  std::string planner_plugin_;

  double distance_convert_ = 0.001;
  double angle_convert_ = M_PI / 180;
  double velocity_convert_ = 0.01 / 60;

  std::string task_name_;
  std::string task_caption_;
  int task_id_;
  int segment_id_;
  int pass_id_;
  geometry_msgs::Transform task_transform_;
  double task_velocity_;
  moveit::task_constructor::TaskPtr task_;

  processit_tasks::CartesianTask cartesian_task_;

  // planner_interface configuration
  double max_acceleration_scaling_;     // [rad/s²]
  double via_max_joint_velocity_;       // [rad/s]
  double away_max_joint_velocity_;      // [rad/s]
  double cartesian_velocity_;           // [m/s]
  double via_velocity_;                 // [m/s]
  double welding_velocity_;             // depends on velocity_convert_
  std::string linear_planner_id_;       //"LIN"
  std::string circular_planner_id_;     //"CIRC"
  std::string curve_planner_id_;        //"SPLINE"
  std::string joint_space_planner_id_;  //"PTP"

  // welding layout
  int pass_count_;
  YAML::Node multi_layer_layout_;

  // Offsets for specific sub-tasks
  //// Welding
  double offset_welding_approach_z_;
  double offset_welding_z_;
  double offset_welding_y_;
  double offset_welding_x_;
  double offset_welding_angle_;
  double offset_welding_speed_;
  //// Scanning
  double offset_scanning_approach_z_;
  double offset_scanning_x_;
  double offset_scanning_y_;
  double offset_scanning_z_;
  double offset_scanning_a_;
  double offset_scanning_b_;
  double offset_scanning_c_;

  // Execution
  actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_;
};
}  // namespace processit_tasks

#endif
