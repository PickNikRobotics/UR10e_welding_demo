#ifndef CARTESIAN_TASK_H
#define CARTESIAN_TASK_H

#include <string>
#include <exception>

// YAML
#include <yaml-cpp/yaml.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/transform.h>

// MoveIt
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

namespace processit_tasks
{
class CartesianTask
{
public:
  // Constructor
  CartesianTask(rclcpp::Node::SharedPtr& node);

  // Destructor
  ~CartesianTask() = default;

  // Initialize
  void init(std::string task_name, std::string task_caption);

  // Add current state
  void addCurrentState();

  // Adding a new stage
  void addStage(std::string stage_caption, geometry_msgs::msg::PoseStamped goal_pose, std::string planner_id,
                double velocity);

  // Setting a stage offset
  void setTaskTransformOffset(double x, double y, double z, double roll, double pitch, double yaw);
  void setTaskTransformOffset(geometry_msgs::msg::Transform new_task_transform);

  // Planning of the motion
  bool plan();

  // Execution of the motion
  bool execute();

  moveit::task_constructor::TaskPtr task_;

private:
  /*************
   * Functions *
   *************/

  // Load the relevant parameters from the YAML file
  void loadParameters();

  // Apply the prevoiusly set offset to the task transform
  void applyTaskTransformOffset();

  void setPlannerProperties(std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>& pipeline_planner,
                            std::string planner_id, double velocity);

  /*******
   * ROS *
   *******/

  rclcpp::Node::SharedPtr node_;

  /**************************
   * task-related variables *
   **************************/

  std::string task_name_;
  std::string task_caption_;
  int task_id_;
  std::string planner_plugin_;

  /***********************************
   * planner_interface configuration *
   ***********************************/

  // Cartesian velocity [m/s]
  double cartesian_velocity_ = 0.02;
  double via_velocity_ = 0.2;
  double cartesian_rot_velocity_ = 1.57;
  double max_trans_vel_;
  double max_rot_vel_;

  // Maximum acceleration scaling
  double max_acceleration_scaling_ = 1.0;

  // Step size
  double step_size_ = 0.01;
  int num_planning_attempts_;                 //.01
  double goal_position_tolerance_ = 1e-4;     //.01
  double goal_orientation_tolerance_ = 1e-3;  //.01

  // Planner IDs
  std::string linear_planner_id_ = "LIN";
  std::string circular_planner_id_ = "CIRC";
  std::string curve_planner_id_ = "SPLINE";
  std::string joint_space_planner_id_ = "PTP";
  std::string free_space_planner_id_ = "RRTConnect";
  std::string constrained_planner_id_ = "RRTstar";
  std::string planner_id_property_name_ = "planner";
  std::string path_constraints_name_ = "linear_system_constraints";  // OMPL Constrained with linear equations system

  // Allowed planning time [s] and maximum number of solutions
  int planning_time_free_space_ = 1;
  int planning_time_constrained_ = 5;
  int planning_time_collisions_ = 10;
  int max_solutions_ = 10;

  // // Planning group and link names
  std::string welding_group_name_;  //"welding_endeffector" // "welding_arm"
  std::string welding_tcp_frame_;   //"welding_gun" // "welding_tcp"

  // Execution
  rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr execute_;
};
}  // namespace processit_tasks

#endif
