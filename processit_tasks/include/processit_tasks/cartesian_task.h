#ifndef CARTESIAN_TASK_H
#define CARTESIAN_TASK_H

#include <string>
#include <exception>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/transform.h>

// MoveIt
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
// #include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

namespace processit_tasks
{
class CartesianTask
{
public:
  // Constructor
  CartesianTask(const rclcpp::Node::SharedPtr& node, const moveit::task_constructor::TaskPtr& task);

  // Destructor
  ~CartesianTask() = default;

  // Initialize
  void init(std::string task_name, std::string task_caption);

  // Add current state
  void addCurrentState();
  // void addFixedState(std::string group_state);
  void viaMotion(std::string planner_id, double velocity);

  // Cartesian motion
  void approachRetreat(const std::string stage_caption, const std::string task_control_frame,
                       const double offset_approach_z);
  void generateStart(std::string stage_caption, geometry_msgs::msg::PoseStamped goal_pose,
                     std::string task_control_frame);
  void addStage(std::string stage_caption, geometry_msgs::msg::PoseStamped goal_pose, std::string task_control_frame,
                std::string planner_id, double velocity);

  void weld(geometry_msgs::msg::PoseStamped start_pose, geometry_msgs::msg::PoseStamped goal_pose);

  moveit::task_constructor::TaskPtr task_;

private:
  /*************
   * Functions *
   *************/

  void setPlannerProperties(moveit::task_constructor::solvers::PipelinePlannerPtr& pipeline_planner,
                            std::string planner_id, double velocity);

  /*******
   * ROS *
   *******/

  rclcpp::Node::SharedPtr node_;

  /**************************
   * task-related variables *
   **************************/

  moveit::task_constructor::Stage* current_state_ptr_;  // Forward current_state on to pose generator
  std::string task_name_;
  std::string task_caption_;
  int task_id_;
  std::string planning_plugin_ = "pilz_industrial_motion_planner::CommandPlanner";
  std::string planning_pipeline_ = "pilz_industrial_motion_planner";
  std::string planner_id_ = "LIN";
  const std::string world_frame_ = "world";    // Default world frame
  std::string workpiece_frame_ = "workpiece";  // Default workpiece frame

  /***********************************
   * planner_interface configuration *
   ***********************************/

  // Cartesian velocity [m/s]
  double welding_velocity_;  // [cm/min], see velocity_convert_ in welding.h
  double cartesian_velocity_;
  double via_velocity_;
  double cartesian_rot_velocity_ = 1.57;
  double max_trans_vel_ = 1.0;
  double max_rot_vel_ = 9.42;

  // Maximum acceleration scaling
  double max_acceleration_scaling_factor_;

  // Step size
  double step_size_ = 0.01;
  int num_planning_attempts_;                 //.01
  double goal_position_tolerance_ = 1e-4;     //.01
  double goal_orientation_tolerance_ = 1e-3;  //.01
  double offset_welding_approach_z_;

  // Planner IDs
  std::string linear_planner_id_ = "LIN";
  std::string circular_planner_id_ = "CIRC";
  std::string curve_planner_id_ = "SPLINE";
  std::string joint_space_planner_id_ = "PTP";
  std::string free_space_planner_id_ = "RRTConnect";
  std::string constrained_planner_id_ = "RRTstar";
  std::string planner_id_property_name_ = "planner";

  // for OMPL constrained planning
  std::string path_constraints_name_ = "linear_system_constraints";  // OMPL Constrained with linear equations system
  // Allowed planning time [s] and maximum number of solutions
  int planning_time_free_space_ = 1;
  int planning_time_constrained_ = 5;
  int planning_time_collisions_ = 10;
  int max_solutions_ = 10;

  // Planning group and link names
  std::string arm_group_name_ = "ur_manipulator";
  std::string welding_group_name_ = "ur_manipulator";
  std::string welding_tcp_frame_ = "tcp_welding_gun_link";
  std::string task_control_frame_ = "tcp_welding_gun_link";
  std::string arm_home_pose_ = "ready";

  // Execution
  rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr execute_;
};
}  // namespace processit_tasks

#endif
