/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sebastian Jahr, Adam Pettinger
 */

#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hybrid_planning_demo/servo_solver.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace hybrid_planning_demo
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

bool ServoSolver::initialize(const rclcpp::Node::SharedPtr& node,
                             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                             const std::string& group_name)
{
  // Load parameter & initialize member variables
  // if (node->has_parameter("velocity_scaling_threshold"))
  //   node->get_parameter<double>("velocity_scaling_threshold", velocity_scaling_threshold_);
  // else
  //   velocity_scaling_threshold_ = node->declare_parameter<double>("velocity_scaling_threshold", 0.0);

  planning_scene_monitor_ = planning_scene_monitor;
  node_ = node;
  // joint_cmd_pub_ = node_->create_publisher<control_msgs::msg::JointJog>("~/delta_joint_cmds", 10);
  // twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("~/delta_twist_cmds", 10);
  // ee_tf_pub_ = node_->create_publisher<geometry_msgs::msg::TransformStamped>("~/eef_position", 10);

  // traj_cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
  //     "/joint_trajectory_controller/joint_trajectory", 10);

  // Get Servo Parameters
  // Get the servo parameters.
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(node, param_namespace);
  servo_parameters_ = servo_param_listener->get_params();

  // Create Servo and start it
  servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_param_listener, planning_scene_monitor_);

  // Create publisher to send servo commands
  // joint_cmd_pub_ =
  //     node_->create_publisher<control_msgs::msg::JointJog>(servo_parameters_.joint_command_in_topic, 1);
  return true;
}

bool ServoSolver::reset()
{
  RCLCPP_INFO(LOGGER, "Reset Servo Solver");
  joint_cmd_rolling_window_.clear();
  return true;
};

moveit_msgs::action::LocalPlanner::Feedback
ServoSolver::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                   const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
                   trajectory_msgs::msg::JointTrajectory& local_solution)
{
  // Create Feedback
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;

  // Transform next robot trajectory waypoint into JointJog message
  moveit_msgs::msg::RobotTrajectory robot_command;
  local_trajectory.getRobotTrajectoryMsg(robot_command);

  if (robot_command.joint_trajectory.points.empty())
  {
    feedback_result.feedback = "Reference trajectory does not contain any points";
    return feedback_result;
  }

  // If we're getting to close to a Replan if velocity scaling is below threshold
  // if (replan_)
  //{
  //  if (!feedback_send_)
  //  {
  //    feedback_result.feedback = "collision_ahead";
  //    auto msg = std::make_unique<control_msgs::msg::JointJog>();
  //    msg->header.stamp = node_->now();
  //    msg->joint_names = robot_command.joint_trajectory.joint_names;
  //    msg->velocities.assign(robot_command.joint_trajectory.joint_names.size(), 0);
  //    joint_cmd_pub_->publish(std::move(msg));
  //    publish_ = true;
  //  }
  //  feedback_send_ = true;  // Give the architecture time to handle feedback
  //}
  // else
  //{
  //  feedback_send_ = false;
  //}

  // Get current state
  const auto current_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();

  // Create goal state
  moveit::core::RobotState target_state(local_trajectory.getRobotModel());
  target_state.setVariablePositions(robot_command.joint_trajectory.joint_names,
                                    robot_command.joint_trajectory.points[0].positions);
  target_state.update();

  // TF planning_frame -> current EE
  Eigen::Isometry3d current_pose = current_state->getFrameTransform("tcp_welding_gun_link");
  // TF planning -> target EE
  Eigen::Isometry3d target_pose = target_state.getFrameTransform("tcp_welding_gun_link");

  // current EE -> planning frame * planning frame -> target EE
  Eigen::Isometry3d diff_pose = current_pose.inverse() * target_pose;
  Eigen::AngleAxisd axis_angle(diff_pose.linear());

  constexpr double fixed_vel = 0.05;
  const double vel_scale = fixed_vel / diff_pose.translation().norm();

  // Calculate Cartesian command delta
  // Transform current pose to command frame
  // Eigen::Isometry3d current_pose = current_state->getFrameTransform(servo_parameters_.robot_link_command_frame);
  // Transform goal pose to command frame
  servo_->setCommandType(moveit_servo::CommandType::TWIST);
  moveit_servo::TwistCommand target_twist{
    "tcp_welding_gun_link",
    { diff_pose.translation().x() * vel_scale, diff_pose.translation().y() * vel_scale,
      diff_pose.translation().z() * vel_scale, axis_angle.axis().x() * axis_angle.angle() * vel_scale,
      axis_angle.axis().y() * axis_angle.angle() * vel_scale, 0.0 }
  };

  std::optional<trajectory_msgs::msg::JointTrajectory> trajectory_msg;
  while (!trajectory_msg)
  {
    moveit_servo::KinematicState joint_state = servo_->getNextJointState(current_state, target_twist);
    const auto status = servo_->getStatus();
    if (status == moveit_servo::StatusCode::INVALID)
    {
      feedback_result.feedback = "Servo StatusCode 'INVALID'";
      return feedback_result;
    }
    moveit_servo::updateSlidingWindow(joint_state, joint_cmd_rolling_window_, servo_parameters_.max_expected_latency,
                                      node_->now());
    if (!joint_cmd_rolling_window_.empty())
    {
      current_state->setJointGroupPositions(current_state->getJointModelGroup(servo_parameters_.move_group_name),
                                            joint_cmd_rolling_window_.back().positions);
      current_state->setJointGroupVelocities(current_state->getJointModelGroup(servo_parameters_.move_group_name),
                                             joint_cmd_rolling_window_.back().velocities);
    }
    trajectory_msg = moveit_servo::composeTrajectoryMessage(servo_parameters_, joint_cmd_rolling_window_);
  }
  local_solution = trajectory_msg.value();

  // Create twist command
  // current EE -> planning frame * planning frame -> target EE
  // Eigen::Isometry3d diff_pose = current_pose.inverse() * target_pose;
  // Eigen::AngleAxisd axis_angle(diff_pose.linear());

  // Scale velocity
  // constexpr double fixed_vel = 0.05;
  // const double vel_scale = fixed_vel / diff_pose.translation().norm();

  // Create twist command msg
  // auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  // msg->header.stamp = node_->now();
  // msg->twist.linear.x = diff_pose.translation().x() * vel_scale;
  // msg->twist.linear.y = diff_pose.translation().y() * vel_scale;
  // The lase correction should only happen along the z axis
  // msg->twist.linear.z = diff_pose.translation().z() * vel_scale - laser_correction_;
  // msg->twist.angular.x = axis_angle.axis().x() * axis_angle.angle() * vel_scale;
  // msg->twist.angular.y = axis_angle.axis().y() * axis_angle.angle() * vel_scale;

  // Rotation joint is laser correction, not from delta-position
  // msg->twist.angular.z = 0;  // laser_correction_;

  // twist_cmd_pub_->publish(std::move(msg));

  // Publish EEF Position
  // geometry_msgs::msg::TransformStamped tf;
  // if (servo_->getEEFrameTransform(tf))
  //  ee_tf_pub_->publish(tf);

  return feedback_result;
}
}  // namespace hybrid_planning_demo

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hybrid_planning_demo::ServoSolver, moveit::hybrid_planning::LocalConstraintSolverInterface);
