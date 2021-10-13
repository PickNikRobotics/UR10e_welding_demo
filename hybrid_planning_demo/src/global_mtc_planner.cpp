#include <hybrid_planning_demo/global_mtc_planner.h>

// MTC Stages
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <processit_tasks/cartesian_task.h>

namespace hybrid_planning_demo
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_mtc_planner_component");
const std::string PLANNING_PIPELINES_NS =
    "ompl.";  // See "https://github.com/ros-planning/moveit_task_constructor/pull/170/files#r719602378"
const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
const std::string UNDEFINED = "<undefined>";
using namespace std::chrono_literals;
using namespace moveit::task_constructor;

bool GlobalMTCPlannerComponent::initialize(const rclcpp::Node::SharedPtr& node)
{
  // Declare planning pipeline paramter
  node->declare_parameter<std::vector<std::string>>(PLANNING_PIPELINES_NS + "pipeline_names",
                                                    std::vector<std::string>({ "pilz_industrial_motion_planner" }));
  node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "namespace", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "planning_plugin",
                                       "pilz_industrial_motion_planner::CommandPlanner");

  // Declare PlanRequestParameters
  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planner_id", "LIN");
  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planning_pipeline", "pilz_industrial_motion_planner");
  node->declare_parameter<int>(PLAN_REQUEST_PARAM_NS + "planning_attempts", 5);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "planning_time", 1.0);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor", 0.1);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor", 0.1);

  task_ = std::make_shared<moveit::task_constructor::Task>();
  node_ptr_ = node;
  return true;
}

bool GlobalMTCPlannerComponent::reset()
{
  return true;
}

moveit_msgs::msg::MotionPlanResponse GlobalMTCPlannerComponent::plan(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
{
  // Process goal
  if ((global_goal_handle->get_goal())->desired_motion_sequence.items.size() > 1)
  {
    RCLCPP_WARN(LOGGER, "Global planner received motion sequence request with more than one item but the "
                        "'moveit_planning_pipeline' plugin only accepts one item. Just using the first item as global "
                        "planning goal!");
  }
  auto motion_plan_req = (global_goal_handle->get_goal())->desired_motion_sequence.items[0].req;
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = "world";
  start_pose.pose.position.x = motion_plan_req.goal_constraints[0].position_constraints[0].target_point_offset.x;
  start_pose.pose.position.y = motion_plan_req.goal_constraints[0].position_constraints[0].target_point_offset.y;
  start_pose.pose.position.z = motion_plan_req.goal_constraints[0].position_constraints[0].target_point_offset.z;
  start_pose.pose.orientation = motion_plan_req.goal_constraints[0].orientation_constraints[0].orientation;
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "world";
  goal_pose.pose.position.x = motion_plan_req.goal_constraints[0].position_constraints[1].target_point_offset.x;
  goal_pose.pose.position.y = motion_plan_req.goal_constraints[0].position_constraints[1].target_point_offset.y;
  goal_pose.pose.position.z = motion_plan_req.goal_constraints[0].position_constraints[1].target_point_offset.z;
  goal_pose.pose.orientation = motion_plan_req.goal_constraints[0].orientation_constraints[0].orientation;

  // // Apply offset to get approach pose
  // Eigen::Isometry3d goal;
  // tf2::fromMsg(goal_pose.pose, goal);
  // Eigen::Isometry3d approach_offset = Eigen::Isometry3d::Identity();
  // approach_offset.translation().z() = -0.1;
  // goal = goal * approach_offset;
  // tf2::convert(goal, goal_pose.pose);

  // Set parameters required by the planning component
  node_ptr_->set_parameter({ PLAN_REQUEST_PARAM_NS + "planner_id", motion_plan_req.planner_id });
  node_ptr_->set_parameter({ PLAN_REQUEST_PARAM_NS + "planning_pipeline", motion_plan_req.pipeline_id });
  node_ptr_->set_parameter({ PLAN_REQUEST_PARAM_NS + "planning_attempts", motion_plan_req.num_planning_attempts });
  node_ptr_->set_parameter({ PLAN_REQUEST_PARAM_NS + "planning_time", motion_plan_req.allowed_planning_time });
  node_ptr_->set_parameter(
      { PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor", motion_plan_req.max_velocity_scaling_factor });
  node_ptr_->set_parameter(
      { PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor", motion_plan_req.max_acceleration_scaling_factor });

  // Result
  moveit_msgs::msg::MotionPlanResponse planning_solution;
  planning_solution.error_code.val = planning_solution.error_code.SUCCESS;
  planning_solution.group_name = "ur_manipulator";

  // task_ = std::make_shared<moveit::task_constructor::Task>();
  moveit::task_constructor::Task& t = *task_;
  t.stages()->setName("global_mtc_task");
  t.loadRobotModel(node_ptr_);

  // Set task properties
  t.setProperty("group", "ur_manipulator");
  t.setProperty("eef", "welding_ee");
  // t.setProperty("hand", hand_group_name_);
  // t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", "tcp_welding_gun_link");

  // Task initialization
  // ROS_DEBUG_NAMED(LOGNAME, "Initializing welding task");
  // task_caption_ = "pass_" + std::to_string(pass_id_) + "_";
  // task_name_ = "task_" + std::to_string(task_id_) + "_segment_" + std::to_string(segment_id_);
  std::string task_name_ = "welding_segment";
  std::string task_control_frame_tech_model_ = "tcp_welding_gun_link";  // "welding_frames/task_control_frame";
  double offset_z = 0.1;
  auto cartesian_task = processit_tasks::CartesianTask(node_ptr_, task_);
  cartesian_task.init("welding_segment", "single_pass");
  cartesian_task.viaMotion("RRTConnect", 1.0);
  cartesian_task.approachRetreat("approach_start", task_control_frame_tech_model_, offset_z);
  cartesian_task.generateStart("start_state ", start_pose, task_control_frame_tech_model_);
  cartesian_task.addStage("welding_motion ", goal_pose, task_control_frame_tech_model_, "RRTstar", 0.2);
  cartesian_task.approachRetreat("retreat_end", task_control_frame_tech_model_, -offset_z);

  /******************************************************
   *          Execution                                 *
   *****************************************************/

  constexpr size_t max_solutions = 1;
  t.plan(max_solutions);
  if (t.numSolutions() == max_solutions)
  {
    moveit_task_constructor_msgs::msg::Solution solution;
    t.solutions().front()->fillMessage(solution, &t.introspection());
    auto& solution_traj = planning_solution.trajectory;
    for (const auto& sub_traj : solution.sub_trajectory)
    {
      const auto& traj = sub_traj.trajectory;
      if (solution_traj.joint_trajectory.joint_names.empty())
        solution_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names;
      if (solution_traj.multi_dof_joint_trajectory.joint_names.empty())
        solution_traj.multi_dof_joint_trajectory.joint_names = traj.multi_dof_joint_trajectory.joint_names;

      const auto& jt = traj.joint_trajectory.points;
      solution_traj.joint_trajectory.points.insert(solution_traj.joint_trajectory.points.end(), jt.begin(), jt.end());
      const auto& mdjt = traj.multi_dof_joint_trajectory.points;
      solution_traj.multi_dof_joint_trajectory.points.insert(solution_traj.multi_dof_joint_trajectory.points.end(),
                                                             mdjt.begin(), mdjt.end());
    }
  }

  // response.getMessage(planning_solution);
  return planning_solution;
}
}  // namespace hybrid_planning_demo

// Register the component as plugin
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(hybrid_planning_demo::GlobalMTCPlannerComponent, moveit_hybrid_planning::GlobalPlannerInterface);
