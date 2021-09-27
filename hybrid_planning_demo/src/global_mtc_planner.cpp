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

#include <processit_tasks/cartesian_task.h>

namespace hybrid_planning_demo
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_mtc_planner_component");
const std::string PLANNING_PIPELINES_NS = "planning_pipelines.";
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

  task_ = std::make_shared<moveit::task_constructor::Task>();
  moveit::task_constructor::Task& t = *task_;
  t.stages()->setName("global_mtc_task");
  t.loadRobotModel(node_ptr_);

  // Sampling planner
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node_ptr_);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  sampling_planner->setProperty("max_velocity_scaling_factor", motion_plan_req.max_velocity_scaling_factor);
  sampling_planner->setProperty("max_acceleration_scaling_factor", motion_plan_req.max_acceleration_scaling_factor);
  sampling_planner->setProperty("planning_attempts", motion_plan_req.num_planning_attempts);
  sampling_planner->setProperty("planning_time", motion_plan_req.allowed_planning_time);
  sampling_planner->setPlannerId("LIN");

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties
  t.setProperty("group", "ur_manipulator");
  t.setProperty("eef", "welding_ee");
  // t.setProperty("hand", hand_group_name_);
  // t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", "tcp_welding_gun_link");

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  using namespace moveit::task_constructor::stages;
  {
    auto current_state = std::make_unique<stages::CurrentState>("current state");
    t.add(std::move(current_state));
  }

  /******************************************************
   *          Joint Goal Motion                         *
   *****************************************************/
  // Move to joint state constraints specified in goal request
  // NOTE: this assumes only joint constraints are specified
  // {
  //   auto stage = std::make_unique<stages::MoveTo>("move to goal", sampling_planner);
  //   stage->setGroup("ur_manipulator");
  //   std::map<std::string, double> goal_state;
  //   for (const auto& jc : motion_plan_req.goal_constraints[0].joint_constraints)
  //   {
  //     goal_state[jc.joint_name] = jc.position;
  //   }
  //   stage->setGoal(goal_state);
  //   t.add(std::move(stage));
  // }

  /******************************************************
   *          Relative Motion                           *
   *****************************************************/
  {
    auto stage = std::make_unique<stages::MoveRelative>("relative motion", cartesian_planner);
    stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    stage->setIKFrame("tcp_welding_gun_link");
    stage->properties().set("marker_ns", "retreat");
    geometry_msgs::msg::Vector3Stamped vec;
    vec.header.frame_id = "world";
    vec.vector.y = 0.5;
    stage->setDirection(vec);
    t.add(std::move(stage));
  }

  /******************************************************
   *          Cartesian Motion                          *
   *****************************************************/

  // TODO Work in progress
  // geometry_msgs::msg::PoseStamped goal_pose;
  // goal_pose.header.frame_id = "world";
  // goal_pose.pose.position.x = 0.35;
  // goal_pose.pose.position.y = 0.11;
  // goal_pose.pose.position.z = 0.57;
  // goal_pose.pose.orientation.x = 0.6;
  // goal_pose.pose.orientation.y = 0.66;
  // goal_pose.pose.orientation.z = -0.3;
  // goal_pose.pose.orientation.w = 0.31;
  // processit_tasks::CartesianTask cartesian_task_(node_ptr_);
  // cartesian_task_.init("test", "test");
  // cartesian_task_.addStage("Test Cartesian Path", goal_pose, "LIN", 0.1);

  // constexpr size_t max_solutions = 1;
  // cartesian_task_.task_->plan();
  // if (cartesian_task_.task_->numSolutions() == max_solutions)
  // {
  //   moveit_task_constructor_msgs::msg::Solution solution;
  //   cartesian_task_.task_->solutions().front()->fillMessage(solution, &cartesian_task_.task_->introspection());
  //   auto& solution_traj = planning_solution.trajectory;
  //   for (const auto& sub_traj : solution.sub_trajectory)
  //   {
  //     const auto& traj = sub_traj.trajectory;
  //     if (solution_traj.joint_trajectory.joint_names.empty())
  //       solution_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names;
  //     if (solution_traj.multi_dof_joint_trajectory.joint_names.empty())
  //       solution_traj.multi_dof_joint_trajectory.joint_names = traj.multi_dof_joint_trajectory.joint_names;

  //     const auto& jt = traj.joint_trajectory.points;
  //     solution_traj.joint_trajectory.points.insert(solution_traj.joint_trajectory.points.end(), jt.begin(),
  //     jt.end()); const auto& mdjt = traj.multi_dof_joint_trajectory.points;
  //     solution_traj.multi_dof_joint_trajectory.points.insert(solution_traj.multi_dof_joint_trajectory.points.end(),
  //                                                            mdjt.begin(), mdjt.end());
  //   }
  // }

  // {
  //   auto stage = std::make_unique<stages::MoveTo>("move to cartesian", cartesian_planner);
  //   stage->properties().configureInitFrom(Stage::PARENT, { "group" });
  //   stage->setGroup("ur_manipulator");
  //   stage->setGoal(goal_pose);
  //   stage->restrictDirection(stages::MoveTo::FORWARD);
  //   t.add(std::move(stage));
  // }

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
