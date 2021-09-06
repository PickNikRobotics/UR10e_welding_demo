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
using namespace std::chrono_literals;
using namespace moveit::task_constructor;
GlobalMTCPlannerComponent::GlobalMTCPlannerComponent(const rclcpp::NodeOptions& options)
  : GlobalPlannerComponent(options)
{
}

bool GlobalMTCPlannerComponent::init()
{
  return true;
}

moveit_msgs::msg::MotionPlanResponse
GlobalMTCPlannerComponent::plan(const moveit_msgs::msg::MotionPlanRequest& planning_problem)
{
  // Result
  moveit_msgs::msg::MotionPlanResponse planning_solution;
  planning_solution.error_code.val = planning_solution.error_code.SUCCESS;
  planning_solution.group_name = "ur_manipulator";

  task_ = std::make_shared<moveit::task_constructor::Task>();
  Task& t = *task_;
  t.stages()->setName("global_mtc_task");
  t.loadRobotModel(shared_from_this());

  // Sampling planner
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(shared_from_this());
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // Set task properties
  t.setProperty("group", "ur_manipulator");
  // t.setProperty("eef", eef_name_);
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
  //   for (const auto& jc : planning_problem.goal_constraints[0].joint_constraints)
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

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hybrid_planning_demo::GlobalMTCPlannerComponent)
