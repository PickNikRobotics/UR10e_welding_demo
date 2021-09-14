#include <processit_tasks/cartesian_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace processit_tasks
{
using namespace moveit::task_constructor;
const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_task");

CartesianTask::CartesianTask(rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  execute_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
      node_, "execute_task_solution");
}

void CartesianTask::loadParameters()
{
  RCLCPP_INFO(LOGGER, "[CartesianTask instance]: Loading task parameters");

  // Critical parameters (no default exists => shutdown if loading fails)
  size_t critical_errors = 0;
  critical_errors += rosparam_shortcuts::get(node_, "welding_group_name", welding_group_name_);
  critical_errors += rosparam_shortcuts::get(node_, "welding_tcp_frame", welding_tcp_frame_);
  rosparam_shortcuts::shutdownIfError(critical_errors);

  // Optional parameters (default value exists => no shutdown required if loading fails)
  size_t warnings = 0;
  warnings += rosparam_shortcuts::get(node_, "max_acceleration_scaling", max_acceleration_scaling_);
  warnings += rosparam_shortcuts::get(node_, "cartesian_rot_velocity", cartesian_rot_velocity_);
  warnings += rosparam_shortcuts::get(node_, "step_size", step_size_);
  warnings += rosparam_shortcuts::get(node_, "num_planning_attempts", num_planning_attempts_);
  warnings += rosparam_shortcuts::get(node_, "goal_position_tolerance", goal_position_tolerance_);
  warnings += rosparam_shortcuts::get(node_, "goal_orientation_tolerance", goal_orientation_tolerance_);
  warnings += rosparam_shortcuts::get(node_, "planning_time_free_space", planning_time_free_space_);
  warnings += rosparam_shortcuts::get(node_, "planning_time_constrained", planning_time_constrained_);
  warnings += rosparam_shortcuts::get(node_, "planning_time_collisions", planning_time_collisions_);
  warnings += rosparam_shortcuts::get(node_, "max_solutions", max_solutions_);

  RCLCPP_WARN(LOGGER, "Failed to load optional parameters.");
}

void CartesianTask::init(std::string task_name, std::string task_caption)
{
  task_name_ = task_name;
  task_caption_ = task_caption;

  loadParameters();
  rosparam_shortcuts::get(node_, "move_group.planning_plugin", planner_plugin_);
  rosparam_shortcuts::get(node_, "robot_description_planning.cartesian_limits.max_trans_vel", max_trans_vel_);
  rosparam_shortcuts::get(node_, "robot_description_planning.cartesian_limits.max_rot_vel", max_rot_vel_);

  RCLCPP_INFO(LOGGER, "Initializing task pipeline");

  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  Task& t = *task_;
  t.stages()->setName(task_caption + task_name);
  t.loadRobotModel(node_);

  // Set task properties
  // t.setProperty("welding", welding_group_name_);
  // t.setProperty("welding_tcp_frame", welding_tcp_frame_);
  // t.setProperty("ik_frame", welding_tcp_frame_);

  // Start from current robot state
  addCurrentState();
}

void CartesianTask::addCurrentState()
{
  Task& t = *task_;
  auto current_state = std::make_unique<stages::CurrentState>("Current state");
  t.add(std::move(current_state));
}

void CartesianTask::setPlannerProperties(
    std::shared_ptr<moveit::task_constructor::solvers::PipelinePlanner>& pipeline_planner, std::string planner_id,
    double velocity)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: setPlannerProperties");
  pipeline_planner->setProperty("step_size", step_size_);
  if (planner_plugin_ == "pilz_industrial_motion_planner::CommandPlanner")
  {
    pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_);
    pipeline_planner->setPlannerId(planner_id);
    if (planner_id == joint_space_planner_id_)
    {
      if (velocity > 0)
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", velocity);
        RCLCPP_INFO_STREAM(LOGGER, "Setting desired Maximum Joint velocity scaling factor: " << velocity);
      }
    }
    else
    {
      double trans_velocity_scaling_factor;
      double rot_velocity_scaling_factor;

      if (velocity >= max_trans_vel_)
      {
        /// desired_trans_cartesian_velocity = max_trans_vel_
        trans_velocity_scaling_factor = 1;
      }
      else
        /// desired_trans_cartesian_velocity = max_velocity_scaling_factor * max_trans_vel_
        trans_velocity_scaling_factor = velocity / max_trans_vel_;

      if (cartesian_rot_velocity_ >= max_rot_vel_)
      {
        /// desired_rot_cartesian_velocity = max_rot_vel_
        rot_velocity_scaling_factor = 1;
      }
      else
        /// desired_rot_cartesian_velocity = max_velocity_scaling_factor * max_rot_vel_
        rot_velocity_scaling_factor = cartesian_rot_velocity_ / max_rot_vel_;

      if (trans_velocity_scaling_factor > rot_velocity_scaling_factor)
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", rot_velocity_scaling_factor);
      }
      else
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", trans_velocity_scaling_factor);
      }
    }
  }
  else if (planner_plugin_ == "ompl_interface/OMPLPlanner")
  {
    if (planner_id == joint_space_planner_id_)
    {
      pipeline_planner->setPlannerId(free_space_planner_id_);
      pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_);
      pipeline_planner->setProperty("max_velocity_scaling_factor", velocity);  // max_joint_velocity
    }
    else
    {
      pipeline_planner->setProperty("goal_orientation_tolerance", goal_orientation_tolerance_);
      pipeline_planner->setPlannerId(constrained_planner_id_);
      // Requires https://github.com/ros-planning/moveit/pull/2856
      // pipeline_planner->setProperty("max_cartesian_speed", velocity);  // cartesian_velocity
      // pipeline_planner->setProperty("cartesian_speed_limited_link", welding_tcp_frame_);
    }
    // pipeline_planner->setProperty(planner_id_property_name_, constrained_planner_id_);
  }
  else
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Planner plugin name not correctly set: " << planner_plugin_);
  }
}

/**
 * @brief Adds a stage to the task. Additionally allows to specify an explicit stage-specific offset. Does not take into
 * account the task_transform_.
 *
 * @param stage_caption How the stage is named
 * @param manufacturingSubFrameID The header.frame_id of the frame to reach in this stage
 * @param manufacturingToTaskFrameID The header.frame_id of ManufacturingToTaskFrame
 * @param stage_offset The stage offset
 * @param planner_id The ID of the planner, e.g. "LIN"/"CIRC"/...
 * @param velocity The velocity which the robot should have
 */
void CartesianTask::addStage(std::string stage_caption, geometry_msgs::msg::PoseStamped goal_pose,
                             std::string planner_id, double velocity)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: Add stage stage_offset");
  // Create an instance of the planner and set its properties
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node_);
  setPlannerProperties(pipeline_planner, planner_id, velocity);

  // Create a stage with desired planner
  auto stage = std::make_unique<stages::MoveTo>(stage_caption, pipeline_planner);
  stage->setGroup(welding_group_name_);
  stage->setGoal(goal_pose);
  // stage->setTaskEnd(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
  if (planner_id == linear_planner_id_ && planner_plugin_ == "ompl_interface/OMPLPlanner")
  {
    // stage->setTaskStart(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
    // moveit_msgs::msg::Constraints path_constraints = stage->setLinConstraints();
    // stage->setPathConstraints(path_constraints);
  }
  else if (planner_id == circular_planner_id_)
  {
    // stage->setTaskStart(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
    // stage->setTaskInterim(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
    // moveit_msgs::msg::Constraints path_constraints = stage->setCircConstraints(planner_plugin_);
    // stage->setPathConstraints(path_constraints);
  }
  else if (planner_id == curve_planner_id_)
  {
    // stage->setSplineConstraints(manufacturingSubFrameID, task_transform_);
  }

  if (planner_id != joint_space_planner_id_ && planner_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_constrained_);
  else if (planner_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_free_space_);

  Task& t = *task_;
  t.add(std::move(stage));
}

bool CartesianTask::plan()
{
  RCLCPP_INFO(LOGGER, "Start searching for task solutions");
  //   ros::NodeHandle pnh("~");
  //   int max_solutions = pnh.param<int>("max_solutions", max_solutions_);

  try
  {
    task_->plan(max_solutions_);
  }
  catch (InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed");
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Planning failed, no solutions found");
    return false;
  }
  return true;
}

bool CartesianTask::execute()
{
  RCLCPP_INFO(LOGGER, "Executing solution trajectory");
  moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal execute_goal;
  task_->solutions().front()->fillMessage(execute_goal.solution);
  auto execute_future = execute_->async_send_goal(execute_goal);
  execute_future.wait();

  if (execute_future.get()->get_status() != rclcpp_action::GoalStatus::STATUS_SUCCEEDED)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return false;
  }

  return true;
}

}  // namespace processit_tasks
