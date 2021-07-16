// MTC Welding IPA
#include "processit_tasks/cartesian_task.h"

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
  critical_errors += rosparam_shortcuts::get(node_, "arm_group_name", arm_group_name_);
  critical_errors += rosparam_shortcuts::get(node_, "moveit_ee_name", moveit_ee_name_);
  critical_errors += rosparam_shortcuts::get(node_, "welding_group_name", welding_group_name_);
  critical_errors += rosparam_shortcuts::get(node_, "welding_tcp_frame", welding_tcp_frame_);

  rosparam_shortcuts::shutdownIfError(critical_errors);

  // Optional parameters (default value exists => no shutdown required if loading fails)
  size_t warnings = 0;
  warnings += rosparam_shortcuts::get(node_, "max_acceleration_scaling", max_acceleration_scaling_);
  warnings += rosparam_shortcuts::get(node_, "via_max_joint_velocity", via_max_joint_velocity_);
  warnings += rosparam_shortcuts::get(node_, "away_max_joint_velocity", away_max_joint_velocity_);
  warnings += rosparam_shortcuts::get(node_, "via_velocity", via_velocity_);
  warnings += rosparam_shortcuts::get(node_, "cartesian_velocity", cartesian_velocity_);
  warnings += rosparam_shortcuts::get(node_, "cartesian_rot_velocity", cartesian_rot_velocity_);
  warnings += rosparam_shortcuts::get(node_, "step_size", step_size_);
  warnings += rosparam_shortcuts::get(node_, "num_planning_attempts", num_planning_attempts_);
  warnings += rosparam_shortcuts::get(node_, "goal_joint_tolerance", goal_joint_tolerance_);
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
  rosparam_shortcuts::get(node_, "/move_group/planning_plugin", planner_plugin_);
  rosparam_shortcuts::get(node_, "/robot_description_planning/cartesian_limits/max_trans_vel", max_trans_vel_);
  rosparam_shortcuts::get(node_, "/robot_description_planning/cartesian_limits/max_rot_vel", max_rot_vel_);

  RCLCPP_INFO(LOGGER, "Initializing task pipeline");

  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  Task& t = *task_;
  t.stages()->setName(task_caption + task_name);
  t.loadRobotModel();

  // Set task properties
  // t.setProperty("arm", arm_group_name_);
  // t.setProperty("eef", moveit_ee_name_);
  // t.setProperty("welding", welding_group_name_);
  // t.setProperty("welding_tcp_frame", welding_tcp_frame_);
  // t.setProperty("ik_frame", welding_tcp_frame_);

  // Set task transform
  task_transform_.translation.x = task_transform_x_;
  task_transform_.translation.y = task_transform_y_;
  task_transform_.translation.z = task_transform_z_;
  tf2::Quaternion task_rotation_tf;
  task_rotation_tf.setEuler(task_transform_roll_, task_transform_pitch_, task_transform_yaw_);
  task_rotation_tf.normalize();
  tf2::convert(task_rotation_tf, task_transform_.rotation);

  // Start from current robot state
  addCurrentState();
}

void CartesianTask::addCurrentState()
{
  Task& t = *task_;
  auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
  {
    auto current = std::make_unique<stages::CurrentState>("Current state");
    t.add(std::move(current));
  }
}

void CartesianTask::applyTaskTransformOffset()
{
  task_transform_.translation.x = cartesian_offset_x_;
  task_transform_.translation.y = cartesian_offset_y_;
  task_transform_.translation.z = cartesian_offset_z_;

  double roll_angle = cartesian_offset_roll_;
  double pitch_angle = cartesian_offset_pitch_;
  double yaw_angle = cartesian_offset_yaw_;

  tf2::Quaternion task_rotation_tf;
  task_rotation_tf.setEuler(roll_angle, pitch_angle, yaw_angle);
  task_rotation_tf.normalize();
  tf2::convert(task_rotation_tf, task_transform_.rotation);
}

void CartesianTask::setTaskTransformOffset(double x, double y, double z, double roll, double pitch, double yaw)
{
  cartesian_offset_x_ = x;
  cartesian_offset_y_ = y;
  cartesian_offset_z_ = z;
  cartesian_offset_roll_ = roll;
  cartesian_offset_pitch_ = pitch;
  cartesian_offset_yaw_ = yaw;
  applyTaskTransformOffset();
}

void CartesianTask::setTaskTransformOffset(geometry_msgs::msg::Transform new_task_transform)
{
  task_transform_ = new_task_transform;
}

/**
 * @brief Adds a new stage to the task. Offset is defined through task_transform_.
 *
 * @param stage_caption How the stage is named
 * @param manufacturingSubFrameID The header.frame_id of the frame to reach in this stage
 * @param planner_id The ID of the planner, e.g. "LIN"/"CIRC"/...
 * @param velocity The velocity which the robot should have
 */
void CartesianTask::addStage(std::string stage_caption, std::string manufacturingSubFrameID, std::string planner_id,
                             double velocity)
{
  // Create an instance of the planner and set the parameters for it
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>();
  pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_);
  pipeline_planner->setProperty("step_size", step_size_);
  // pipeline_planner->setProperty("goal_joint_tolerance", 1e-5);
  if (planner_plugin_ == "pilz_industrial_motion_planner::CommandPlanner")
  {
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
        RCLCPP_ERROR_STREAM(LOGGER, "The desired Cartesian translational velocity is above Pilz Maximum, setting "
                                    "translation scaling factor to 1");
      }
      else
        /// desired_trans_cartesian_velocity = max_velocity_scaling_factor * max_trans_vel_
        trans_velocity_scaling_factor = velocity / max_trans_vel_;

      if (cartesian_rot_velocity_ >= max_rot_vel_)
      {
        /// desired_rot_cartesian_velocity = max_rot_vel_
        rot_velocity_scaling_factor = 1;
        RCLCPP_ERROR_STREAM(LOGGER, "The desired Cartesian rotational velocity is above Pilz Maximum, setting rotation "
                                    "scaling factor to 1");
      }
      else
        /// desired_rot_cartesian_velocity = max_velocity_scaling_factor * max_rot_vel_
        rot_velocity_scaling_factor = cartesian_rot_velocity_ / max_rot_vel_;

      if (trans_velocity_scaling_factor > rot_velocity_scaling_factor)
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", rot_velocity_scaling_factor);
        RCLCPP_INFO_STREAM(LOGGER, "Setting scaling factor to fullfill desired/max Cartesian rotational velocity: "
                                       << rot_velocity_scaling_factor);
      }
      else
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", trans_velocity_scaling_factor);
        RCLCPP_INFO_STREAM(LOGGER, "Setting scaling factor to fullfill desired/max Cartesian translational velocity: "
                                       << trans_velocity_scaling_factor);
      }
    }
  }
  else if (planner_plugin_ == "ompl_interface/OMPLPlanner")
  {
    if (planner_id == joint_space_planner_id_)
    {
      pipeline_planner->setPlannerId(free_space_planner_id_);
      pipeline_planner->setProperty("max_velocity_scaling_factor", velocity);  // max_joint_velocity
    }
    else
    {
      pipeline_planner->setProperty("goal_orientation_tolerance", goal_orientation_tolerance_);
      pipeline_planner->setPlannerId(constrained_planner_id_);
      pipeline_planner->setProperty("max_cartesian_speed", velocity);  // cartesian_velocity
      pipeline_planner->setProperty("cartesian_speed_end_effector_link", welding_tcp_frame_);
      RCLCPP_INFO_STREAM(LOGGER, "Setting max " << welding_tcp_frame_
                                                << " speed to fullfill desired Cartesian translational velocity [m/s]: "
                                                << velocity);
    }
    // pipeline_planner->setProperty(planner_id_property_name_, constrained_planner_id_);
  }
  else
    RCLCPP_ERROR_STREAM(LOGGER, "Planner plugin name not correctly set: " << planner_plugin_);

  // Create a stage and include the planner in it
  auto stage = std::make_unique<stages::MoveToTaskFrame>(stage_caption, pipeline_planner);
  stage->setGroup(welding_group_name_);
  stage->setTaskEnd(manufacturingSubFrameID, task_transform_);
  if (planner_id == linear_planner_id_ && planner_plugin_ == "ompl_interface/OMPLPlanner")
  {
    stage->setTaskStart(manufacturingSubFrameID, task_transform_);
    moveit_msgs::msg::Constraints path_constraints = stage->setLinConstraints();
    stage->setPathConstraints(path_constraints);
  }
  else if (planner_id == circular_planner_id_)
  {
    stage->setTaskStart(manufacturingSubFrameID, task_transform_);
    stage->setTaskInterim(manufacturingSubFrameID, task_transform_);
    moveit_msgs::msg::Constraints path_constraints = stage->setCircConstraints(planner_plugin_);
    stage->setPathConstraints(path_constraints);
  }
  else if (planner_id == curve_planner_id_)
  {
    // stage->setSplineConstraints(manufacturingSubFrameID, task_transform_);
  }

  if (planner_id != joint_space_planner_id_ && planner_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_constrained_);
  else if (planner_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_free_space_);

  // Add the stage to the task
  Task& t = *task_;
  t.add(std::move(stage));
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
void CartesianTask::addStage(std::string stage_caption, std::string manufacturingSubFrameID,
                             std::string manufacturingToTaskFrameID, geometry_msgs::msg::Transform stage_offset,
                             std::string planner_id, double velocity)
{
  // Create an instance of the planner and set the parameters for it
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>();
  pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_);
  pipeline_planner->setProperty("step_size", step_size_);
  // pipeline_planner->setProperty("goal_joint_tolerance", 1e-5);
  if (planner_plugin_ == "pilz_industrial_motion_planner::CommandPlanner")
  {
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
        RCLCPP_ERROR_STREAM(LOGGER, "The desired Cartesian translational velocity is above Pilz Maximum, setting "
                                    "translation scaling factor to 1");
      }
      else
        /// desired_trans_cartesian_velocity = max_velocity_scaling_factor * max_trans_vel_
        trans_velocity_scaling_factor = velocity / max_trans_vel_;

      if (cartesian_rot_velocity_ >= max_rot_vel_)
      {
        /// desired_rot_cartesian_velocity = max_rot_vel_
        rot_velocity_scaling_factor = 1;
        RCLCPP_ERROR_STREAM(LOGGER, "The desired Cartesian rotational velocity is above Pilz Maximum, setting rotation "
                                    "scaling factor to 1");
      }
      else
        /// desired_rot_cartesian_velocity = max_velocity_scaling_factor * max_rot_vel_
        rot_velocity_scaling_factor = cartesian_rot_velocity_ / max_rot_vel_;

      if (trans_velocity_scaling_factor > rot_velocity_scaling_factor)
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", rot_velocity_scaling_factor);
        RCLCPP_INFO_STREAM(LOGGER, "Setting scaling factor to fullfill desired/max Cartesian rotational velocity: "
                                       << rot_velocity_scaling_factor);
      }
      else
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", trans_velocity_scaling_factor);
        RCLCPP_INFO_STREAM(LOGGER, "Setting scaling factor to fullfill desired/max Cartesian translational velocity: "
                                       << trans_velocity_scaling_factor);
      }
    }
  }
  else if (planner_plugin_ == "ompl_interface/OMPLPlanner")
  {
    if (planner_id == joint_space_planner_id_)
    {
      pipeline_planner->setPlannerId(free_space_planner_id_);
      pipeline_planner->setProperty("max_velocity_scaling_factor", velocity);  // max_joint_velocity
    }
    else
    {
      pipeline_planner->setProperty("goal_orientation_tolerance", goal_orientation_tolerance_);
      pipeline_planner->setPlannerId(constrained_planner_id_);
      pipeline_planner->setProperty("max_cartesian_speed", velocity);  // cartesian_velocity
      pipeline_planner->setProperty("cartesian_speed_end_effector_link", welding_tcp_frame_);
      RCLCPP_INFO_STREAM(LOGGER, "Setting max " << welding_tcp_frame_
                                                << " speed to fullfill desired Cartesian translational velocity [m/s]: "
                                                << velocity);
    }
    // pipeline_planner->setProperty(planner_id_property_name_, constrained_planner_id_);
  }
  else
    RCLCPP_ERROR_STREAM(LOGGER, "Planner plugin name not correctly set: " << planner_plugin_);

  // Create a stage and include the planner in it
  auto stage = std::make_unique<stages::MoveToTaskFrame>(stage_caption, pipeline_planner);
  stage->setGroup(welding_group_name_);
  stage->setTaskEnd(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
  if (planner_id == linear_planner_id_ && planner_plugin_ == "ompl_interface/OMPLPlanner")
  {
    stage->setTaskStart(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
    moveit_msgs::msg::Constraints path_constraints = stage->setLinConstraints();
    stage->setPathConstraints(path_constraints);
  }
  else if (planner_id == circular_planner_id_)
  {
    stage->setTaskStart(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
    stage->setTaskInterim(manufacturingSubFrameID, manufacturingToTaskFrameID, stage_offset);
    moveit_msgs::msg::Constraints path_constraints = stage->setCircConstraints(planner_plugin_);
    stage->setPathConstraints(path_constraints);
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
    RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0)
  {
    RCLCPP_ERROR(LOGGER, "Planning failed, no solutions found");
    return false;
  }
  return true;
}

bool CartesianTask::execute()
{
  RCLCPP_INFO(LOGGER, "Executing solution trajectory");
  moveit_task_constructor_msgs::action::ExecuteTaskSolutionGoal execute_goal;
  task_->solutions().front()->fillMessage(execute_goal.solution);
  execute_->sendGoal(execute_goal);
  execute_->waitForResult();
  moveit_msgs::msg::MoveItErrorCodes execute_result = execute_->getResult()->error_code;

  if (execute_result->val != moveit_msg::msgs::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_->getState().toString());
    return false;
  }

  return true;
}
}  // namespace processit_tasks
