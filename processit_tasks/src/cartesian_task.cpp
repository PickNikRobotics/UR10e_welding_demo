// MTC Welding IPA
#include <processit_tasks/cartesian_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace processit_tasks
{
using namespace moveit::task_constructor;
const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_task");

CartesianTask::CartesianTask(const rclcpp::Node::SharedPtr& node, const moveit::task_constructor::TaskPtr& task)
{
  node_ = node;
  execute_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
      node_, "execute_task_solution");
  task_ = task;
}

void CartesianTask::init(std::string task_name, std::string task_caption)
{
  task_name_ = task_name;
  task_caption_ = task_caption;

  // loadParameters();
  // nh_.param<std::string>("/move_group/planning_plugin", planner_plugin_, "");
  // nh_.param<double>("/robot_description_planning/cartesian_limits/max_trans_vel", max_trans_vel_, 0.0);
  // nh_.param<double>("/robot_description_planning/cartesian_limits/max_rot_vel", max_rot_vel_, 0.0);

  RCLCPP_INFO(LOGGER, "Initializing task pipeline");

  // Reset ROS introspection before constructing the new object
  // TODO(henningkayser): verify this is a bug, fix if possible
  // task_.reset();
  // task_.reset(new moveit::task_constructor::Task());

  // Task &t = *task_;
  // t.stages()->setName(task_caption + task_name);
  // t.loadRobotModel();

  // Set task properties
  // t.setProperty("welding", welding_group_name_);
  // t.setProperty("welding_tcp_frame", welding_tcp_frame_);
  // t.setProperty("ik_frame", welding_tcp_frame_);

  current_state_ptr_ = nullptr;  // Forward current_state on to pose generator
  // Start from current robot state
  addCurrentState();
}

void CartesianTask::addCurrentState()
{
  Task& t = *task_;
  auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
  {
    auto current = std::make_unique<stages::CurrentState>("Current state");
    current_state_ptr_ = current.get();
    t.add(std::move(current));
  }
}

//     void CartesianTask::addFixedState(std::string group_state)
// {
//         Task &t = *task_;
//         RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: addFixedState");
//         auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
//         auto &state = scene->getCurrentStateNonConst();
//         state.setToDefaultValues(state.getJointModelGroup(arm_group_name_), group_state);

//         auto fixed = std::make_unique<stages::FixedState>("final state");
//         fixed->setState(scene);
//         t.add(std::move(fixed));
//     }

void CartesianTask::viaMotion(std::string planner_id, double velocity)
{
  Task& t = *task_;
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: viaMotion");
  // Create an instance of the planner and set its properties
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>();
  pipeline_planner->setProperty("goal_joint_tolerance", 1e-5);
  pipeline_planner->setPlannerId(free_space_planner_id_);
  pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_);
  pipeline_planner->setProperty("max_velocity_scaling_factor", 1.0);  // max_joint_velocity

  // setPlannerProperties(pipeline_planner, planner_id, velocity);
  auto connect = std::make_unique<stages::Connect>(
      "Move between processes", stages::Connect::GroupPlannerVector{ { arm_group_name_, pipeline_planner } });
  connect->setTimeout(5.0);
  // connect->init();
  // connect->properties().configureInitFrom(Stage::PARENT);
  t.add(std::move(connect));
}

void CartesianTask::approachRetreat(const std::string stage_caption, const std::string task_control_frame,
                                    const double offset_approach_z)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: approachRetreat");
  // Create a stage with desired planner
  auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
  cartesian_interpolation->setMaxVelocityScaling(0.2);
  cartesian_interpolation->setMaxAccelerationScaling(max_acceleration_scaling_);
  cartesian_interpolation->setStepSize(.01);
  auto stage = std::make_unique<stages::MoveRelative>(stage_caption, cartesian_interpolation);
  stage->properties().set("marker_ns", "stage_caption");
  // stage->setIKFrame(task_control_frame);
  stage->setGroup(welding_group_name_);
  // stage->setMinMaxDistance(offset_approach_z, offset_approach_z * 1.1);

  // Set hand forward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = welding_tcp_frame_;
  vec.vector.z = offset_approach_z;
  stage->setDirection(vec);

  // stage->insert(std::move(ik));
  Task& t = *task_;
  t.add(std::move(stage));
}

void CartesianTask::setPlannerProperties(moveit::task_constructor::solvers::PipelinePlannerPtr& pipeline_planner,
                                         std::string planner_id, double velocity)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: setPlannerProperties");
  pipeline_planner->setProperty("step_size", step_size_);
  pipeline_planner->setProperty("goal_joint_tolerance", 1e-5);
  if (planner_plugin_ == "pilz_industrial_motion_planner::CommandPlanner")
  {
    pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_);
    pipeline_planner->setPlannerId(planner_id);
    if (planner_id != joint_space_planner_id_)
    {
      double trans_velocity_scaling_factor;
      double rot_velocity_scaling_factor;

      if (velocity >= max_trans_vel_)
      {
        /// desired_trans_cartesian_velocity = max_trans_vel_
        trans_velocity_scaling_factor = 1;
        RCLCPP_ERROR_STREAM(LOGGER, "[CartesianTask instance]: The desired Cartesian translational velocity is above "
                                    "Pilz Maximum, setting translation scaling factor to 1");
      }
      else
        /// desired_trans_cartesian_velocity = max_velocity_scaling_factor * max_trans_vel_
        trans_velocity_scaling_factor = velocity / max_trans_vel_;

      if (cartesian_rot_velocity_ >= max_rot_vel_)
      {
        /// desired_rot_cartesian_velocity = max_rot_vel_
        rot_velocity_scaling_factor = 1;
        RCLCPP_INFO_STREAM(LOGGER, "[CartesianTask instance]: The desired Cartesian rotational velocity is above Pilz "
                                   "Maximum, setting rotation scaling factor to 1");
      }
      else
        /// desired_rot_cartesian_velocity = max_velocity_scaling_factor * max_rot_vel_
        rot_velocity_scaling_factor = cartesian_rot_velocity_ / max_rot_vel_;

      if (trans_velocity_scaling_factor > rot_velocity_scaling_factor)
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", rot_velocity_scaling_factor);
        RCLCPP_INFO_STREAM(LOGGER, "[CartesianTask instance]: Setting scaling factor to fullfill desired/max Cartesian "
                                   "rotational velocity: "
                                       << rot_velocity_scaling_factor);
      }
      else
      {
        pipeline_planner->setProperty("max_velocity_scaling_factor", trans_velocity_scaling_factor);
        RCLCPP_INFO_STREAM(LOGGER, "[CartesianTask instance]: Setting scaling factor to fullfill desired/max Cartesian "
                                   "translational velocity: "
                                       << trans_velocity_scaling_factor);
      }
    }
    else if (velocity > 0.0)
    {
      pipeline_planner->setProperty("max_velocity_scaling_factor", velocity);
      RCLCPP_INFO_STREAM(LOGGER, "[CartesianTask instance]: Setting desired Maximum Joint velocity scaling factor: "
                                     << velocity);
    }
    else
      RCLCPP_ERROR_STREAM(LOGGER,
                          "[CartesianTask instance]: Desired velocity was not properly set (0 is not possible)");
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
      // pipeline_planner->setProperty("max_cartesian_speed", velocity); // cartesian_velocity
      // pipeline_planner->setProperty("cartesian_speed_limited_link", welding_tcp_frame_);
      RCLCPP_INFO_STREAM(LOGGER,
                         "[CartesianTask instance]: Setting max "
                             << welding_tcp_frame_
                             << " speed to fullfill desired Cartesian translational velocity [m/s]: " << velocity);
    }
    // pipeline_planner->setProperty(planner_id_property_name_, constrained_planner_id_);
  }
  else
    RCLCPP_ERROR_STREAM(LOGGER, "Planner plugin name not correctly set: " << planner_plugin_);
}

void CartesianTask::generateStart(std::string stage_caption, geometry_msgs::msg::PoseStamped goal_pose,
                                  std::string task_control_frame)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: Add stage stage_offset");
  // Create a stage with desired planner
  auto stage = std::make_unique<stages::GeneratePose>("generate start pose");
  stage->setPose(goal_pose);
  stage->setMonitoredStage(current_state_ptr_);  // Hook into current state
  // Compute IK
  auto ik = std::make_unique<stages::ComputeIK>("start pose IK", std::move(stage));
  // ik->insert(std::move(initial));
  ik->setGroup(welding_group_name_);
  // ik->setTargetPose(target_pose);
  ik->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
  ik->properties().set("ignore_collisions", true);
  // ik->setTimeout(5.0);
  // ik->setMinSolutionDistance(0.0);
  // ik->setMaxIKSolutions(100);

  ik->setIKFrame(task_control_frame);
  // ik->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
  // ik->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});

  // auto cl_cost{std::make_unique<cost::Clearance>()};
  // cl_cost->cumulative = nh.param("cumulative", false); // sum up pairwise distances?
  // cl_cost->with_world = nh.param("with_world", true);  // consider distance to world objects?
  // ik->setCostTerm(std::move(cl_cost));

  // stage->insert(std::move(ik));
  Task& t = *task_;
  t.add(std::move(ik));
}

/**
 * @brief Adds a new stage to the task. Offset is defined through task_transform_.
 *
 * @param stage_caption How the stage is named
 * @param goal_pose the goal pose to reach in this stage
 * @param task_control_frame The name of the attached subframe to be controlled and used of IK
 * @param planner_id The ID of the planner, e.g. "LIN"/"CIRC"/...
 * @param velocity The velocity which the robot should have
 */
void CartesianTask::addStage(std::string stage_caption, geometry_msgs::msg::PoseStamped goal_pose,
                             std::string task_control_frame, std::string planner_id, double velocity)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: addStage");
  // Create an instance of the planner and set its properties
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node_);
  setPlannerProperties(pipeline_planner, planner_id, velocity);

  // Create a stage with desired planner
  auto stage = std::make_unique<stages::MoveTo>(stage_caption, pipeline_planner);
  stage->setGroup(welding_group_name_);
  stage->setIKFrame(task_control_frame);
  stage->setGoal(goal_pose);

  if (planner_id != joint_space_planner_id_ && planner_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_constrained_);
  else if (planner_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_free_space_);

  // Add the stage to the task
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
