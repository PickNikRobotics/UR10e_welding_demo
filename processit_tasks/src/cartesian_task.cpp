// MTC Welding IPA
#include <moveit/planning_scene/planning_scene.h>
#include <processit_tasks/cartesian_task.h>

namespace processit_tasks
{
const std::string PLANNING_PIPELINES_NS =
    "ompl.";  // See "https://github.com/ros-planning/moveit_task_constructor/pull/170/files#r719602378"
const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
const std::string WELDING_PARAM_NS = "welding_params.";
using namespace moveit::task_constructor;
const rclcpp::Logger LOGGER = rclcpp::get_logger("cartesian_task");

CartesianTask::CartesianTask(const rclcpp::Node::SharedPtr& node, const moveit::task_constructor::TaskPtr& task)
{
  node_ = node;
  // Declare WeldingParameters
  node_->declare_parameter<std::string>(WELDING_PARAM_NS + "arm_group_name", "ur_manipulator");
  node_->declare_parameter<std::string>(WELDING_PARAM_NS + "welding_tcp_frame", "tcp_welding_gun_link");
  node_->declare_parameter<std::string>(WELDING_PARAM_NS + "task_control_frame", "welding_frames/task_control_frame");
  node_->declare_parameter<std::string>(WELDING_PARAM_NS + "arm_home_pose", "ready");
  node_->declare_parameter<std::string>(WELDING_PARAM_NS + "workpiece_frame", "workpiece");
  node_->declare_parameter<double>(WELDING_PARAM_NS + "welding_velocity", 0.0);    // [cm/min], see velocity_convert_
  node_->declare_parameter<double>(WELDING_PARAM_NS + "cartesian_velocity", 0.0);  // [m/s]
  node_->declare_parameter<double>(WELDING_PARAM_NS + "via_velocity", 0.0);        // velocity scaling factor [0.0001;1]
  node_->declare_parameter<double>(WELDING_PARAM_NS + "offset_welding_approach_z", 0.0);
  node_->declare_parameter<double>(WELDING_PARAM_NS + "goal_orientation_tolerance", 0.0);

  execute_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
      node_, "execute_task_solution");
  task_ = task;
}

void CartesianTask::init(std::string task_name, std::string task_caption)
{
  task_name_ = task_name;
  task_caption_ = task_caption;

  node_->get_parameter<std::string>(PLANNING_PIPELINES_NS + "planning_plugin", planning_plugin_);
  node_->get_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planner_id", planner_id_);
  node_->get_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planning_pipeline", planning_pipeline_);
  node_->get_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor",
                               max_acceleration_scaling_factor_);
  // Load parameter & initialize member variables
  node_->get_parameter<std::string>("arm_group_name", arm_group_name_);
  node_->get_parameter<std::string>("welding_tcp_frame", welding_tcp_frame_);
  node_->get_parameter<std::string>("task_control_frame", task_control_frame_);
  node_->get_parameter<std::string>("arm_home_pose", arm_home_pose_);
  node_->get_parameter<std::string>("workpiece_frame", workpiece_frame_);
  node_->get_parameter<double>("welding_velocity", welding_velocity_);      // [cm/min], see velocity_convert_
  node_->get_parameter<double>("cartesian_velocity", cartesian_velocity_);  // [m/s]
  node_->get_parameter<double>("via_velocity", via_velocity_);              // velocity scaling factor [0.0001;1]
  node_->get_parameter<double>("offset_welding_approach_z", offset_welding_approach_z_);
  node_->get_parameter<double>("goal_orientation_tolerance", goal_orientation_tolerance_);

  // loadParameters();
  // nh_.param<std::string>("/move_group/planning_plugin", planning_plugin_, "");
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

void CartesianTask::viaMotion(std::string planner_id, double velocity)
{
  Task& t = *task_;
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: viaMotion");
  // Create an instance of the planner and set its properties
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node_, planning_pipeline_);
  setPlannerProperties(pipeline_planner, planner_id, velocity);

  auto connect = std::make_unique<stages::Connect>(
      "free-space_motion", stages::Connect::GroupPlannerVector{ { arm_group_name_, pipeline_planner } });
  // connect->setTimeout(5.0);
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
  cartesian_interpolation->setMaxVelocityScaling(cartesian_velocity_);
  cartesian_interpolation->setMaxAccelerationScaling(max_acceleration_scaling_factor_);
  cartesian_interpolation->setStepSize(step_size_);
  auto stage = std::make_unique<stages::MoveRelative>(stage_caption, cartesian_interpolation);
  stage->properties().set("marker_ns", "stage_caption");
  stage->setIKFrame(task_control_frame_);
  stage->setGroup(welding_group_name_);
  // stage->setMinMaxDistance(offset_approach_z, offset_approach_z * 1.1);

  // Set hand forward direction
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = welding_tcp_frame_;
  vec.vector.z = offset_approach_z;
  stage->setDirection(vec);

  Task& t = *task_;
  t.add(std::move(stage));
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
  auto pipeline_planner = std::make_shared<solvers::PipelinePlanner>(node_, planning_pipeline_);
  setPlannerProperties(pipeline_planner, planner_id, velocity);

  // Create a stage with desired planner
  auto stage = std::make_unique<stages::MoveTo>(stage_caption, pipeline_planner);
  stage->setGroup(welding_group_name_);
  stage->setIKFrame(task_control_frame);
  stage->setGoal(goal_pose);

  if (planner_id != joint_space_planner_id_ && planning_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_constrained_);
  else if (planning_plugin_ == "ompl_interface/OMPLPlanner")
    stage->setTimeout(planning_time_free_space_);

  // Add the stage to the task
  Task& t = *task_;
  t.add(std::move(stage));
}

void CartesianTask::setPlannerProperties(moveit::task_constructor::solvers::PipelinePlannerPtr& pipeline_planner,
                                         std::string planner_id, double velocity)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "[CartesianTask instance]: setPlannerProperties");
  pipeline_planner->setProperty("step_size", step_size_);
  pipeline_planner->setProperty("goal_joint_tolerance", 1e-5);
  if (planning_plugin_ == "pilz_industrial_motion_planner::CommandPlanner")
  {
    pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_factor_);
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
    else if (velocity >= 0.0001)  // Pilz error: Velocity scaling not in range [0.0001, 1]
    {
      pipeline_planner->setProperty("max_velocity_scaling_factor", velocity);
      RCLCPP_INFO_STREAM(LOGGER, "[CartesianTask instance]: Setting desired Maximum Joint velocity scaling factor: "
                                     << velocity);
    }
    else
      RCLCPP_ERROR_STREAM(
          LOGGER, "[CartesianTask instance]: Desired velocity was not properly set, scaling not in range [0.0001, 1]");
  }
  else if (planning_plugin_ == "ompl_interface/OMPLPlanner")
  {
    if (planner_id == joint_space_planner_id_)
    {
      pipeline_planner->setPlannerId(free_space_planner_id_);
      pipeline_planner->setProperty("max_acceleration_scaling_factor", max_acceleration_scaling_factor_);
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
    RCLCPP_ERROR_STREAM(LOGGER, "Planner plugin name not correctly set: " << planning_plugin_);
}

void CartesianTask::weld(geometry_msgs::msg::PoseStamped start_pose, geometry_msgs::msg::PoseStamped goal_pose)
{
  init("welding_segment", "single_pass");
  viaMotion(joint_space_planner_id_, via_velocity_);
  approachRetreat("approach_start", task_control_frame_, offset_welding_approach_z_);
  generateStart("start_state ", start_pose, task_control_frame_);
  addStage("welding_motion ", goal_pose, task_control_frame_, linear_planner_id_, welding_velocity_);
  approachRetreat("retreat_end", task_control_frame_, -offset_welding_approach_z_);
}

}  // namespace processit_tasks
