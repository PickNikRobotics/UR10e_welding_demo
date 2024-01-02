// MTC Welding IPA
#include <processit_tasks/welding.h>

namespace processit_tasks
{
using namespace moveit::task_constructor;
const rclcpp::Logger LOGGER = rclcpp::get_logger("welding");

constexpr char LOGNAME[] = "welding";
Welding::Welding(const int task_id, const int segment_id, const rclcpp::Node::SharedPtr& node)
  : task_id_(task_id), segment_id_(segment_id), execute_("execute_task_solution", true), cartesian_task_(node)
{
  node_ = node;
  welding_src_client_ = node_->create_client<processit_msgs::SetDigitalOut>("/io_controller/set_io");
  loadParameters();
  loadWeldingLayout();
  nh_.param<std::string>("/move_group/planning_plugin", planner_plugin_, "");
}

void Welding::loadParameters()
{
  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
  ros::NodeHandle pnh("~");

  // planner_interface configuration
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(node_, "max_acceleration_scaling", max_acceleration_scaling_);
  errors += !rosparam_shortcuts::get(node_, "via_max_joint_velocity", via_max_joint_velocity_);
  errors += !rosparam_shortcuts::get(node_, "away_max_joint_velocity", away_max_joint_velocity_);
  errors += !rosparam_shortcuts::get(node_, "cartesian_velocity", cartesian_velocity_);
  errors += !rosparam_shortcuts::get(node_, "via_velocity", via_velocity_);
  errors += !rosparam_shortcuts::get(node_, "welding_velocity", welding_velocity_);
  errors += !rosparam_shortcuts::get(node_, "linear_planner_id", linear_planner_id_);
  errors += !rosparam_shortcuts::get(node_, "circular_planner_id", circular_planner_id_);
  errors += !rosparam_shortcuts::get(node_, "curve_planner_id", curve_planner_id_);
  errors += !rosparam_shortcuts::get(node_, "joint_space_planner_id", joint_space_planner_id_);

  // Offsets for specific sub-tasks
  //// Welding
  errors += !rosparam_shortcuts::get(node_, "offset_welding_approach_z", offset_welding_approach_z_);
  errors += !rosparam_shortcuts::get(node_, "offset_welding_z", offset_welding_z_);
  errors += !rosparam_shortcuts::get(node_, "offset_welding_y", offset_welding_y_);
  errors += !rosparam_shortcuts::get(node_, "offset_welding_x", offset_welding_x_);
  errors += !rosparam_shortcuts::get(node_, "offset_welding_angle", offset_welding_angle_);
  errors += !rosparam_shortcuts::get(node_, "offset_welding_speed", offset_welding_speed_);

  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void Welding::loadWeldingLayout()
{
  std::string welding_layout_path = ros::package::getPath("processit_tasks") + "/config/multi_layer_layout_ipa.yaml";
  multi_layer_layout_ = YAML::LoadFile(welding_layout_path);
  pass_count_ = multi_layer_layout_.size();
}

void Welding::setPass(int pass_id)
{
  pass_id_ = pass_id;
  if (pass_id_ == -1)
  {
    task_velocity_ = velocity_convert_ * welding_velocity_;
    ROS_INFO_NAMED(LOGNAME, "Welding single pass with parameters from Technology Model");
  }
  else
  {
    // Get data from YAML file
    double layout_offset_x = multi_layer_layout_[pass_id_]["pos_x"].as<double>();
    double layout_offset_z = multi_layer_layout_[pass_id_]["pos_z"].as<double>();
    double welding_angle = multi_layer_layout_[pass_id_]["welding_angle"].as<double>();
    double welding_speed = multi_layer_layout_[pass_id_]["welding_speed"].as<double>();
    std::cout << "Pass ID: " << pass_id_ << std::endl;
    std::cout << "Offset X: " << layout_offset_x << std::endl;
    std::cout << "Offset Z: " << layout_offset_z << std::endl;
    std::cout << "Welding angle: " << welding_angle << std::endl;
    std::cout << "Welding speed: " << welding_speed << std::endl;

    // Convert and apply translational offsets
    task_transform_.translation.x = distance_convert_ * (layout_offset_x + offset_welding_x_);
    task_transform_.translation.y = distance_convert_ * offset_welding_y_;
    task_transform_.translation.z = distance_convert_ * (layout_offset_z + offset_welding_z_);

    // Convert and apply rotational offsets
    double roll_angle = angle_convert_ * (welding_angle + offset_welding_angle_);
    double pitch_angle = angle_convert_ * 0;
    double yaw_angle = angle_convert_ * 0;
    tf2::Quaternion task_rotation_tf;
    task_rotation_tf.setEuler(roll_angle, pitch_angle, yaw_angle);
    task_rotation_tf.normalize();
    tf2::convert(task_rotation_tf, task_transform_.rotation);

    // Convert and apply velocity
    task_velocity_ = velocity_convert_ * (welding_speed + offset_welding_speed_);
  }
}

void Welding::resetTask()
{
  if (pass_id_ == -1)
  {
    cartesian_task_.init(task_name_ + "_single_pass", task_caption_);
  }
  else
  {
    cartesian_task_.init(task_name_, task_caption_);
  }
}

void Welding::approach()
{
  // Task initialization
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
  task_caption_ = "Approaching ";
  task_name_ = "task_" + std::to_string(task_id_) + "_segment_" + std::to_string(segment_id_);
  resetTask();

  // Stage: Approach start of welding seam (distance from seam: offset_welding_approach_z_)
  if (pass_id_ == -1)
  {
    geometry_msgs::msg::Transform stage_offset;
    stage_offset.translation.z = -distance_convert_ * offset_welding_approach_z_;
    // Hack because robot configuration change with Pilz PTP works bad
    if (planner_plugin_ == "pilz_industrial_motion_planner/CommandPlanner")
      cartesian_task_.addStage("Approach path", task_name_ + "_start", "ManufacturingToTaskFrame", stage_offset,
                               linear_planner_id_, via_velocity_);
    else
      cartesian_task_.addStage("Approach path", task_name_ + "_start", "ManufacturingToTaskFrame", stage_offset,
                               joint_space_planner_id_, via_max_joint_velocity_);
  }
  else
  {
    geometry_msgs::msg::Transform task_transform = task_transform_;
    task_transform.translation.z -= distance_convert_ * offset_welding_approach_z_;
    cartesian_task_.setTaskTransformOffset(task_transform);
    // Hack because robot configuration change with Pilz PTP works bad
    if (planner_plugin_ == "pilz_industrial_motion_planner/CommandPlanner")
      cartesian_task_.addStage("Approach path", task_name_ + "_start", linear_planner_id_, via_velocity_);
    else
      cartesian_task_.addStage("Approach path", task_name_ + "_start", joint_space_planner_id_, via_max_joint_velocity_);
  }

  // Stage: Reach start of welding seam
  cartesian_task_.setTaskTransformOffset(task_transform_);
  if (pass_id_ == -1)
  {
    geometry_msgs::msg::Transform stage_offset;
    cartesian_task_.addStage("Start state root seam", task_name_ + "_start", "ManufacturingToTaskFrame", stage_offset,
                             joint_space_planner_id_, away_max_joint_velocity_);
  }
  else
  {
    cartesian_task_.addStage("Start state of seam", task_name_ + "_start", joint_space_planner_id_,
                             away_max_joint_velocity_);
  }

  // Planning and execution of the task
  if (plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
    std::cout << "Press Enter to approach pass." << std::endl;
    std::cin.get();
    // if (nh_.param("execute", false))
    {
      bool approach_success = execute();
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    }
    // else
    // {
    // 	ROS_INFO_NAMED(LOGNAME, "Execution disabled");
    // }
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }
}

void Welding::weld()
{
  // Task initialization
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
  task_caption_ = "Weld pass " + std::to_string(pass_id_) + " of ";
  task_name_ = "task_" + std::to_string(task_id_) + "_segment_" + std::to_string(segment_id_);
  resetTask();

  // Stage: Weld until the end of the welding seam
  cartesian_task_.setTaskTransformOffset(task_transform_);
  if (pass_id_ == -1)
  {
    geometry_msgs::msg::Transform stage_offset;
    if (segment_id_ == 1)  // HACK to plan circular path for segment 1. Should be replaced by reading task_description
                           // or reading the TF tree for _circ
    {
      cartesian_task_.addStage("Welding root seam ", task_name_ + "_end", "ManufacturingToTaskFrame", stage_offset,
                               circular_planner_id_, task_velocity_);
    }
    else
    {
      cartesian_task_.addStage("Welding root seam ", task_name_ + "_end", "ManufacturingToTaskFrame", stage_offset,
                               linear_planner_id_, task_velocity_);
    }
  }
  else
  {
    if (segment_id_ == 1)  // HACK to plan circular path for segment 1. Should be replaced by reading task_description
                           // or reading the TF tree for _circ
    {
      cartesian_task_.addStage("Welding seam ", task_name_ + "_end", circular_planner_id_, task_velocity_);
    }
    else
    {
      cartesian_task_.addStage("Welding seam ", task_name_ + "_end", linear_planner_id_, task_velocity_);
    }
  }

  // Planning and execution of the task
  if (plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
    processit_msgs::SetDigitalOut srv;
    srv.request.pin = 1;
    std::cout << "Enter 'WELD' to weld pass with welding source. Otherwise, enter anything else." << std::endl;
    std::string operation_mode;
    std::cin >> operation_mode;

    // if (nh_.param("execute", false))
    {
      if (operation_mode == "WELD")
      {
        ROS_WARN_NAMED(LOGNAME, "Welding source will activate. Stay away and keep doors shut.");
        srv.request.state = true;
        bool io_success = welding_src_client_.call(srv);
        ROS_INFO_NAMED(LOGNAME, "Started welding");
      }

      bool weld_success = execute();
      ROS_INFO_NAMED(LOGNAME, "Execution complete");

      if (operation_mode == "WELD")
      {
        srv.request.state = false;
        bool io_success = welding_src_client_.call(srv);
        ROS_INFO_NAMED(LOGNAME, "Welding stopped");
      }
    }
    // else
    // {
    // 	ROS_INFO_NAMED(LOGNAME, "Execution disabled");
    // }
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }
}

void Welding::retrieve()
{
  // Task initialization
  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
  task_caption_ = "Retrieve ";
  task_name_ = "task_" + std::to_string(task_id_) + "_segment_" + std::to_string(segment_id_);
  resetTask();

  // Stage: Retrieve from welding seam with distance offset_welding_approach_z_
  if (pass_id_ == -1)
  {
    geometry_msgs::msg::Transform stage_offset;
    stage_offset.translation.z = -distance_convert_ * offset_welding_approach_z_;
    cartesian_task_.addStage("Retrieve path", task_name_ + "_end", "ManufacturingToTaskFrame", stage_offset,
                             joint_space_planner_id_, away_max_joint_velocity_);
  }
  else
  {
    geometry_msgs::msg::Transform task_transform = task_transform_;
    task_transform.translation.z -= distance_convert_ * offset_welding_approach_z_;
    cartesian_task_.setTaskTransformOffset(task_transform);
    cartesian_task_.addStage("Retrieve path", task_name_ + "_end", joint_space_planner_id_, away_max_joint_velocity_);
  }

  // Planning and execution of the task
  if (plan())
  {
    ROS_INFO_NAMED(LOGNAME, "Planning succeeded");
    std::cout << "Press Enter to retrieve from pass." << std::endl;
    std::cin.get();
    std::cin.get();
    // if (nh_.param("execute", false))
    {
      bool retrieval_success = execute();
      ROS_INFO_NAMED(LOGNAME, "Execution complete");
    }
    // else
    // {
    // 	ROS_INFO_NAMED(LOGNAME, "Execution disabled");
    // }
  }
  else
  {
    ROS_INFO_NAMED(LOGNAME, "Planning failed");
  }
}

bool Welding::plan()
{
  return cartesian_task_.plan();
}

bool Welding::execute()
{
  return cartesian_task_.execute();
}

void Welding::weldFillet(int task_id)
{
  task_id_ = task_id;
  segment_id_ = 0;
  pass_id_ = -1;  // Welding with Technology Model Trafo
  setPass(pass_id_);
  approach();
  weld();
  // Retrieve only required if no collision-free planner
  if (planner_plugin_ == "pilz_industrial_motion_planner/CommandPlanner")
    retrieve();
}

void Welding::weldAllFillets(int last_task_id)
{
  for (int i = 0; i < last_task_id; i++)
  {
    weldFillet(i);
  }
}

void Welding::weldSinglePass(int pass_id)
{
  setPass(pass_id);
  approach();
  weld();
  retrieve();
}

void Welding::weldAllPasses()
{
  for (int i = 0; i < multi_layer_layout_.size(); i++)
  {
    weldSinglePass(i);
  }
}

}  // namespace processit_tasks
