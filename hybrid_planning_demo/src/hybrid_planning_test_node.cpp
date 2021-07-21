/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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

/* Author: Sebastian Jahr
 */

#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/action/hybrid_planning.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

using namespace std::chrono_literals;
const rclcpp::Logger LOGGER = rclcpp::get_logger("ipa_hybrid_planning_demo");

class HybridPlanningDemo
{
public:
  HybridPlanningDemo(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;
    hp_action_client_ = rclcpp_action::create_client<moveit_msgs::action::HybridPlanning>(node_, "run_hybrid_planning"),
    robot_state_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);

    // collision_object_1_.header.frame_id = "panda_link0";
    // collision_object_1_.id = "box1";

    // collision_object_2_.header.frame_id = "panda_link0";
    // collision_object_2_.id = "box2";

    // collision_object_3_.header.frame_id = "panda_link0";
    // collision_object_3_.id = "box3";

    // box_1_.type = box_1_.BOX;
    // box_1_.dimensions = { 0.5, 0.8, 0.01 };

    // box_2_.type = box_2_.BOX;
    // box_2_.dimensions = { 1.0, 0.4, 0.01 };

    // // Add new collision object as soon as global trajectory is available.
    // global_solution_subscriber_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
    //     "global_trajectory", 1, [this](const moveit_msgs::msg::MotionPlanResponse::SharedPtr msg) {
    //       // Remove old collision objects
    //       collision_object_1_.operation = collision_object_1_.REMOVE;

    //       // Add new collision objects
    //       geometry_msgs::msg::Pose box_pose_2;
    //       box_pose_2.position.x = 0.2;
    //       box_pose_2.position.y = 0.4;
    //       box_pose_2.position.z = 0.95;

    //       collision_object_2_.primitives.push_back(box_2_);
    //       collision_object_2_.primitive_poses.push_back(box_pose_2);
    //       collision_object_2_.operation = collision_object_2_.ADD;

    //       geometry_msgs::msg::Pose box_pose_3;
    //       box_pose_3.position.x = 0.2;
    //       box_pose_3.position.y = -0.4;
    //       box_pose_3.position.z = 0.95;

    //       collision_object_3_.primitives.push_back(box_2_);
    //       collision_object_3_.primitive_poses.push_back(box_pose_3);
    //       collision_object_3_.operation = collision_object_3_.ADD;

    //       // Add object to planning scene
    //       {  // Lock PlanningScene
    //         planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    //         scene->processCollisionObjectMsg(collision_object_2_);
    //         scene->processCollisionObjectMsg(collision_object_3_);
    //         scene->processCollisionObjectMsg(collision_object_1_);
    //       }  // Unlock PlanningScene
    //     });
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize Planning Scene Monitor");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
    //     node_, "robot_description", tf_buffer_, "planning_scene_monitor");
    // if (planning_scene_monitor_->getPlanningScene() != nullptr)
    // {
    //   planning_scene_monitor_->startStateMonitor();
    //   planning_scene_monitor_->providePlanningSceneService();  // let RViz display query PlanningScene
    //   planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
    //   planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
    //                                                         "/planning_scene");
    //   planning_scene_monitor_->startSceneMonitor();
    // }

    // if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5))
    // {
    //   RCLCPP_ERROR(LOGGER, "Timeout when waiting for /joint_states updates. Is the robot running?");
    //   return;
    // }

    if (!hp_action_client_->wait_for_action_server(20s))
    {
      RCLCPP_ERROR(LOGGER, "Hybrid planning action server not available after waiting");
      return;
    }

    // geometry_msgs::msg::Pose box_pose;
    // box_pose.position.x = 0.4;
    // box_pose.position.y = 0.0;
    // box_pose.position.z = 0.85;

    // collision_object_1_.primitives.push_back(box_1_);
    // collision_object_1_.primitive_poses.push_back(box_pose);
    // collision_object_1_.operation = collision_object_1_.ADD;

    // // Add object to planning scene
    // {  // Lock PlanningScene
    //   planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
    //   scene->processCollisionObjectMsg(collision_object_1_);
    // }  // Unlock PlanningScene

    RCLCPP_INFO(LOGGER, "Wait 2s see collision object");
    rclcpp::sleep_for(2s);

    // Setup motion planning goal taken from motion_planning_api tutorial
    const std::string planning_group = "ur_manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

    // Create a RobotState and JointModelGroup
    const auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

    // Configure a valid robot state
    robot_state->setToDefaultValues(joint_model_group, "test_configuration");

    auto goal_msg = moveit_msgs::action::HybridPlanning::Goal();

    moveit::core::robotStateToRobotStateMsg(*robot_state, goal_msg.request.start_state);
    goal_msg.request.group_name = planning_group;
    goal_msg.request.num_planning_attempts = 10;
    goal_msg.request.max_velocity_scaling_factor = 0.1;
    goal_msg.request.max_acceleration_scaling_factor = 0.1;
    goal_msg.request.allowed_planning_time = 2.0;
    goal_msg.request.planner_id = "RRTConnectkConfigDefault";

    moveit::core::RobotState goal_state(robot_model);
    std::vector<double> joint_values = { -0.19, -0.29, -1.29, -1.57, -4.53, -4.71 };
    goal_state.setJointGroupPositions(joint_model_group, joint_values);

    goal_msg.request.goal_constraints.resize(1);
    goal_msg.request.goal_constraints[0] =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::HybridPlanning>::SendGoalOptions();
    send_goal_options.result_callback =
        [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanning>::WrappedResult& result) {
          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(LOGGER, "Hybrid planning goal succeded");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
              return;
            default:
              RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
              return;
              RCLCPP_INFO(LOGGER, "Hybrid planning result received");
          }
        };
    send_goal_options.feedback_callback =
        [](rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanning>::SharedPtr /*unused*/,
           const std::shared_ptr<const moveit_msgs::action::HybridPlanning::Feedback> feedback) {
          RCLCPP_INFO(LOGGER, feedback->feedback);
        };

    RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = hp_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::HybridPlanning>::SharedPtr hp_action_client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_subscriber_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit_msgs::msg::CollisionObject collision_object_1_;
  moveit_msgs::msg::CollisionObject collision_object_2_;
  moveit_msgs::msg::CollisionObject collision_object_3_;
  shape_msgs::msg::SolidPrimitive box_1_;
  shape_msgs::msg::SolidPrimitive box_2_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("hybrid_planning_test_node", "", node_options);

  HybridPlanningDemo demo(node);
  std::thread run_demo([&demo]() {
    rclcpp::sleep_for(5s);
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
