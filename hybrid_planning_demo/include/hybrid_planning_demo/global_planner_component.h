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

/* Author: Henning Kayser */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>

namespace hybrid_planning_demo
{
// Component node containing the global planner
class GlobalPlannerComponent : public rclcpp::Node
{
public:
  GlobalPlannerComponent(const rclcpp::NodeOptions& options) : rclcpp::Node("global_planner_component", options)
  {
    // Initialize global planning request action server
    global_planning_request_server_ = rclcpp_action::create_server<moveit_msgs::action::GlobalPlanner>(
        this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(),
        this->get_node_waitables_interface(), "global_planning_action",
        [](const rclcpp_action::GoalUUID& /*unused*/,
           std::shared_ptr<const moveit_msgs::action::GlobalPlanner::Goal> /*unused*/) {
          RCLCPP_INFO(rclcpp::get_logger("global_planner_component"), "Received global planning goal request");
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& /*unused*/) {
          RCLCPP_INFO(rclcpp::get_logger("global_planner_component"),
                      "Received request to cancel global planning goal");
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>>& goal_handle) {
          // TODO(sjahr): Add feedback
          const auto goal = goal_handle->get_goal();

          // Plan global trajectory
          moveit_msgs::msg::MotionPlanResponse planning_solution = plan(goal->request);

          // Publish global planning solution to the local planner
          if (planning_solution.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            global_trajectory_pub_->publish(planning_solution);

          // Send action response
          auto result = std::make_shared<moveit_msgs::action::GlobalPlanner::Result>();
          result->response = planning_solution;
          goal_handle->succeed(result);

          // Save newest planning solution
          last_global_solution_ = planning_solution;  // TODO(sjahr) Add Service to expose this
        });

    global_trajectory_pub_ = this->create_publisher<moveit_msgs::msg::MotionPlanResponse>("global_trajectory", 1);

    // Initialize global planner after construction
    // TODO(sjahr) Remove once life cycle component nodes are available
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(1ms, [this]() {
      if (initialized_)
      {
        timer_->cancel();
      }
      else
      {
        initialized_ = this->init();
        if (!initialized_)
        {
          const std::string error = "Failed to initialize global planner";
          timer_->cancel();
          throw std::runtime_error(error);
        }
      }
    });
  }

private:
  // Initialize planning scene monitor and load pipelines
  virtual bool init() = 0;

  // Plan global trajectory
  virtual moveit_msgs::msg::MotionPlanResponse plan(const moveit_msgs::msg::MotionPlanRequest& planning_problem) = 0;

  // Global solution
  moveit_msgs::msg::MotionPlanResponse last_global_solution_;

  // Global planning request action server
  rclcpp_action::Server<moveit_msgs::action::GlobalPlanner>::SharedPtr global_planning_request_server_;

  // Global trajectory publisher
  rclcpp::Publisher<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_trajectory_pub_;

  // Delayed initialization
  rclcpp::TimerBase::SharedPtr timer_;
  bool initialized_{ false };
};
}  // namespace hybrid_planning_demo
