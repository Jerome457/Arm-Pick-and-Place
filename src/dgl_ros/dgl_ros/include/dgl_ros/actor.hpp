// Copyright (c) 2023 Sebastian Peralta
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>

namespace dgl
{
/**
 * @brief This class is responsible for generating actions over a ROS2 action server.
 */
template <typename ActionT>
class Actor : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Actor object
   *
   * @param options
   * @param action_generator_func
   */
  Actor(const rclcpp::NodeOptions& options,
        std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func)
    : Node("actor", options), action_generator_func_(action_generator_func)
  {
    this->declare_parameter("action_topic", "sample_grasp_poses");
    RCLCPP_INFO(this->get_logger(), "Grasp detection action server ready");

    namespace sp = std::placeholders;
    action_server_ = rclcpp_action::create_server<ActionT>(this,  this->get_parameter("action_topic").as_string(), 
    std::bind(&Actor::handle_goal, this, sp::_1, sp::_2),
                                                    std::bind(&Actor::handle_cancel, this, sp::_1),
                                                    std::bind(&Actor::handle_accepted, this, sp::_1));
  }

private:
  typedef std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> GoalHandleSharedPtr;

  rclcpp_action::CancelResponse handle_cancel(const GoalHandleSharedPtr goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const GoalHandleSharedPtr& goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "New goal accepted");

    // Spawn a thread to execute the goal so it doesn’t block the executor
    std::thread{ [this, goal_handle]() {
        execute(goal_handle);
    }}.detach();
  }


void execute(const GoalHandleSharedPtr &goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing GPD grasp detection goal...");

  auto result = std::make_shared<typename ActionT::Result>();

  try
  {
    // Generate grasp poses (calls Gpd::actionFromObs internally)
    auto feedback = action_generator_func_();

    // Check for cancel request before sending feedback
    if (goal_handle->is_canceling())
    {
      RCLCPP_WARN(this->get_logger(), "Goal canceled before completion");
      goal_handle->canceled(result);
      return;
    }

    // Publish all detected grasps as feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Published %zu grasp candidates", feedback->grasp_candidates.size());

    // --- Select the best grasp (lowest cost) ---
    if (!feedback->grasp_candidates.empty() && feedback->grasp_candidates.size() == feedback->costs.size())
    {
      // Find index of grasp with minimum cost
      auto min_it = std::min_element(feedback->costs.begin(), feedback->costs.end());
      size_t best_idx = std::distance(feedback->costs.begin(), min_it);

      const auto &best_grasp = feedback->grasp_candidates[best_idx];
      double best_cost = feedback->costs[best_idx];

      // Assign best grasp pose to the result
      result->best_grasp_pose = best_grasp;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No valid grasps detected");
      result->best_grasp_pose.header.frame_id = "world";
    }

    // Optional small delay
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Mark goal as succeeded
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded — best grasp returned as result");
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Exception during grasp detection: %s", e.what());
    goal_handle->abort(result);
  }
}

  /**
   * @brief Called every time feedback is received for the goal
   * @param feedback - pointer to the feedback message
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const typename ActionT::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  std::function<typename ActionT::Feedback::SharedPtr()> action_generator_func_;
  typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
};
}  // namespace dgl