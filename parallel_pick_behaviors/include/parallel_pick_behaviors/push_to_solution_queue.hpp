// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_studio_behavior_interface/async_behavior_base.hpp>

namespace moveit_studio::behaviors
{
/**
 * @brief Push a new solution to the solution queue
 *
 * @details
 * | Data Port Name   | Port Type        | Object Type                                             |
 * |------------------|------------------|---------------------------------------------------------|
 * | solution         | Input            | moveit_task_constructor_msgs::msg::Solution             |
 * | solution queue   | Input/Output     | std::queue<moveit_task_constructor_msgs::msg::Solution> |
 */
class PushToSolutionQueue final : public AsyncBehaviorBase
{
public:
  PushToSolutionQueue(const std::string& name, const BT::NodeConfiguration& config,
                      const std::shared_ptr<BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

private:
  /**
   * @brief Read queue from the blackboard, push a new solution read from a second port to it and write it back to the blackboard
   */
  fp::Result<bool> doWork() override;

  /**
   * @brief Halts an in-progress planning process by preempting the stored MTC task.
   * @return Always returns void, since preempting an MTC task always succeeds.
   */
  fp::Result<void> doHalt() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<fp::Result<bool>> future_;
};
}  // namespace moveit_studio::behaviors
