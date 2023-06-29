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
 * @brief Pops the solution queue to get the next solution to be processed. If the queue is empty it will return RUNNING
 *
 * @details
 * | Data Port Name   | Port Type | Object Type                                             |
 * |------------------|-----------|---------------------------------------------------------|
 * | solution queue   | Input     | std::queue<moveit_task_constructor_msgs::msg::Solution> |
 * | solution         | Output    | moveit_task_constructor_msgs::msg::Solution             |
 */
class WaitAndPopSolutionQueue final : public AsyncBehaviorBase
{
public:
  WaitAndPopSolutionQueue(const std::string& name, const BT::NodeConfiguration& config,
                          const std::shared_ptr<BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

private:
  /**
   * @brief Read queue from the blackboard, check whether or not it is empty and pop the next solution if it is not
   * empty. The solution will be written to the BT output port.
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
