// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>

namespace parallel_pick_behaviors
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
class WaitAndPopSolutionQueue final : public BT::ActionNodeBase
{
public:
  WaitAndPopSolutionQueue(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  /**
   * @brief Read queue from the blackboard, check whether or not it is empty and pop the next solution if it is not
   * empty. The solution will be written to the BT output port.
   */
  BT::NodeStatus tick() override;

  void halt() override;
};
}  // namespace parallel_pick_behaviors
