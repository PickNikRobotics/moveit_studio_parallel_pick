// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

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
class PushToSolutionQueue final : public BT::SyncActionNode
{
public:
  PushToSolutionQueue(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  /**
   * @brief Read queue from the input data port, push a new solution read from a second port to it and write it back to the blackboard
   * @return BT::NodeStatus::SUCCESS if successful.
   * @return BT::NodeStatus::FAILURE if a value could not be retrieved from an input data port.
   */
  BT::NodeStatus tick() override;

};
}  // namespace moveit_studio::behaviors
