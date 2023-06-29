// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

namespace parallel_pick_behaviors
{
/**
 * @brief Given an existing MTC Task object, appends an MTC FixedState Stage to the Task.
 *
 * @details
 * | Data Port Name       | Port Type     | Object Type                                     |
 * | -------------------- |---------------|-------------------------------------------------|
 * | planning_scene_msg   | input         | moveit_msgs::msg::PlanningScene                 |
 * | joint_state_msg      | input         | sensor_msgs::msg::JointState                    |
 * | task                 | Bidirectional | std::shared_ptr<moveit::task_constructor::Task> |
 */
class SetupMTCFixedJointState final : public BT::SyncActionNode
{
public:
  SetupMTCFixedJointState(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
}  // namespace parallel_pick_behaviors
