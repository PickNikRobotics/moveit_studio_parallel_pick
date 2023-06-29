// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/basic_types.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include <parallel_pick_behaviors/setup_mtc_fixed_joint_state.hpp>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{
constexpr auto kPortIDPlanningSceneMsg = "planning_scene_msg";
constexpr auto kPortIDJointState = "joint_state";
constexpr auto kPortIDTask = "task";

// NOTE: this is just set to 'current state' to ensure compatibility with other MTC-based behaviors that expect the first stage to be named this
constexpr auto kStageNameCurrentState = "current state";

}  // namespace

namespace parallel_pick_behaviors
{
SetupMTCFixedJointState::SetupMTCFixedJointState(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList SetupMTCFixedJointState::providedPorts()
{
  return {
    BT::InputPort<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMsg),
    BT::InputPort<sensor_msgs::msg::JointState>(kPortIDJointState),
    BT::BidirectionalPort<moveit::task_constructor::TaskPtr>(kPortIDTask),
  };
}

BT::NodeStatus SetupMTCFixedJointState::tick()
{
  const auto planning_scene_msg = getInput<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMsg);
  const auto joint_state = getInput<sensor_msgs::msg::JointState>(kPortIDJointState);
  const auto task = getInput<moveit::task_constructor::TaskPtr>(kPortIDTask);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(planning_scene_msg, joint_state, task); error)
  {
    return BT::NodeStatus::FAILURE;
  }

  const auto robot_model = task.value()->getRobotModel();

  const auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
	if (!scene->setPlanningSceneMsg(planning_scene_msg.value()))
  {
    return BT::NodeStatus::FAILURE;
  }

  try
  {
    moveit::core::RobotState current_state{robot_model};
    current_state.setToDefaultValues();
    current_state.setVariablePositions(joint_state.value().name, joint_state.value().position);
    scene->setCurrentState(current_state);
  }
  catch(const std::exception& e)
  {
    return BT::NodeStatus::FAILURE;
  }

  auto fixed_state = std::make_unique<moveit::task_constructor::stages::FixedState>(kStageNameCurrentState);
  fixed_state->setState(scene);
  task.value()->add(std::move(fixed_state));

  return BT::NodeStatus::SUCCESS;
}
}  // parallel_pick_behaviors
