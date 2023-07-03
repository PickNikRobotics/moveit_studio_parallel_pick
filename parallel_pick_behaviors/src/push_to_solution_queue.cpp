// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <behaviortree_cpp/action_node.h>
#include <parallel_pick_behaviors/push_to_solution_queue.hpp>

#include <behaviortree_cpp/basic_types.h>

#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include <queue>

namespace
{
constexpr auto kPortQueue = "solution_queue";
constexpr auto kPortSolution = "solution";
}  // namespace

namespace parallel_pick_behaviors
{
PushToSolutionQueue::PushToSolutionQueue(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList PushToSolutionQueue::providedPorts()
{
  return { BT::BidirectionalPort<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue),
           BT::InputPort<moveit_task_constructor_msgs::msg::Solution>(kPortSolution) };
}

BT::NodeStatus PushToSolutionQueue::tick()
{
  // Get and validate required inputs
  const auto solution_input = getInput<moveit_task_constructor_msgs::msg::Solution>(kPortSolution);
  auto solution_queue_input = getInput<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue);

  if (const auto error = moveit_studio::behaviors::maybe_error(solution_queue_input, solution_queue_input); error)
  {
    return BT::NodeStatus::FAILURE;
  }

  solution_queue_input.value().push(solution_input.value());

  // Write updated queue back to the blackboard
  setOutput(kPortQueue, std::move(solution_queue_input.value()));
  return BT::NodeStatus::SUCCESS;
}
}  // namespace parallel_pick_behaviors
