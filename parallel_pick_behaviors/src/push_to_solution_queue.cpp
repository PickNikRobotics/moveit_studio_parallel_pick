// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <parallel_pick_behaviors/wait_and_pop_solution_queue.hpp>

#include <behaviortree_cpp/basic_types.h>

#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include <queue>

namespace
{
constexpr auto kPortQueue = "solution_queue";
constexpr auto kPortSolution = "solution";
}  // namespace

namespace moveit_studio::behaviors
{
PushToSolutionQueue::PushToSolutionQueue(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList PushToSolutionQueue::providedPorts()
{
  return { BT::BidirectionalPort<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue),
           BT::InputPort<moveit_task_constructor_msgs::msg::Solution>(kPortSolution) };
}

fp::Result<bool> PushToSolutionQueue::doWork()
{
  // Get and validate required inputs
  auto solution_queue_input = getInput<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue);
  if (const auto error = maybe_error(solution_queue_input); error)
  {
    return tl::make_unexpected(fp::Internal("Failed to get required values from input data ports: " + error.value()));
  }

  auto const solution_input = getInput<moveit_task_constructor_msgs::msg::Solution>(kPortSolution);
  if (const auto error = maybe_error(solution_input); error)
  {
    return tl::make_unexpected(fp::Internal("Failed to get required values from input data ports: " + error.value()));
  }

  solution_queue_input.value().push(solution_input.value());

  // Write updated queue back to the blackboard
  setOutput(kPortQueue, std::move(solution_queue_input.value()));
  return true;
}

fp::Result<void> PushToSolutionQueue::doHalt()
{
  return {};
}
}  // namespace moveit_studio::behaviors
