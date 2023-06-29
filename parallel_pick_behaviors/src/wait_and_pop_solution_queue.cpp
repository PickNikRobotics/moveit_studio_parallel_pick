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
WaitAndPopSolutionQueue::WaitAndPopSolutionQueue(const std::string& name, const BT::NodeConfiguration& config,
                                                 const std::shared_ptr<BehaviorContext>& shared_resources)
  : AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList WaitAndPopSolutionQueue::providedPorts()
{
  return { BT::BidirectionalPort<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue),
           BT::OutputPort<moveit_task_constructor_msgs::msg::Solution>(kPortSolution) };
}

fp::Result<bool> WaitAndPopSolutionQueue::doWork()
{
  // Get and validate required inputs
  auto solution_queue_input = getInput<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue);
  if (const auto error = maybe_error(solution_queue_input); error)
  {
    return tl::make_unexpected(fp::Internal("Failed to get required values from input data ports: " + error.value()));
  }

  // If the queue is empty the Node will return running
  if (solution_queue_input.value().empty())
  {
    return false;
  }

  // Push the task solution in the queue to the output port.
  setOutput(kPortSolution, std::move(solution_queue_input.value().front()));
  solution_queue_input.value().pop();

  // Write updated queue back to the blackboard
  setOutput(kPortQueue, std::move(solution_queue_input.value()));
  return true;
}

fp::Result<void> WaitAndPopSolutionQueue::doHalt()
{
  return {};
}
}  // namespace moveit_studio::behaviors
