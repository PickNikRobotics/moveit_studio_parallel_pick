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
constexpr auto kPortInterrupt = "fail_if_queue_empty";
constexpr auto kPortQueue = "solution_queue";
constexpr auto kPortSolution = "solution";
}  // namespace

namespace parallel_pick_behaviors
{
WaitAndPopSolutionQueue::WaitAndPopSolutionQueue(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config)
{
}

BT::PortsList WaitAndPopSolutionQueue::providedPorts()
{
  return { BT::BidirectionalPort<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue),
           BT::OutputPort<moveit_task_constructor_msgs::msg::Solution>(kPortSolution),
           BT::InputPort<bool>(kPortInterrupt),
            };
}

BT::NodeStatus WaitAndPopSolutionQueue::tick()
{
  auto interrupt_input = getInput<bool>(kPortInterrupt);

  // Get and validate required inputs
  auto solution_queue_input = getInput<std::queue<moveit_task_constructor_msgs::msg::Solution>>(kPortQueue);
  if (const auto error = moveit_studio::behaviors::maybe_error(solution_queue_input); error)
  {
    return BT::NodeStatus::RUNNING;
  }

  // If the queue is empty the Node will return running
  if (solution_queue_input.value().empty())
  {
    if (interrupt_input.has_value() && interrupt_input.value())
    {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  // Push the task solution in the queue to the output port.
  setOutput(kPortSolution, std::move(solution_queue_input.value().front()));
  solution_queue_input.value().pop();

  // Write updated queue back to the blackboard
  setOutput(kPortQueue, std::move(solution_queue_input.value()));
  return BT::NodeStatus::SUCCESS;
}

void WaitAndPopSolutionQueue::halt()
{
  return;
}
}  // namespace parallel_pick_behaviors
