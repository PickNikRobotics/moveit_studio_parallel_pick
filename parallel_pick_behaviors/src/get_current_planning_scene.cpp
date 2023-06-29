// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <moveit_msgs/msg/detail/planning_scene__struct.hpp>
#include <parallel_pick_behaviors/get_current_planning_scene.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace
{
constexpr std::chrono::seconds kServiceResponseTimeoutSeconds{ 3 };

constexpr auto kGetPlanningSceneServiceName = "/get_planning_scene";
constexpr auto kPortIDPlanningSceneMsg = "planning_scene_msg";
}  // namespace

namespace parallel_pick_behaviors
{
GetCurrentPlanningScene::GetCurrentPlanningScene(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<GetPlanningScene>(name, config, shared_resources)
{
}

GetCurrentPlanningScene::GetCurrentPlanningScene(const std::string& name, const BT::NodeConfiguration& config,
                                       const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
                                       std::unique_ptr<ClientInterfaceBase> client_interface)
  : ServiceClientBehaviorBase<GetPlanningScene>(name, config, shared_resources, std::move(client_interface))
{
}

BT::PortsList GetCurrentPlanningScene::providedPorts()
{
  return { BT::OutputPort<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMsg) };
}

fp::Result<std::string> GetCurrentPlanningScene::getServiceName()
{
  return kGetPlanningSceneServiceName;
}

fp::Result<std::chrono::duration<double>> GetCurrentPlanningScene::getResponseTimeout()
{
  return kServiceResponseTimeoutSeconds;
}

fp::Result<GetPlanningScene::Request> GetCurrentPlanningScene::createRequest()
{
  // Create request message.
  moveit_msgs::srv::GetPlanningScene::Request request{};

  request.components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
                                  moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                  moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
                                  moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                  moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                  moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
                                  moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
                                  moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                  moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
                                  moveit_msgs::msg::PlanningSceneComponents::OBJECT_COLORS;

  return request;
}

fp::Result<bool> GetCurrentPlanningScene::processResponse(const GetPlanningScene::Response& response)
{
  setOutput<moveit_msgs::msg::PlanningScene>(kPortIDPlanningSceneMsg, response.scene);
  return true;
}
}  // namespace moveit_studio::behaviors

template class moveit_studio::behaviors::ServiceClientBehaviorBase<moveit_msgs::srv::GetPlanningScene>;
