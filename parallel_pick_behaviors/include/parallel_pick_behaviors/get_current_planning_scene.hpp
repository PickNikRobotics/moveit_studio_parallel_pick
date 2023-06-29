// Copyright 2022 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <moveit_studio_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>

#include <moveit_msgs/srv/get_planning_scene.hpp>

namespace parallel_pick_behaviors
{
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

/**
 * @brief Get the current planning scene state from the MoveIt PlanningSceneMonitor by sending a /get_planning_scene service request
 *
 * @details
 * | Data Port Name       | Port Type  | Object Type                     |
 * | -------------------- | ---------- | ------------------------------- |
 * | planning_scene_msg   | output     | moveit_msgs::msg::PlanningScene |
 */
class GetCurrentPlanningScene final : public moveit_studio::behaviors::ServiceClientBehaviorBase<GetPlanningScene>
{
public:
  explicit GetCurrentPlanningScene(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  explicit GetCurrentPlanningScene(const std::string& name, const BT::NodeConfiguration& config,
                              const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources,
                              std::unique_ptr<ClientInterfaceBase> client_interface);

  static BT::PortsList providedPorts();

private:
  fp::Result<std::string> getServiceName() override;

  fp::Result<std::chrono::duration<double>> getResponseTimeout() override;

  /**
   * @brief Creates a service request message.
   * @return Returns an instance of GetPlanningScene::Request.
   */
  fp::Result<GetPlanningScene::Request> createRequest() override;

  /**
   * @brief Sets the planning scene message received in the service response to the output data port.
   * @param response Response message received from the service server.
   * @return Returns true.
   */
  fp::Result<bool> processResponse(const GetPlanningScene::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return future_;
  }

  /**
   * @brief Holds the result of calling the service asynchronously.
   */
  std::shared_future<fp::Result<bool>> future_;
};
}  // namespace parallel_pick_behaviors
