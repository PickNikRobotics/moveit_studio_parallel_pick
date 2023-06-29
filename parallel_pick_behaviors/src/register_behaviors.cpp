#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <parallel_pick_behaviors/get_current_planning_scene.hpp>
#include <parallel_pick_behaviors/setup_mtc_fixed_joint_state.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace parallel_pick_behaviors
{
  class ParallelPickBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
  {
  public:
    void registerBehaviors(BT::BehaviorTreeFactory &factory,
                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources) override
    {
      using namespace moveit_studio::behaviors;

      registerBehavior<GetCurrentPlanningScene>(factory, "GetCurrentPlanningScene", shared_resources);
      registerBehavior<SetupMTCFixedJointState>(factory, "SetupMTCFixedJointState");
    }
  };
} // namespace setup_mtc_pick_from_pose

PLUGINLIB_EXPORT_CLASS(parallel_pick_behaviors::ParallelPickBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
