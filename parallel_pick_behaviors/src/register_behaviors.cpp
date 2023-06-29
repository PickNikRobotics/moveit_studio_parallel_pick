#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace setup_mtc_pick_from_pose
{
  class ParallelPickBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
  {
  public:
    void registerBehaviors(BT::BehaviorTreeFactory &factory,
                           const std::shared_ptr<moveit_studio::behaviors::BehaviorContext> &shared_resources) override
    {
      using namespace moveit_studio::behaviors;

      // TODO: register custom behaviors
      // registerBehavior<SetupMtcPickFromPose>(factory, "SetupMtcPickFromPose", shared_resources);
    }
  };
} // namespace setup_mtc_pick_from_pose

PLUGINLIB_EXPORT_CLASS(setup_mtc_pick_from_pose::ParallelPickBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
