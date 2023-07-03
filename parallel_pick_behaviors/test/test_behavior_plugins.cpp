#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>

/**
 * @brief This test makes sure that the Behaviors provided in this package can be successfully registered and
 * instantiated by the behavior tree factory.
 */
TEST(ParallelPickBehaviors, test_load_behavior_plugins)
{
  pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("parallel_pick_behaviors::ParallelPickBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }

  // Test that ClassLoader is able to find and instantiate each behavior using the package's plugin description info.
  factory.instantiateTreeNode("test_behavior_name", "PushToSolutionQueue",
                              BT::NodeConfiguration());
  factory.instantiateTreeNode("test_behavior_name", "SetupMTCFixedJointState",
                              BT::NodeConfiguration());
  factory.instantiateTreeNode("test_behavior_name", "GetCurrentPlanningScene",
                            BT::NodeConfiguration());
  factory.instantiateTreeNode("test_behavior_name", "WaitAndPopSolutionQueue",
                              BT::NodeConfiguration());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
