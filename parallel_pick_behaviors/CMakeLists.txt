cmake_minimum_required(VERSION 3.16.3)
project(parallel_pick_behaviors)

# Set MoveIt's typical CMake settings, including C++ standard version
find_package(moveit_common REQUIRED)
moveit_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_studio_behavior_interface
  moveit_task_constructor_core
  pluginlib
  sensor_msgs
  tf2_eigen
)

# AprilTag Behavior
add_library(parallel_pick_behaviors SHARED
  src/get_current_planning_scene.cpp
  src/push_to_solution_queue.cpp
  src/register_behaviors.cpp
  src/setup_mtc_fixed_joint_state.cpp
  src/wait_and_pop_solution_queue.cpp
)
target_include_directories(parallel_pick_behaviors
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(parallel_pick_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_features(parallel_pick_behaviors PUBLIC cxx_std_17)

# Install Libraries
install(
  TARGETS parallel_pick_behaviors
  EXPORT parallel_pick_behaviorsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the behavior plugins defined in this package so they are available to plugin loaders that load the
# behavior base class library from the moveit_studio_behavior package.
pluginlib_export_plugin_description_file(moveit_studio_behavior_interface parallel_pick_behaviors_plugin_description.xml)

ament_export_targets(parallel_pick_behaviorsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
