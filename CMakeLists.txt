cmake_minimum_required(VERSION 3.8)
project(teleop_twist_stamped_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(teleop_twist_stamped_node_parameters
  src/teleop_twist_stamped_node_parameters.yaml
)

ament_auto_add_library(teleop_twist_stamped_joy SHARED
  src/converter.cpp
  src/teleop_twist_stamped_node.cpp
)
target_include_directories(teleop_twist_stamped_joy
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(teleop_twist_stamped_joy
  teleop_twist_stamped_node_parameters
)
rclcpp_components_register_node(teleop_twist_stamped_joy
  PLUGIN "teleop_twist_stamped_joy::TeleopTwistStampedNode"
  EXECUTABLE teleop_twist_stamped_joy_node
)

install(
  TARGETS
    teleop_twist_stamped_node_parameters
    teleop_twist_stamped_joy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/
)
install(
  DIRECTORY
    launch
    config
  DESTINATION
   "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
