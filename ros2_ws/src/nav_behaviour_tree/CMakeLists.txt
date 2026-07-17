cmake_minimum_required(VERSION 3.8)
project(nav_behaviour_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

# The executor. Leaf nodes are header-only for now (include/nav_behaviour_tree/*.hpp),
# so they compile straight into this target. If a leaf grows a .cpp, add it
# to this add_executable list or build a shared lib and link it.
add_executable(bt_main src/bt_main.cpp)
target_include_directories(bt_main PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(bt_main ${dependencies})

install(TARGETS bt_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(PROGRAMS
  scripts/imu_hdas_translator.py
  DESTINATION lib/${PROJECT_NAME})


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
