cmake_minimum_required(VERSION 3.8)
project(robot_chase)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(robot_chase_node src/robot_chase.cpp)
ament_target_dependencies(robot_chase_node rclcpp)

install(TARGETS
   robot_chase_node
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
