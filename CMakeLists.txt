cmake_minimum_required(VERSION 3.5)
project(system_modes_benchmark)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(system_modes REQUIRED)
find_package(system_modes_msgs REQUIRED)

set(dependencies
  rclcpp  
  rclcpp_lifecycle
  lifecycle_msgs
  system_modes
  system_modes_msgs
)

add_executable(nav2 src/nav2.cpp)
add_executable(manipulator src/manipulator.cpp)
add_executable(camera src/camera.cpp)
add_executable(guard src/guard.cpp)

ament_target_dependencies(nav2 ${dependencies})
ament_target_dependencies(manipulator ${dependencies})
ament_target_dependencies(camera ${dependencies})
ament_target_dependencies(guard ${dependencies})

install(FILES benchmark_modes.yaml
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(TARGETS
  nav2
  manipulator
  camera
  guard
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
