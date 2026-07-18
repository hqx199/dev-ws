cmake_minimum_required(VERSION 3.8)
project(racing_inertial_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(inertial_nav_controller 
  src/inertial_nav_controller.cpp
)

# Link libraries
ament_target_dependencies(inertial_nav_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ai_msgs
)
target_link_libraries(inertial_nav_controller ${OpenCV_LIBS})

# Install targets
install(TARGETS
  inertial_nav_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
