cmake_minimum_required(VERSION 3.8)
project(pcd2octomap)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pcd2octomap.cpp
  src/pcd_occlusion_fileter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "Pcd2Octomap"
  EXECUTABLE pcd2octomap_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "PcdOcclusionFilter"
  EXECUTABLE pcd_occlusion_filter_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)

install(FILES DESTINATION share/${PROJECT_NAME})