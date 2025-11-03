cmake_minimum_required(VERSION 3.14)
project(v2i_interface)

find_package(autoware_cmake QUIET)
if(autoware_cmake_FOUND)
  autoware_package()
else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
endif()

# Node executable
ament_auto_add_executable(v2i_interface_node
  src/main.cpp
  src/v2i_interface_node.cpp
  src/state_converter.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
