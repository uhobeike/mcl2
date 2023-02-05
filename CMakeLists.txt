cmake_minimum_required(VERSION 3.8)
project(mcl2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

ament_auto_add_library(mcllib SHARED
  src/mcl/likelihoodField.cpp
  src/mcl/mcl.cpp
  src/mcl/motionModelUpdate.cpp
  src/mcl/observationModelUpdate.cpp
  src/mcl/resampling.cpp
)

ament_auto_add_library(mcl2_node SHARED
  src/mcl2_node.cpp
)

target_link_libraries(mcl2_node
  mcllib
)

rclcpp_components_register_node(mcl2_node
  PLUGIN "mcl2::Mcl2Node"
  EXECUTABLE ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
