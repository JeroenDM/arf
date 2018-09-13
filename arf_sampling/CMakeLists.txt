cmake_minimum_required(VERSION 2.8.3)
project(arf_sampling)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_visual_tools
  roscpp
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES arf_sampling
#  CATKIN_DEPENDS moveit_core moveit_ros_planning roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sampling.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test test/test_sampling.cpp)
set_target_properties(${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME test PREFIX "")
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test
 gtest
 ${PROJECT_NAME}
 ${catkin_LIBRARIES}
)