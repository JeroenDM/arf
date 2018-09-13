cmake_minimum_required(VERSION 2.8.3)
project(arf_graph)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES arf_graph
#  CATKIN_DEPENDS moveit_core moveit_ros_planning roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(arf_graph src/graph.cpp)
target_link_libraries(arf_graph ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_test_graph test/test_graph.cpp)
set_target_properties(${PROJECT_NAME}_test_graph PROPERTIES OUTPUT_NAME test_graph PREFIX "")
add_dependencies(${PROJECT_NAME}_test_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_graph
 gtest
 arf_graph
 ${catkin_LIBRARIES}
)
