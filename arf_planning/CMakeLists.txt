cmake_minimum_required(VERSION 2.8.3)
project(arf_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  arf_graph
  arf_moveit_wrapper
  arf_trajectory
  moveit_visual_tools
  roscpp
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS moveit_visual_tools roscpp
   #DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/planner.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/test_planner.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME test PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})