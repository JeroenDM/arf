cmake_minimum_required(VERSION 2.8.3)
project(arf_moveit_wrapper)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  opw_kinematics
  arf_sampling
)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS opw_kinematics moveit_core moveit_ros_planning opw_kinematics
  DEPENDS EIGEN3
)

###########
## Build ##
###########

# library
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/moveit_wrapper.cpp src/redundant_robot.cpp
)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# main node for quick testing purpose
add_executable(
  ${PROJECT_NAME}_main
  src/main.cpp
)

set_target_properties(
  ${PROJECT_NAME}_main
  PROPERTIES OUTPUT_NAME main PREFIX ""
)

target_link_libraries(${PROJECT_NAME}_main
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)