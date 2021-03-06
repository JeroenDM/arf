cmake_minimum_required(VERSION 2.8.3)
project(arf_demo)


add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  arf_moveit_wrapper 
  arf_graph
  arf_trajectory
  arf_tsr
  moveit_visual_tools
  arf_planning
  moveit_ros_planning_interface
  std_msgs
  trajectory_msgs
  geometry_msgs
  message_generation
  nexon_msgs
  simple_moveit_wrapper
)

# Let's try to use open-mp parallization if we can
# find_package(OpenMP)
# if (OPENMP_FOUND)
#   message(STATUS "ARF will use OpenMP")
#   set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
# endif()

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

add_service_files(
  FILES
  TrajOptPlanningServer.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arf_demo
#  CATKIN_DEPENDS roscpp moveit_simplified arf_sampling arf_path opw_kinematics
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_demo src/demo.cpp)
set_target_properties(${PROJECT_NAME}_demo PROPERTIES OUTPUT_NAME demo PREFIX "")
target_link_libraries(${PROJECT_NAME}_demo
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_demo2 src/demo2.cpp)
add_executable(${PROJECT_NAME}_demo_table src/demo_table.cpp)
add_executable(${PROJECT_NAME}_sampling_comparison src/sampling_comparison.cpp)


set_target_properties(${PROJECT_NAME}_demo2 PROPERTIES OUTPUT_NAME demo2 PREFIX "")
set_target_properties(${PROJECT_NAME}_demo_table PROPERTIES OUTPUT_NAME demo_table PREFIX "")
set_target_properties(${PROJECT_NAME}_sampling_comparison PROPERTIES OUTPUT_NAME sampling_comparison PREFIX "")


target_link_libraries(${PROJECT_NAME}_demo2
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_demo_table
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_sampling_comparison
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_planning src/planning.cpp)
set_target_properties(${PROJECT_NAME}_planning PROPERTIES OUTPUT_NAME planning PREFIX "")
target_link_libraries(${PROJECT_NAME}_planning ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_arf_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
