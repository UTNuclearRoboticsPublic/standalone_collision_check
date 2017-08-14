cmake_minimum_required(VERSION 2.8.3)
project(standalone_collision_check)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
)

# What should we make available to other packages that depend on this one?
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    roscpp
  DEPENDS
  LIBRARIES
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(standalone_collision_check src/standalone_collision_check.cpp)
add_dependencies(standalone_collision_check ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(standalone_collision_check ${catkin_LIBRARIES} )
