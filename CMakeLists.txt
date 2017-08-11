cmake_minimum_required(VERSION 2.8.3)
project(standalone_collision_check)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# What should we make available to other packages that depend on this one?
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
  LIBRARIES standalone_collision_check
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(standalone_collision_check src/standalone_collision_check.cpp)
add_dependencies(standalone_collision_check ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(standalone_collision_check ${catkin_LIBRARIES} )
