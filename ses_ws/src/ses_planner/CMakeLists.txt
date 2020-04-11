cmake_minimum_required(VERSION 2.8.3)
project(ses_planner)

add_compile_options(-std=c++11)
add_definitions(-std=c++11)
find_package(catkin REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

cs_add_library(
  ${PROJECT_NAME}
  src/map_manager_voxblox_erc.cpp
  )


cs_add_executable(sesplanner_node src/sesplanner_ros_node.cpp)
target_link_libraries(sesplanner_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)
