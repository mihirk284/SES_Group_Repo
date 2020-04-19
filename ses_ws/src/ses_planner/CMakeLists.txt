cmake_minimum_required(VERSION 2.8.3)
project(ses_planner)

add_compile_options(-std=c++11)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   geometry_msgs
   std_msgs
   catkin_simple
   message_generation 
)
find_package(Eigen3 REQUIRED)


add_service_files(
FILES
checkBoxes.srv
)

# add_message_files(
#   FILES
#   geometry_msgs
#   std_msgs
# )

# generate_messages(
# DEPENDENCIES
# std_msgs
# geometry_msgs
# )




catkin_simple(ALL_DEPS_REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

cs_add_library(
  ${PROJECT_NAME}
  src/map_manager_voxblox_erc.cpp
  src/voxblox_interface.cpp
  src/voxbloxinterface_ros_node.cpp
  )


cs_add_executable(sesplanner_node src/voxbloxinterface_ros_node.cpp src/map_manager_voxblox_erc.cpp src/voxblox_interface.cpp)


target_link_libraries(sesplanner_node ${PROJECT_NAME})

cs_install()
cs_export(LIBRARIES)
