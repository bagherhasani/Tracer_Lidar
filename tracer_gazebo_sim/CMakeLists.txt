cmake_minimum_required(VERSION 2.8.3)
project(tracer_gazebo_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roslint 
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2 
    tf2_ros
)
catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(tracer_gazebo STATIC src/tracer_skid_steer.cpp)
target_link_libraries(tracer_gazebo ${catkin_LIBRARIES})

add_executable(tracer_skid_steer_controller src/tracer_skid_steer_controller.cpp)
target_link_libraries(tracer_skid_steer_controller tracer_gazebo ${catkin_LIBRARIES})


#############
## Install ##
#############

roslaunch_add_file_check(launch)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
