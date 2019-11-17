cmake_minimum_required(VERSION 2.8.3)
project(getPose_ENU_312)

#启用c++11
set(CMAKE_CXX_FLAGS "-std=c++11")

# List C++ dependencies on ros packages
set(ROS_CXX_DEPENDENCIES
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  std_msgs
)

#寻找EIGEN3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS
    ${ROS_CXX_DEPENDENCIES}
    )



include_directories(
include
${catkin_INCLUDE_DIRS}
)

INCLUDE_DIRECTORIES(/opt/ros/kinetic/include/)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_ekf
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(${PROJECT_NAME} src/Convert.cpp src/EKF_Attitude.cpp src/cal_position.cpp src/main.cpp)
#add_executable(pose_ekf src/test.cpp src/ins.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
