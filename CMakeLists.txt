cmake_minimum_required(VERSION 3.0.2)
project(rflm_AKF_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)


catkin_package(
  # INCLUDE_DIRS include
#  LIBRARIES rflm_AKF_pkg
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)

add_executable(akf_node src/KalmanFilter.cpp src/kalman_filter.cpp)
target_link_libraries(akf_node
  ${catkin_LIBRARIES}
)