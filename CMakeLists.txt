cmake_minimum_required(VERSION 2.8.3)
project(aer_auto_exposure_gradient)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

#### main
add_executable(exp_node src/exp_node.cpp src/auto_exp.cpp)
target_link_libraries(exp_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})