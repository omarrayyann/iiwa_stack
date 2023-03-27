cmake_minimum_required(VERSION 2.8.3)
project(actuonix_control)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++0x -Wno-psabi ${CMAKE_CXX_FLAGS}")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
)

catkin_package(CATKIN_DEPENDS std_msgs roscpp)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(ActuonixROS src/ActuonixROS.cpp src/firgelli.cpp)
target_link_libraries(ActuonixROS ${catkin_LIBRARIES} -lusb-1.0)

add_executable(ActuonixROS_open_closepublisher src/ActuonixROS_open_closepublisher.cpp src/firgelli.cpp)
target_link_libraries(ActuonixROS_open_closepublisher ${catkin_LIBRARIES} -lusb-1.0)
