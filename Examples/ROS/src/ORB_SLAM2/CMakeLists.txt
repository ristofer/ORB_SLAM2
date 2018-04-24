cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(orb_slam_ros)
#rosbuild_init()

#IF(NOT ROS_BUILD_TYPE)
#  SET(ROS_BUILD_TYPE Release)
#ENDIF()

# optional: debug mode
#SET(ROS_BUILD_TYPE Debug)

#MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++11.")
#elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#  message(STATUS "Using flag -std=c++0x.")
#else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    image_transport
    tf
)
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
#find_package(Eigen3 REQUIRED)
#find_package(Pangolin REQUIRED)
catkin_package(
  INCLUDE_DIRS
    src/maqui
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    std_msgs
    image_transport
    tf
  DEPENDS
    Boost
)
include_directories(
  src/maqui
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
include_directories(
${PROJECT_SOURCE_DIR}
/home/nao/ORB_SLAM2/include
/home/nao/ORB_SLAM2/
#${Pangolin_INCLUDE_DIRS}
)



set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
/home/nao/ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
/home/nao/ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
/home/nao/ORB_SLAM2/lib/libORB_SLAM2.so
)

# node for Odometry
add_executable(Maqui
    ${PROJECT_SOURCE_DIR}/src/maqui/main.cpp
    ${PROJECT_SOURCE_DIR}/src/maqui/SubscribeHandler.cpp
    )
target_link_libraries(Maqui
${LIBS}
${Boost_LIBRARIES}
${catkin_LIBRARIES}
)


# #Node for monocular camera
# rosbuild_add_executable(Mono
# src/ros_mono.cc
# )

# target_link_libraries(Mono
# ${LIBS}
# )

## Node for monocular camera (Augmented Reality Demo)
# rosbuild_add_executable(MonoAR
# src/AR/ros_mono_ar.cc
# src/AR/ViewerAR.h
# src/AR/ViewerAR.cc
# )

# target_link_libraries(MonoAR
# ${LIBS}
# )

## Node for stereo camera
# rosbuild_add_executable(Stereo
# src/ros_stereo.cc
# )

# target_link_libraries(Stereo
# ${LIBS}
# )

# Node for RGB-D camera
# rosbuild_add_executable(RGBD
# src/ros_rgbd.cc
# )

# target_link_libraries(RGBD
# ${LIBS}
# )

