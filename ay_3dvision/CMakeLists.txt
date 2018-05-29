cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# ++++OPENCV-2.x++++
find_package(OpenCV 2 REQUIRED)
message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ++++OPENCV-2.4.13++++
# # NOTE: Install OpenCV 2.4.13 on ~/.local from source which includes cv::fisheye
# # We don't use opencv found by any prior running of find_package
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# set(TMP_PREFIX_PATH ${CMAKE_PREFIX_PATH})
# set(CMAKE_PREFIX_PATH "$ENV{HOME}/.local")
# find_package(OpenCV 2.4.13 REQUIRED)
# # restore CMAKE_PREFIX_PATH and other cached variables
# # in order to avoid other packages find this opencv.
# set(CMAKE_PREFIX_PATH ${TMP_PREFIX_PATH})
# unset(OpenCV_CONFIG_PATH CACHE)
# unset(OpenCV_DIR CACHE)
# message("OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message("OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
# ----OPENCV----

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#Building libraries:

rosbuild_add_library(ay3dv_pcl_util
      src/pcl_util.cpp
      )

if(EXISTS /usr/include/libm100)
  rosbuild_add_library(ay3dv_sentis_m100
        src/sentis_m100.cpp
        )
  include_directories(/usr/include/libm100)
  target_link_libraries(ay3dv_sentis_m100
        ${OpenCV_LIBRARIES}
        m100
        )
endif(EXISTS /usr/include/libm100)


#Building executables:

rosbuild_add_executable(rt_pose_estimator_node
      src_ros/rt_pose_estimator_node.cpp
      )
target_link_libraries(rt_pose_estimator_node
      ayv_rt_pose_estimator
      ay3dv_pcl_util
      ay_vision
      ${OpenCV_LIBRARIES}
      pcl_segmentation
      )

if(EXISTS /usr/include/libm100)
  rosbuild_add_executable(sentis_m100_node
        src_ros/sentis_m100_node.cpp
        )
  include_directories(/usr/include/libm100)
  target_link_libraries(sentis_m100_node
        ay3dv_sentis_m100
        ${OpenCV_LIBRARIES}
        m100
        )
endif(EXISTS /usr/include/libm100)

rosbuild_add_executable(usb_stereo_node
      src_ros/usb_stereo_node.cpp
      )
target_link_libraries(usb_stereo_node
      ay3dv_pcl_util
      ayv_color_detector
      ayv_flow_finder
      ayv_usb_stereo
      ayv_edge_fit
      ay_vision
      ay_cpp
      ${OpenCV_LIBRARIES}
      pcl_segmentation
      boost_thread
      boost_system
      )
