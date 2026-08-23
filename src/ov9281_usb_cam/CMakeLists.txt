cmake_minimum_required(VERSION 2.8.12)
project(ov9281_usb_cam)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  image_transport
  roscpp
  sensor_msgs
  std_srvs
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec REQUIRED libavcodec)
pkg_check_modules(avutil REQUIRED libavutil)
pkg_check_modules(swscale REQUIRED libswscale)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${avutil_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

add_executable(ov9281_usb_cam_node
  nodes/usb_cam_node.cpp
  src/usb_cam.cpp
)
target_link_libraries(ov9281_usb_cam_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${avcodec_LIBRARIES}
  ${avutil_LIBRARIES}
  ${swscale_LIBRARIES}
)

install(TARGETS ov9281_usb_cam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
