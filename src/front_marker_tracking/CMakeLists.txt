cmake_minimum_required(VERSION 3.0.2)
project(front_marker_tracking)

add_compile_options(-std=c++14)

if(EXISTS "/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
  set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4" CACHE PATH "OpenCV config path" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  mavros_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED COMPONENTS
  aruco
  calib3d
  core
  imgproc
  videoio
)

add_message_files(
  FILES
  HomogeneousTransform.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES front_marker_pose_estimator
  CATKIN_DEPENDS
    camera_info_manager
    cv_bridge
    geometry_msgs
    mavros_msgs
    message_runtime
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(front_marker_detector_node
  src/front_marker_detector_node.cpp
)

add_executable(front_marker_csi_camera_node
  src/front_marker_csi_camera_node.cpp
)

add_library(front_marker_pose_estimator
  src/marker_pose_estimator.cpp
)

add_dependencies(front_marker_pose_estimator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(front_marker_pose_estimator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(front_marker_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(front_marker_csi_camera_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(front_marker_detector_node
  front_marker_pose_estimator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(front_marker_csi_camera_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    scripts/front_marker_ab_report.py
    scripts/front_marker_base_transform_node
    scripts/front_marker_tracking_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS front_marker_detector_node front_marker_csi_camera_node front_marker_pose_estimator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_math_and_control.py)
  catkin_add_gtest(test_marker_pose_estimator test/test_marker_pose_estimator.cpp)
  if(TARGET test_marker_pose_estimator)
    target_link_libraries(test_marker_pose_estimator
      front_marker_pose_estimator
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
    )
  endif()
  add_rostest(test/test_rejected_frame_no_publish.test)
  add_rostest(test/test_sample_driven_tracking.test)
  add_rostest(test/test_yaw_control_switch.test)
endif()
