cmake_minimum_required(VERSION 3.0.2)
project(front_marker_vision)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++14)

if(EXISTS "/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
  set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4" CACHE PATH "OpenCV config path" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
)
find_package(OpenCV REQUIRED COMPONENTS
  aruco
  calib3d
  core
  imgproc
)

add_message_files(FILES HomogeneousTransform.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES front_marker_vision_pose_estimator front_marker_vision_roi_tracker
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(front_marker_vision_pose_estimator
  src/marker_pose_estimator.cpp
)
add_dependencies(front_marker_vision_pose_estimator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(front_marker_vision_pose_estimator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(front_marker_vision_roi_tracker
  src/roi_tracker.cpp
)
add_dependencies(front_marker_vision_roi_tracker
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(front_marker_vision_roi_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(front_marker_vision_node
  src/front_marker_vision_node.cpp
)
add_dependencies(front_marker_vision_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(front_marker_vision_node
  front_marker_vision_pose_estimator
  front_marker_vision_roi_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(
  TARGETS front_marker_vision_node front_marker_vision_pose_estimator front_marker_vision_roi_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_marker_pose_estimator test/test_marker_pose_estimator.cpp)
  if(TARGET test_marker_pose_estimator)
    target_link_libraries(test_marker_pose_estimator
      front_marker_vision_pose_estimator
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_roi_tracker test/test_roi_tracker.cpp)
  if(TARGET test_roi_tracker)
    target_link_libraries(test_roi_tracker
      front_marker_vision_roi_tracker
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
    )
  endif()
  add_rostest(test/test_rejected_frame_no_publish.test)
  add_rostest(test/test_bootstrap_ambiguity_no_publish.test)
  add_rostest(test/test_marker_tf_publication.test)
  add_rostest(test/test_roi_tracking.test)
endif()
