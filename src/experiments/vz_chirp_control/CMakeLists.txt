cmake_minimum_required(VERSION 3.0.2)
project(vz_chirp_control)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
)

add_message_files(
  FILES
  VzChirpStatus.msg
  VzRecorderHealth.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vz_chirp_profile
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime nav_msgs roscpp rospy std_msgs tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vz_chirp_profile
  src/chirp_profile.cpp
)
target_link_libraries(vz_chirp_profile ${catkin_LIBRARIES})

add_executable(vz_chirp_control_node
  src/vz_chirp_control_node.cpp
)
add_dependencies(vz_chirp_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vz_chirp_control_node vz_chirp_profile ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS scripts/offboard_bag_recorder.py scripts/recorder_session.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS vz_chirp_profile vz_chirp_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_chirp_profile test/test_chirp_profile.cpp)
  if(TARGET test_chirp_profile)
    target_link_libraries(test_chirp_profile vz_chirp_profile ${catkin_LIBRARIES})
  endif()
endif()
