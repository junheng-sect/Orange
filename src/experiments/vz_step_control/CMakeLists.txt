cmake_minimum_required(VERSION 3.0.2)
project(vz_step_control)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
)

add_message_files(
  FILES
  VzStepStatus.msg
  VzRecorderHealth.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vz_step_profile
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime nav_msgs roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(vz_step_profile
  src/step_profile.cpp
  src/step_controller.cpp
)
target_link_libraries(vz_step_profile ${catkin_LIBRARIES})

add_executable(vz_step_control_node
  src/vz_step_control_node.cpp
)
add_dependencies(vz_step_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vz_step_control_node vz_step_profile ${catkin_LIBRARIES})

catkin_install_python(
  PROGRAMS scripts/offboard_bag_recorder.py scripts/recorder_session.py
            scripts/sequence_allocator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS vz_step_profile vz_step_control_node
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
  catkin_add_gtest(test_step_profile test/test_step_profile.cpp)
  if(TARGET test_step_profile)
    target_link_libraries(test_step_profile vz_step_profile ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_step_controller test/test_step_controller.cpp)
  if(TARGET test_step_controller)
    target_link_libraries(test_step_controller vz_step_profile ${catkin_LIBRARIES})
  endif()
endif()
