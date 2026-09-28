cmake_minimum_required(VERSION 3.0.2)
project(front_marker_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    nav_msgs
    rospy
    std_msgs
    tf2_geometry_msgs
    tf2_ros
)

catkin_install_python(
  PROGRAMS scripts/front_marker_control_node scripts/front_camera_static_tf_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
  catkin_add_nosetests(test/test_lock_scope.py)
  catkin_add_nosetests(test/test_localization_geometry.py)
  catkin_add_nosetests(test/test_refinement.py)
  catkin_add_nosetests(test/test_refinement_node.py)
  add_rostest(test/test_position_hold.test)
  add_rostest(test/test_refinement_integration.test)
  add_rostest(test/test_sample_driven_control.test)
  add_rostest(test/test_yaw_control_switch.test)
  add_rostest(test/test_ov13855_input.test)
endif()
