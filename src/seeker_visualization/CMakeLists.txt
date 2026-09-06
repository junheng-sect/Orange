cmake_minimum_required(VERSION 3.0.2)
project(seeker_visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  tf
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    rospy
    tf
    tf2_ros
    visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/mavros_odom_tf_node.py
    scripts/tf_frame_labels_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_mavros_odom_tf.test)
  add_rostest(test/test_tf_frame_labels.test)
endif()
