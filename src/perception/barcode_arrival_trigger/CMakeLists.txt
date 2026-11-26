cmake_minimum_required(VERSION 3.0.2)
project(barcode_arrival_trigger)
find_package(catkin REQUIRED COMPONENTS
  barcode_rc_trigger
  barcode_reader_camera
  barcode_reader_serial
  mavros_msgs
  message_generation
  ov13855_apriltag_vision
  rospy
  std_msgs)

catkin_python_setup()

add_message_files(FILES ArrivalReadResult.msg)
generate_messages(DEPENDENCIES std_msgs barcode_reader_camera barcode_reader_serial)

catkin_package(CATKIN_DEPENDS
  barcode_rc_trigger
  barcode_reader_camera
  barcode_reader_serial
  mavros_msgs
  message_runtime
  ov13855_apriltag_vision
  rospy
  std_msgs
)
catkin_install_python(PROGRAMS scripts/barcode_arrival_trigger_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_policy.py)
  add_rostest(test/node.test)
endif()
