cmake_minimum_required(VERSION 3.0.2)
project(barcode_reader_camera)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge geometry_msgs message_generation ov13855_apriltag_vision roscpp roslib sensor_msgs std_msgs std_srvs)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc imgcodecs)
find_package(ZXing REQUIRED)  # /usr/local/lib/cmake/ZXing, ZXing-C++ 3.0 (read-only build)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(APRILTAG_INCLUDE_DIR apriltag.h PATH_SUFFIXES apriltag)
find_library(APRILTAG_LIBRARY apriltag)

add_message_files(FILES Barcode.msg BarcodeArray.msg)
add_service_files(FILES DecodeBarcodes.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES barcode_reader_camera_decoder
  CATKIN_DEPENDS cv_bridge geometry_msgs message_runtime roscpp sensor_msgs std_msgs std_srvs)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS}
  ${APRILTAG_INCLUDE_DIR})

# ROS-free decoder core; ZXing stays private to it.
add_library(barcode_reader_camera_decoder src/multi_decoder.cpp src/config_yaml.cpp src/frame_recorder.cpp)
target_link_libraries(barcode_reader_camera_decoder PRIVATE ZXing::ZXing
  PUBLIC ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES} Threads::Threads)

add_executable(decode_image src/decode_image_main.cpp)
target_link_libraries(decode_image barcode_reader_camera_decoder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(barcode_reader_camera_node src/barcode_reader_camera_node.cpp)
add_dependencies(barcode_reader_camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(barcode_reader_camera_node barcode_reader_camera_decoder ${catkin_LIBRARIES})

install(TARGETS barcode_reader_camera_decoder decode_image barcode_reader_camera_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
catkin_install_python(PROGRAMS scripts/trigger_decode DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_barcode_fusion test/test_fusion.cpp)
  catkin_add_gtest(test_multi_decoder test/test_multi_decoder.cpp)
  catkin_add_gtest(test_frame_recorder test/test_frame_recorder.cpp)
  if(TARGET test_frame_recorder)
    target_link_libraries(test_frame_recorder barcode_reader_camera_decoder)
  endif()
  if(TARGET test_multi_decoder)
    target_link_libraries(test_multi_decoder barcode_reader_camera_decoder ${catkin_LIBRARIES}
      ${APRILTAG_LIBRARY})
  endif()
  add_rostest_gtest(test_barcode_reader_camera_node test/node.test test/test_node.cpp)
  if(TARGET test_barcode_reader_camera_node)
    add_dependencies(test_barcode_reader_camera_node barcode_reader_camera_node
      ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(test_barcode_reader_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
      ${YAML_CPP_LIBRARIES} ${APRILTAG_LIBRARY})
  endif()
endif()
