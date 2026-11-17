cmake_minimum_required(VERSION 3.10)
project(fastlio_dll)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs geometry_msgs message_filters nav_msgs roscpp sensor_msgs std_msgs tf2_msgs tf2_ros)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

catkin_package()

add_executable(fastlio_dll_node src/localization.cpp src/registration.cpp)
target_compile_features(fastlio_dll_node PRIVATE cxx_std_17)
target_compile_options(fastlio_dll_node PRIVATE -Wall -Wextra)
target_include_directories(fastlio_dll_node SYSTEM PRIVATE
  ${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(fastlio_dll_node PRIVATE
  ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads OpenSSL::Crypto)

install(TARGETS fastlio_dll_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/replay.sh scripts/record_observation.sh
  scripts/summarize_replay.py scripts/replay_legacy_seed.py
  scripts/prepare_outdoor_map.py scripts/replay_pair_outdoor.sh scripts/record_outdoor_dll_ywl
  scripts/record_outdoor_rc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
