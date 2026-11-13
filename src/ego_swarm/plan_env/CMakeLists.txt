cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
  traj_utils
  nav_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs traj_utils nav_msgs
 DEPENDS OpenCV
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/grid_map.cpp 
    src/raycast.cpp
    src/obj_predictor.cpp 
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )  

add_executable(obj_generator
    src/obj_generator.cpp 
)
target_link_libraries(obj_generator 
    ${catkin_LIBRARIES}
    )

set_property(TARGET plan_env PROPERTY CXX_STANDARD 14)

add_dependencies(plan_env ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_unknown_space test/unknown_space.test test/test_unknown_space.cpp)
  if(TARGET test_unknown_space)
    target_link_libraries(test_unknown_space plan_env ${catkin_LIBRARIES})
  endif()
endif()
