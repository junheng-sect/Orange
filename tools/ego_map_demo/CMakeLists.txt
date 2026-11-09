cmake_minimum_required(VERSION 3.10)
project(ego_saved_map_demo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs geometry_msgs nav_msgs sensor_msgs visualization_msgs cv_bridge message_filters tf)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
# The manager has an unused DataDisp include. Avoid generating unused messages
# while keeping all vendored source files unchanged.
file(READ "${CMAKE_SOURCE_DIR}/official/plan_manage/include/plan_manage/planner_manager.h" H)
string(REPLACE "#include <traj_utils/DataDisp.h>" "" H "${H}")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated/plan_manage")
file(WRITE "${CMAKE_BINARY_DIR}/generated/plan_manage/planner_manager.h" "${H}")
file(READ "${CMAKE_SOURCE_DIR}/official/bspline_opt/include/bspline_opt/gradient_descent_optimizer.h" G)
string(REPLACE "{1e10}" "{2147483647}" G "${G}")
file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/generated/bspline_opt")
file(WRITE "${CMAKE_BINARY_DIR}/generated/bspline_opt/gradient_descent_optimizer.h" "${G}")
add_executable(ego_map_planner
  src/demo.cpp
  official/plan_manage/src/planner_manager.cpp
  official/plan_env/src/grid_map.cpp official/plan_env/src/raycast.cpp official/plan_env/src/obj_predictor.cpp
  official/path_searching/src/dyn_a_star.cpp
  official/bspline_opt/src/uniform_bspline.cpp official/bspline_opt/src/bspline_optimizer.cpp official/bspline_opt/src/gradient_descent_optimizer.cpp
  official/traj_utils/src/planning_visualization.cpp official/traj_utils/src/polynomial_traj.cpp)
target_include_directories(ego_map_planner PRIVATE
  "${CMAKE_BINARY_DIR}/generated"
  official/plan_manage/include official/plan_env/include official/path_searching/include official/bspline_opt/include official/traj_utils/include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_compile_options(ego_map_planner PRIVATE -O2)
target_link_libraries(ego_map_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
set_target_properties(ego_map_planner PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
