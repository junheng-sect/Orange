cmake_minimum_required(VERSION 3.0.2)
project(camera_calibration_pkg)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/auto_save_calibrator.py scripts/save_calibration.py
 scripts/generate_checkerboard.py scripts/recalibrate.py scripts/scale_calibration.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch board DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
if(CATKIN_ENABLE_TESTING)
 catkin_add_nosetests(test/test_scripts.py)
endif()
