cmake_minimum_required(VERSION 3.0.2)
project(pilz_industrial_motion_testutils)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
   src/${PROJECT_NAME}/xml_testdata_loader.py
   src/${PROJECT_NAME}/acceptance_test_utils.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
