cmake_minimum_required(VERSION 3.0.2)
project(pilz_testutils)

add_definitions(-std=c++14)
add_definitions(-Wall)
add_definitions(-Wextra)
add_definitions(-Wno-unused-parameter)
add_definitions(-Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pilz_utils
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

################
## Clang tidy ##
################
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/joint_state_publisher_mock.cpp
)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    APPEND_COVERAGE_COMPILER_FLAGS()
  endif()

  catkin_add_gmock(unittest_async_test
    test/unittest_async_test.cpp
  )
  target_link_libraries(unittest_async_test
    ${catkin_LIBRARIES}
  )

  catkin_add_gmock(unittest_logger_mock
  test/unittest_logger_mock.cpp
  )
  target_link_libraries(unittest_logger_mock
    ${catkin_LIBRARIES}
  )

  catkin_add_gmock(unittest_service_client_mock
  test/unittest_service_client_mock.cpp
  )
  target_link_libraries(unittest_service_client_mock
    ${catkin_LIBRARIES}
  )

  add_rostest_gmock(unittest_joint_state_publisher_mock
    test/unittest_joint_state_publisher_mock.test
    test/unittest_joint_state_publisher_mock.cpp
  )
  target_link_libraries(unittest_joint_state_publisher_mock
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  # to run:
  #   catkin_make -DENABLE_COVERAGE_TESTING=ON package_name
  #   catkin_make -DENABLE_COVERAGE_TESTING=ON package_name_coverage (adding -j1 recommended)
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      # specifying dependencies in a reliable way is on open issue
      # see https://github.com/mikeferguson/code_coverage/pull/14
    )
  endif()
endif()
