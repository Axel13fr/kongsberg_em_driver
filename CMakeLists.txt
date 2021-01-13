cmake_minimum_required(VERSION 2.8.3)
project(kongsberg_em_driver)

set(CMAKE_CXX_STANDARD 14)
# ext-numeric-literals is needed for Kinetic compilation
add_compile_options(-std=c++14 -Wall -Werror=return-type -fext-numeric-literals)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        std_msgs
        ds_core_msgs
        ds_kongsberg_msgs
        ds_multibeam_msgs
        librosasio)

find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lib_kongsberg_em
  CATKIN_DEPENDS std_msgs roscpp ds_core_msgs
  ds_kongsberg_msgs ds_multibeam_msgs librosasio
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
 ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE LIB_SOURCES src/em_driver_library/*.cpp src/em_driver_library/*.h)
# Declare a C++ library
add_library(lib_kongsberg_em
  ${LIB_SOURCES}
)
add_dependencies(lib_kongsberg_em ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

file(GLOB_RECURSE NODE_SOURCES src/em_driver_node/*.cpp src/em_driver_node/*.h)
add_executable(kongsberg_em_node ${NODE_SOURCES})

target_link_libraries(lib_kongsberg_em
   ${catkin_LIBRARIES}
)

target_link_libraries(kongsberg_em_node
    lib_kongsberg_em
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS lib_kongsberg_em kongsberg_em_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/lib_kongsberg_em/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_em2040_parse
        launch/em2040_parse_test.test
        src/test/test_kongsberg_em2040_parsing.cpp)
    target_link_libraries(test_em2040_parse lib_kongsberg_em)
    catkin_add_gtest(test_em2040_utils src/test/test_kongsberg_em2040_util.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
    target_link_libraries(test_em2040_utils lib_kongsberg_em)
#endif(CATKIN_ENABLE_TESTING)
