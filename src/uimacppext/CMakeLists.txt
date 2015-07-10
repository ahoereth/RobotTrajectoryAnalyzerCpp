cmake_minimum_required(VERSION 2.8.3)
project(uimacppext)

################################################################################
## CMake Configuration                                                        ## ################################################################################
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

################################################################################
## Dependencies                                                               ##
################################################################################
find_package(catkin REQUIRED)
find_package(uimacpp REQUIRED)

################################################################################
## Catkin Configuration                                                       ##
################################################################################
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include ${uimacpp_INCLUDE_DIRS}
)

################################################################################
## Includes                                                                   ##
################################################################################
include_directories(
  ${uimacpp_INCLUDE_DIRS}
  include
)

################################################################################
## Libraries                                                                  ##
################################################################################
add_library(uimautils SHARED src/utils.cpp)
target_link_libraries(uimautils ${uimacpp_LIBRARIES})

add_library(StdCoutLogger SHARED src/StdCoutLogger.cpp)
target_link_libraries(StdCoutLogger ${uimacpp_LIBRARIES})
