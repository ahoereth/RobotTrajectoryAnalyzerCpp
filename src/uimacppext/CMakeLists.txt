cmake_minimum_required(VERSION 2.8.3)
project(uimacppext)

################################################################################
## Include sub cmake files                                                    ##  ################################################################################
include(cmake/uima.cmake)

################################################################################
## Configure package                                                           ##
################################################################################
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
)

include_directories(include)

################################################################################
## Libraries                                                                  ##
################################################################################
uima_add_library(uimautils src/utils.cpp)

uima_add_library(StdCoutLogger src/StdCoutLogger.cpp)
