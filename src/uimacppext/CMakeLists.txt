cmake_minimum_required(VERSION 2.8.3)
project(uimacppext)

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
add_library(uimautils SHARED src/utils.cpp src/uimautils.cpp)
target_link_libraries(uimautils ${uimacpp_LIBRARIES})

add_library(StdCoutLogger SHARED src/StdCoutLogger.cpp)
target_link_libraries(StdCoutLogger ${uimacpp_LIBRARIES})

add_library(AnnotationIterator SHARED src/AnnotationIterator.cpp)
target_link_libraries(AnnotationIterator ${uimacpp_LIBRARIES})
