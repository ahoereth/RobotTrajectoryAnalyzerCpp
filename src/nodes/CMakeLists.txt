cmake_minimum_required(VERSION 2.8.3)
project(rta_nodes)

################################################################################
## Dependencies                                                               ##
################################################################################
find_package(catkin REQUIRED)
find_package(roscpp REQUIRED)
find_package(uimacppext REQUIRED)
find_package(rta_annotators REQUIRED)

################################################################################
## Catkin Configuration                                                       ##
################################################################################
catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
)

################################################################################
## Includes                                                                   ##
################################################################################
include_directories(
  ${roscpp_INCLUDE_DIRS}
  ${uimacppext_INCLUDE_DIRS}
  ${rta_annotators_INCLUDE_DIRS}
  include
)

################################################################################
## Executables                                                                ##
################################################################################
add_library(Topics src/Topics.cpp)
target_link_libraries(Topics ${roscpp_LIBRARIES} AnnotationGateway uimautils)

add_executable(plot src/plot.cpp)
target_link_libraries(plot
  ${roscpp_LIBRARIES}
  AnnotationGateway
  uimautils
  Topics
)
