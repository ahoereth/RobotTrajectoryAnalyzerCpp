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
add_executable(acceleration_topic src/acceleration_topic.cpp)
target_link_libraries(acceleration_topic ${roscpp_LIBRARIES} AnnotationGateway)
