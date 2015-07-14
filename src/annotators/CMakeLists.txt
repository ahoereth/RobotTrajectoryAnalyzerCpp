cmake_minimum_required(VERSION 2.8.3)
project(rta_annotators)

################################################################################
## CMake Configuration                                                        ##
################################################################################
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

################################################################################
## Dependencies                                                               ##
################################################################################
find_package(catkin REQUIRED)
find_package(uimacpp REQUIRED)
find_package(mongoclient REQUIRED)
find_package(uimacppext REQUIRED)

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
  ${uimacpp_INCLUDE_DIRS}
  ${mongoclient_INCLUDE_DIRS}
  ${uimacppext_INCLUDE_DIRS}
  include
)

################################################################################
## Macros                                                                     ##
################################################################################
macro(add_uima_annotator libname)
  add_library(${libname} SHARED ${ARGN})
  target_link_libraries(${libname} ${uimacpp_LIBRARIES})
  set_target_properties(${libname} PROPERTIES PREFIX "")
endmacro(add_uima_annotator)

################################################################################
## Annotators                                                                 ##
################################################################################
add_uima_annotator(JointStatePopulator src/JointStatePopulator.cpp)
target_link_libraries(JointStatePopulator
  ${mongoclient_LIBRARIES}
  ${Boost_LIBRARIES}
  uimautils)

add_uima_annotator(AccelerationAnnotator src/AccelerationAnnotator.cpp)
target_link_libraries(AccelerationAnnotator uimautils)

add_uima_annotator(MovementAnnotator src/MovementAnnotator.cpp)
target_link_libraries(MovementAnnotator uimautils)

add_uima_annotator(MovementDirectionAnnotator src/MovementDirectionAnnotator.cpp)
target_link_libraries(MovementDirectionAnnotator uimautils)

add_uima_annotator(OscillationAnnotator src/OscillationAnnotator.cpp)
target_link_libraries(OscillationAnnotator uimautils)

add_uima_annotator(ControllerAnnotator src/ControllerAnnotator.cpp)
target_link_libraries(ControllerAnnotator
#  ${mongoclient_LIBRARIES}
#  ${Boost_LIBRARIES}
  uimautils)

add_uima_annotator(ErrorAnnotator src/ErrorAnnotator.cpp)
target_link_libraries(ErrorAnnotator uimautils)

add_uima_annotator(PlannedMovementAnnotator src/PlannedMovementAnnotator.cpp)
target_link_libraries(PlannedMovementAnnotator uimautils)

################################################################################
## Libraries                                                                  ##
################################################################################
add_library(AnnotationGateway SHARED src/AnnotationGateway.cpp)
target_link_libraries(AnnotationGateway
  ${uimacpp_LIBRARIES}
  uimautils
  StdCoutLogger
  AnnotationIterator)

################################################################################
## Executables                                                                ##
################################################################################
add_executable(demo src/demo.cpp)
target_link_libraries(demo ${uimacpp_LIBRARIES} AnnotationGateway)
