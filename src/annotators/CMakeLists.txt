cmake_minimum_required(VERSION 2.8.3)
project(rta_annotators)

################################################################################
## Include sub cmake files                                                    ##  ################################################################################
include(cmake/uima.cmake)
include(cmake/mongoclient.cmake)

################################################################################
## Configure package                                                          ##
################################################################################
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
)

include_directories(include)

################################################################################
## Configure dependencies                                                     ##
################################################################################
find_package(uimacppext REQUIRED)
include_directories(${uimacppext_INCLUDE_DIRS})

################################################################################
## Annotators                                                                 ##
################################################################################
uima_add_library(JointStatePopulator src/JointStatePopulator.cpp)
target_link_libraries(JointStatePopulator
  ${MongoDB_LIBRARIES}
  ${Boost_LIBRARIES}
  uimautils
)

uima_add_library(AccelerationAnnotator src/AccelerationAnnotator.cpp)
target_link_libraries(AccelerationAnnotator uimautils)

uima_add_library(MovementAnnotator src/MovementAnnotator.cpp)
target_link_libraries(MovementAnnotator uimautils)

uima_add_library(MovementDirectionAnnotator src/MovementDirectionAnnotator.cpp)
target_link_libraries(MovementDirectionAnnotator uimautils)

uima_add_library(OscillationAnnotator src/OscillationAnnotator.cpp)
target_link_libraries(OscillationAnnotator uimautils)

################################################################################
## Libraries                                                                  ##
################################################################################
uima_add_library(AnnotationGateway src/AnnotationGateway.cpp)
target_link_libraries(AnnotationGateway
  uimautils
  StdCoutLogger
)

################################################################################
## Executables                                                                ##
################################################################################
uima_add_executable(demo src/demo.cpp)
target_link_libraries(demo AnnotationGateway)
