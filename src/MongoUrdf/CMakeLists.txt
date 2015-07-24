cmake_minimum_required(VERSION 2.8.3)
project(mongourdf)

################################################################################
## Dependencies                                                               ##
################################################################################
find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(urdf REQUIRED)
find_package(mongoclient REQUIRED)
find_package(TinyXML REQUIRED)
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
  ${urdf_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${mongoclient_INCLUDE_DIRS}
  ${uimacppext_INCLUDE_DIRS}
  include
)

################################################################################
## Libraries                                                                  ##
################################################################################
#add_library(MongoUrdf SHARED src/MongoUrdf.cpp)
add_library(MongoUrdf SHARED src/MongoUrdf.cpp)
target_link_libraries(MongoUrdf
# See https://github.com/ahoereth/RobotTrajectoryAnalyzerCpp/issues/3
#  ${mongoclient_LIBRARIES}
#  ${Boost_LIBRARIES}
  ${urdf_LIBRARIES}
  ${TinyXML_LIBRARIES}
  uimautils)
