# Includes the uima and apr include directories and defines makros
# for adding uima libraries and executables.
#
# Requires the UIMACPP_HOME environment variable to be set correctly.
#
# Exports
# * uima_add_library
# * uima_add_executable

##
## Props to the iai_robosherlock project.
## Institute for Artificial Intelligence, University of Bremen
## iai_robosherlock/master/iai_rs_cpp/cmake/uimabuild.cmake
##


################################################################################
## Enable all warnings..                                                      ##
################################################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# ..but the deprecated and unused variable warnings - because uima.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-unused-but-set-variable")

################################################################################
## UIMA PATHS                                                                 ##
################################################################################
if(DEFINED ENV{UIMACPP_HOME})
  include_directories($ENV{UIMACPP_HOME}/include)
  include_directories($ENV{UIMACPP_HOME}/include/apr-1)
  link_directories($ENV{UIMACPP_HOME}/lib)
endif()

################################################################################
## Add library macro                                                          ##
################################################################################
macro(uima_add_library libname)
  add_library(${libname} SHARED ${ARGN})
  target_link_libraries(${libname} uima apr-1 xerces-c icuuc icuio icui18n icudata dl)
  set_target_properties(${libname} PROPERTIES PREFIX "")
endmacro(uima_add_library)

################################################################################
## Add executable macro                                                       ##
################################################################################
macro(uima_add_executable execname)
  add_executable(${execname} ${ARGN})
  target_link_libraries(${execname} uima apr-1 xerces-c icuuc icuio icui18n icudata dl)
endmacro(uima_add_executable)
