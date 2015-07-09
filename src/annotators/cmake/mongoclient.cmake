# Find the MongoDB includes and client library + the required boost
# dependencies. Includes the include directories if found. Throws an fatal
# error if the mongo libraries cannot be found.
#
# Exports
# * MongoDB_LIBRARIES, the libraries needed to use MongoDB
# * MongoDB_INCLUDE_DIR, where to find mongo/client/dbclient.h
# * Boost_LIBRARIES, the boost libraries required for mongo
# * Boost_INCLUDE_DIRS, where to find the boost headers

################################################################################
## Find the mongo libraries                                                   ##
################################################################################
if(NOT MongoDB_INCLUDE_DIR AND NOT MongoDB_LIBRARIES)
  find_path(MongoDB_INCLUDE_DIR mongo/client/dbclient.h
    /usr/include/
    /usr/local/include/
    /usr/include/mongo/
    /usr/local/include/mongo/
    /opt/mongo/include/
  )

  find_library(MongoDB_LIBRARIES NAMES mongoclient
    PATHS
    /usr/lib
    /usr/lib/mongo
    /usr/local/lib
    /usr/local/lib/mongo
    /opt/mongo/lib
  )

  if(MongoDB_INCLUDE_DIR AND MongoDB_LIBRARIES)
    message(STATUS "Found MongoDB: ${MongoDB_INCLUDE_DIR}, ${MongoDB_LIBRARIES}")
  else(MongoDB_INCLUDE_DIR AND MongoDB_LIBRARIES)
    message(FATAL_ERROR "MongoDB not found.")
  endif(MongoDB_INCLUDE_DIR AND MongoDB_LIBRARIES)

  mark_as_advanced(MongoDB_INCLUDE_DIR MongoDB_LIBRARIES)
endif(NOT MongoDB_INCLUDE_DIR AND NOT MongoDB_LIBRARIES)

################################################################################
## Threads and boost libraries                                                ##
################################################################################
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS thread regex system REQUIRED)

################################################################################
## Configure includes                                                         ##
################################################################################
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${MongoDB_INCLUDE_DIR})
