# - Try to find the Mongo Client Libraries
# Once done this will define
#  mongoclient_FOUND
#  mongoclient_INCLUDE_DIRS
#  mongoclient_LIBRARIES

################################################################################
## Dependencies                                                               ##
################################################################################
find_package(Threads REQUIRED)
find_package(Boost COMPONENTS thread regex system REQUIRED)

################################################################################
## mongoclient_INCLUDE_DIRS                                                   ##
################################################################################
find_path(mongoclient_INCLUDE_DIR
  NAMES mongo/client/dbclient.h
  PATHS /usr/include/
        /usr/local/include/
        /usr/include/mongo/
        /usr/local/include/mongo/
        /opt/mongo/include/
)

set(mongoclient_INCLUDE_DIRS ${mongoclient_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

################################################################################
## mongoclient_LIBRARIES                                                      ##
################################################################################
find_library(mongoclient_LIBRARIES
  NAMES mongoclient
  PATHS /usr/lib
        /usr/lib/mongo
        /usr/local/lib
        /usr/local/lib/mongo
        /opt/mongo/lib
)

################################################################################
## Error handling and advancing                                               ##
################################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(mongoclient DEFAULT_MSG
  mongoclient_LIBRARIES mongoclient_INCLUDE_DIR)

mark_as_advanced(mongoclient_INCLUDE_DIR)
