# - Try to find UIMACPP
# Once done this will define
#  uimacpp_FOUND
#  uimacpp_INCLUDE_DIRS
#  uimacpp_LIBRARIES


################################################################################
## uimacpp_INCLUDE_DIRS                                                       ##
################################################################################
find_path(uimacpp_INCLUDE_DIR
  NAMES uima/api.hpp uima/api.h
  PATHS ENV UIMACPP_HOME
  PATH_SUFFIXES include)

find_path(uimacpp_apr_INCLUDE_DIR
  NAMES apr.hpp apr.h apr-1.hpp apr-1.h
  PATHS ENV UIMACPP_HOME
  PATH_SUFFIXES include/apr-1 include/apr)

set(uimacpp_INCLUDE_DIRS ${uimacpp_INCLUDE_DIR} ${uimacpp_apr_INCLUDE_DIR})

################################################################################
## uimacpp_LIBRARIES                                                          ##
################################################################################
# Find the uima base library.
find_library(uimacpp_LIBRARY
  NAMES uima
  PATHS ENV UIMACPP_HOME
  PATH_SUFFIXES lib
  NO_DEFAULT_PATH
)

# Find all those required libraries.
find_library(apr_LIBRARY NAMES apr apr-1 PATHS $ENV{UIMACPP_HOME}/lib NO_DEFAULT_PATH)
find_library(xercesc_LIBRARY NAMES xerces-c PATHS $ENV{UIMACPP_HOME}/lib NO_DEFAULT_PATH)
find_library(icuuc_LIBRARY NAMES icuuc PATHS $ENV{UIMACPP_HOME}/lib NO_DEFAULT_PATH)
find_library(icuio_LIBRARY NAMES icuio PATHS $ENV{UIMACPP_HOME}/lib NO_DEFAULT_PATH)
find_library(icui18n_LIBRARY NAMES icui18n PATHS $ENV{UIMACPP_HOME}/lib NO_DEFAULT_PATH)
find_library(icudata_LIBRARY NAMES icudata PATHS $ENV{UIMACPP_HOME}/lib NO_DEFAULT_PATH)
find_library(dl_LIBRARY NAMES dl)

set(uimacpp_LIBRARIES ${uimacpp_LIBRARY})
LIST(APPEND uimacpp_LIBRARIES ${apr_LIBRARY})
LIST(APPEND uimacpp_LIBRARIES ${xercesc_LIBRARY})
LIST(APPEND uimacpp_LIBRARIES ${icuuc_LIBRARY})
LIST(APPEND uimacpp_LIBRARIES ${icuio_LIBRARY})
LIST(APPEND uimacpp_LIBRARIES ${icui18n_LIBRARY})
LIST(APPEND uimacpp_LIBRARIES ${icudata_LIBRARY})
LIST(APPEND uimacpp_LIBRARIES ${dl_LIBRARY})

################################################################################
## Error handling and advancing                                               ##
################################################################################
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(uimacpp DEFAULT_MSG
  uimacpp_LIBRARY uimacpp_INCLUDE_DIR
  apr_LIBRARY uimacpp_apr_INCLUDE_DIR
  xercesc_LIBRARY
  icuuc_LIBRARY
  icuio_LIBRARY
  icui18n_LIBRARY
  icudata_LIBRARY
  dl_LIBRARY)

mark_as_advanced(
  uimacpp_INCLUDE_DIR
  uimacpp_apr_INCLUDE_DIR
  apr_LIBRARY
  xercesc_LIBRARY
  icuuc_LIBRARY
  icuio_LIBRARY
  icui18n_LIBRARY
  icudata_LIBRARY
  dl_LIBRARY
)
