find_path(ROBOTCONTROL_INCLUDE_DIR robotcontrol.h
  HINTS "/usr/include" )

find_library(ROBOTCONTROL_LIBRARY robotcontrol
  HINTS "/usr/lib" )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(robotcontrol  DEFAULT_MSG
  ROBOTCONTROL_LIBRARY ROBOTCONTROL_INCLUDE_DIR)

mark_as_advanced(ROBOTCONTROL_INCLUDE_DIR ROBOTCONTROL_LIBRARY )

set(ROBOTCONTROL_LIBRARIES ${ROBOTCONTROL_LIBRARY} )
set(ROBOTCONTROL_INCLUDE_DIRS ${ROBOTCONTROL_INCLUDE_DIR} )
