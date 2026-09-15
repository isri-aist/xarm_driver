find_path(XARM_INCLUDE_DIR NAMES xarm/wrapper/xarm_api.h)

find_library(XARM_LIBRARY NAMES xarm)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  xarm REQUIRED_VARS XARM_INCLUDE_DIR XARM_LIBRARY
)

if(xarm_FOUND AND NOT TARGET xarm_sdk)
  add_library(xarm_sdk UNKNOWN IMPORTED)

  set_target_properties(
    xarm_sdk PROPERTIES IMPORTED_LOCATION "${XARM_LIBRARY}"
                        INTERFACE_INCLUDE_DIRECTORIES "${XARM_INCLUDE_DIR}"
  )
endif()

mark_as_advanced(XARM_INCLUDE_DIR XARM_LIBRARY)
