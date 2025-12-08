#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ur10e_hl_interface::ur10e_hl_interface" for configuration ""
set_property(TARGET ur10e_hl_interface::ur10e_hl_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ur10e_hl_interface::ur10e_hl_interface PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libur10e_hl_interface.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ur10e_hl_interface::ur10e_hl_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_ur10e_hl_interface::ur10e_hl_interface "${_IMPORT_PREFIX}/lib/libur10e_hl_interface.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
