#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "sfbot_can::sfbot_can" for configuration ""
set_property(TARGET sfbot_can::sfbot_can APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(sfbot_can::sfbot_can PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libsfbot_can.so"
  IMPORTED_SONAME_NOCONFIG "libsfbot_can.so"
  )

list(APPEND _cmake_import_check_targets sfbot_can::sfbot_can )
list(APPEND _cmake_import_check_files_for_sfbot_can::sfbot_can "${_IMPORT_PREFIX}/lib/libsfbot_can.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
