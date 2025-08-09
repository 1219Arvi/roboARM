#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "serial::serial" for configuration ""
set_property(TARGET serial::serial APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(serial::serial PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libserial.so"
  IMPORTED_SONAME_NOCONFIG "libserial.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS serial::serial )
list(APPEND _IMPORT_CHECK_FILES_FOR_serial::serial "${_IMPORT_PREFIX}/lib/libserial.so" )

# Import target "serial::serial_example" for configuration ""
set_property(TARGET serial::serial_example APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(serial::serial_example PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/serial_example"
  )

list(APPEND _IMPORT_CHECK_TARGETS serial::serial_example )
list(APPEND _IMPORT_CHECK_FILES_FOR_serial::serial_example "${_IMPORT_PREFIX}/bin/serial_example" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
