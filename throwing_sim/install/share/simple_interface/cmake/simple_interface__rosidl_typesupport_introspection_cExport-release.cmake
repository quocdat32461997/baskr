#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "simple_interface::simple_interface__rosidl_typesupport_introspection_c" for configuration "Release"
set_property(TARGET simple_interface::simple_interface__rosidl_typesupport_introspection_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(simple_interface::simple_interface__rosidl_typesupport_introspection_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsimple_interface__rosidl_typesupport_introspection_c.so"
  IMPORTED_SONAME_RELEASE "libsimple_interface__rosidl_typesupport_introspection_c.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS simple_interface::simple_interface__rosidl_typesupport_introspection_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_simple_interface::simple_interface__rosidl_typesupport_introspection_c "${_IMPORT_PREFIX}/lib/libsimple_interface__rosidl_typesupport_introspection_c.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
