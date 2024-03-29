# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_get_node_data_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED get_node_data_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(get_node_data_FOUND FALSE)
  elseif(NOT get_node_data_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(get_node_data_FOUND FALSE)
  endif()
  return()
endif()
set(_get_node_data_CONFIG_INCLUDED TRUE)

# output package information
if(NOT get_node_data_FIND_QUIETLY)
  message(STATUS "Found get_node_data: 0.0.0 (${get_node_data_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'get_node_data' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${get_node_data_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(get_node_data_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${get_node_data_DIR}/${_extra}")
endforeach()
