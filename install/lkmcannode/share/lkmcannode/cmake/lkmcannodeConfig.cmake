# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lkmcannode_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lkmcannode_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lkmcannode_FOUND FALSE)
  elseif(NOT lkmcannode_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lkmcannode_FOUND FALSE)
  endif()
  return()
endif()
set(_lkmcannode_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lkmcannode_FIND_QUIETLY)
  message(STATUS "Found lkmcannode: 0.0.0 (${lkmcannode_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lkmcannode' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${lkmcannode_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lkmcannode_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${lkmcannode_DIR}/${_extra}")
endforeach()
