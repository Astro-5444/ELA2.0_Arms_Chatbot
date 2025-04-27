# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ela2_arms_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ela2_arms_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ela2_arms_FOUND FALSE)
  elseif(NOT ela2_arms_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ela2_arms_FOUND FALSE)
  endif()
  return()
endif()
set(_ela2_arms_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ela2_arms_FIND_QUIETLY)
  message(STATUS "Found ela2_arms: 0.0.0 (${ela2_arms_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ela2_arms' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${ela2_arms_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ela2_arms_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${ela2_arms_DIR}/${_extra}")
endforeach()
