# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_autonomy_computer_vision_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED autonomy_computer_vision_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(autonomy_computer_vision_FOUND FALSE)
  elseif(NOT autonomy_computer_vision_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(autonomy_computer_vision_FOUND FALSE)
  endif()
  return()
endif()
set(_autonomy_computer_vision_CONFIG_INCLUDED TRUE)

# output package information
if(NOT autonomy_computer_vision_FIND_QUIETLY)
  message(STATUS "Found autonomy_computer_vision: 1.0.0 (${autonomy_computer_vision_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'autonomy_computer_vision' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT autonomy_computer_vision_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(autonomy_computer_vision_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${autonomy_computer_vision_DIR}/${_extra}")
endforeach()
