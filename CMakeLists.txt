#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

cmake_minimum_required(VERSION 3.5)

project(new_model_haptic_exploration
        LANGUAGES CXX
        VERSION 0.0.100)

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Default build type to Release
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
  endif()

add_subdirectory(environment)
add_subdirectory(plugins)
add_subdirectory(robots)


# Add standard uninstall target
include(AddUninstallTarget)

