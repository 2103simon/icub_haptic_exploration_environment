# Copyright: (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 3.5)

project(icub-gazebo)

# Add models (this are tipically contained in a models subdirectory, 
#             but for the time being we keep them in the main folder to 
#             preserve back-compatibility)

# List the subdirectories (http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory)
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Copy the iCub_manual folder in the build tree
SUBDIRLIST(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/iCub_manual)
foreach(subdir ${subdirs})
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/iCub_manual/${subdir}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/iCub/${subdir})
endforeach()

# Add the model.config just for a limited number of models that are known
# to work in Gazebo, unless the ICUB_MODELS_INSTALL_ALL_GAZEBO_MODELS option
# is enabled
option(ICUB_MODELS_INSTALL_ALL_GAZEBO_MODELS OFF)

# Workaround for https://github.com/robotology/icub-models/issues/39 and
# https://github.com/osrf/gazebo/issues/2728
set(ICUB_MODELS_SDF_VERSION "1.5")
find_package(GAZEBO QUIET)
if(GAZEBO_FOUND AND GAZEBO_VERSION VERSION_GREATER_EQUAL 11.0)
  set(ICUB_MODELS_SDF_VERSION "1.7")
endif()

# Deal with manually generated models
set(GAZEBO_SUPPORTED_MODELS "")
list(APPEND GAZEBO_SUPPORTED_MODELS "iCubGazeboV2_5_hapticmanip")

SUBDIRLIST(ROBOTS_NAMES ${CMAKE_CURRENT_SOURCE_DIR}/iCub_manual/robots)
foreach(ROBOT_NAME ${ROBOTS_NAMES})
  if(ROBOT_NAME IN_LIST GAZEBO_SUPPORTED_MODELS)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/iCub_manual/robots/${ROBOT_NAME}/model.config
                   ${CMAKE_CURRENT_BINARY_DIR}/iCub/robots/${ROBOT_NAME}/model.config
                   @ONLY)
  endif()
endforeach()

# Install the whole iCub directory
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/iCub DESTINATION share)

# Add standard uninstall target
include(AddUninstallTarget)
