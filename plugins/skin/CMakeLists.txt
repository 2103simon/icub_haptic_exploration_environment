#===============================================================================
#
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(PLUGIN_TARGET_NAME gazebo_yarp_skin)

find_package(YARP REQUIRED
              COMPONENTS dev os
)

find_package(GazeboYARPPlugins REQUIRED)

find_package(gazebo REQUIRED)

find_package(ICUB REQUIRED)

set(${PLUGIN_TARGET_NAME}_HDR
    include/gazebo/Skin.hh
)

set(${PLUGIN_TARGET_NAME}_SRC
    src/Skin.cc
)

add_library(${PLUGIN_TARGET_NAME} SHARED ${${PLUGIN_TARGET_NAME}_HDR} ${${PLUGIN_TARGET_NAME}_SRC})

target_include_directories(${PLUGIN_TARGET_NAME}
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/include/gazebo
                           ${GAZEBO_INCLUDE_DIRS}
)

target_link_libraries(${PLUGIN_TARGET_NAME}
                      PRIVATE
                      GazeboYARPPlugins::gazebo_yarp_singleton
		      ICUB::skinDynLib
                      ${GAZEBO_LIBRARIES}
)

install(TARGETS ${PLUGIN_TARGET_NAME}
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
 )

