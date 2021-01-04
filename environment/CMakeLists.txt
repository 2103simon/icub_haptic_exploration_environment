##############################################################################
#                                                                            #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

cmake_minimum_required(VERSION 3.5)

project(haptic_exploration)

install (DIRECTORY models/vte_box DESTINATION share/gazebo/models)
install (DIRECTORY models/vte_shelf DESTINATION share/gazebo/models)
install (DIRECTORY models/vte_table DESTINATION share/gazebo/models)
install (DIRECTORY models/vte_mustard DESTINATION share/gazebo/models)

install (FILES worlds/vte_scenario.sdf DESTINATION share/gazebo/worlds)

# Add standard uninstall target
include(AddUninstallTarget)
