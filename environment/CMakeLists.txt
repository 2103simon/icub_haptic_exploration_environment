##############################################################################
#                                                                            #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

cmake_minimum_required(VERSION 3.5)

project(haptic_exploration)

install (DIRECTORY models/haptic_exploration/he_box DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_shelf DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_table DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_mustard DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_cylinder DESTINATION share/gazebo/models)

install (DIRECTORY models/contour_following/cf_triangle_3d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_triangle_2_5d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_triangle_2d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_circle_3d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_circle_2_5d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_circle_2d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_square_3d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_square_2_5d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_square_2d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_star_3d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_star_2_5d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_star_2d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_infinity_3d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_infinity_2_5d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_infinity_2d DESTINATION share/gazebo/models)

install (FILES worlds/he_scenario.sdf DESTINATION share/gazebo/worlds)

# Add standard uninstall target
include(AddUninstallTarget)
