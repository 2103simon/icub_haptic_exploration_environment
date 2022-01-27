##############################################################################
#                                                                            #
# Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

cmake_minimum_required(VERSION 3.5)

project(haptic_exploration)

# YCB model selection
install (DIRECTORY models/haptic_exploration/he_box DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_shelf DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_table DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_mustard DESTINATION share/gazebo/models)
install (DIRECTORY models/haptic_exploration/he_cylinder DESTINATION share/gazebo/models)

# contour following
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
install (DIRECTORY models/contour_following/cf_line_2_5d DESTINATION share/gazebo/models)
install (DIRECTORY models/contour_following/cf_line_2d DESTINATION share/gazebo/models)

# braille letters
install (DIRECTORY models/braille/br_A DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_B DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_C DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_D DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_E DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_F DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_G DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_H DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_I DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_J DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_K DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_L DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_M DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_N DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_O DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_P DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_Q DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_R DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_S DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_T DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_U DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_V DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_W DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_X DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_Y DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_Z DESTINATION share/gazebo/models)
install (DIRECTORY models/braille/br_Space DESTINATION share/gazebo/models)

# world environment with table
install (FILES worlds/he_scenario.sdf DESTINATION share/gazebo/worlds)

# Add standard uninstall target
include(AddUninstallTarget)
