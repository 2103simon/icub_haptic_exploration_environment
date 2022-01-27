/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

#ifndef GAZEBOYARP_SKIN_HH
#define GAZEBOYARP_SKIN_HH

// gazebo
#include <gazebo/common/Plugin.hh>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

// icub-main
#include <iCub/skinDynLib/common.h>
#include <iCub/skinDynLib/skinContactList.h>

// std
#include <string>
#include <unordered_map>
#include <vector>
using namespace std;

typedef std::unordered_map<std::string, gazebo::physics::LinkPtr> linksMap;

enum class linkNumberEnum;

struct ContactSensor
{
  // Some of these may be useful for future implementations
  // of the Skin plugin
  std::string sensorName;
  std::string collisionName;
  gazebo::physics::LinkPtr parentLink;
  gazebo::sensors::ContactSensorPtr sensor;

  // Required to reconstrct
  // the output of the skinManager
  iCub::skinDynLib::BodyPart bodyPart;
  linkNumberEnum linkNumber;

  iCub::skinDynLib::SkinPart skinPart;

  // In this simplificative simulation
  // each contact is associated with
  // one taxel only. Taxels IDs are used
  // to distinguish which finger tip was
  // involved in the contact.
  unsigned int taxelId;
};

enum class linkNumberEnum
{
  // Fingertips and palm are considered in this implementation
  HAND = 6
};

namespace gazebo
{
  /// \class GazeboYarpSkin
  ///
  class GazeboYarpSkin : public ModelPlugin
  {
  public:
    /**
	 * Constructor.
	 */
    GazeboYarpSkin();

    ~GazeboYarpSkin();

    /**
	 * Store pointer to the model,
	 * and connect to the World update event of Gazebo.
	 */
    void Load(gazebo::physics::ModelPtr, sdf::ElementPtr);

  private:
    /**
	 * Instance of yarp::os::Network
	 */
    yarp::os::Network m_yarp;

    /**
	 * Buffered port where skinContactList is sent
	 */
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> m_portSkin;

    /**
	 * Parameters of the plugin
	 */
    yarp::os::Property m_parameters;

    /**
	 * Driver required to access the IFrameTransform interface
	 */
    yarp::dev::PolyDriver m_drvTransformClient;

    /**
	 * Pointer to the IFrameTransform view
	 */
    yarp::dev::IFrameTransform *m_tfClient;

    /**
	 * Gaussian noise generator
	 */
    std::random_device m_rndDev;
    std::mt19937 m_rndGen;
    std::normal_distribution<> m_gaussianGen;
    bool m_noiseEnabled;

    /**
	 * Indicates whether the transform from /inertial
	 * to the robot root frame has been received
	 */
    bool m_robotRootFrameReceived;

    /**
	 * Pointer to the model where the plugin is inserted
	 */
    gazebo::physics::ModelPtr m_model;

    /**
	 * Pointer to the SDF associated to the model
	 */
    sdf::ElementPtr m_sdf;

    /**
	 * Connection to the World update event of Gazebo
	 */
    gazebo::event::ConnectionPtr m_worldUpdateConnection;

    /**	
	 * Transformation from inertial to the root link of the robot
	 */
    ignition::math::Pose3d m_inertialToRobot;

    /**
	 * List of ContactSensor(s)
	 */
    std::vector<ContactSensor> m_contactSensors;

    /**
	 * Map between links local names and gazebo::physics::LinkPtr(s)
	 */
    std::unordered_map<std::string, gazebo::physics::LinkPtr> m_linksMap;
    std::vector<std::string> linksLocalNames;

    /**
	 * String indicating which hand is considered, left or right
	 */
    std::string m_whichHand;

    /**
	 * Name of the robot
	 */
    std::string m_robotName;

    /**
	 * Name of source frame requried to retrieve the pose of the robot
	 */
    std::string m_robotSourceFrameName;

    /**
	 * Name of target frame required to retrieve the pose of the robot
	 */
    std::string m_robotTargetFrameName;

    /**
	 * Name of the local port used to instantiate the driver
	 * for the torso encoders
	 */
    std::string m_torsoControlBoardLocalPort;

    /**
	 * Name of the local port used to instantiate the driver
	 * for the arm encoders
	 */
    std::string m_armControlBoardLocalPort;

    /**
	 * Name of the local port used to instantiate the driver
	 * for FrameTransformClient
	 */
    std::string m_transformClientLocalPort;

    /**
	 * Name of the output port
	 */
    std::string m_outputPortName;

    /**
	 * Load a parameter from the SDF.
	 */
    template <typename T>
    bool LoadParam(const std::string &name, T &param);

    /*
	 * Retrieve the pose of the robot root frame that is published
	 * in the FrameTransformServer.
	 */
    bool RetrieveRobotRootFrame(ignition::math::Pose3d &pose);

    /**
	 * Retrieve a links given their local, i.e. not scoped, names.
	 * A local name is supposed to be unique within the model.
	 */
    bool RetrieveLinksFromLocalNames(const std::vector<std::string> &linksLocalNames,
                                     linksMap &map);

    /**
	 * Configure the gazebo contact sensor corresponding to the link
	 * with local name linkLocalName.
	 */
    bool ConfigureGazeboContactSensor(const std::string &linkLocalName,
                                      ContactSensor &sensor);
    /**
	 * Configure all the contacts sensors attached to the links
	 * listed in the .ini configuration file.
	 */
    bool ConfigureAllContactSensors();

    /**
	 *
	 */
    void OnWorldUpdate();
  };

  /**
      * taxel placement finger (MK1)
      */
  /*
  unordered_map<string, vector<vector<double>>> taxel_placement_finger = {
      {"r_ail3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},         // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},       // y-coordinates
        {-2.1613e-3, -5.6631e-3, -5.6631e-3, -2.1613e-3, -2.4749e-3, -3.5e-3, 0, 3.5e-3, 2.4749e-3, 2.1613e-3, 5.6631e-3, 2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                       // forces t-1
      {"r_mil3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},         // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},       // y-coordinates
        {-2.1613e-3, -5.6631e-3, -5.6631e-3, -2.1613e-3, -2.4749e-3, -3.5e-3, 0, 3.5e-3, 2.4749e-3, 2.1613e-3, 5.6631e-3, 2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                       // forces t-1
      {"r_ril3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},         // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},       // y-coordinates
        {-2.1613e-3, -5.6631e-3, -5.6631e-3, -2.1613e-3, -2.4749e-3, -3.5e-3, 0, 3.5e-3, 2.4749e-3, 2.1613e-3, 5.6631e-3, 2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                       // forces t-1
      {"r_lil3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},         // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},       // y-coordinates
        {-2.1613e-3, -5.6631e-3, -5.6631e-3, -2.1613e-3, -2.4749e-3, -3.5e-3, 0, 3.5e-3, 2.4749e-3, 2.1613e-3, 5.6631e-3, 2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                       // forces t-1
      {"r_tl4",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},         // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},       // y-coordinates
        {-2.1613e-3, -5.6631e-3, -5.6631e-3, -2.1613e-3, -2.4749e-3, -3.5e-3, 0, 3.5e-3, 2.4749e-3, 2.1613e-3, 5.6631e-3, 2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                       // forces t-1

      {"l_ail3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},        // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},      // y-coordinates
        {2.1613e-3, 5.6631e-3, 5.6631e-3, 2.1613e-3, 2.4749e-3, 3.5e-3, 0, -3.5e-3, -2.4749e-3, -2.1613e-3, -5.6631e-3, -2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                      // forces t-1
      {"l_mil3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},        // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},      // y-coordinates
        {2.1613e-3, 5.6631e-3, 5.6631e-3, 2.1613e-3, 2.4749e-3, 3.5e-3, 0, -3.5e-3, -2.4749e-3, -2.1613e-3, -5.6631e-3, -2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                      // forces t-1
      {"l_ril3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},        // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},      // y-coordinates
        {2.1613e-3, 5.6631e-3, 5.6631e-3, 2.1613e-3, 2.4749e-3, 3.5e-3, 0, -3.5e-3, -2.4749e-3, -2.1613e-3, -5.6631e-3, -2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                      // forces t-1
      {"l_lil3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},        // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},      // y-coordinates
        {2.1613e-3, 5.6631e-3, 5.6631e-3, 2.1613e-3, 2.4749e-3, 3.5e-3, 0, -3.5e-3, -2.4749e-3, -2.1613e-3, -5.6631e-3, -2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}},                                                                                      // forces t-1
      {"l_tl4",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {7.75e-3, 7.75e-3, 13.25e-3, 13.25e-3, 16.75e-3, 16.75e-3, 17.5e-3, 16.75e-3, 16.75e-3, 13.25e-3, 13.25e-3, 7.75e-3},        // x-coordinates
        {6.6574e-3, 4.1145e-3, 4.1145e-3, 6.6574e-3, 3.5e-3, 4.1145e-3, 0, 4.1145e-3, 3.4e-3, 6.6574e-3, 4.1145e-3, 6.6574e-3},      // y-coordinates
        {2.1613e-3, 5.6631e-3, 5.6631e-3, 2.1613e-3, 2.4749e-3, 3.5e-3, 0, -3.5e-3, -2.4749e-3, -2.1613e-3, -5.6631e-3, -2.1613e-3}, // z-coordinates
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}}                                                                                       // forces t-1
  };

  // thresholds for hand and palm are different due to differnent sensor types
  // variables might get arrays with entries for every pixel? -> fine tuning
  double dist_th_palm = 8e-3;
  double f_max_palm = 4.0;
  double sigma_max_palm = dist_th_palm / 2;
  double k_exp_palm = log(0.01) / f_max_palm;
  double m_lin_palm = sigma_max_palm / f_max_palm;
  double b_lin_palm = 0;
  double force_th_palm = 0;
  double delta_force_th_palm = 1e-4;

  double dist_th_finger = 5e-3;
  double f_max_finger = 4.0;
  double sigma_max_finger = dist_th_finger / 2;
  double k_exp_finger = log(0.001) / f_max_finger;
  double m_lin_finger = sigma_max_finger / f_max_finger;
  double b_lin_finger = 0;
  double force_th_finger = 0;
  double delta_force_th_finger = 1e-4;

  double sigma;

  int number_spikes;
  double number_spikes_max = 100;
  double k_exp_spikes = log(0.1) / number_spikes_max * 10000;
  double m_spikes = 100 / 0.01;
  double b_spikes = 0;

  double force_tax;
  size_t number_increments_lengths = 101;
  */
  /**
      * taxel placement finger (MK2 design)
      */
  unordered_map<string, vector<vector<double>>> taxel_placement_finger = {
      {"r_hand_index_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {-3.25e-3, 0.0, 3.25e-3, 6.5e-3, -6.5e-3, 3.25e-3, 0.0, -3.25e-3, -3.25e-3, 0.0, 3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"r_hand_middle_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {-3.25e-3, 0.0, 3.25e-3, 6.5e-3, -6.5e-3, 3.25e-3, 0.0, -3.25e-3, -3.25e-3, 0.0, 3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"r_hand_ring_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {-3.25e-3, 0.0, 3.25e-3, 6.5e-3, -6.5e-3, 3.25e-3, 0.0, -3.25e-3, -3.25e-3, 0.0, 3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"r_hand_little_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {-3.25e-3, 0.0, 3.25e-3, 6.5e-3, -6.5e-3, 3.25e-3, 0.0, -3.25e-3, -3.25e-3, 0.0, 3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"r_hand_thumb_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {-3.25e-3, 0.0, 3.25e-3, 6.5e-3, -6.5e-3, 3.25e-3, 0.0, -3.25e-3, -3.25e-3, 0.0, 3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1

      {"l_hand_index_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {3.25e-3, 0.0, -3.25e-3, -6.5e-3, 6.5e-3, -3.25e-3, 0.0, 3.25e-3, 3.25e-3, 0.0, -3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"l_hand_middle_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {3.25e-3, 0.0, -3.25e-3, -6.5e-3, 6.5e-3, -3.25e-3, 0.0, 3.25e-3, 3.25e-3, 0.0, -3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"l_hand_ring_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {3.25e-3, 0.0, -3.25e-3, -6.5e-3, 6.5e-3, -3.25e-3, 0.0, 3.25e-3, 3.25e-3, 0.0, -3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"l_hand_little_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {3.25e-3, 0.0, -3.25e-3, -6.5e-3, 6.5e-3, -3.25e-3, 0.0, 3.25e-3, 3.25e-3, 0.0, -3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
      {"l_hand_thumb_3",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        {12e-3, 12e-3, 12e-3, 13.5e-3, 13.5e-3, 15e-3, 15e-3, 15e-3, 19.625e-3, 21.25e-3, 19.625e-3, 24.5e-3},                // x-coordinates
        {3.25e-3, 6.5e-3, 3.25e-3, -1.5e-3, -1.5e-3, 3.25e-3, 6.5e-3, 3.25e-3, 1.625e-3, 3.25e-3, 1.625e-3, 0.0},             // y-coordinates
        {3.25e-3, 0.0, -3.25e-3, -6.5e-3, 6.5e-3, -3.25e-3, 0.0, 3.25e-3, 3.25e-3, 0.0, -3.25e-3, 0.0},                       // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                         // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                       // forces t-1
  };

  unordered_map<string, vector<vector<double>>> taxel_placement_palm = {
      {"r_hand",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47},
        {53.1e-3, 57.6e-3, 53.1e-3, 57.6e-3, 57.6e-3, 56.6e-3, 62.6e-3, 62.6e-3, 62.6e-3, 62.1e-3, 61.1e-3, 60.6e-3, 38.6e-3, 42.6e-3, 47.6e-3, 47.6e-3, 47.6e-3, 52.1e-3, 40.6e-3, 46.1e-3, 51.1e-3, 48.6e-3, 43.6e-3, 36.6e-3, 52.6e-3, 56e-3, 57.6e-3, 57.6e-3, 62.6e-3, 63.1e-3, 62.6e-3, 62.6e-3, 57.6e-3, 62.6e-3, 57.6e-3, 54.6e-3, 37.1e-3, 30.6e-3, 32.1e-3, 27.1e-3, 32.1e-3, 27.6e-3, 37.1e-3, 24.1e-3, 32.1e-3, 37.1e-3, 42.1e-3, 42.1e-3},        // x-coordinates
        {-6.2e-3, -6.2e-3, -1.2e-3, 1.2e-3, -11.2e-3, -16.2e-3, -1.2e-3, -6.2e-3, -11.2e-3, -16.2e-3, -21.2e-3, -25.9e-3, -21.7e-3, -12.2e-3, -6.2e-3, -1.2e-3, -11.7e-3, -17.2e-3, -16.2e-3, -16.2e-3, -20.7e-3, -21.2e-3, -17.7e-3, 4.3e-3, 11.3e-3, 14.3e-3, 19.3e-3, 19.3e-3, 24.3e-3, 14.3e-3, 9.3e-3, 9.3e-3, 4.3e-3, 4.3e-3, 16.3e-3, -12.2e-3, -17.2e-3, -12.2e-3, -14.2e-3, -7.2e-3, -9.2e-3, -7.2e-3, -11.2e-3, -2.2e-3, -2.2e-3, -2.2e-3, -7.2e-3}, //y-coordinates
        {25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3},                                                                                                      // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                                                                                                                                                                      // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                                                                                                                                                                    // forces t-1                                                                                     // forces t-1
      {"l_hand",
       {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47},
        {53.1e-3, 57.6e-3, 53.1e-3, 57.6e-3, 57.6e-3, 56.6e-3, 62.6e-3, 62.6e-3, 62.6e-3, 62.1e-3, 61.1e-3, 60.6e-3, 38.6e-3, 42.6e-3, 47.6e-3, 47.6e-3, 47.6e-3, 52.1e-3, 40.6e-3, 46.1e-3, 51.1e-3, 48.6e-3, 43.6e-3, 36.6e-3, 52.6e-3, 56e-3, 57.6e-3, 57.6e-3, 62.6e-3, 63.1e-3, 62.6e-3, 62.6e-3, 57.6e-3, 62.6e-3, 57.6e-3, 54.6e-3, 37.1e-3, 30.6e-3, 32.1e-3, 27.1e-3, 32.1e-3, 27.6e-3, 37.1e-3, 24.1e-3, 32.1e-3, 37.1e-3, 42.1e-3, 42.1e-3}, // x-coordinates
        {6.2e-3, 6.2e-3, 1.2e-3, -1.2e-3, 11.2e-3, 16.2e-3, 1.2e-3, 6.2e-3, 11.2e-3, 16.2e-3, 21.2e-3, 25.9e-3, 21.7e-3, 12.2e-3, 6.2e-3, 1.2e-3, 11.7e-3, 17.2e-3, 16.2e-3, 16.2e-3, 20.7e-3, 21.2e-3, 17.7e-3, -4.3e-3, -11.3e-3, -14.3e-3, -19.3e-3, -19.3e-3, -24.3e-3, -14.3e-3, -9.3e-3, -9.3e-3, -4.3e-3, -4.3e-3, -16.3e-3, 12.2e-3, 17.2e-3, 12.2e-3, 14.2e-3, 7.2e-3, 9.2e-3, 7.2e-3, 11.2e-3, 2.2e-3, 2.2e-3, 2.2e-3, 7.2e-3},               //y-coordinates
        {25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3, 25e-3},                                                                                               // z-coordinates
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                                                                                                                                                                                               // forces t
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}}},                                                                                                                                                                                             // forces t-1   
  };

  // thresholds for hand and palm are different due to differnent sensor types
  // variables might get arrays with entries for every pixel? -> fine tuning
  double dist_th_palm = 8e-3;
  double f_max_palm = 4.0;
  double sigma_max_palm = dist_th_palm / 2;
  double k_exp_palm = log(0.01) / f_max_palm;
  double m_lin_palm = sigma_max_palm / f_max_palm;
  double b_lin_palm = 0;
  // double force_th_palm = 0;
  double delta_force_th_palm = 0;
  bool init_finger = true;

  double dist_th_finger = 5e-3;
  double f_max_finger = 4.0;
  double sigma_max_finger = dist_th_finger / 2;
  double k_exp_finger = log(0.001) / f_max_finger;
  double m_lin_finger = sigma_max_finger / f_max_finger;
  double b_lin_finger = 0;
  double force_th_finger = 0;
  double delta_force_th_finger = 0;
  bool init_palm = true;

  double sigma;

  int number_spikes;
  double number_spikes_max = 100;
  double k_exp_spikes = log(0.1) / number_spikes_max * 10000;
  double m_spikes = 100 / 0.01;
  double b_spikes = 0;

  double force_tax;
  double force_th = 0.01;
  size_t number_increments_lengths = 101;

  // variables for force distribution
  bool calc_lin = false;
  bool calc_exp = true;
  bool calc_sig = false;

  // event or sample based
  bool event_driven = false;
  // relation between number of spikes and delta force
  bool spikes_lin = true; // true = lin, false = exp

} // namespace gazebo

namespace gazebo
{
  template <typename T>
  bool GazeboYarpSkin::LoadParam(const std::string &name,
                                 T &param)
  {
    // Check if the element exists
    if (!(m_sdf->HasElement(name)))
    {
      yError() << "GazeboYarpSkin::Load error:"
               << "cannot find parameter"
               << name
               << "for the"
               << m_whichHand
               << "hand";
      return false;
    }

    // Get the associated parameter
    sdf::ParamPtr paramPtr = m_sdf->GetElement(name)->GetValue();

    // Check if the value can be interpreted
    // as the required type
    if (!paramPtr->Get<T>(param))
    {
      yError() << "GazeboYarpSkin::Load error:"
               << "parameter"
               << name
               << "for the"
               << m_whichHand
               << "hand";
      return false;
    }

    return true;
  }
} // namespace gazebo
#endif
