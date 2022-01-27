
/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia iCub Facility
 * Authors: see AUTHORS file.
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 */

// gazebo
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo/physics/physics.hh>

// ignition
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// GazeboYarpPlugins
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/ConfHelpers.hh>

// yarp
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Searchable.h>
#include <yarp/math/FrameTransform.h>

// icub-main
#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/dynContact.h>

// boost
#include <boost/bind.hpp>

// std
#include <numeric>

#include "Skin.hh"

using namespace yarp::math;

GZ_REGISTER_MODEL_PLUGIN(gazebo::GazeboYarpSkin)

namespace gazebo
{

    GazeboYarpSkin::GazeboYarpSkin() : m_rndGen(m_rndDev()) {}

    GazeboYarpSkin::~GazeboYarpSkin()
    {
        // Close the port
        m_portSkin.close();

        // Close the drivers
        m_drvTransformClient.close();
    }

    void GazeboYarpSkin::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Store pointer to the model
        m_model = _parent;

        // Store point to the SDF associated to the model
        m_sdf = _sdf;

        // Check yarp network availability
        if (!m_yarp.checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
        {
            yError() << "GazeboYarpSkin::Load error:"
                     << "yarp network does not seem to be available, is the yarpserver running?";
            return;
        }

        // Get .ini configuration file from plugin sdf
        bool ok = GazeboYarpPlugins::loadConfigModelPlugin(_parent, _sdf, m_parameters);
        if (!ok)
        {
            yError() << "GazeboYarpSkin::Load error:"
                     << "error loading .ini configuration from plugin SDF";
            return;
        }

        // Get hand type
        yarp::os::Value &whichHandValue = m_parameters.find("whichHand");
        if (whichHandValue.isNull())
        {
            yError() << "GazeboYarpSkin::ConfigureAllContactSensors error:"
                     << "configuration parameter 'whichHand' not found.";
            return;
        }

        m_whichHand = whichHandValue.asString();

        // Get robot name from the SDF
        if (!LoadParam<std::string>("robotName", m_robotName))
            return;

        // Get source frame name required to retrieve the pose of the robot
        if (!LoadParam<std::string>("robotSourceFrameName", m_robotSourceFrameName))
            return;

        // Get target frame name required to retrieve the pose of the robot
        if (!LoadParam<std::string>("robotTargeFrameName", m_robotTargetFrameName))
            return;

        if (!LoadParam<std::string>("transformClientLocalPort", m_transformClientLocalPort))
            return;

        // Get the output port name
        if (!LoadParam<std::string>("outputPortName", m_outputPortName))
            return;

        // Get the noiseEnabled flag
        if (!LoadParam<bool>("enableNoise", m_noiseEnabled))
            return;

        // Configure gaussian random generator if required
        if (m_noiseEnabled)
        {
            // Load mean and standard deviation for gaussian noise
            double mean;
            double std;
            if (!LoadParam<double>("noiseMean", mean))
                return;
            if (!LoadParam<double>("noiseStd", std))
                return;

            // Configure the gaussian random rumber generator
            std::normal_distribution<>::param_type params(mean, std);
            m_gaussianGen.param(params);
        }

        // Prepare properties for the FrameTransformClient
        yarp::os::Property propTfClient;
        propTfClient.put("device", "transformClient");
        propTfClient.put("local", m_transformClientLocalPort);
        propTfClient.put("remote", "/transformServer");

        // try to open the driver
        ok = m_drvTransformClient.open(propTfClient);
        if (!ok)
        {
            yError() << "GazeboYarpSkin::Load error:"
                     << "unable to open the FrameTransformClient driver for the"
                     << m_whichHand
                     << "arm.";
            return;
        }

        // Try to retrieve the view
        ok = m_drvTransformClient.view(m_tfClient);
        if (!ok || m_tfClient == 0)
        {
            yError() << "GazeboYarpSkin::Load error:"
                     << "unable to retrieve the FrameTransformClient view for the"
                     << m_whichHand
                     << "arm.";
            return;
        }

        // Set default value
        m_robotRootFrameReceived = false;

        // Configure all the contact sensors
        ok = ConfigureAllContactSensors();
        if (!ok)
        {
            return;
        }

        // Open skin port
        ok = m_portSkin.open(m_outputPortName); //"/" + m_whichHand + "_hand/skinManager/skin_events:o"
        if (!ok)
        {
            yError() << "GazeboYarpSkin::Load error:"
                     << "cannot open port /skinManager/skin_events:o";
            return;
        }

        // listen to the update event
        auto worldUpdateBind = boost::bind(&GazeboYarpSkin::OnWorldUpdate, this);
        m_worldUpdateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(worldUpdateBind);
    }

    bool GazeboYarpSkin::RetrieveRobotRootFrame(ignition::math::Pose3d &pose)
    {
        // Get the pose of the root frame of the robot
        // TODO: get source and target from configuration file
        yarp::sig::Matrix inertialToRobot(4, 4);
        std::string source = m_robotSourceFrameName; //"/inertial";
        std::string target = m_robotTargetFrameName; //"/iCub/frame";

        // Get the transform
        if (!m_tfClient->getTransform(target, source, inertialToRobot))
            return false;

        // Convert to ignition::math::Pose3d
        yarp::math::FrameTransform frame;
        frame.fromMatrix(inertialToRobot);

        yarp::math::FrameTransform::Translation_t &pos = frame.translation;
        yarp::math::Quaternion &quat = frame.rotation;
        pose = ignition::math::Pose3d(pos.tX, pos.tY, pos.tZ,
                                      quat.w(), quat.x(), quat.y(), quat.z());
        return true;
    }

    bool GazeboYarpSkin::RetrieveLinksFromLocalNames(const std::vector<std::string> &linksLocalNames,
                                                     linksMap &map)
    {
        // Get all the links within the robot
        const gazebo::physics::Link_V &links = m_model->GetLinks();

        // Search for the given links
        // Lots of redundancy here, room for improvements...
        //
        for (size_t i = 0; i < links.size(); i++)
        {
            // Get the scoped name of the current link
            std::string currentLinkScopedName = links[i]->GetScopedName();

            for (size_t j = 0; j < linksLocalNames.size(); j++)
            {
                // Check if the ending of the name of the current link corresponds to
                // that of one of the given links
                std::string linkNameScopedEnding = "::" + linksLocalNames[j];
                if (GazeboYarpPlugins::hasEnding(currentLinkScopedName, linkNameScopedEnding))
                {
                    // Store the link into the map
                    map[linksLocalNames[j]] = links[i];

                    break;
                }
            }
        }

        // Return false if not all the links were found
        if (map.size() != linksLocalNames.size())
            return false;

        return true;
    }

    bool GazeboYarpSkin::ConfigureGazeboContactSensor(const std::string &linkLocalName,
                                                      ContactSensor &sensor)
    {
        // Retrieve the link
        sensor.parentLink = m_linksMap[linkLocalName];
        if (sensor.parentLink == NULL)
            return false;

        // Check if this link contains any sensor
        size_t nSensors = sensor.parentLink->GetSensorCount();
        // std::cout << nSensors << std::endl;

        if (nSensors <= 0)
            return false;

        // One contact sensor per link is expected
        // however many sensors can be attached to the same link
        // hence it is required to search for the contact sensor
        sdf::ElementPtr modelSdf = sensor.parentLink->GetSDF();
        sdf::ElementPtr child;

        bool foundContactSensor = false;
        for (child = modelSdf->GetElement("sensor");
             child != sdf::ElementPtr(nullptr);
             child = child->GetNextElement("sensor"))
        {
            if ((child->GetAttribute("type")->GetAsString()) == "contact")
            {
                // check if a child "contact" tag exists since it is not required
                // by default
                if (child->HasElement("contact"))
                {
                    foundContactSensor = true;
                    break;
                }
            }
        }
        if (!foundContactSensor)
            return false;

        // Extract scoped sensor name
        std::string localSensorName = child->GetAttribute("name")->GetAsString();
        std::vector<std::string> scopedNameList = m_model->SensorScopedName(localSensorName);
        sensor.sensorName = std::accumulate(std::begin(scopedNameList),
                                            std::end(scopedNameList),
                                            sensor.sensorName);
        // Extract collision name
        sdf::ElementPtr contact = child->GetElement("contact");
        sensor.collisionName = contact->GetElement("collision")->GetValue()->GetAsString();

        // Get the sensor from the sensor manager
        gazebo::sensors::SensorManager *sensorMgr = gazebo::sensors::SensorManager::Instance();
        if (!sensorMgr->SensorsInitialized())
            return false;

        gazebo::sensors::SensorPtr genericPtr = sensorMgr->GetSensor(sensor.sensorName);
        sensor.sensor = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(genericPtr);
        if (sensor.sensor == nullptr)
            return false;

        // Activate sensor
        sensor.sensor->SetActive(true);

        return true;
    }

    bool GazeboYarpSkin::ConfigureAllContactSensors()
    {
        // Configure skinManager parameters
        iCub::skinDynLib::BodyPart bodyPart;
        iCub::skinDynLib::SkinPart skinPart;
        linkNumberEnum linkNumber = linkNumberEnum::HAND;

        if (m_whichHand == "right")
        {
            bodyPart = iCub::skinDynLib::BodyPart::RIGHT_ARM;
            skinPart = iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND;
        }
        else
        {
            bodyPart = iCub::skinDynLib::BodyPart::LEFT_ARM;
            skinPart = iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND;
        }

        // Get local link names of the finger tips
        yarp::os::Bottle linksLocalNamesBottle = m_parameters.findGroup("linkNames");

        if (linksLocalNamesBottle.isNull())
        {
            yError() << "GazeboYarpSkin::ConfigureAllContactSensors error:"
                     << "configuration parameter 'linkNames' not found.";
            return false;
        }

        int numberOfLinks = linksLocalNamesBottle.size() - 1;
        for (size_t i = 0; i < numberOfLinks; i++)
            linksLocalNames.push_back(linksLocalNamesBottle.get(i + 1).asString().c_str());
        // std::cout << "numberOfLinks:" << numberOfLinks << std::endl;

        // Get taxel ids associated to each collision
        yarp::os::Bottle taxelIdsBottle = m_parameters.findGroup("taxelIds");
        std::vector<unsigned int> taxelIds;
        if (taxelIdsBottle.isNull())
        {
            yError() << "GazeboYarpSkin::ConfigureAllContactSensors error:"
                     << "configuration parameter 'taxelIds' not found.";
            return false;
        }

        for (size_t i = 0; i < numberOfLinks; i++)
            taxelIds.push_back(taxelIdsBottle.get((i + 1)).asInt32());

        // Retrieve the links from the model
        RetrieveLinksFromLocalNames(linksLocalNames, m_linksMap);

        // Configure contact sensors
        std::string linkLocalName;
        bool ok;
        for (size_t i = 0; i < numberOfLinks; i++)
        {
            ContactSensor sensor;

            // Copy skinManager parameters
            sensor.bodyPart = bodyPart;
            sensor.linkNumber = linkNumber;
            sensor.skinPart = skinPart;
            sensor.taxelId = taxelIds[i];

            // Configure Gazebo contact sensor
            linkLocalName = linksLocalNames[i];

            ok = ConfigureGazeboContactSensor(linkLocalName, sensor);
            if (!ok)
            {
                yError() << "GazeboYarpSkin::ConfigureAllContactSensors error:"
                         << "cannot configure link"
                         << linkLocalName;
                return false;
            }

            // Add sensor to the list
            m_contactSensors.push_back(sensor);
        }

        return true;
    }

    void GazeboYarpSkin::OnWorldUpdate()
    {
        // set publish_data to false to prevent data output
        // from non taxels
        bool publish_data = false;

        // The first time this is executed and
        // until m_robotRootFrameReceived is true
        // the transform between m_robotSourceFrameName
        // and m_robotTargetFrameName is retrieved
        if (!m_robotRootFrameReceived)
        {
            m_robotRootFrameReceived = RetrieveRobotRootFrame(m_inertialToRobot);
            if (!m_robotRootFrameReceived)
            {
                yError() << "GazeboYarpSkin:OnWorldUpdate error:"
                         << "unable to get the pose of the root link of the robot.";
                return;
            }
        }

        // Process contacts for each contact sensor
        iCub::skinDynLib::skinContactList &skinContactList = m_portSkin.prepare();
        skinContactList.clear();

        // every fingertip is taken as a single contact sensor
        // m_contactSensors.size() = 6 (5 finger + palm)
        std::cout << "switching hand\n"
                  << std::endl;
        for (size_t i = 0; i < m_contactSensors.size(); i++)
        {
            ContactSensor &sensor = m_contactSensors[i];
            msgs::Contacts contacts = sensor.sensor->Contacts();
            unsigned int collision_count = sensor.sensor->GetCollisionCount();
            // Get the number of collisions that the sensor is observing. (https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ContactSensor.html#ae0ef4f0b0232cc0d4ab6c9a1d42b9c2d)
            std::cout << "contact_count: " << collision_count << std::endl; // returns always 1
            for (size_t collision_counter = 0; collision_counter < collision_count; collision_counter++)
            {
                std::string collision_name = sensor.sensor->GetCollisionName(collision_counter);
                std::cout << "collision_name: " << collision_name << std::endl;
                unsigned int collision_contact_count = sensor.sensor->GetCollisionContactCount(collision_name);
                // Return the number of contacts for an observed collision. (https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ContactSensor.html#afabffb9fe7a9a71bdce58afed1227cf2)
                std::cout << "collision_contact_count: " << collision_contact_count << std::endl; // returns always more than 1
            }

            // Skip to next sensor if no contacts
            if (contacts.contact_size() == 0)
            {
                std::cout << "skipping sensor nr: " << i << " of: " << m_contactSensors.size() << std::endl;
                continue;
            }
            std::cout << "detected " << contacts.contact_size() << " contacts at sensor nr: " << i << std::endl;
            // get pointer to the link
            gazebo::physics::LinkPtr link_name_ptr = m_model->GetLink("iCub::iCub::" + linksLocalNames[i]);
            std::cout << "linkLocalNames[i]: " << linksLocalNames[i] << std::endl;

            // get link coordinates by using the pointer to the model
            ignition::math::Pose3d link_coord;
            link_coord = link_name_ptr->WorldPose();
            std::cout << "linkCoordinates:" << link_coord << std::endl;

            // every contact is treated independently
            // for (size_t k = 0; k < contacts.contact_size(); k++)
            std::cout << "contacts.contact_size(): " << contacts.contact_size() << std::endl;
            for (size_t j = 0; j < contacts.contact_size(); j++)
            {
                std::cout << "contacts.contact(j).position_size(): " << contacts.contact(j).position_size() << std::endl;
                for (size_t k = 0; k < contacts.contact(j).position_size(); k++)
                {
                    std::cout << "contacts.contact(j).position(k): " << contacts.contact(j).position(k).x() << " " << contacts.contact(j).position(k).y() << " " << contacts.contact(j).position(k).z() << std::endl;
                }
            }
            init_palm = true;
            init_finger = true;

            // TODO change to the double loop from above contact_size + position
            size_t j = 0;
            for (size_t k = 0; k < contacts.contact(j).position_size(); k++)
            {
                // initialising taxelID
                uint taxelId_link = 0;

                // std::cout << "contacts.contact_size(): " << contacts.contact_size() << std::endl;
                // std::cout << "contacts.contact(j).position_size(): " << contacts.contact(j).position_size() << std::endl;
                // std::cout << "k: " << k << std::endl;
                // Extract position from message
                auto position = contacts.contact(j).position(k);
                // Convert to a pose with no rotation
                ignition::math::Pose3d point(position.x(), position.y(), position.z(),
                                             0, 0, 0);
                // std::cout << "point: " << point << std::endl;

                // calculate the contact position at the fingertip
                ignition::math::Pose3d cont_tip = point - link_coord;
                // std::cout << "contact in Link Coordinates: \n" << cont_tip << std::endl;

                // normal
                // TODO transpose in respect to fingertip; yet in world frame!!!
                yarp::sig::Vector normVector(3, 0.0);
                normVector[0] = contacts.contact(j).normal(k).x();
                normVector[1] = contacts.contact(j).normal(k).y();
                normVector[2] = contacts.contact(j).normal(k).z();

                // force
                const gazebo::msgs::JointWrench &wrench = contacts.contact(j).wrench(k);
                yarp::sig::Vector forceVector(3, 0.0);
                forceVector[0] = wrench.body_1_wrench().force().x();
                forceVector[1] = wrench.body_1_wrench().force().y();
                forceVector[2] = wrench.body_1_wrench().force().z();

                // calc geo. resulting force // add normal force as new option and default
                auto force_res = sqrt(pow(forceVector[0], 2) + pow(forceVector[1], 2) + pow(forceVector[2], 2));
                // std::cout << "force_res: " << force_res << std::endl;

                // initialize for gaussian (needed cause otherwise variables unkown down the code TODO improve)
                double max_val_gau = 0;
                double f_max = 0;
                double m_lin = 0;
                double b_lin = 0;
                double sigma_max = 0;
                double k_exp = 0;
                double dist_th = 0;
                auto taxel_placement = taxel_placement_finger.at(linksLocalNames[i]);

                // update init calculation with variable for palm or finger
                if (linksLocalNames[i] == "r_hand" || linksLocalNames[i] == "l_hand")
                {
                    f_max = f_max_palm;
                    m_lin = m_lin_palm;
                    b_lin = b_lin_palm;
                    sigma_max = sigma_max_palm;
                    k_exp = k_exp_palm;
                    dist_th = dist_th_palm;
                    auto taxel_placement = taxel_placement_palm.at(linksLocalNames[i]);
                    if (init_palm)
                    {
                        std::cout << "Init for palm!" << std::endl;
                        taxel_placement_palm.at(linksLocalNames[i])[4] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                        init_palm = false;
                    }
                }
                else
                {
                    f_max = f_max_finger;
                    m_lin = m_lin_finger;
                    b_lin = b_lin_finger;
                    sigma_max = sigma_max_finger;
                    k_exp = k_exp_finger;
                    dist_th = dist_th_finger;
                    auto taxel_placement = taxel_placement_finger.at(linksLocalNames[i]);
                    if (init_finger)
                    {
                        std::cout << "Init for finger!" << std::endl;
                        taxel_placement_finger.at(linksLocalNames[i])[4] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    }
                }
                // std::cout << "calc. sigma" << std::endl;
                if (calc_lin)
                {
                    if (force_res < f_max)
                    {
                        sigma = m_lin * force_res + b_lin;
                    }
                    else
                    {
                        sigma = sigma_max;
                    }
                    // std::cout << "sigma: " << sigma << std::endl;
                }
                else if (calc_exp)
                {
                    sigma = sigma_max - sigma_max * exp(k_exp * force_res);
                    // std::cout << "sigma: " << sigma << std::endl;
                }
                else if (calc_sig)
                {
                    sigma = (1 / (1 + exp(-(2 * force_res - f_max)))) * sigma_max;
                    // std::cout << "sigma: " << sigma << std::endl;
                }
                // safe max. val. of gaussian for normalization
                for (size_t l = 0; l < number_increments_lengths; l++)
                {
                    double calc_lengths = -dist_th + l * (2 * dist_th / number_increments_lengths);
                    // TODO check if assigning a new variable is faster then to do the computation twice (memory vs. speed)
                    if (exp(-(pow(calc_lengths, 2) / pow(2 * sigma, 2))) > max_val_gau)
                    {
                        max_val_gau = exp(-(pow(calc_lengths, 2) / pow(2 * sigma, 2)));
                    }
                }

                // init
                yarp::sig::Vector diffVector(3, 0.0);
                diffVector = {0.0, 0.0, 0.0};

                // calc force per taxel
                for (size_t m = 0; m < taxel_placement[0].size(); m++)
                {
                    // diff between contact position and taxel position
                    // yarp::sig::Vector diffVector(3, 0.0);
                    diffVector[0] = taxel_placement[1][m] - cont_tip.Pos().X();
                    diffVector[1] = taxel_placement[2][m] - cont_tip.Pos().Y();
                    diffVector[2] = taxel_placement[3][m] - cont_tip.Pos().Z();
                    ;

                    // calc force according scaled gaussian
                    force_tax = force_res * (exp(-((pow(taxel_placement[1][m] - cont_tip.Pos().X(), 2) / pow(2 * sigma, 2)) + (pow(taxel_placement[2][m] - cont_tip.Pos().Y(), 2) / pow(2 * sigma, 2)) + (pow(taxel_placement[3][m] - cont_tip.Pos().Z(), 2) / pow(2 * sigma, 2)))) / max_val_gau);

                    // store the highest force value per taxel per fingertip
                    if (force_tax > taxel_placement[4][m])
                    {
                        taxel_placement[4][m] = force_tax;
                    }
                }

                // transmit force per taxel
                for (size_t m = 0; m < taxel_placement[0].size(); m++)
                {
                    taxelId_link = taxel_placement[0][m];
                    // move out of this loop (event driven & conventional)! final force per taxel needs to be known after all contacts per fingertip are calculated
                    if (event_driven)
                    {
                        auto delta_force = taxel_placement[4][m] - taxel_placement[5][m];
                        taxel_placement[5][m] = force_tax;
                        if (delta_force > delta_force_th_palm)
                        {
                            // calc # spikes
                            if (spikes_lin)
                            {
                                number_spikes = m_spikes * delta_force + b_spikes;
                            }
                            else
                            {
                                number_spikes = number_spikes_max - number_spikes_max * exp(k_exp_spikes * delta_force);
                            }
                            // std::cout << "# of spikes: " << number_spikes << std::endl;
                            // std::cout << "force: " << force_tax << " " << "delta force: " << delta_force << " " << "taxelID: " << taxelId_link << std::endl;
                        }
                    }
                    else
                    {
                        if (force_tax > force_th_palm)
                        {
                            // write force_tax to variable at position taxelId_link
                            // taxel_placement[4][m] = taxel_placement[4][m] + force_tax;

                            // old code:
                            // std::cout << "force at taxel: " << force_tax << " " << "taxelID: " << taxelId_link << std::endl;
                            publish_data = true;

                            iCub::skinDynLib::dynContact dynContact(sensor.bodyPart,
                                                                    static_cast<int>(sensor.linkNumber),
                                                                    yarp::sig::Vector(3, 0.0));
                            iCub::skinDynLib::skinContact skinContact(dynContact);
                            skinContact.setSkinPart(sensor.skinPart);
                            skinContact.setGeoCenter(diffVector); // distance from center of taxel to contact position
                            skinContact.setNormalDir(normVector); // only related to force_res
                            skinContact.setForce(forceVector);    // irrelevant, only geo. mean of force is concidered
                            skinContact.setPressure(force_tax);   // instead of pressure

                            // Suppose each contact is due to one taxel only
                            skinContact.setActiveTaxels(1); // in the actual implementaion only 1 works

                            // Set the right taxel id depending on the finger
                            // involved in the contact
                            std::vector<unsigned int> taxelIds;
                            taxelIds.push_back(sensor.taxelId + taxelId_link);
                            skinContact.setTaxelList(taxelIds);

                            // Add contact to the list
                            skinContactList.push_back(skinContact);
                        }
                    }
                }
            }
        }

        // Send data over port in case of contacts
        if (publish_data)
        {
            m_portSkin.write();
        }
    }

} // namespace gazebo
