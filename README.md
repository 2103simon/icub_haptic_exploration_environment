# iTouch - Haptic Exploration with iCub in gazebo

This plugin contains the haptic exploration environment, and the [`YCB model dataset`](https://github.com/sea-bass/ycb-tools) and is based on the [`Tactile sensors (aka Skin)`](http://wiki.icub.org/wiki/Tactile_sensors_(aka_Skin)) using the [`contact sensor plugin`](http://gazebosim.org/tutorials?tut=contact_sensor) for [`iCub`](http://wiki.icub.org/wiki/Main_Page). The sensors are limited to the fingertip and the palm.

The skin at the fingertip is not the same as the one at the palm. Both are using conductance-based sensors, but have different properties. For further information about the physical skin, and the read-out, look [`here`](http://wiki.icub.org/images/c/cd/SkinTutorial.pdf).
![physical model](https://github.com/2103simon/haptic_exploration/blob/master/assets/sensor_physical_old.png)
*On the left the design of the fingertip is shown without the conductance foam, the assembled hand is shown in the mid, and on the right, the triangle taxels mounted at the palm are shown.*

### Dependencies

- [`robotlogy-superbuild`](https://github.com/robotology/robotology-superbuild)

### Installation

```
git clone https://github.com/2103simon/haptic_exploration.git
cd haptic_exploration
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=<installation_path> ../
make install
```

then make sure that `GAZEBO_PLUGIN_PATH` contains `<installation_path>/lib`.

### Force to area distribution

Because [`gazebo`](http://gazebosim.org/) (with the [`contact sensor plugin`](http://gazebosim.org/tutorials?tut=contact_sensor)) only provides point contact, and no surface deformation, the force to area distribution has to be calculated mathematically. The calculation of the force measured by the single taxels is based on a gaussian force to area distribution, whereas the area depends on the contact force. 

The relation between force and area can be set to:
- linear 
- exponential saturation
- sigmoid

Further, the relationship is defined by the following two parameters:
- maximum force with effect on the area size
- maximum area distribution

![gaussian force distribution](https://github.com/2103simon/haptic_exploration/blob/master/assets/force_to_area_distr.png)
*At the top different relations between force and area are shown, namely: linear, exponential saturation, sigmoid. The bottom shows the impact on the gaussian force to area distribution with the highest impact on low forces.*

- taxel position hard coded. Might be defined in an external .txt file or within the SDF
- output port name hardcoded to `/right_hand/skinManager/skin_events:o`, and `/left_hand/skinManager/skin_events:o`
- only point contact and one contact per link is taken into account

One can read the output of the above-mentioned port from the command line using the following command:

`yarp read ... /skinManager/skin_events:o`

Every skinContactList is represented with the following format: (SKIN_CONTACT_VECTOR_1) ... (SKIN_CONTACT_VECTOR_N). There are as many SKIN_CONTACT_VECTORs as there were (clusters of) contacts detected on the whole skin of the robot by the skinManager. If case of no contact, the skinContactList is empty. If there was contact, every SKIN_CONTACT_VECTOR is enclosed by brackets and has the following format: ((contactId bodyPartId linkNumber skinPart) (centerOfPressure_x cOP_y cOP_z) (force_x f_y f_z) (moment_x m_y m_z) (geometricCenter_x gC_y gC_z) (surfaceNormalDirection_x sND_y sND_z) (activatedTaxelId1 aTId2 .. aTIdN) average_pressure). Here more information on some of the data:
- bodyPart: the part of the body (TORSO=1, HEAD=2, LEFT_ARM=3, RIGHT_ARM=4, LEFT_LEG=5, RIGHT_LEG=6)
- linkNumber: the link number relative to the specified body part (e.g. upper arm=2, forearm=4, hand=6)
- skinPart: the part of the skin (SKIN_LEFT_HAND=1, SKIN_LEFT_FOREARM=2, SKIN_LEFT_UPPER_ARM=3, SKIN_RIGHT_HAND=4, SKIN_RIGHT_FOREARM=5, SKIN_RIGHT_UPPER_ARM=6, SKIN_FRONT_TORSO=7)
- CoP: the center of pressure (expressed in link reference frame)
- force: force applied at contact (expressed in link reference frame)
- moment: moment applied at contact (expressed in link reference frame)
- geoCenter: the geometric center of the contact area (expressed in link reference frame)
- normalDir: normal direction of the contact area (expressed in link reference frame)
- taxelList: list of ids of the activated taxels
- pressure: average output of the activated taxels (now representing the force at taxel)


### Event Driven Output

The event-based output has two different working principles implemented: 
- linear relation from delta force to number of spikes per time step
- saturated delta force to number of spikes based on the same principles as the gaussian area to force distribution

![linear relation between 3 of spikes and delta force](https://github.com/2103simon/haptic_exploration/blob/master/assets/spikes_lin.png)
*The plot shows an arbitrary relation between delta force and number of spikes per timestep. Spikes on the left side are called "off" and the one on the right "on" representing de-/increase of force.*


### Haptic Exploration Environment

The haptic exploration environment contains the [`iCub model`](https://github.com/robotology/icub-models), a table to place objects in front of it, and the [`YCB model dataset`](https://github.com/sea-bass/ycb-tools).


### Other Useful Tools

To list available `YARP ports` type `yarp name list`. All names ending with `:o` are available output ports. To visualize (for example joint states) run `yarpscope --remote /icubSim/left_hand_finger/state:o --index "(0, 1, 2, 3)"`, whereas the numbers in breakets represent the number of displayed variables.

The environment can also be downloaded as a [`Docker`](https://www.docker.com/) environment containing the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild) and all the above-mentioned content. To do so, run `sudo docker pull 2103simon/event_driven_image:latest` 
