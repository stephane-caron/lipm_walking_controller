# LIPM Walking Controller

[![Stair climbing by the HRP-4 humanoid robot](https://scaron.info/images/stair-climbing.jpg)](https://www.youtube.com/watch?v=vFCFKAunsYM)

Source code of the walking and stair climbing controller used in the experiments of [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://hal.archives-ouvertes.fr/hal-01875387/document).

## Installation

Compilation has been tested on Ubuntu 14.04 (gcc/clang) with ROS Indigo and Ubuntu 16.04 (gcc) with ROS Kinetic.

### Dependencies

Compilation requires:

* [ROS](http://www.ros.org/) with a working [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg): spatial vector algebra
* [RBDyn](https://github.com/jrl-umi3218/RBDyn/): rigid body dynamics
* [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol): quadratic programming (if you have the LSSOL licence ask us this library)
* [eigen-qld](https://github.com/jrl-umi3218/eigen-qld): quadratic programming
* [sch-core](https://github.com/jrl-umi3218/sch-core): collision detection
* [Tasks](https://github.com/jrl-umi3218/Tasks/): inverse kinematics
* [mc\_rbdyn\_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf): robot model loader
* [copra](https://github.com/vsamy/copra): linear model predictive control

The following dependencies are not publicly released yet but available upon request to [Pierre Gergondet](mailto:pierre.gergondet@gmail.com):

* [mc\_rtc](https://gite.lirmm.fr/multi-contact/mc_rtc): robot controller library (includes mc\_control, mc\_rbdyn, mc\_solver and mc\_tasks)
* [mc\_rtc\_ros](https://gite.lirmm.fr/multi-contact/mc_rtc_ros): ROS tools for mc\_rtc
* [mc\_rtc\_ros\_data](https://gite.lirmm.fr/multi-contact/mc_rtc_ros_data): ROS environment and object descriptions for mc\_rtc

### Building from source on Linux

Link `lipm_walking_controller` from the source folder of your catkin workspace, then follow the standard catkin workflow:
```sh
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo && catkin_make install
```
To avoid a ``sudo`` at ``catkin_make install`` you can change ownership or permissions of the ``/usr/local/lib/mc_controller`` folder.

## Usage

First, launch RViz for the HRP-4 model by:
```sh
roslaunch lipm_walking_controller display_hrp4.launch
```
Enable the controller in your mc\_rtc configuration:
```sh
{
  "MainRobot": "HRP4",
  "Enabled": ["LIPMWalking"]
}
```
Finally, run your mc\_rtc interface, for instance ``mc_vrep`` or ``MCUDPControl``.

## Thanks

To [@gergondet](https://github.com/gergondet) for developing and helping with the mc\_rtc framework.
