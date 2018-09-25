# LIPM Walking Controller

Source code of the walking controller used in the experiments of [Stair
Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance
Control](https://hal.archives-ouvertes.fr/hal-01875387/document).

## Installation

Compilation has been tested on Ubuntu 14.04 (gcc/clang) with ROS Indigo.

### Dependencies

Compilation requires:

* [ROS](http://www.ros.org/) with a working [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg): spatial vector algebra
* [RBDyn](https://github.com/jrl-umi3218/RBDyn/): rigid body dynamics
* [eigen-qld](https://github.com/jrl-umi3218/eigen-qld): quadratic programming
* [sch-core](https://github.com/jrl-umi3218/sch-core): collision detection
* [Tasks](https://github.com/jrl-umi3218/Tasks/): inverse kinematics
* [mc\_rbdyn\_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf): robot model loader
* [eigen-cddlib](https://github.com/vsamy/eigen-cddlib): polyhedron double description
* [Copra](https://github.com/vsamy/Copra-deprecated): linear model predictive control

The following dependencies are not publicly released yet but available upon
request to [Pierre Gergondet](mailto:pierre.gergondet@gmail.com):

* [mc\_rtc](https://gite.lirmm.fr/multi-contact/mc_rtc): robot controller library (includes mc_control, mc_rbdyn, mc_solver and mc_tasks)
* [mc\_rtc\_ros](https://gite.lirmm.fr/multi-contact/mc_rtc_ros): ROS tools for mc_rtc
* [mc\_rtc\_ros\_data](https://gite.lirmm.fr/multi-contact/mc_rtc_ros_data): ROS environment and object descriptions for mc_rtc

There is currently an additional dependency to the *EigenQP* library. It is
redundant with *eigen-qld* and will be removed in a future update.

### Building from source on Linux

Link `lipm_walking_controller` from the source folder of your catkin workspace,
then follow the standard catkin workflow:
```sh
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo && catkin_make install
```

## Usage

First, launch RViz by:
```sh
roslaunch lipm_walking_controller display.launch
```
Then, run your mc\_rtc interface on the main confuguration file:
```sh
<mc_rtc_interface> lipm_walking_controller/etc/mc_rtc.conf
```
where ``<mc_rtc_interface>`` is for instance ``mc_vrep`` or ``MCControlTCP``.

## Thanks

- To Pierre Gergondet for developing and helping with the mc\_rtc framework
