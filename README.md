# LIPM Walking Controller

Source code of the walking controller used in the experiments of [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://hal.archives-ouvertes.fr/hal-01875387/document).

## Installation

Compilation has been tested on Ubuntu 14.04 (gcc/clang) with ROS Indigo and Ubuntu 16.04 (gcc) with ROS Kinetic.

### Dependencies

Compilation requires:

* [ROS](http://www.ros.org/) with a working [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg): spatial vector algebra
* [RBDyn](https://github.com/jrl-umi3218/RBDyn/): rigid body dynamics
* [eigen-qld](https://github.com/jrl-umi3218/eigen-qld): quadratic programming
* [sch-core](https://github.com/jrl-umi3218/sch-core): collision detection
* [Tasks](https://github.com/jrl-umi3218/Tasks/): inverse kinematics
* [mc\_rbdyn\_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf): robot model loader
* [copra](https://github.com/vsamy/copra): linear model predictive control

The following dependencies are not publicly released yet but available upon request to [Pierre Gergondet](mailto:pierre.gergondet@gmail.com):

* [mc\_rtc](https://gite.lirmm.fr/multi-contact/mc_rtc): robot controller library (includes mc\_control, mc\_rbdyn, mc\_solver and mc\_tasks)
* [mc\_rtc\_ros](https://gite.lirmm.fr/multi-contact/mc_rtc_ros): ROS tools for mc\_rtc
* [mc\_rtc\_ros\_data](https://gite.lirmm.fr/multi-contact/mc_rtc_ros_data): ROS environment and object descriptions for mc\_rtc

There is currently an additional dependency to the *eigen-lssol* library. It will be replaced by [eigen-qld](https://github.com/jrl-umi3218/eigen-qld) in a future update.

### Building from source on Linux

Link `lipm_walking_controller` from the source folder of your catkin workspace, then follow the standard catkin workflow:
```sh
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo && catkin_make install
```

## Usage

First, launch RViz for the HRP-4 model by:
```sh
roslaunch lipm_walking_controller display_hrp4.launch
```
Then, run your mc\_rtc interface on the main configuration file:
```sh
<mc_rtc_interface> lipm_walking_controller/etc/mc_rtc.conf
```
where ``<mc_rtc_interface>`` is for instance ``mc_vrep`` or ``MCControlTCP``.

## Thanks

- To Pierre Gergondet for developing and helping with the mc\_rtc framework
