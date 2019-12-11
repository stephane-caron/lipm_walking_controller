# LIPM Walking Controller

[![Stair climbing by the HRP-4 humanoid robot](https://scaron.info/images/stair-climbing.jpg)](https://www.youtube.com/watch?v=vFCFKAunsYM)

Source code of the walking and stair climbing controller used in the experiments of [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://hal.archives-ouvertes.fr/hal-01875387/document), as well as in an industrial demonstrator at the [Airbus Saint-Nazaire factory](https://cordis.europa.eu/project/rcn/194280/brief/en?WT.mc_id=exp).

## Getting started

The easiest way to get started with the controller is to run its Docker image from an Ubuntu Linux distribution:

```
xhost +local:docker
docker run -it --rm --user ayumi -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    stephanecaron/lipm_walking_controller \
    lipm_walking --floor
```

See the [documentation](https://scaron.info/doc/lipm_walking_controller/docker.html) for more usage instructions.

## Installation

The controller has been tested on Ubuntu 14.04 with ROS Indigo and Ubuntu 16.04 with ROS Kinetic. See the instructions to [build from source](https://scaron.info/doc/lipm_walking_controller/build.html) in the documentation.

### Dependencies

Compilation requires:

* [ROS](http://www.ros.org/) with a working [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
* [Eigen3ToPython](https://github.com/jrl-umi3218/Eigen3ToPython): Python bindings for Eigen
* [SpaceVecAlg](https://github.com/jrl-umi3218/SpaceVecAlg): spatial vector algebra
* [RBDyn](https://github.com/jrl-umi3218/RBDyn/): rigid body dynamics
* [eigen-qld](https://github.com/jrl-umi3218/eigen-qld): quadratic programming
* [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog): quadratic programming
* [sch-core](https://github.com/jrl-umi3218/sch-core): collision detection
* [sch-core-python](https://github.com/jrl-umi3218/sch-core-python): Python bindings for sch-core
* [Tasks](https://github.com/jrl-umi3218/Tasks/): inverse kinematics
* [mc\_rbdyn\_urdf](https://github.com/jrl-umi3218/mc_rbdyn_urdf): robot model loader
* [copra](https://github.com/vsamy/copra): linear model predictive control

The following dependencies are not publicly released but available upon request to [Pierre Gergondet](mailto:pierre.gergondet@gmail.com):

* [mc\_rtc](https://gite.lirmm.fr/multi-contact/mc_rtc): robot controller library (includes mc\_control, mc\_rbdyn, mc\_solver and mc\_tasks)
* [mc\_rtc\_ros](https://gite.lirmm.fr/multi-contact/mc_rtc_ros): ROS tools for mc\_rtc
* [mc\_rtc\_ros\_data](https://gite.lirmm.fr/multi-contact/mc_rtc_ros_data): ROS environment and object descriptions for mc\_rtc

## Usage

If it is not there already, enable the controller in your mc\_rtc configuration:
```json
{
  "MainRobot": "JVRC1",
  "Enabled": ["LIPMWalking"]
}
```
Launch RViz for the JVRC-1 model by:
```sh
roslaunch lipm_walking_controller display.launch robot:=jvrc1
```
Finally, start the controller from your mc\_rtc interface. Here is the example
of the [Choreonoid](https://choreonoid.org/en/) project installed from
[jvrc\_choreonoid](https://gite.lirmm.fr/multi-contact/jvrc_choreonoid) in the
Docker image:
```sh
cd /usr/local/share/hrpsys/samples/JVRC1
choreonoid --start-simulation sim_mc.cnoid
```
You should end up with the following windows:
![Choreonoid and RViz GUI of the controller](https://user-images.githubusercontent.com/1189580/64157945-ead71c80-ce37-11e9-9081-7936702c5fbc.png)
See the [Graphical user interface](https://github.com/stephane-caron/lipm_walking_controller/wiki/Graphical-user-interface) page of the
wiki for further instructions on how to use this GUI.

## Thanks

To [@gergondet](https://github.com/gergondet) for developing and helping with the mc\_rtc framework.
