# LIPM Walking Controller

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://scaron.info/doc/lipm_walking_controller/)

[![Stair climbing by the HRP-4 humanoid robot](https://scaron.info/images/stair-climbing.jpg)](https://www.youtube.com/watch?v=vFCFKAunsYM&t=22)

Source code of the walking and stair climbing controller used in the experiments of [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://hal.archives-ouvertes.fr/hal-01875387/document), as well as in an industrial demonstrator at the [Airbus Saint-Nazaire factory](https://hal-lirmm.ccsd.cnrs.fr/lirmm-02303117/document).

This repository is meant for learning and sharing of experimental knowledge. It keeps walking components in the same place, and in the simple form they had in 2019. If you have questions, go ahead and ask them in the issue tracker after checking that they are not already answered in the [frequently asked questions](https://github.com/stephane-caron/lipm_walking_controller/wiki) ;) And if you want to take part in development, reach out for the [active fork at jrl-umi3218](https://github.com/jrl-umi3218/lipm_walking_controller).

## Getting started

The easiest way to try the controller in action is to run its Docker image from an Ubuntu Linux distribution:

```sh
xhost +local:docker
docker run -it --rm --user ayumi -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    stephanecaron/lipm_walking_controller \
    lipm_walking --floor
```

This image runs the exact controller we used in 2019 during experiments and industrial demonstrations.

## Dependencies

The controller has been tested on Ubuntu 14.04 with ROS Indigo and Ubuntu 16.04 with ROS Kinetic. It builds on the shoulders of the following software:

* [ROS](http://www.ros.org/): robotics middleware
* [Eigen](https://eigen.tuxfamily.org/): linear algebra
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
* [mc\_rtc](https://github.com/jrl-umi3218/mc_rtc): robot controller library

Instructions to build an active development version of the controller from source are available for the [jrl-umi3218 fork](https://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/build.html).

## Usage

Make sure the controller is enabled in your mc\_rtc configuration:

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
of the [Choreonoid](https://choreonoid.org/en/) project installed in the
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

To [@gergondet](https://github.com/gergondet) and [@arntanguy](https://github.com/arntanguy) for developing and helping with the mc\_rtc framework.
