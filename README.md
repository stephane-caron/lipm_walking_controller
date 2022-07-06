# LIPM Walking Controller

[**Trying the controller**](https://github.com/stephane-caron/lipm_walking_controller#trying-the-controller)
| [**Dependencies**](https://github.com/stephane-caron/lipm_walking_controller#dependencies)
| [**Usage**](https://github.com/stephane-caron/lipm_walking_controller#usage)
| [**Known bugs**](https://github.com/stephane-caron/lipm_walking_controller#known-bugs)
| [**Thanks**](https://github.com/stephane-caron/lipm_walking_controller#thanks)

[![License](https://img.shields.io/badge/License-BSD%202--Clause-green.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://scaron.info/doc/lipm_walking_controller/)
![Status](https://img.shields.io/badge/status-archive-lightgrey.svg)

[![Stair climbing by the HRP-4 humanoid robot](https://scaron.info/images/stair-climbing.jpg)](https://www.youtube.com/watch?v=vFCFKAunsYM&t=22)

Source code of the walking and stair climbing controller used in the experiments of [Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control](https://hal.archives-ouvertes.fr/hal-01875387/document), as well as in an industrial demonstrator at the [Airbus Saint-Nazaire factory](https://hal-lirmm.ccsd.cnrs.fr/lirmm-02303117/document).

This repository is meant for learning and sharing of experimental knowledge. It keeps walking components in the same place, and in the simple form they had in 2019. If you have questions, go ahead and ask them in the [discussions](https://github.com/stephane-caron/lipm_walking_controller/discussions) forum, after checking that they are not already answered in the [frequently asked questions](https://github.com/stephane-caron/lipm_walking_controller/wiki) ;) If you want to take part in future development, reach out for the [fork at jrl-umi3218](https://github.com/jrl-umi3218/lipm_walking_controller).

## Trying the controller

You can try the controller directly by running its Docker image:

```sh
xhost +local:docker
docker run -it --rm --user ayumi -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    stephanecaron/lipm_walking_controller \
    lipm_walking --floor
```

This image runs the exact controller we used in 2019 during experiments and industrial demonstrations. Replace `--floor` with `--staircase` for stair climbing.

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

Instructions to build a more recent version of the controller from source are available from the [jrl-umi3218 fork](https://jrl-umi3218.github.io/lipm_walking_controller/doxygen/HEAD/build.html).

## Usage

The `lipm_walking` script in the Docker images starts two processes. First, it launches RViz for visualization and interaction with the controller:

```sh
roslaunch lipm_walking_controller display.launch robot:=jvrc1
```

Second, it starts the controller itself in a [Choreonoid](https://choreonoid.org/en/) simulation:

```sh
cd /usr/local/share/hrpsys/samples/JVRC1
choreonoid --start-simulation sim_mc.cnoid
```

We end up with three windows: one for RViz (left), one for Choreonoid (top right) and one for the terminal (bottom right) where the controller logs information and debug messages.

![Choreonoid and RViz GUI of the controller](https://user-images.githubusercontent.com/1189580/64157945-ead71c80-ce37-11e9-9081-7936702c5fbc.png)

See the [Graphical user interface](https://github.com/stephane-caron/lipm_walking_controller/wiki/How-to-use-the-graphical-user-interface%3F) page of the
wiki for further instructions on how to interact with the controller in RViz.

## Known bugs

This repository is an archive of the code as it ran in 2019, bugs included. Here is a non-exhaustive list of the big ones that have been found since then:

- Swing foot velocities and accelerations computed in incorrect frames [#29](https://github.com/jrl-umi3218/lipm_walking_controller/issues/29)
- Missing transformation in foot force difference control [#72](https://github.com/stephane-caron/lipm_walking_controller/discussions/72)
- Missing contact reference velocity update [#37](https://github.com/jrl-umi3218/lipm_walking_controller/issues/37)
- Overwritten swing foot task velocities [#41](https://github.com/jrl-umi3218/lipm_walking_controller/issues/41)

Head over to the [fork at jrl-umi3218](https://github.com/jrl-umi3218/lipm_walking_controller) for updates.

## Thanks

Thanks to:

- [@gergondet](https://github.com/gergondet) for developing and helping with mc\_rtc
- [@arntanguy](https://github.com/arntanguy) for developing and helping with mc\_rtc
- [@Saeed-Mansouri](https://github.com/Saeed-Mansouri) for bug hunting and discussion around the project - *Best Debugger Award* 🏅
- [@mmurooka](https://github.com/mmurooka) for bug hunting and discussion around the project
