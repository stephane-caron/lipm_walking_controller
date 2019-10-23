# Changelog

All notable changes to this project will be documented in this file.

## Unreleased

### Added

- Stabilizer: alternative DCM gain tuning by pole placement under ZMP lag model

### Changed

- Switch license to BSD 2-clause for compatibility with other mc\_rtc projects

## [v1.2] - 2019/10/11

The main change in this release is the switch from ZMP to DCM derivative term
in DCM tracking, following [Biped locomotion control for uneven terrain with
narrow support region](https://doi.org/10.1109/SII.2014.7028007) (Morisawa _et
al._, 2014). The DCM derivative is computed from DCM and ZMP measurements using
a model-based estimator.

### Added

- Default footstep plans and settings for HRP-2Kai
- Model-based DCM derivative estimator ("derivator")
- Sample curved footstep plans
- StationaryOffsetFilter class used by DCM derivator

### Fixed

- Initialization of reference pendulum ZMP
- Stabilizer: damping term of foot force difference control
- Stabilizer: formula of proportional term in desired CoM acceleration

### Changed

- Cleaned up legacy code
- Footstep plans are now robot-specific
- Removed legacy mass estimation phase
- Stabilizer: switch from ZMP to DCM derivative term in DCM tracking
- Updated CMake configuration
- Updated copra and mc\_rtc dependencies

## [v1.1] - 2019/04/15

This release corresponds to the stair climbing and walking control that ran in
the final demonstrator of the COMANOID project.

## [v1.0] - 2018/10/17

First public release of the controller.
