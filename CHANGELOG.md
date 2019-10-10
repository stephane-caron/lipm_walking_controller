# Changelog

All notable changes to this project will be documented in this file.

## Unreleased (will be v1.2)

### Added

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
- Updated CMake configuration
- Updated copra and mc\_rtc dependencies
- Stabilizer: switch from ZMP to DCM derivative term in DCM tracking

## [v1.1] - 2019/04/15

This release corresponds to the stair climbing and walking control that ran in
the final demonstrator of the COMANOID project.

## [v1.0] - 2018/10/17

First public release of the controller.
