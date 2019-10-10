# Changelog

All notable changes to this project will be documented in this file.

## Unreleased (will be v1.2)

### Added

- Curved footstep plans

### Fixed

- Initialization of reference pendulum ZMP
- Stabilizer: damping term of foot force difference control for stair climbing
- Stabilizer: formula of proportional term in desired CoM acceleration

### Changed

- Footstep plans are now robot-specific
- Removed legacy mass estimation phase
- Updated CMake configuration
- Updated copra and mc\_rtc dependencies

## [v1.1] - 2019/04/15

This release corresponds to the stair climbing and walking control that ran in
the final demonstrator of the COMANOID project.

## [v1.0] - 2018/10/17

First public release of the controller.
