# EqVIO (Equivariant Visual Inertial Odometry)

This repository contains the implementation of EqVIO: An Equivariant Filter (EqF) for Visual Inertial Odometry (VIO).

Please see https://pvangoor.github.io/eqvio_docs/ for the documentation.

This fork of the EqVIO allows for attitude estimates to be provided by ArduPilot. This allows for more accurate attitude data 

## Dependencies

- Eigen 3: `sudo apt install libeigen3-dev`
- Yaml-cpp: `sudo apt install libyaml-cpp-dev`
- GIFT: https://github.com/pvangoor/GIFT

### Optional Dependencis

- FreeGLUT (for visualisations): `sudo apt install freeglut3-dev`
- ROS (for reading ROS-Bags): http://wiki.ros.org/ROS/Installation
- Doxygen (for documentation): `sudo apt install doxygen`
