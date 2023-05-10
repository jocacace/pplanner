# pplanner, a 3D path planner
This repository contains a 3D path planner implemented as a shared library in C++. This library can be included in other C++ programs to plan quickly obstacle free paths in a 3D scene.

This software uses [ompl](https://ompl.kavrakilab.org/)  ompl for the generation of the path and  [fcl](https://github.com/flexible-collision-library/fcl)  for the obstacle collision checking phase. 

### Installation 

To install this library use the CMakeLists.txt file included in the directory. 

      $ cd pplanner
      $ mkdir build && cd build
      $ cmake ..
      $ make
      $ sudo make install

### Requirements
This library requires omple (verison 1.6) and FCL (version 0.6)

### Documentation
 Use the docs/html/index.html file to check the Doxygen documentation. An example using ROS on how to use the planner can be found at https://github.com/jocacace/test_3d_planner






