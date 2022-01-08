# TRO2021-Dynamic-Modelling

A MATLAB & Simulink implementation of the forward and inverse dynamics solutions for mechanisms driven by screw-based transmission systems.

## Relevant Publications

[[1]](https://ieeexplore.ieee.org/abstract/document/9353237) A. Mablekos-Alexiou, L. da Cruz, and C. Bergeles, "Friction-Inclusive Modeling of Sliding Contact Transmission Systems in Robotics," *IEEE Transactions on Robotics*, vol. 37, no. 4, pp. 1252-1267, 2021

[[2]]() A. Mablekos-Alexiou, L. da Cruz, and C. Bergeles, "On the Geometry and Dynamics of the 4-DoF Serial Spherical Mechanism," in *IEEE International Conference on Robotics and Automation*, 2022 (under review).

## Prerequisites

The package requires the MATLAB Signal Processing, Optimization, and Robotics System Toolbox.

## System Object [Transmission.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/Transmission.m)

[Transmission.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/Transmission.m) defines the `Transmission` class. This class encapsulates all the dynamics related data of a screw-based drive and the operations performed on that data for use in Simulink. The five basic public properties of this class, each predefined as a structure array with values set to zero, are the following

* `geometry` -
* `preload` - 
* `friction` - 
* `inertia` - 
* `actuation` - 

## Simulation Examples

### Single-DoF Dynamics Verification

### Inverse Dynamics Calculations of the 4-DoF Serial Spherical Mechanism
