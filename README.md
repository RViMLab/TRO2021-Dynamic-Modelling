# TRO2021-Dynamic-Modelling

A MATLAB & Simulink implementation of the forward and inverse dynamics solutions for mechanisms driven by screw-based transmission systems.

## Relevant Publications

[[1]](https://ieeexplore.ieee.org/abstract/document/9353237) A. Mablekos-Alexiou, L. da Cruz, and C. Bergeles, "Friction-Inclusive Modeling of Sliding Contact Transmission Systems in Robotics," *IEEE Transactions on Robotics*, vol. 37, no. 4, pp. 1252-1267, 2021

[[2]]() A. Mablekos-Alexiou, L. da Cruz, and C. Bergeles, "On the Geometry and Dynamics of the 4-DoF Serial Spherical Mechanism," in *IEEE International Conference on Robotics and Automation*, 2022 (under review).

## Prerequisites

The package requires the MATLAB Signal Processing, Optimization, and Robotics System Toolbox.

## System Object [Transmission.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/Transmission.m)

The main script of this package is [Transmission.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/Transmission.m), which defines the `Transmission` class. This class encapsulates all the dynamics related data of a screw-based drive and the operations performed on that data for use in Simulink. The five basic public properties of this class, each predefined as a structure array with values set to zero, are the following:

* `geometry` - Property that contains the geometric parameters of a screw-based drive, including the drive type, the input element lead and lead angle, the diameter of the input element, the diameter of the output element in the case of the worm drive, the reduction ratio, and the thread and pressure angles. The value for the drive type, where `1` corresponds to the simple lead screw drive, `2` to the antibacklash lead screw drive, and `3` to the worm drive, is assigned to the field with name `type`. The rest geometric values are assigned to the corresponding fields with names `lead`, `leadAngle`, `inElemDia`, `outElemDia`, `reductionRatio`, `threadAngle`, and `pressureAngle`. For the simple and antibacklash lead screw drives, the value to `outElemDia` is left zero. Note that the lead, lead angle, input and output element diameters, and the reduction ratio are parameters that are correlated for a screw-based drive. The user must provide values only for the necessary number of parameters so as not to under- or over-define the geometry of the system.

* `preload` - Property that contains the preload spring force of an antibacklash lead screw drive. The force value is assigned to the field with name `springForce`. For the simple lead screw and the worm drive, this value is left zero.

* `friction` - Property that contains the friction parameters of a screw-based drive, including the breakaway and viscosity terms of the support elements, and the friction coefficient, the Stribeck coefficient, the Stribeck velocity, the rising static friction constant, and the memory constant of the main elements. The frictional values are assigned to the corresponding fields with names `inSupBreak`, `inSupVisc`, `outSupBreak`, `outSupVisc`, `kinFriction`, `strbInf`, `strbVelocity`, `risingCst`, and `memoryCst`. In case the friction model used for the simulation does not include all the above parameters, then values are assigned to the relevant fields while the rest are left zero.

* `inertia` - Property that contains the inertial parameters of a screw-based drive, including the input and the main output element inertia, as well as the secondary output element mass in the case of an antibacklash drive. The inertial values are assigned to the corresponding fields with names `inElemInertia`, `mnOutElemInertia`, and `secOutElemInertia` For the simple lead screw and the worm drive, the value to `secOutElemInertia` is left zero.

* `actuation` - Property that contains some actuation parameters of a screw-based drive, including the motor nominal torque and torque constant as well as the encoder counts. The parameter values are assigned to the corresponding fields with names `nominalTorque`, `torqueCst`, and `encoderCounts`. Assigning values to the fields of that structure is optional and not needed for the simulation. However, it can prove useful in case experimental data, e.g., motor currents and encoder signals, are used for model verification.

`Transmission` can be used in a model to simulate in Simulink by defining the `setupImpl` and `stepImpl` methods. Both methods perform calculations by calling a series of static methods, which implement the corresponding system dynamics, as described in [[1]](https://ieeexplore.ieee.org/abstract/document/9353237).

## Simulation Examples

### Single-DoF Dynamics Verification

The script [sd_1dof.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/sd_1dof.m), which needs to be run first, creates three `Transmission` instances, each enclosing the parameters of the respective single-DoF mechanism that has been tested in [[1]](https://ieeexplore.ieee.org/abstract/document/9353237). Each object can be added to [simINVD_1dof.slx](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/simINVD_1dof.slx) or [simFWD_1dof.slx](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/simFWD_1dof.slx) for simulating the inverse or the forward dynamics of the corresponding drive, respectively. The scripts [id_1dof_static.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/id_1dof_static.m) and [id_1dof_dyn.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/id_1dof_dyn.m) are examples of the static and dynamic identification framework, respectively. Data used for the identification are listed in the [experiments](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/tree/main/experiments) folder and read using the [getEposData.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/getEposData.m) function.

### Inverse Dynamics Calculations of the 4-DoF Serial Spherical Mechanism (SSM)

The script [ssmBuild.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/ssmBuild.m) creates a model of the 4-DoF SSM that has been studied in [[2]](). The script [sd_ssm.m](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/sd_ssm.m) creates three `Transmission` instances that correspond to the robot's transmission systems. These objects can be added to [simINVD_ssm.slx](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/simINVD_ssm.slx) for simulating the robot's inverse dynamics. The script [ssmMotorization.m]() creates a desired motion response for a selected joint of the 4-DoF SSM and, after running [simINVD_ssm.slx](https://github.com/RViMLab/TRO2021-Dynamic-Modelling/blob/main/simINVD_ssm.slx), provides estimations for the torque requirements.
