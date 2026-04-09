# 4006 Task — Flight Dynamics Simulation Model

An undergraduate project developing a flight dynamics simulation model built from first principles, covering forces and torques, control surface aerodynamics, and trim analysis.

## Project Overview

This project constructs a complete aircraft simulation model by integrating three core sub-models:

### 1. Forces and Torques Model
Derives the aerodynamic and gravitational forces and moments acting on the aircraft body. Includes:
- Lift, drag, and side force computation
- Rolling, pitching, and yawing moment equations
- Body-axis force and moment summation

### 2. Control Surface Model
Models the effect of control surface deflections on aerodynamic forces and moments. Includes:
- Aileron, elevator, and rudder deflection effects
- Control surface aerodynamic derivatives
- Coupling between control inputs and aircraft response

### 3. Trim Model
Computes the equilibrium (trim) conditions for steady-level flight and other flight states. Includes:
- Trim state computation for given flight conditions
- Solving for required control deflections and attitudes at trim
- Validation of trim solutions against equations of motion

## Repository Structure

```
4006_Task/
├── forces_torques/     # Forces and torques model
├── control_surface/    # Control surface aerodynamic model
├── trim/               # Trim analysis model
└── README.md
```

## Requirements

- MATLAB / Simulink (recommended)

## Usage

Each sub-model can be run independently or integrated into a full simulation pipeline. Refer to individual module directories for setup and run instructions.

## Author

Undergraduate project — Department of Aerospace Engineering
