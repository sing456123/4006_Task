# 4006 Task — Flight Dynamics Simulation Model

An undergraduate project developing a flight dynamics simulation model built from first principles, covering forces and torques, control surface aerodynamics, and trim analysis.

## Project Overview

This project constructs a complete aircraft simulation model by integrating three core sub-models:

### 1. Forces and Torques Model

Derives the aerodynamic and gravitational forces and moments acting on the aircraft body.
The total aerodynamic force vector in the body frame is:

$$
\mathbf{F}_{aero} = \begin{bmatrix} F_x \\ F_y \\ F_z \end{bmatrix}
= \frac{1}{2} \rho V_a^2 S \begin{bmatrix} -C_D(\alpha) \\ C_Y \\ -C_L(\alpha) \end{bmatrix}
$$

where $\rho$ is air density, $V_a$ is airspeed, $S$ is wing area, and $C_D$, $C_Y$, $C_L$ are the drag, side-force, and lift coefficients respectively.

### 2. Control Surface Model

Models the effect of control surface deflections on aerodynamic forces and moments.
Control inputs are the aileron $\delta_a$, elevator $\delta_e$, and rudder $\delta_r$ deflections.
The pitching moment contribution from the elevator is:

$$
m = \frac{1}{2} \rho V_a^2 S \, \bar{c} \left( C_{m_0} + C_{m_\alpha} \alpha + C_{m_q} \frac{\bar{c}}{2 V_a} q + C_{m_{\delta_e}} \delta_e \right)
$$

### 3. Trim Model

Computes the equilibrium (trim) conditions for steady-level flight.
At trim, accelerations are zero — the equations of motion reduce to:

$$
\mathbf{F}_{aero} + \mathbf{F}_{gravity} = \mathbf{0}, \qquad \mathbf{M}_{aero} = \mathbf{0}
$$

Trim solving finds the state $[\alpha^*, \delta_e^*, \delta_t^*]$ satisfying these constraints.

### 4. Autopilot

Gains are computed from linearised dynamics. The autopilot uses successive loop closure:

$$
K_p = \frac{2 \zeta \omega_n}{k_a}, \qquad K_i = \frac{\omega_n^2}{k_a}
$$

where $\zeta$ is the damping ratio and $\omega_n$ is the natural frequency.

## Repository Structure

```
4006_Task/
├── uavsim/
│   ├── Autopolit/      # Autopilot & dynamics simulation (Simulink)
│   ├── parameters/     # Aircraft, sensor, wind, simulation parameters
│   └── tools/          # Rotation and quaternion utilities
├── report/
│   ├── report.tex      # LaTeX report template
│   └── references.bib  # Bibliography
└── README.md
```

## Requirements

- MATLAB / Simulink (recommended)

## Usage

Each sub-model can be run independently or integrated into a full simulation pipeline.
Refer to individual module directories for setup and run instructions.

## Report

The full LaTeX report template is available here: [report/report.tex](report/report.tex)

## Authors

Undergraduate project — Department of Aerospace Engineering
