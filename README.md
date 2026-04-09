# 4006 Task — Flight Dynamics Simulation Model

An undergraduate project developing a flight dynamics simulation model built from first principles, covering forces and torques, control surface aerodynamics, trim analysis, and autopilot design.

## Simulation Pipeline

The four sub-models are connected in a closed loop:

```
Autopilot  ──►  Trim Model  ──►  Control Surface Model  ──►  aircraft_dynamics
   ▲                                                                │
   └────────────────────── 12 states (feedback) ───────────────────┘
```

---

## Sub-Models

### 1. Forces and Torques — `aircraft_dynamics.m`

Integrates Newton–Euler equations of motion to propagate the full aircraft state.

**Input:** body-frame forces and moments $[F_x,\ F_y,\ F_z,\ l,\ m,\ n]$

**Output:** 12 states

$$
\mathbf{x} = [p_n,\ p_e,\ p_d,\ u,\ v,\ w,\ \phi,\ \theta,\ \psi,\ p,\ q,\ r]^\top
$$

where $(p_n, p_e, p_d)$ are NED positions, $(u, v, w)$ body velocities, $(\phi, \theta, \psi)$ Euler angles, and $(p, q, r)$ body angular rates.

---

### 2. Control Surface Model — `forces_moments.m`

Takes control surface deflections as input and outputs body-frame forces and moments to feed into `aircraft_dynamics`.

**Input:** $[\delta_e,\ \delta_a,\ \delta_r,\ \delta_t]$

**Output:** $[F_x,\ F_y,\ F_z,\ l,\ m,\ n]$

The model computes forces and moments through the following steps:

**Step 1 — Wind in body frame**

Transform steady wind through the vehicle-to-body rotation $R^v_b$ and add the body-frame gust component:

$$
\mathbf{w}_b = R^v_b \, \mathbf{w}_{steady} + \mathbf{w}_{gust}
$$

**Step 2 — Airspeed vector**

Subtract wind from ground velocity and compute airspeed $V_a$, angle of attack $\alpha$, and sideslip angle $\beta$:

$$
\mathbf{u}_r = \mathbf{u} - \mathbf{w}_b, \qquad
V_a = \|\mathbf{u}_r\|, \qquad
\alpha = \arctan\!\left(\frac{w_r}{u_r}\right), \qquad
\beta = \arcsin\!\left(\frac{v_r}{V_a}\right)
$$

**Step 3 — Aerodynamic coefficient blending**

Compute $\sigma(\alpha)$ blending, lift and drag coefficients, and their rate and deflection derivatives:

$$
C_L(\alpha) = (1-\sigma)\left(C_{L_0} + C_{L_\alpha}\alpha\right) + \sigma\left(2\,\text{sgn}(\alpha)\sin^2\!\alpha\cos\alpha\right)
$$

$$
C_D(\alpha) = C_{D_p} + \frac{(C_{L_0}+C_{L_\alpha}\alpha)^2}{\pi e AR}, \qquad
C_X = -C_D\cos\alpha + C_L\sin\alpha, \qquad
C_Z = -C_D\sin\alpha - C_L\cos\alpha
$$

**Step 4 — Gravitational force**

Evaluate the three-component body-frame gravity vector from Euler angles:

$$
\mathbf{F}_{gravity} = m g \begin{bmatrix} -\sin\theta \\ \cos\theta\sin\phi \\ \cos\theta\cos\phi \end{bmatrix}
$$

**Step 5 — Aerodynamic forces and moments**

Evaluate longitudinal and lateral components scaled by dynamic pressure $\bar{q} = \tfrac{1}{2}\rho V_a^2 S$:

$$
F_x = \bar{q}\!\left(C_X + C_{X_q}\frac{\bar{c}}{2V_a}q + C_{X_{\delta_e}}\delta_e\right), \quad
F_z = \bar{q}\!\left(C_Z + C_{Z_q}\frac{\bar{c}}{2V_a}q + C_{Z_{\delta_e}}\delta_e\right)
$$

$$
l = \bar{q} b\!\left(C_p + C_{p_p}\frac{b}{2V_a}p + C_{p_r}\frac{b}{2V_a}r + C_{p_{\delta_a}}\delta_a + C_{p_{\delta_r}}\delta_r\right)
$$

**Step 6 — Propulsive force and torque**

Evaluate thrust $F_{prop}$ and propeller reaction torque $Q_{prop}$; add propeller torque to roll moment $l$:

$$
F_{prop} = \tfrac{1}{2}\rho S_{prop} C_{prop}\!\left[(k_\text{motor}\,\delta_t)^2 - V_a^2\right], \qquad
l \mathrel{+}= -k_{T_p}(k_\Omega\,\delta_t)^2
$$

---

### 3. Trim Model — `compute_trim.m`

Finds the equilibrium deflections for a commanded steady flight condition using numerical optimisation.

**Method:** Search over $[\delta_e,\ \delta_a,\ \delta_r,\ \delta_t]$ until the aircraft sustains the commanded airspeed, altitude, and course with zero acceleration:

$$
\min_{\delta_e,\,\delta_a,\,\delta_r,\,\delta_t} \left\| \dot{\mathbf{x}} \right\|^2 \quad \text{subject to} \quad V_a = V_a^*, \quad \dot{h} = 0, \quad \dot{\chi} = 0
$$

The resulting trim deflections are passed directly to the **Control Surface Model**.

---

### 4. Autopilot — `autopilot.m`, `compute_autopilot_gains.m`

Tracks a commanded altitude $h^*$, airspeed $V_a^*$, and course angle $\chi^*$ using successive loop closure with PID controllers.

$$
\delta_e = K_{p,\theta}(\theta^* - \theta) + K_{d,\theta}\dot{\theta} + K_{i,\theta}\int(\theta^* - \theta)\,dt
$$

$$
K_p = \frac{2\zeta\omega_n}{k_a}, \qquad K_i = \frac{\omega_n^2}{k_a}
$$

The autopilot feeds the commanded flight condition to the **Trim Model** to obtain the equilibrium operating point, then drives the aircraft toward it via control surface commands.

---

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
