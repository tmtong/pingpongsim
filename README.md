# Table Tennis Ball Trajectory Simulation

## Purpose
This project simulates the trajectories of table tennis balls during topspin shots against topspin shots, taking into account realistic physical parameters such as ball dynamics, rubber properties, and bounce mechanics. The goal is to analyze how different rubbers and impact conditions affect the ball's trajectory and behavior on the table. This simulation can be useful for:
- Table tennis players who want to understand the effects of different rubbers.
- Coaches and researchers studying the physics of table tennis.
- Equipment manufacturers designing new rubber materials.

The simulation generates visual plots of ball trajectories and organizes them into an HTML report for easy analysis.

---


# Key Equations and Physics Modeling
## Ball Dynamics
The motion of the ball is governed by Newton's second law, incorporating forces such as gravity, drag, and the Magnus effect. Below are the key equations used in the simulation:

### Drag Force
The drag force opposes the ball's motion and depends on its velocity:

```
F_drag = (1/2) * rho * A * C_D * v^2
Where:
rho: Air density (1.225 kg/m³)
A: Cross-sectional area of the ball (pi * r^2)
C_D: Drag coefficient (0.47, adjusted for a table tennis ball)
v: Ball velocity
```
### Magnus Force
The Magnus force arises due to spin and creates lift perpendicular to the ball's motion:
```
F_Magnus = (1/2) * rho * A * C_L * r * |omega| * v
Where:
C_L: Lift coefficient (0.40, adjusted for topspin shots )
r: Radius of the ball (0.02 m)
omega: Angular velocity (spin rate)
```

### Acceleration Components
The acceleration components in the x and z directions are computed as:
```
a_x = -(F_drag / m) * (v_x / v)
a_z = -g - (F_drag / m) * (v_z / v) - sign(omega) * (F_Magnus / m)
```

## Impact Model
When the ball hits the racket, its post-impact velocities and spin are calculated based on the rubber's properties and the incoming ball's state. The key parameters include:

```
v_out_x = e_n * v_in_x + k_v * v_racket_x + 0.07 * omega_out * r
v_out_z = e_n * v_in_z + k_z * v_racket_z
omega_out = mu * omega_in + k_w * (v_rel_tangential / r) + 0.1 * (v_rel_normal / r)
Coefficient of restitution (e) : Determines how much kinetic energy is retained after impact.
Friction coefficient (mu) : Governs tangential velocity changes and spin generation.
Spin transfer coefficients (k_w) : Control how much spin is imparted to the ball.
The post-impact velocities and spin are computed as:
```
## Bounce Model
After hitting the table, the ball's velocities and spin are updated using a physics-based bounce model. The vertical velocity is reduced by the coefficient of restitution (e_table), while the horizontal velocity and spin are affected by friction and spin transfer:
```
v_z_post = -e_table * v_z_pre
delta_v_x = (5/7) * mu_bounce * r * omega
v_x_post = mu_bounce * v_x_pre + delta_v_x
omega_post = spin_transfer * omega_pre - (5 / (2 * r)) * (v_x_pre - delta_v_x)
```
## Rubber Properties
The coefficients for different rubbers were determined based on experimental data and manufacturers specifications. These coefficients include:
- **Friction coefficient ($\mu$)**: Represents the grip of the rubber and its ability to generate spin.
- **Restitution coefficient ($e$)**: Describes how "bouncy" the rubber is.
- **Velocity transfer coefficients ($k_v$, $k_z$)**: Indicate how much of the racket's velocity is transferred to the ball.
- **Spin transfer coefficient ($k_w$)**: Controls how effectively the rubber imparts spin to the ball.

Example values for popular rubbers:
```python
rubbers = {
    "Tenergy 05 FX": {"mu": 0.82, "e": 0.78, "kv_x": 0.67, "kv_z": 0.50, "kw": 0.75},
    "Tenergy 05": {"mu": 0.88, "e": 0.83, "kv_x": 0.74, "kv_z": 0.59, "kw": 0.85},
    "Dignics 05": {"mu": 0.93, "e": 0.87, "kv_x": 0.81, "kv_z": 0.67, "kw": 0.90},
    "Dignics 80": {"mu": 0.91, "e": 0.88, "kv_x": 0.81, "kv_z": 0.68, "kw": 0.90},
    "Dignics 09C": {"mu": 1.08, "e": 0.90, "kv_x": 0.85, "kv_z": 0.74, "kw": 1.05}
}
```

These values were fine-tuned to ensure realistic behavior in simulations. For example, Dignics rubbers are known for their high spin generation, so their kw values are higher compared to Tenergy rubbers.

## How We Obtained the Coefficients
The coefficients were derived from a combination of:

Experimental Data : High-speed camera studies and laboratory experiments measuring ball-rubber interactions.
Manufacturer Specifications : Information provided by rubber manufacturers about spin generation and control.
Iterative Testing : Adjusting coefficients in the simulation until the results matched observed real-world behavior.

## Usage
### Install the required libraries:
pip install numpy matplotlib

### Run the script:
python table_tennis_simulation.py

Open results/all_trajectories.html for an overview.