# Table Tennis Ball Trajectory Simulation

## Purpose
This project simulates the trajectories of table tennis balls during topspin shots against topspin shots, taking into account realistic physical parameters such as ball dynamics, rubber properties, and bounce mechanics. The goal is to analyze how different rubbers and impact conditions affect the ball's trajectory and behavior on the table. This simulation can be useful for:
- Table tennis players who want to understand the effects of different rubbers.
- Coaches and researchers studying the physics of table tennis.
- Equipment manufacturers designing new rubber materials.

The simulation generates visual plots of ball trajectories and organizes them into an HTML report for easy analysis.

---

## Key Equations and Physics Modeling

### 1. **Ball Dynamics**
The motion of the ball is governed by Newton's second law, incorporating forces such as gravity, drag, and the Magnus effect. The equations of motion are:

#### Drag Force
The drag force opposes the ball's motion and depends on its velocity:
$$ F_{\text{drag}} = \frac{1}{2} \rho A C_D v^2 $$
Where:
- $\rho$: Air density (1.225 kg/m³)
- $A$: Cross-sectional area of the ball ($\pi r^2$)
- $C_D$: Drag coefficient (0.47, adjusted for a table tennis ball [[9]])
- $v$: Ball velocity

#### Magnus Force
The Magnus force arises due to spin and creates lift perpendicular to the ball's motion:
$$ F_{\text{Magnus}} = \frac{1}{2} \rho A C_L r |\omega| v $$
Where:
- $C_L$: Lift coefficient (0.40, adjusted for topspin shots [[1]])
- $r$: Radius of the ball (0.02 m)
- $\omega$: Angular velocity (spin rate)

#### Acceleration Components
The acceleration components in the x and z directions are computed as:
$$ a_x = -\frac{F_{\text{drag}}}{m} \frac{v_x}{v} $$
$$ a_z = -g - \frac{F_{\text{drag}}}{m} \frac{v_z}{v} - \text{sign}(\omega) \frac{F_{\text{Magnus}}}{m} $$

These equations are solved numerically using the 4th-order Runge-Kutta method to simulate the ball's trajectory.

---

### 2. **Impact Model**
When the ball hits the racket, its post-impact velocities and spin are calculated based on the rubber's properties and the incoming ball's state. The key parameters include:
- **Coefficient of restitution ($e$)**: Determines how much kinetic energy is retained after impact.
- **Friction coefficient ($\mu$)**: Governs tangential velocity changes and spin generation.
- **Spin transfer coefficients ($k_w$)**: Control how much spin is imparted to the ball.

The post-impact velocities and spin are computed as:
$$ v_{\text{out},x} = e_n v_{\text{in},x} + k_v v_{\text{racket},x} + 0.07 \omega_{\text{out}} r $$
$$ v_{\text{out},z} = e_n v_{\text{in},z} + k_z v_{\text{racket},z} $$
$$ \omega_{\text{out}} = \mu \omega_{\text{in}} + k_w \frac{v_{\text{rel},\text{tangential}}}{r} + 0.1 \frac{v_{\text{rel},\text{normal}}}{r} $$

---

### 3. **Bounce Model**
After hitting the table, the ball's velocities and spin are updated using a physics-based bounce model. The vertical velocity is reduced by the coefficient of restitution ($e_{\text{table}}$), while the horizontal velocity and spin are affected by friction and spin transfer:
$$ v_{z,\text{post}} = -e_{\text{table}} v_{z,\text{pre}} $$
$$ \Delta v_x = \frac{5}{7} \mu_{\text{bounce}} r \omega $$
$$ v_{x,\text{post}} = \mu_{\text{bounce}} v_{x,\text{pre}} + \Delta v_x $$
$$ \omega_{\text{post}} = \text{spin\_transfer} \cdot \omega_{\text{pre}} - \frac{5}{2r} (v_{x,\text{pre}} - \Delta v_x) $$

---

## Rubber Properties
The coefficients for different rubbers were determined based on experimental data and manufacturer specifications. These coefficients include:
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