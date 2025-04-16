import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Switch to Tkinter backend [[5]]
import matplotlib.pyplot as plt
import os




# Physical Constants
g = 9.81       # Gravitational acceleration (m/s²)
m = 0.0027     # Ball mass (kg)
r = 0.02       # Ball radius (m)
rho = 1.225    # Air density (kg/m³)
A = np.pi*r**2 # Cross-sectional area (m²)
C_D = 0.47     # Drag coefficient
C_L = 0.18     # Lift coefficient (adjusted down)

# Table Dimensions (ITTF standard)
table_length = 2.74
table_height = 0.76
net_height = 0.1525
half_table = table_length/2
total_net_height = table_height + net_height

# Simulation Parameters
x0 = -2.37            # Starting position (m)
impact_heights = [0.15, 0.20, 0.25]  # Realistic impact heights (m)
incoming_speed = 7.5  # m/s (27 km/h)
incoming_angle = -4.0*np.pi/180  # Slight downward angle
incoming_spin = 1800*2*np.pi/60  # 1800 rpm backspin
racket_speed = 7.0    # m/s (25.2 km/h)

# Realistic Rubber Properties
# Realistic Rubber Properties (Updated with new rubbers)
rubbers = {
    "Tenergy 05 FX": {"mu": 0.75, "e": 0.72, "kv_x": 0.55, "kv_z": 0.48, "kw": 0.65},
    "Tenergy 05": {"mu": 0.78, "e": 0.75, "kv_x": 0.62, "kv_z": 0.55, "kw": 0.72},
    "Tenergy 05 Hard": {"mu": 0.80, "e": 0.76, "kv_x": 0.64, "kv_z": 0.56, "kw": 0.74},  
    "Tenergy 19": {"mu": 0.77, "e": 0.74, "kv_x": 0.60, "kv_z": 0.52, "kw": 0.70},
    "Dignics 09C": {"mu": 0.85, "e": 0.80, "kv_x": 0.70, "kv_z": 0.65, "kw": 0.82},
    "Dignics 05": {"mu": 0.82, "e": 0.78, "kv_x": 0.68, "kv_z": 0.60, "kw": 0.78},
    "Dignics 80": {"mu": 0.80, "e": 0.77, "kv_x": 0.65, "kv_z": 0.58, "kw": 0.75},
    "Dignics 64": {"mu": 0.83, "e": 0.79, "kv_x": 0.69, "kv_z": 0.62, "kw": 0.80},      
}

# Improved bounce parameters (Adjusted for realism)
e_table = 0.70  # Vertical Coefficient of Restitution (adjusted)
mu_bounce = 0.80  # Horizontal friction coefficient (adjusted)
spin_transfer = 0.55  # Adjusted spin retention after bounce
I = (2 / 5) * m * r**2  # Moment of inertia for solid sphere

def calculate_post_impact(rubber, impact_height, swing_angle):
    """Improved impact model with rotational energy consideration."""
    mu, e_n = rubber["mu"], rubber["e"]
    kv_x, kv_z, kw = rubber["kv_x"], rubber["kv_z"], rubber["kw"]

    # Pre-impact velocities
    v_in_x = incoming_speed * np.cos(incoming_angle)
    v_in_z = incoming_speed * np.sin(incoming_angle)
    v_racket_x = racket_speed * np.cos(swing_angle)
    v_racket_z = racket_speed * np.sin(swing_angle)

    # Normal and tangential components
    v_rel_tangential = (v_racket_x - v_in_x) + (incoming_spin * r)
    v_rel_normal = v_racket_z - v_in_z

    # Enhanced spin generation
    omega_out = (mu * incoming_spin + kw * v_rel_tangential / r + 0.1 * v_rel_normal / r)

    # Energy-corrected velocity outputs
    v_out_x = e_n * v_in_x + kv_x * v_racket_x + 0.07 * omega_out * r
    v_out_z = e_n * v_in_z + kv_z * v_racket_z

    return v_out_x, v_out_z, omega_out

def ball_dynamics(y, omega):
    """Defines the dynamics of the ball in flight."""
    x, z, vx, vz = y
    v = np.sqrt(vx**2 + vz**2)

    # Drag force
    F_drag = 0.5 * rho * A * C_D * v**2
    a_drag = F_drag / m

    # Magnus force
    a_lift = 0.5 * rho * A * C_L * r * abs(omega) / m

    # Acceleration components
    ax = -a_drag * (vx / v) if v != 0 else 0
    az = -g - a_drag * (vz / v) - np.sign(omega) * a_lift if v != 0 else -g

    return np.array([vx, vz, ax, az])

def simulate_trajectory(y0, omega, dt=0.0005, max_time=2.0):
    """Simulates the ball trajectory using 4th-order Runge-Kutta integration."""
    trajectory = [y0]
    t = 0
    while t < max_time:
        current_state = trajectory[-1]
        if current_state[1] < table_height or abs(current_state[0]) > half_table + 1:
            break  # Stop if ball hits the table or goes out of bounds
        derivatives = ball_dynamics(current_state, omega)
        new_state = current_state + derivatives * dt
        trajectory.append(new_state)
        t += dt
    return np.array(trajectory).T

def simulate_after_bounce(y0, omega, dt=0.0005, max_time=2.0):
    """Simulates the ball trajectory after bouncing."""
    trajectory = [y0]
    t = 0
    while t < max_time:
        current_state = trajectory[-1]
        if current_state[1] < 0 or abs(current_state[0]) > half_table + 1:  # Ball hits the floor or goes out of bounds
            break
        derivatives = ball_dynamics(current_state, omega)
        new_state = current_state + derivatives * dt
        trajectory.append(new_state)
        t += dt
    return np.array(trajectory).T

def refined_bounce(landing_vx, landing_vz, omega):
    """Physics-based bounce model incorporating spin effects."""
    # Normal direction
    vz_post = -e_table * landing_vz

    # Tangential direction with spin interaction
    delta_vx = (5 / 7) * mu_bounce * r * omega
    vx_post = mu_bounce * landing_vx + delta_vx

    # Spin alteration during bounce
    omega_post = (spin_transfer * omega - (5 / (2 * r)) * (landing_vx - delta_vx))

    return vx_post, vz_post, omega_post

def generate_plot(rubber_name, impact_height, angles):
    """Generates plots for ball trajectories."""
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_xlim(-3, 3)
    ax.set_ylim(0, 2.5)
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Height (m)')
    ax.set_title(f'Trajectories: {rubber_name}, Height: {impact_height*100}cm')
    ax.grid(True)

    # Draw table and net
    ax.plot([-half_table, half_table], [table_height, table_height], 'k-', linewidth=2, label="Table")
    ax.plot([0, 0], [table_height, total_net_height], 'r-', linewidth=2, label="Net")

    # Draw people
    colors = plt.cm.tab10.colors
    for i, swing_angle in enumerate(angles):
        swing_angle_rad = swing_angle * np.pi / 180
        v_out_x, v_out_z, omega_out = calculate_post_impact(rubbers[rubber_name], impact_height, swing_angle_rad)
        y0 = np.array([x0, table_height + impact_height, v_out_x, v_out_z])

        # Simulate pre-bounce trajectory
        trajectory = simulate_trajectory(y0, omega_out)
        x_positions, z_positions = trajectory[0], trajectory[1]

        # Find landing point and adjust for bounce
        landing_x = x_positions[-1]
        landing_vx = trajectory[2][-1]
        landing_vz = trajectory[3][-1]
        landing_omega = omega_out

        # Post-bounce conditions
        post_bounce_vx, post_bounce_vz, post_bounce_omega = refined_bounce(landing_vx, landing_vz, landing_omega)
        y0_bounce = np.array([landing_x, table_height, post_bounce_vx, post_bounce_vz])

        # Simulate post-bounce trajectory
        trajectory_bounce = simulate_after_bounce(y0_bounce, post_bounce_omega)
        x_bounce, z_bounce = trajectory_bounce[0], trajectory_bounce[1]

        # Combine trajectories
        x_combined = np.concatenate((x_positions, x_bounce))
        z_combined = np.concatenate((z_positions, z_bounce))

        # Plot trajectory
        ax.plot(x_combined, z_combined, color=colors[i % len(colors)], linewidth=1.5, alpha=0.7,
                label=f"{swing_angle}°")

    # Add legend
    ax.legend(loc='upper right')

    # Save figure
    filename = f"trajectories_{rubber_name.replace(' ', '_')}_{int(impact_height*100)}cm.png"
    plt.savefig('results/' + filename, dpi=300, bbox_inches='tight')
    plt.close()
    return filename

def generate_all_plots():
    """Generates plots for all rubbers and impact heights."""
    results = []
    sample_rubbers = list(rubbers.keys())
    sample_heights = [0.10, 0.20, 0.30]
    sample_angles = [20, 30, 40, 50, 60, 70, 80]

    for rubber_name in sample_rubbers:
        for height in sample_heights:
            try:
                filepath = generate_plot(rubber_name, height, sample_angles)
                results.append({
                    'rubber': rubber_name,
                    'height_cm': height * 100,
                    'filepath': filepath
                })
                print(f"Generated trajectories for {rubber_name}, height: {height*100}cm")
            except Exception as e:
                print(f"Error generating trajectories for {rubber_name}, height: {height*100}cm: {e}")

    # Generate HTML file
    with open('./results/all_trajectories.html', 'w') as f:
        f.write("<html><body>\n")
        f.write("<h1>Table Tennis Ball Trajectories</h1>\n")
        for result in results:
            f.write(f"<h2>{result['rubber']}, Height: {result['height_cm']}cm</h2>\n")
            f.write(f'<img src="{result["filepath"]}" alt="Trajectories">\n')
        f.write("</body></html>")
    with open('./results/all_trajectories.md', 'w') as f:
        for result in results:

            f.write('![alt text](' + result["filepath"] + ' "' + result['rubber'] + ' at height ' + str(result['height_cm']) + '")\n')

generate_all_plots()