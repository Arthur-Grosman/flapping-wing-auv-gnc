import yaml
import numpy as np
from src.environment import Environment
from src.dynamics import AUVDynamics
from src.controls import Controller
from src.guidance import GuidanceSystem
from src.visualization import plot_telemetry, animate_trajectory


def main():
    # 1. Load Configuration
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)

    # 2. Initialize Modules
    env = Environment()
    dynamics = AUVDynamics(config)
    controller = Controller(config)
    guidance = GuidanceSystem(env, config)

    # Dynamics start slightly off depth to demonstrate PID
    dynamics.state[4] = 0.0  # start at surface

    # 3. Data Logging
    history = {k: [] for k in ['t', 'x', 'y', 'theta', 'v', 'z', 'f', 'u_omega', 'u_z']}

    time_steps = np.arange(0, config['simulation']['max_time'], config['simulation']['dt'])

    print("Starting Simulation...")

    # 4. Simulation Loop
    for t in time_steps:
        x, y, theta, v, z, z_dot = dynamics.state

        # Guidance (APF + Waypoint)
        theta_des, mission_complete = guidance.get_desired_heading(x, y)

        # Control
        f = controller.compute_flapping_freq(v, config['control']['v_target'])
        u_omega = controller.compute_turn_rate(theta, theta_des)
        u_z = controller.compute_depth_control(z, config['control']['z_target'])

        # Stop flapping if mission is complete
        if mission_complete:
            f = 0.0

        # Step Dynamics
        dynamics.step(f, u_omega, u_z)

        # Record Telemetry
        history['t'].append(t)
        history['x'].append(x)
        history['y'].append(y)
        history['theta'].append(theta)
        history['v'].append(v)
        history['z'].append(z)
        history['f'].append(f)
        history['u_omega'].append(u_omega)
        history['u_z'].append(u_z)

        if mission_complete and v < 0.1:
            print(f"Mission complete at t={t:.2f}s")
            break

    print("Simulation finished. Generating visualizations...")

    # 5. Visualization
    plot_telemetry(history, config)
    animate_trajectory(history, env)


if __name__ == "__main__":
    main()