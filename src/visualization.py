import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches


def plot_telemetry(history, config):
    t = history['t']

    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle('AUV GNC Telemetry', fontsize=16, fontweight='bold')

    # 1. Depth
    axs[0].plot(t, history['z'], label='Actual Depth', color='b', lw=2)
    axs[0].axhline(config['control']['z_target'], color='r', linestyle='--', label='Target Depth')
    axs[0].set_ylabel('Depth (m)')
    axs[0].legend()
    axs[0].grid(True, alpha=0.3)

    # 2. Velocity
    axs[1].plot(t, history['v'], label='Forward Velocity', color='g', lw=2)
    axs[1].axhline(config['control']['v_target'], color='r', linestyle='--', label='Target Velocity')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].legend()
    axs[1].grid(True, alpha=0.3)

    # 3. Flapping Frequency
    axs[2].plot(t, history['f'], label='Flapping Frequency', color='purple', lw=2)
    axs[2].set_ylabel('Frequency (Hz)')
    axs[2].legend()
    axs[2].grid(True, alpha=0.3)

    # 4. Control Signals
    axs[3].plot(t, history['u_omega'], label='Rate of Turn Cmd ($u_\\omega$)', color='orange', lw=2)
    axs[3].plot(t, history['u_z'], label='Vertical Thrust Cmd ($u_z$)', color='cyan', lw=2)
    axs[3].set_ylabel('Control Input')
    axs[3].set_xlabel('Time (s)')
    axs[3].legend()
    axs[3].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('telemetry_dashboard.png', dpi=300)
    print("Telemetry saved as telemetry_dashboard.png")


def animate_trajectory(history, env):
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_aspect('equal')
    ax.set_xlim(-2, 50)
    ax.set_ylim(-10, 15)
    ax.set_title('AUV Autonomous Navigation (APF + Waypoints)')
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.grid(True, linestyle=':', alpha=0.6)

    # Plot Environment
    for obs in env.obstacles:
        circle = patches.Circle((obs[0], obs[1]), obs[2], color='darkgreen', alpha=0.6, label='Kelp Obstacle')
        ax.add_patch(circle)

    wx, wy = env.waypoints[:, 0], env.waypoints[:, 1]
    ax.plot(wx, wy, 'rx', markersize=10, label='Waypoints')
    ax.plot(wx, wy, 'r--', alpha=0.3)

    # Remove duplicate labels
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper left')

    # Initialize Animation elements
    trail, = ax.plot([], [], 'b-', alpha=0.5, lw=2)
    robot, = ax.plot([], [], 'bo', markersize=8)
    heading_line, = ax.plot([], [], 'r-', lw=2)

    # Text element for displaying speed on screen (using axis-relative coordinates transform)
    speed_text = ax.text(0.95, 0.95, '', transform=ax.transAxes, ha='right', va='top',
                         fontsize=12, fontweight='bold', bbox=dict(facecolor='white', alpha=0.7, edgecolor='gray'))

    x_data = np.array(history['x'])
    y_data = np.array(history['y'])
    theta_data = np.array(history['theta'])
    v_data = np.array(history['v'])  # Extract velocity array

    def init():
        trail.set_data([], [])
        robot.set_data([], [])
        heading_line.set_data([], [])
        speed_text.set_text('')  # Reset text on initialization
        return trail, robot, heading_line, speed_text

    def update(frame):
        # Only plot every Nth frame to speed up animation rendering
        idx = frame * 5
        if idx >= len(x_data):
            idx = len(x_data) - 1

        trail.set_data(x_data[:idx], y_data[:idx])
        robot.set_data([x_data[idx]], [y_data[idx]])

        # Heading indicator
        hx = [x_data[idx], x_data[idx] + 1.5 * np.cos(theta_data[idx])]
        hy = [y_data[idx], y_data[idx] + 1.5 * np.sin(theta_data[idx])]
        heading_line.set_data(hx, hy)

        # Update the text overlay with current speed formatted to 2 decimal places
        speed_text.set_text(f'Speed: {v_data[idx]:.2f} m/s')

        return trail, robot, heading_line, speed_text

    frames = len(x_data) // 5
    # Note: speed_text is added to blit tracking list to ensure smooth rendering updates
    ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=50)

    try:
        ani.save('trajectory_animation.mp4', writer='ffmpeg', fps=30, dpi=200)
        print("Animation saved as trajectory_animation.mp4")
    except Exception as e:
        print(f"FFMPEG not found or error occurred. Saving as GIF instead. ({e})")
        ani.save('trajectory_animation.gif', writer='pillow', fps=30, dpi=150)