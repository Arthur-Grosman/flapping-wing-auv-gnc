import numpy as np


class AUVDynamics:
    def __init__(self, config):
        self.m = config['vehicle']['mass']
        self.k_f = config['vehicle']['k_f']
        self.c_d = config['vehicle']['c_d']
        self.c_dz = config['vehicle']['c_dz']
        self.dt = config['simulation']['dt']

        # State: [x, y, theta, v, z, z_dot]
        self.state = np.zeros(6)

    def step(self, f, u_omega, u_z):
        x, y, theta, v, z, z_dot = self.state

        # Thrust generation: F = k_f * f^2
        thrust = self.k_f * (f ** 2)

        # Surge dynamics
        drag = self.c_d * v * abs(v)
        v_dot = (thrust - drag) / self.m

        # Heave/Depth dynamics (simplified 2nd order)
        drag_z = self.c_dz * z_dot
        z_ddot = (u_z - drag_z) / self.m

        # Kinematics
        x_dot = v * np.cos(theta)
        y_dot = v * np.sin(theta)
        theta_dot = u_omega

        # Euler Integration
        self.state[0] += x_dot * self.dt
        self.state[1] += y_dot * self.dt
        self.state[2] += theta_dot * self.dt
        self.state[3] += v_dot * self.dt
        self.state[4] += z_dot * self.dt
        self.state[5] += z_ddot * self.dt

        # Wrap heading to [-pi, pi]
        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi