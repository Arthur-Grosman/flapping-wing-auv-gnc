import numpy as np


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class Controller:
    def __init__(self, config):
        self.config = config

        # Depth PID States
        self.int_z = 0.0
        self.prev_e_z = 0.0
        self.dt = config['simulation']['dt']

    def compute_depth_control(self, z, z_target):
        # PID to compute vertical control force u_z
        e_z = z_target - z
        self.int_z += e_z * self.dt
        d_e_z = (e_z - self.prev_e_z) / self.dt
        self.prev_e_z = e_z

        kp = self.config['control']['kp_z']
        ki = self.config['control']['ki_z']
        kd = self.config['control']['kd_z']

        u_z = (kp * e_z) + (ki * self.int_z) + (kd * d_e_z)
        return u_z

    def compute_flapping_freq(self, v, v_target):
        # P-control on velocity to demand thrust, mapped to frequency
        e_v = v_target - v
        thrust_demand = self.config['control']['kp_v'] * e_v + (self.config['vehicle']['c_d'] * v * abs(v))

        if thrust_demand <= 0:
            return 0.0

        f_sq = thrust_demand / self.config['vehicle']['k_f']
        f = np.sqrt(f_sq)
        return np.clip(f, 0, self.config['vehicle']['max_f'])

    def compute_turn_rate(self, theta, theta_des):
        # P-control on heading
        e_theta = wrap_to_pi(theta_des - theta)
        u_omega = self.config['control']['kp_psi'] * e_theta
        max_turn = self.config['vehicle']['max_turn_rate']
        return np.clip(u_omega, -max_turn, max_turn)