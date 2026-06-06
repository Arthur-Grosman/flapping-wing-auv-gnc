import numpy as np


class GuidanceSystem:
    def __init__(self, env, config):
        self.env = env
        self.config = config
        self.current_wp_idx = 1  # Start heading to WP 1 (WP 0 is start)

    def get_desired_heading(self, x, y):
        # 1. Waypoint Management
        if self.current_wp_idx >= len(self.env.waypoints):
            # End of mission: hold last heading
            wp = self.env.waypoints[-1]
        else:
            wp = self.env.waypoints[self.current_wp_idx]
            dist_to_wp = np.hypot(wp[0] - x, wp[1] - y)
            if dist_to_wp < self.config['guidance']['waypoint_radius']:
                self.current_wp_idx += 1
                if self.current_wp_idx < len(self.env.waypoints):
                    wp = self.env.waypoints[self.current_wp_idx]

        # 2. Attractive Vector (Line of Sight to Waypoint)
        v_att = np.array([wp[0] - x, wp[1] - y])
        v_att = v_att / (np.linalg.norm(v_att) + 1e-6)  # Normalize

        # 3. Repulsive Vector (APF from Obstacles)
        v_rep = np.array([0.0, 0.0])
        rho_0 = self.config['guidance']['apf_influence_radius']
        eta = self.config['guidance']['apf_repulsive_gain']

        for obs in self.env.obstacles:
            ox, oy, rad = obs
            dist = np.hypot(x - ox, y - oy) - rad

            if dist < rho_0 and dist > 0:
                # Repulsive magnitude grows as distance approaches 0
                rep_mag = eta * (1.0 / dist - 1.0 / rho_0) * (1.0 / (dist ** 2))
                vec_away = np.array([x - ox, y - oy])
                vec_away = vec_away / np.linalg.norm(vec_away)
                v_rep += rep_mag * vec_away

        # 4. Resultant Vector
        v_res = v_att + v_rep
        theta_des = np.arctan2(v_res[1], v_res[0])

        mission_complete = self.current_wp_idx >= len(self.env.waypoints)
        return theta_des, mission_complete