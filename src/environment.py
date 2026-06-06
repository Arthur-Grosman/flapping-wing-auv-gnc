import numpy as np


class Environment:
    def __init__(self):
        # [x, y] coordinates for waypoints
        self.waypoints = np.array([
            [0.0, 0.0],
            [10.0, 5.0],
            [20.0, -2.0],
            [35.0, 8.0],
            [45.0, 0.0]
        ])

        # [x, y, radius] for kelp/obstacles
        self.obstacles = np.array([
            [5.0, 2.0, 1.5],
            [15.0, 2.0, 2.0],
            [15.0, -1.0, 1.5],
            [22.0, 2.0, 2.5],
            [25.0, 5.0, 2.0],
            [30.0, 0.0, 1.5],
            [30.0, 5.5, 1.0],
            [35.0, 2.0, 2.5],
            [39.0, 7.0, 1.5],
            [41.5, 4.0, 1.5]
        ])