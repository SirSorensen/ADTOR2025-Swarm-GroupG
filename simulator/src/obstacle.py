import numpy as np


class Obstacle:
    def __init__(self, pos, radius):
        self.pos = np.array(pos, dtype=float)
        self.radius = radius
        self.type = "obstacle"  # used in sensing

OBSTACLES : list[Obstacle] = [
    #Obstacle(pos=(200, 150), radius=20),
    #Obstacle(pos=(600, 120), radius=30),
]