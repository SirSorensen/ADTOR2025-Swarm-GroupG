from enum import Enum


class Message:
    def __init__(self, heading: float, comm_signal):
        self.heading = heading
        self.comm_signal = comm_signal


class Objects(Enum):
    Robot = 0
    Obstacle = 1
    Wall = 2


class Signal:
    def __init__(
        self,
        message: Message,
        distance: float,
        bearing: float,
        sensor_idx: int,
        intensity: float,
    ):
        self.message = message
        self.distance = distance
        self.bearing = bearing
        self.sensor_idx = sensor_idx
        self.intensity = intensity


class Reading:
    def __init__(self, distance: float, reading_type: Objects):
        self.distance = distance
        self.reading_type = reading_type
