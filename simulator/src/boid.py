import math
from random import uniform
import numpy as np
from readings import Objects

from robot import Robot, MAX_SPEED


class Boid(Robot):
    def __init__(
        self,
        id: int,
        pos: np.ndarray[tuple[int, ...], np.dtype[np.float64]],
        heading: float,
    ):
        super().__init__(id, pos, heading)
        
    def robot_controller(self, dispersion=True):
        """
        Implement your control logic here.
        You can access:
        - self.rab_signals: list of received messages from other robots
        - self.prox_readings: proximity sensor data
        - self.light_intensity: light at current location

        Use only self.set_rotation_and_speed(...) to move the robot.

        DO NOT modify robot._linear_velocity or robot._angular_velocity directly. DO NOT modify move()
        """
        avoid_wall = self.avoid_wall()
        if not avoid_wall:
            if dispersion:
                avoid_robots = self.disperse()

            else:
                self.flocking()

    def avoid(self, angle_readings):
        average_angle = sum(angle_readings) / len(angle_readings)
        if average_angle >= 0:  # If wall is to the left or infront
            opposite_angle = average_angle - math.pi
        else:  # If wall is to the right
            opposite_angle = average_angle + math.pi

        # Add randomness
        random_angle_diff = uniform(math.pi * 0.1, math.pi * -0.1)
        target_angle = opposite_angle + random_angle_diff

        if self.verbose:
            print(f"Avoiding! Turning {target_angle}")
        self.set_rotation_and_speed(target_angle, MAX_SPEED)

    def disperse(self):
        robot_angles = [
            r.bearing
            for r in self.rab_signals
            if r.distance > self.light_intensity * 100
        ]  # Distances seem to be around 70 - 150 and light_intensity goes from 0 to 1 it seems.
        should_disperse = len(robot_angles) > 0

        if should_disperse:
            self.avoid(robot_angles)

        else:
            if self.verbose:
                print("Nothing is wrong:) Stopping and chilling.")
            self.set_rotation_and_speed(0, 0)

        return should_disperse

    def avoid_wall(self):
        wall_reading_angles = []

        for i, angle in enumerate(self.prox_angles):
            if self.prox_readings[i].reading_type == Objects.Wall:
                if angle > math.pi:
                    wall_reading_angles.append(angle - 2 * math.pi)
                else:
                    wall_reading_angles.append(angle)

        # See if we register any wall not behind us
        filered_list = [
            r
            for r in wall_reading_angles
            if r != self.prox_angles[2] and r != (self.prox_angles[3] - 2 * math.pi)
        ]
        should_activate = len(filered_list) > 0

        if should_activate:
            self.avoid(wall_reading_angles)

        return should_activate

    def _get_boids(self) -> list[dict[str, float]]:
        boids = []
        for sig in self.rab_signals:
            sig_angle = self._heading + self.rab_angles[sig.sensor_idx]
            relative_dir = np.array([np.cos(sig_angle), np.sin(sig_angle)])  # radians

            boids.append(
                {
                    "distance": sig.distance,
                    "angle": relative_dir,
                    "heading": sig.message.heading,
                }
            )

        return boids