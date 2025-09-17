import math
from random import uniform
import numpy as np
from robot import Robot, MAX_SPEED, RAB_RANGE
from readings import Signal, Objects
import pygame

ALIGN_COEFFICIENT = 10
SEPARATION_COEFFICIENT = 3
COHESION_COEFFICIENT = 0.1

CLOSE_RANGE_RADIUS = RAB_RANGE / 3

class Boid(Robot):
    def __init__(
        self,
        id: int,
        pos: np.ndarray[tuple[int, ...], np.dtype[np.float64]],
        heading: float,
    ):
        super().__init__(id, pos, heading)
        

    def robot_controller(self, dispersion = True):
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
                self.disperse()
            else:
                self.flocking()

    def flocking(self):
        align_vector = self.calc_align_vector()
        separation_vector = self.calc_separation_vector()
        cohesion_vector = self.calc_cohesion_vector()

        total_vector = (
            align_vector * ALIGN_COEFFICIENT
            + separation_vector * SEPARATION_COEFFICIENT
            + cohesion_vector * COHESION_COEFFICIENT
        )

        delta_bearing = np.arctan2(total_vector[0], total_vector[1])
        if self.verbose:
            print(f"Aligning with other robots! Turning {delta_bearing}")
        self.set_rotation_and_speed(delta_bearing, MAX_SPEED)

        return True

    def get_close_boids(self) -> list[Signal]:
        boids : list[Signal] = self.rab_signals
        close_boids = [boid for boid in boids if boid.distance < CLOSE_RANGE_RADIUS]
        print("Close birds =", [cb.distance for cb in close_boids])
        return close_boids
    
    def get_far_boids(self) -> list[Signal]:
        boids : list[Signal] = self.rab_signals
        far_boids = [boid for boid in boids if boid.distance >= CLOSE_RANGE_RADIUS]
        print("Far birds =", [fb.distance for fb in far_boids])
        return far_boids
    
    def calc_align_vector(self):
        align_boids = self.get_far_boids()
        if len(align_boids) == 0:
            return np.array([0,0])
        align_boids_headings = [r.message.heading for r in align_boids]
        average_heading = calc_average_radian(align_boids_headings)
        align_vector = np.array([np.sin(average_heading), np.cos(average_heading)])
        return align_vector
    
    def calc_separation_vector(self):
        sep_boids = self.get_close_boids()
        if len(sep_boids) == 0:
            return np.array([0,0])
        bearings = [sb.bearing for sb in sep_boids]
        average_bearing = calc_average_radian(bearings)
        opposite_angle = calc_opposite_angle(average_bearing)
        separation_vector = np.array([np.sin(opposite_angle), np.cos(opposite_angle)])
        return separation_vector
    
    def calc_cohesion_vector(self):
        coh_boids = self.get_far_boids()
        if len(coh_boids) == 0:
            return np.array([0,0])
        vectors_to_other_boids = [np.array([np.sin(boid.bearing)*boid.distance, np.cos(boid.bearing)*boid.distance]) for boid in coh_boids]
        cohesion_vector = np.mean(vectors_to_other_boids, axis=0)
        return cohesion_vector



    def avoid(self, angle_readings):
        average_angle = sum(angle_readings) / len(angle_readings)
        if average_angle >= 0: # If wall is to the left or infront
            opposite_angle = average_angle - math.pi
        else:  # If wall is to the right
            opposite_angle = average_angle + math.pi

        # Add randomness
        random_angle_diff = uniform(math.pi * 0.1,math.pi * -0.1)
        target_angle = opposite_angle + random_angle_diff

        if self.verbose:
            print(f"Avoiding! Turning {target_angle}")
        self.set_rotation_and_speed(target_angle, MAX_SPEED)

    def calc_repulsion_distance(self):
        return RAB_RANGE * (1 - (self.light_intensity/2))

    def disperse(self):
        robot_angles = [
            r.bearing for r in self.rab_signals if r.distance <= self.calc_repulsion_distance()
        ] # Distances seem to be around 70 - 150 and light_intensity goes from 0 to 1 it seems.
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
        filered_list = [r for r in wall_reading_angles if r != self.prox_angles[2] and r != (self.prox_angles[3] - 2 * math.pi)]
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

    def draw_vector(self, screen, color, vector):
        pygame.draw.line(screen, color, self._pos, (self._pos + vector) * 0.5)
    
    def draw_vectors(self, screen):
        align_vector = self.calc_align_vector()
        self.draw_vector(screen, pygame.Color(255, 70, 255), align_vector)

        separation_vector = self.calc_separation_vector()
        self.draw_vector(screen, pygame.Color(255, 69, 69), separation_vector)

        cohesion_vector = self.calc_cohesion_vector()
        self.draw_vector(screen, pygame.Color(70, 255, 255), cohesion_vector)

        total_vector = (
            align_vector * ALIGN_COEFFICIENT
            + separation_vector * SEPARATION_COEFFICIENT
            + cohesion_vector * COHESION_COEFFICIENT
        )
        self.draw_vector(screen, pygame.Color(255, 255, 70), total_vector)



def calc_average_radian(radian_list):
    vector_list = []
    for angle in radian_list:
        vector_list.append([np.sin(angle), np.cos(angle)])

    average_vector = np.mean(vector_list, axis=0)
    average_angle = np.arctan2(average_vector[0], average_vector[1])
    
    return average_angle

def calc_opposite_angle(angle):
    if angle >= 0:
        return angle - math.pi
    else:
        return angle + math.pi