import math
from random import uniform
import numpy as np
from robot import Robot, MAX_SPEED, RAB_RANGE, rotate_vector
from readings import Signal, Objects
import pygame
from log import log_calculation, log_decorator

ALIGN_COEFFICIENT = 0
SEPARATION_COEFFICIENT = 0
COHESION_COEFFICIENT = 10

CLOSE_RANGE_RADIUS = RAB_RANGE / 3

class Boid(Robot):
    def __init__(
        self,
        id: int,
        pos: np.ndarray[tuple[int, ...], np.dtype[np.float64]],
        heading: float,
    ):
        self.align_vector = np.ndarray([0,0])
        self.separation_vector = np.ndarray([0,0])
        self.cohesion_vector = np.ndarray([0,0])
        self.target_vector = np.ndarray([0,0])
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
        # avoid_wall = self.avoid_wall()
        # if not avoid_wall:
        if dispersion:
            self.disperse()
        else:
            self.flocking()

    def flocking(self):
        target_vector = self._get_target_vector()

        delta_bearing = np.arctan2(target_vector[1], target_vector[0])
        print(f"{self.id = }", end =" ")
        log_calculation(np.arctan2, [target_vector], delta_bearing)
        self.set_rotation_and_speed(delta_bearing, MAX_SPEED)

        return True
    
    def _get_target_vector(self):
        self.align_vector = self.calc_align_vector() * ALIGN_COEFFICIENT
        self.separation_vector = self.calc_separation_vector() * SEPARATION_COEFFICIENT
        self.cohesion_vector = self.calc_cohesion_vector() * COHESION_COEFFICIENT

        self.target_vector = (self.align_vector + self.separation_vector + self.cohesion_vector)

        print(f"\nRobot[{str(self.id)}]: \n \
              \t target_vector = {np.round(self.target_vector, 2)} \n \
              \t \t separation_vector = {np.round(self.separation_vector, 2)} \n \
              \t \t separation_vector = {np.round(self.separation_vector, 2)} \n \
              \t \t cohesion_vector = {np.round(self.cohesion_vector, 2)}")

        return self.target_vector


    def get_close_boids(self) -> list[Signal]:
        boids : list[Signal] = self.rab_signals
        close_boids = [boid for boid in boids if boid.distance < CLOSE_RANGE_RADIUS]
        return close_boids
    
    def get_far_boids(self) -> list[Signal]:
        boids : list[Signal] = self.rab_signals
        far_boids = [boid for boid in boids if boid.distance >= CLOSE_RANGE_RADIUS]
        return far_boids
    
    def calc_align_vector(self):
        align_boids = self.get_far_boids()
        if len(align_boids) == 0:
            return np.array([0,0])

        align_boids_headings = [r.message.heading for r in align_boids]
        average_heading = calc_average_radian(align_boids_headings)
        align_vector = np.array([np.cos(average_heading), np.sin(average_heading)])
        return align_vector
    
    def calc_separation_vector(self):
        sep_boids = self.get_close_boids()
        if len(sep_boids) == 0:
            return np.array([0,0])
        avg_bearing = self.get_average_bearing(sep_boids)
        separation_vector = avg_bearing * -1
        return np.array(separation_vector)
    
    def calc_cohesion_vector(self):
        coh_boids = self.get_far_boids()
        if len(coh_boids) == 0:
            return np.array([0,0])

        print(f"\nRobot[{str(self.id)}]: \n \
              \t boids_in_range = {[f"sensor_idx={sb.sensor_idx}, bearing={sb.bearing}, distance={sb.distance}, Message:(heading={sb.message.heading}, comm_signal={sb.message.comm_signal})" for sb in coh_boids]}")
        avg_bearing = self.get_average_bearing(coh_boids) # cohesion vector
        return avg_bearing

    def get_average_bearing(self, boids_in_range: list[Signal]):
        bearings = np.array([(np.cos(sb.bearing), np.sin(sb.bearing)) for sb in boids_in_range])
        average_bearing = np.mean(bearings, axis=0)
        return average_bearing


        



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
            relative_dir = np.array([np.sin(sig_angle), np.cos(sig_angle)])  # radians

            boids.append(
                {
                    "distance": sig.distance,
                    "angle": relative_dir,
                    "heading": sig.message.heading,
                }
            )

        return boids

    def draw_vector(self, screen, color, vector):
        # print(f"draw_vector(screen, {color}, {vector}) -> ({self._pos}, {self._pos + vector * 5})")
        pygame.draw.line(screen, color, self._pos, self._pos + vector * 5, 2)

    def draw_vectors(self, screen):
        self.draw_vector(screen, pygame.Color(255, 70, 255), self.align_vector) # Magenta
        self.draw_vector(screen, pygame.Color(255, 69, 69), self.separation_vector) # Red
        self.draw_vector(screen, pygame.Color(70, 255, 255), self.cohesion_vector) # Cyan
        self.draw_vector(screen, pygame.Color(255, 255, 70), self.target_vector) # Yellow



def calc_average_radian(radian_list):
    vector_list = []
    for angle in radian_list:
        vector_list.append([np.cos(angle), np.sin(angle)])

    average_vector = np.mean(vector_list, axis=0)
    average_angle = np.arctan2(average_vector[1], average_vector[0])


    # print(f"calc_average_radian({radian_list}) -> {average_vector} -> {average_angle}")
    
    return average_angle

def calc_opposite_angle(angle):
    if angle >= 0:
        # print(f"calc_opposite_angle({angle}) -> {angle - math.pi}")
        return angle - math.pi
    else:
        # print(f"calc_opposite_angle({angle}) -> {angle + math.pi}")
        return angle + math.pi