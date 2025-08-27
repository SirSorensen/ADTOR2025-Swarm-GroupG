import numpy as np
import pygame
from consts import HEIGHT, WIDTH
from light_source import _get_light_intensity

# Parameters
NUM_ROBOTS = 2
ROBOT_RADIUS = 10

NUM_PROX_SENSORS = 6
NUM_RAB_SENSORS = 12

PROX_SENSOR_RANGE = 60  # pixels
RAB_RANGE = 150  # pixels

MAX_SPEED = 50
MAX_TURN = 3  # radians/sec - a real robot like the e-puck or TurtleBot typically turns at 90–180 deg/sec (≈ 1.5–3.1 rad/sec)

ROBOT_COLOR = (200, 255, 255)

# noise in the motion model (simulates actuation/motor errors)
MOTION_NOISE_STD = 0 # Try 0.5   # Positional noise in dx/dy (pixels)
HEADING_NOISE_STD = 0 # Try 0.01 # Rotational noise in heading (radians)

# sensor noise and dropout
RAB_NOISE_BEARING = 0 # std dev of directional noise to bearing in RAB:  0.1 rad. = ~5.7 degree in the bearing
RAB_DROPOUT = 0  # chance to drop a signal
ORIENTATION_NOISE_STD = 0 # noise in IMU readings of the robot’s own orientation

class Robot:
    def __init__(self, id, pos, heading):
        self.id = id
        self._pos = np.array(pos, dtype=float)
        self._heading = heading
        self._radius = ROBOT_RADIUS
        self._linear_velocity = MAX_SPEED * 0.5
        self._angular_velocity = 0

        #### Sensor readings
        # proximity sensors
        self.prox_angles = np.pi / NUM_PROX_SENSORS + np.linspace(0, 2 * np.pi, NUM_PROX_SENSORS, endpoint=False)
        self.prox_readings = [
            {"distance": PROX_SENSOR_RANGE, "type": None}
            for _ in range(NUM_PROX_SENSORS)
        ]
        # RAB sensor
        self.rab_angles = np.pi / NUM_RAB_SENSORS + np.linspace(0, 2 * np.pi, NUM_RAB_SENSORS, endpoint=False)
        self.rab_signals = []
        # light sensor
        self.light_intensity = 0.0
        # IMU (Inertial Measurement Unit) sensor providing robot's orientation
        self.orientation = 0.0


    def move(self, dt):
        # Update heading
        self._heading += self._angular_velocity * dt
        self._heading += np.random.normal(0, HEADING_NOISE_STD)
        self._heading %= 2 * np.pi  # keep in [0, 2π)

        # Update position
        dx = self._linear_velocity * np.cos(self._heading) * dt
        dy = self._linear_velocity * np.sin(self._heading) * dt
        dx += np.random.normal(0, MOTION_NOISE_STD)
        dy += np.random.normal(0, MOTION_NOISE_STD)
        self._pos += np.array([dx, dy])

        # Arena bounds clipping
        self._pos[0] = np.clip(self._pos[0], self._radius, WIDTH - self._radius)
        self._pos[1] = np.clip(self._pos[1], self._radius, HEIGHT - self._radius)


    def compute_distance_to_wall(self, direction, bounds, max_range):
        x, y = self._pos
        dx, dy = direction
        distances = []
        if dx != 0:
            if dx > 0:
                t = (bounds['right'] - x) / dx
            else:
                t = (bounds['left'] - x) / dx
            if t > 0:
                distances.append(t)
        if dy != 0:
            if dy > 0:
                t = (bounds['bottom'] - y) / dy
            else:
                t = (bounds['top'] - y) / dy
            if t > 0:
                distances.append(t)
        wall_distance = min(distances) if distances else max_range
        wall_distance = max(wall_distance,0)
        return min(wall_distance, max_range)

    def read_sensors(self, robots, obstacles, arena_bounds):

        # Empty the sensors
        self.prox_readings = [
            {"distance": PROX_SENSOR_RANGE, "type": None}
            for _ in range(NUM_PROX_SENSORS)
        ]
        self.rab_signals = []

        # Light sensing
        self.light_intensity = _get_light_intensity(self._pos)

        # Detect other robots
        for other in robots:
            if other.id == self.id:
                continue

            rel_vec = other._pos - self._pos
            distance = max(0,np.linalg.norm(rel_vec) - other._radius)
            if distance > max(PROX_SENSOR_RANGE, RAB_RANGE):
                continue

            bearing = (np.arctan2(rel_vec[1], rel_vec[0]) - self._heading) % (2 * np.pi)

            # Communication (RAB)
            if distance <= RAB_RANGE:
                # signal dropout
                dropout_probability = RAB_DROPOUT  # chance to drop a signal

                if np.random.rand() > dropout_probability:
                    # adding noise (directional error) to bearing
                    bearing = (bearing + np.random.normal(0, RAB_NOISE_BEARING)) % (2 * np.pi)

                    rab_idx = int((bearing / (2 * np.pi)) * NUM_RAB_SENSORS)

                    self.rab_signals.append({
                        'message': {'heading': other.orientation},
                        'distance': distance,
                        'bearing': self.rab_angles[rab_idx], # local
                        'sensor_idx': rab_idx,
                        'intensity': 1 / ((distance / RAB_RANGE) ** 2 + 1),
                    })

            # Also treat robot as obstacle (for IR)
            if distance <= PROX_SENSOR_RANGE:
                prox_idx = int((bearing / (2 * np.pi)) * NUM_PROX_SENSORS)
                if distance < self.prox_readings[prox_idx]["distance"]:
                    self.prox_readings[prox_idx] = {"distance": distance, "type": "robot"}

        # Detect obstacles
        for obs in obstacles:
            rel_vec = obs.pos - self._pos
            distance = max(0,np.linalg.norm(rel_vec) - obs.radius)
            if distance <= PROX_SENSOR_RANGE:
                bearing = (np.arctan2(rel_vec[1], rel_vec[0]) - self._heading) % (2 * np.pi)
                prox_idx = int((bearing / (2 * np.pi)) * NUM_PROX_SENSORS)
                if distance < self.prox_readings[prox_idx]["distance"]:
                    self.prox_readings[prox_idx] = {"distance": distance, "type": "obstacle"}

        # Wall sensing (raycast style)
        for i, angle in enumerate(self.prox_angles):
            global_angle = (self._heading + angle) % (2 * np.pi)
            direction = np.array([np.cos(global_angle), np.sin(global_angle)])
            wall_dist = self.compute_distance_to_wall(direction, arena_bounds, PROX_SENSOR_RANGE)
            if wall_dist <self.prox_readings[i]["distance"]:
                self.prox_readings[i] = {"distance": wall_dist, "type": "wall"}

        # Read IMU for own orientation
        self.orientation = (self._heading + np.random.normal(0, ORIENTATION_NOISE_STD)) % (2 * np.pi)


    def _set_velocity(self, linear, angular):
        # Internal use only. Use set_rotation_and_speed instead
        assert 0 <= linear <= MAX_SPEED, "Linear velocity out of bounds"
        assert -MAX_TURN <= angular <= MAX_TURN, "Angular velocity out of bounds"
        self._linear_velocity = linear
        self._angular_velocity = angular

    def compute_angle_diff(self, target_angle):
        # Returns shortest signed angle between current heading and target
        return (target_angle - self._heading + np.pi) % (2 * np.pi) - np.pi

    def get_relative_heading(self, neighbor_heading):
        # Convert a neighbor's global heading into this robot's local frame (radians).
        #     Positive = CCW, Negative = CW.
        return (neighbor_heading - self.orientation + np.pi) % (2 * np.pi) - np.pi

    def set_rotation_and_speed(self, delta_bearing, target_speed, kp=0.5):
        """
        Sets angular and linear velocity using a proportional controller
        to achieve the given relative turn (rotation) with the given target speed.
        Robot-frame API: delta_bearing is relative to current heading (rad)
        + = turn left (CCW), - = turn right (CW).
        """
        target_heading = (self._heading + delta_bearing) % (2 * np.pi)
        angle_diff = self.compute_angle_diff(target_heading)
        angular_velocity = np.clip(kp * angle_diff, -MAX_TURN, MAX_TURN)
        target_speed = np.clip(target_speed, 0, MAX_SPEED)
        # Slow down when turning sharply
        linear_velocity = target_speed * (1 - min(abs(angle_diff) / np.pi, 1)) * 0.9 + 0.1
        self._set_velocity(linear_velocity, angular_velocity)

    def robot_controller(self):
        """
            Implement your control logic here.
            You can access:
            - self.rab_signals: list of received messages from other robots
            - self.prox_readings: proximity sensor data
            - self.light_intensity: light at current location

            Use only self.set_rotation_and_speed(...) to move the robot.

            DO NOT modify robot._linear_velocity or robot._angular_velocity directly. DO NOT modify move()
            """
        # Example: move forward
        self.set_rotation_and_speed(0, MAX_SPEED * 0.5)

    def draw(self, screen):
        # --- IR proximity sensors ---
        for i, reading in enumerate(self.prox_readings):
            dist = reading["distance"]
            obj_type = reading["type"]

            angle = self._heading + self.prox_angles[i]
            sensor_dir = np.array([np.cos(angle), np.sin(angle)])
            end_pos = self._pos + sensor_dir * dist

            # Color code by detected object type
            if obj_type == "robot":
                color = (0, 150, 255)  # Blue
            elif obj_type == "obstacle":
                color = (255, 165, 0)  # Orange
            elif obj_type == "wall":
                color = (255, 255, 100)  # Yellow
            else:
                color = (20, 80, 20)  # Green (no hit)

            pygame.draw.line(screen, color, self._pos, end_pos, 2)
            pygame.draw.circle(screen, color, end_pos.astype(int), 3)

        # --- RAB signals ---
        for sig in self.rab_signals:
            sig_angle = self._heading + self.rab_angles[sig['sensor_idx']]
            sensor_dir = np.array([np.cos(sig_angle), np.sin(sig_angle)])

            start = self._pos + sensor_dir * (self._radius + 3)
            end = self._pos + sensor_dir * (self._radius + 3 + sig['distance'])

            intensity_color = 55+int(200 * (sig['intensity']*2-1))
            color = (intensity_color, 50, intensity_color)

            pygame.draw.line(screen, color, start, end, 2)

        # --- Robot body ---
        pygame.draw.circle(screen, ROBOT_COLOR, self._pos.astype(int), self._radius)

        # --- Heading indicator ---
        heading_vec = rotate_vector(np.array([self._radius + 2, 0]), self._heading)
        pygame.draw.line(screen, ROBOT_COLOR, self._pos, self._pos + heading_vec, 3)

def rotate_vector(vec, angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([c*vec[0] - s*vec[1], s*vec[0] + c*vec[1]])