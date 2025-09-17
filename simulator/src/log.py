import pygame
from robot import Robot
from consts import WIDTH, HEIGHT

pixel_map : list[list[int]] = [[0 for _ in range(WIDTH)] for _ in range(HEIGHT)]

def logging_init(): #initialize your log file
    pass

def log_metrics(frame_count, total_time, metrics): # write to your log file
    pass

def logging_close(): # close your log file
    pass

# Lav et heatmap
def compute_metrics(robots : list[Robot]): # pass as many arguments as you need and compute relevant metrics to be logged for performance analysis
    from robot import ROBOT_RADIUS
    for robot in robots:
        x, y = robot._pos
        x = int(x)
        y = int(y)
        for dx in range(-ROBOT_RADIUS, ROBOT_RADIUS + 1):
            for dy in range(-ROBOT_RADIUS, ROBOT_RADIUS + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < len(pixel_map[0]) and 0 <= ny < len(pixel_map):
                    if is_within_range(dx,dy): #Circle equation
                        pixel_map[ny][nx] += 1
        # print(f"Robot {robot.id} at ({x}, {y}) increments surrounding pixels.")
    return []

def is_within_range(x,y): #Check if point is within circles
    from robot import ROBOT_RADIUS
    return x**2+y**2 <= ROBOT_RADIUS**2