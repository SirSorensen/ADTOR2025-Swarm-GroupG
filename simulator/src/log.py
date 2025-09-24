
from robot import Robot, ROBOT_RADIUS
from consts import WIDTH, HEIGHT
import matplotlib.pyplot as plt
import numpy as np

pixel_map : list[list[int]] = [[0 for _ in range(WIDTH)] for _ in range(HEIGHT)]

def logging_init(): #initialize your log file
    pass

def log_metrics(frame_count, total_time, metrics): # write to your log file
    pass

def logging_close(): # close your log file
    pixels_undiscovered = 0
    pixels_discovered = 0
    for y in range(len(pixel_map)):
        for x in range(len(pixel_map[y])):
            pixel_map[y][x] = min(100, pixel_map[y][x]) # Cap at 100
            if pixel_map[y][x] > 0:
                pixels_discovered += 1
            else:
                pixels_undiscovered += 1


    a = np.array(pixel_map)
    plt.imshow(a, cmap='hot', interpolation='nearest')
    plt.savefig('heatmap.png')

    total_pixels = pixels_undiscovered + pixels_discovered
    print(f"pixels_undiscovered = {pixels_undiscovered}")
    print(f"pixels_discovered = {pixels_discovered}")
    print(f"total_pixels = {total_pixels}")
    print(f"Percentage discovered = {round((pixels_discovered / total_pixels)*100, 2)}%")



# Lav et heatmap
def compute_metrics(robots : list[Robot]): # pass as many arguments as you need and compute relevant metrics to be logged for performance analysis
    for robot in robots:
        robot_x,robot_y = robot._pos
        robot_x = int(robot_x)
        robot_y = int(robot_y)
        xs = range(robot_x - ROBOT_RADIUS, robot_x + ROBOT_RADIUS + 1)
        ys = range(robot_y - ROBOT_RADIUS, robot_y + ROBOT_RADIUS + 1)

        for y in ys:
            for x in xs:
                # If distance from the center of the robot is greater than the radius, then we know that this pixel is outside the bounding box of the robot's body.
                distance_from_robot_center = np.sqrt((x - robot_x)^2 + (y - robot_y)^2)
                if distance_from_robot_center <= ROBOT_RADIUS:
                    if y >= 0 and y < len(pixel_map) and x >= 0 and x < len(pixel_map[y]):
                        pixel_map[y][x] = pixel_map[y][x] + 1