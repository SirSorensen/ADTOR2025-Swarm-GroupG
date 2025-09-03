
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
    for robot in robots:
        x, y = robot._pos
        x = int(x)
        y = int(y)

        pixel_map[y][x] = pixel_map[y][x] + 1
        print(f"({x}, {y}) => {pixel_map[y][x]}")


    return []