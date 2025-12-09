
from robot import Robot, ROBOT_RADIUS
from consts import WIDTH, HEIGHT
import matplotlib.pyplot as plt
import numpy as np

frame_pos_list : list[list[tuple[float, float]]] = []

def logging_init(): #initialize your log file
    pass

def log_metrics(frame_count, total_time, metrics): # write to your log file
    pass

def logging_close(): # close your log file
    print("\n\nWow we did it!\n\n" + str(frame_pos_list))
    #compute_heatmap()
    compute_flocking()


# Lav et heatmap
def compute_metrics(robots : list[Robot]): # pass as many arguments as you need and compute relevant metrics to be logged for performance analysis
    frame_pos_list.append([robot._pos for robot in robots])

def compute_heatmap():
    pixel_map : list[list[int]] = [[0 for _ in range(WIDTH)] for _ in range(HEIGHT)]

    for pos_list in frame_pos_list:
        for robot_x, robot_y in pos_list:
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



from sklearn import metrics
from sklearn.cluster import DBSCAN


def compute_flocking():
    for pos_list in frame_pos_list:
        pos_matrix = np.array(pos_list)
        

        db = DBSCAN(eps=150, min_samples=3).fit(pos_matrix)
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

        print("Estimated number of clusters: %d" % n_clusters_)
        print("Estimated number of noise points: %d" % n_noise_)
    else:
        unique_labels = set(labels)
        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True

        colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = [0, 0, 0, 1]

            class_member_mask = labels == k

            xy = pos_list[class_member_mask & core_samples_mask]
            plt.plot(
                xy[:, 0],
                xy[:, 1],
                "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                markersize=14,
            )

            xy = pos_list[class_member_mask & ~core_samples_mask]
            plt.plot(
                xy[:, 0],
                xy[:, 1],
                "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                markersize=6,
            )

        plt.title(f"Estimated number of clusters: {n_clusters_}")
        plt.savefig('../figs/cluster-plot.png')
        plt.show()

        plt.scatter(pos_matrix[:, 0], pos_matrix[:, 1])
        plt.savefig('../figs/scatter-plot.png')
        plt.show()


def log_decorator(func):
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        # You can add logging functionality here if needed
        print(f"{func.__name__}({args}, {kwargs}) -> {result}")
        return result
    return wrapper

def log_calculation(func, args: list, result, *, doprint = True):
    s = f"{func.__name__}({', '.join(map(str, args))}) -> {result}"
    if doprint:
        print(s)
    return s