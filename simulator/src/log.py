
from datetime import datetime
import os
from robot import CLOSE_RANGE_RADIUS, NUM_ROBOTS, RAB_RANGE, Robot, ROBOT_RADIUS
from consts import WIDTH, HEIGHT
import matplotlib.pyplot as plt
import numpy as np

def logging_init(seed): #initialize your log file
    global frame_pos_list, output_dir
    frame_pos_list = np.array([])
    output_dir = f'../figs/{NUM_ROBOTS} robots + {seed} seed -> {datetime.now().hour}-{datetime.now().minute}'
    os.mkdir(output_dir)

def log_metrics(frame_count, total_time, metrics): # write to your log file
    print(f"\nCurrent simulation time = {np.round(total_time, 2)}")

def logging_close(): # close your log file
    print("\n\nWow we did it!\n\n")
    save_log_to_file(str(frame_pos_list), "frame_pos_list.log")

    #compute_heatmap()
    compute_flocking()


# Lav et heatmap
def compute_metrics(robots : list[Robot]): # pass as many arguments as you need and compute relevant metrics to be logged for performance analysis
    global frame_pos_list
    #print(f"Before {len(frame_pos_list) = }")
    if len(frame_pos_list) == 0:
        #print("Remaking!")
        frame_pos_list = np.array([[robot._pos for robot in robots]])
    else:
        #print("Appending!")
        frame_pos_list = np.append(frame_pos_list, np.array([[robot._pos for robot in robots]]), axis=0)

def compute_heatmap():
    pixel_map : list[list[int]] = [[0 for _ in range(WIDTH)] for _ in range(HEIGHT)]

    for pos_list in frame_pos_list:
        #print(f"{pos_list = }")
        for robot_x, robot_y in pos_list:
            #print(f"{robot_x, robot_y = }")
            robot_x = int(robot_x)
            robot_y = int(robot_y)
            xs = range(robot_x - ROBOT_RADIUS, robot_x + ROBOT_RADIUS + 1)
            ys = range(robot_y - ROBOT_RADIUS, robot_y + ROBOT_RADIUS + 1)

            for y in ys:
                for x in xs:
                    # If distance from the center of the robot is greater than the radius, then we know that this pixel is outside the bounding box of the robot's body.
                    distance_from_robot_center = np.sqrt(np.pow(x - robot_x, 2) + np.pow(y - robot_y, 2))
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
    plt.savefig(f'{output_dir}/heatmap.png')

    total_pixels = pixels_undiscovered + pixels_discovered
    print(f"pixels_undiscovered = {pixels_undiscovered}")
    print(f"pixels_discovered = {pixels_discovered}")
    print(f"total_pixels = {total_pixels}")
    print(f"Percentage discovered = {round((pixels_discovered / total_pixels)*100, 2)}%")



from sklearn import metrics
from sklearn.cluster import DBSCAN


def compute_flocking():
    min_distance = (RAB_RANGE + CLOSE_RANGE_RADIUS)/2
    clusters = []
    noise = []
    for pos_matrix in frame_pos_list:
        #pos_matrix = np.array(pos_list)
        #print(pos_matrix)
        

        db = DBSCAN(eps=min_distance, min_samples=2).fit(pos_matrix)
        labels = db.labels_

        # Number of clusters in labels, ignoring noise if present.
        unique_labels = set(labels)
        n_clusters_ = len(unique_labels) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1) if n_clusters_ > 0 else NUM_ROBOTS
        
        clusters.append(n_clusters_)
        noise.append(n_noise_)

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

            xy = pos_matrix[class_member_mask & core_samples_mask]
            plt.plot(
                xy[:, 0],
                xy[:, 1],
                "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                markersize=14,
            )

            xy = pos_matrix[class_member_mask & ~core_samples_mask]
            plt.plot(
                xy[:, 0],
                xy[:, 1],
                "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                markersize=6,
            )

        plt.title(f"Estimated number of clusters, with min_dist {np.round(min_distance, 2)} = {n_clusters_}")
        plt.gca().invert_yaxis()
        plt.grid(True)
        plt.xlim(0, WIDTH)
        plt.ylim(0, HEIGHT)
        plt.savefig(f'{output_dir}/cluster-plot.png')
        #plt.show()
        plt.close()

        plt.scatter(pos_matrix[:, 0], pos_matrix[:, 1])
        plt.gca().invert_yaxis() 
        plt.grid(True)
        plt.xlim(0, WIDTH)
        plt.ylim(0, HEIGHT)
        plt.savefig(f'{output_dir}/scatter-plot.png')
        #plt.show()
        plt.close()

    save_log_to_file(str(clusters), "clusters.log")
    save_log_to_file(str(noise), "noise.log")
    
    xs = range(1, len(clusters)+1)
    plt.plot(xs, clusters, '-.')
    plt.plot(xs, noise, '-.')

    plt.legend(['Clusters', 'Noise'])

    plt.xlabel("Frame")
    plt.ylabel("Amount")
    plt.xlim(1, len(clusters)+1)
    plt.ylim(0, max(max(clusters), max(noise))+2)
    # Plot the normalized coordinates
    plt.grid(True)
    plt.title(f'Clusters over time with min_dist {np.round(min_distance, 2)}')
    plt.savefig(f'{output_dir}/line-plot.png')
    #plt.show()
    plt.close()



def log_decorator(func):
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        # You can add logging functionality here if needed
        print(f"{func.__name__}({args}, {kwargs}) -> {result}")
        return result
    return wrapper

def log_calculation(func, args: list, result, *, doprint = False):
    s = f"{func.__name__}({', '.join(map(str, args))}) -> {result}"
    if doprint:
        print(s)
    return s

def save_log_to_file(s, filename):
    with open(f"{output_dir}/{filename}", "w", encoding="utf-8") as f:
        f.write(s)
