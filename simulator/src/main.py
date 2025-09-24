import numpy as np
import pygame
from consts import HEIGHT, WIDTH
from drawer import draw_light_sources, draw_obstacles
from game import init_pygame
from log import compute_metrics, log_metrics, logging_close, logging_init
from obstacle import OBSTACLES
from robot import NUM_ROBOTS, ROBOT_RADIUS, Robot
from boid import Boid

# Pygame setup
screen, font = init_pygame()

# Colours
BG_COLOR = (30, 30, 30)
OBSTACLE_COLOR = (200, 50, 50)
FONT_COLOR = (255, 255, 255)

# Simulator
SIM_DT = 1 / 60.0

# Surrounding walls
ARENA_BOUNDS = {
    'left': 0,
    'right': WIDTH,
    'top': 0,
    'bottom': HEIGHT
}

def main(_seed = 42):
    clock = pygame.time.Clock()
    dt = SIM_DT
    robots : list[Robot] = []

    np.random.seed(_seed)
    for i in range(NUM_ROBOTS):
        pos = np.random.uniform([ROBOT_RADIUS, ROBOT_RADIUS], [WIDTH - ROBOT_RADIUS, HEIGHT - ROBOT_RADIUS])
        heading = np.random.uniform(0, 2 * np.pi)
        robots.append(Boid(i, pos, heading))

    # initialize robot controllers
    for robot in robots:
        robot.controller_init()

    logging_init()

    frame_count = 0
    total_time = 0.0
    running = True
    paused = False
    visualize = True
    dispersion = True
    verbose = False
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    paused = not paused
                    print("Paused" if paused else "Resumed")
                elif event.key == pygame.K_SPACE:
                    visualize = not visualize
                    print("Visualization", "enabled" if visualize else "disabled", "at", total_time)
                elif event.key == pygame.K_r:
                    # If shift as well, generate a new seed:
                    if event.mod != pygame.KMOD_NONE and (event.mod & pygame.KMOD_LSHIFT or event.mod & pygame.KMOD_RSHIFT):
                        new_seed = np.random.randint(0, 100)
                        main(new_seed)
                    else: # If not, use the current seed:
                        main(_seed)
                elif event.key == pygame.K_s:
                    dispersion = not dispersion
                elif event.key == pygame.K_v:
                    verbose = not verbose
                    for r in robots:
                        r.verbose = verbose
                elif event.key == pygame.K_e:
                    running = False
                


        if not paused:
            total_time += dt  # accumulate time

            for robot in robots:
                robot.read_sensors(robots, OBSTACLES, ARENA_BOUNDS)

            for robot in robots:
                robot.robot_controller(dispersion=dispersion)

            for robot in robots:
                robot.move(dt)

            metrics = compute_metrics(robots)
            log_metrics(frame_count, total_time, metrics)

            frame_count += 1

        if visualize:
            clock.tick(60 if not paused else 10)
            screen.fill(BG_COLOR)
            draw_light_sources(screen)
            draw_obstacles(screen)
            for robot in robots:
                robot.draw(screen)
            if paused:
                txt = font.render("PAUSED", True, (255, 100, 100))
                screen.blit(txt, (10, 10))
            pygame.display.flip()
            pygame.display.set_caption("Robot Sim — VISUAL MODE")
        else:
            pygame.display.set_caption("Robot Sim — PAUSED in HEADLESS" if paused else "Robot Sim — HEADLESS")


    pygame.quit()
    logging_close()

if __name__ == "__main__":
    main()
