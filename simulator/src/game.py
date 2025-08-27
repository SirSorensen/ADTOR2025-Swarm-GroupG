import pygame
from consts import HEIGHT, WIDTH

def init_pygame():
	pygame.init()
	screen = pygame.display.set_mode((WIDTH, HEIGHT))
	pygame.display.set_caption("Robots Detecting Each Other and Obstacles with Proximity & RAB")
	font = pygame.font.SysFont(None, 20)
	return (screen, font)
