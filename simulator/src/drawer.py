import pygame
from light_source import LIGHT_SOURCES
from obstacle import OBSTACLES


def draw_obstacles(screen):
    for obs in OBSTACLES:
        pygame.draw.circle(screen, (120, 120, 120), obs.pos.astype(int), obs.radius)

def draw_light_sources(screen):
    for light in LIGHT_SOURCES:
        # Draw fading light circle
        for r in range(light.decay_radius, light.core_radius, -10):
            alpha = int(255 *light.get_intensity_at(r))
            surface = pygame.Surface((r * 2, r * 2), pygame.SRCALPHA)
            pygame.draw.circle(surface, (light.intensity*255, light.intensity*235, 0, alpha), (r, r), r) # type: ignore
            screen.blit(surface, (light.pos[0] - r, light.pos[1] - r))

        # Draw core of the light
        pygame.draw.circle(screen, (light.intensity*255, light.intensity*255, 0), light.pos.astype(int), max(5,light.core_radius)) # type: ignore