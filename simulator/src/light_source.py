import numpy as np

# sensor noise and dropout
LIGHT_NOISE_STD = 0 # noise in perceived light

class LightSource:
    def __init__(self, pos, intensity=1.0, core_radius=0, decay_radius=200):
        self.pos = np.array(pos, dtype=float)
        self.intensity = intensity
        self.core_radius = core_radius
        self.decay_radius = decay_radius

    def get_intensity_at(self, dist):
        if dist > self.decay_radius:
            return 0.0
        if dist < self.core_radius:
            return self.intensity
        # Inverse-square decay beyond the center radius (you can change to linear or exponential)
        return self.intensity * max(0.0, 1.0 - ((dist - self.core_radius) / (self.decay_radius - self.core_radius)) ** 2)


LIGHT_SOURCES : list[LightSource] = [
    #LightSource(pos=(100, 100), intensity=1.0, core_radius=50, decay_radius=300),
    #LightSource(pos=(700, 500), intensity=0.9, core_radius=10, decay_radius=100)
]


# Utility function to sum light intensity from all sources
def _get_light_intensity(pos):
    raw_intensity = sum(source.get_intensity_at(np.linalg.norm(pos - source.pos)) for source in LIGHT_SOURCES)
    noise_factor = np.random.normal(1.0, LIGHT_NOISE_STD)
    return np.clip(raw_intensity * noise_factor, 0.0, 1.0)