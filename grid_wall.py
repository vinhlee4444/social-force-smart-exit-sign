import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from sfm.config import Config
from sfm.floorplan import FloorPlan

# 1. Load the floorplan image (Make sure it's in the same folder)
# Black pixels (0) = Walls, White pixels (255) = Walkable floor
cfg = Config()
floor = FloorPlan(cfg.map, cfg.exits, Path(__file__).resolve().parent)

# 2. Calculate the distance map
# This assigns every pixel a number representing its distance to the nearest wall.
# Walls are 0, pixels right next to the wall are 1, center of the room is highest.
distance_map = floor.sdf

# 3. Calculate Repulsion Vectors (Gradients)
# np.gradient looks at the distance map and creates directional arrows (dx, dy) 
# pointing away from the walls and towards the open space.
grad_y, grad_x = np.gradient(distance_map)

# 4. Define your Wall Repulsion Function
def get_grid_wall_repulsion(pos_x, pos_y):
    # Convert agent's real-world coordinates to image pixel indices
    px = int(pos_x / floor.s)
    py = int(pos_y / floor.s)
    
    # Check boundaries so the script doesn't crash if an agent walks off the map
    if px < 0 or px >= floor.W or py < 0 or py >= floor.H:
        return np.zeros(2)
        
    # Get the distance to the nearest wall at this specific pixel
    dist = distance_map[py, px]
    
    if 0 < dist < 1.0:
        # The gradient gives us the direction pointing away from the wall
        direction = np.array([grad_x[py, px], grad_y[py, px]])
        
        # Normalize the direction vector
        norm = np.linalg.norm(direction)
        if norm > 0:
            direction = direction / norm
            
        # Apply exponential force (stronger as they get closer)
        A = 15.0 # Force strength
        B = 0.2  # Falloff rate (metres)
        force = direction * A * np.exp(-dist / B)
        return force
        
    return np.zeros(2)

# --- Visualization (Optional) ---
# You can plot the distance map to see the "magnetic cushion" your walls create
plt.title("Wall Distance Map (Brighter = Further from wall)")
plt.imshow(distance_map, cmap='magma')
plt.colorbar()
plt.show()
