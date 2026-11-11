import cv2
import numpy as np
import matplotlib.pyplot as plt

# 1. Load the floorplan image (Make sure it's in the same folder)
# Black pixels (0) = Walls, White pixels (255) = Walkable floor
img = cv2.imread('1.png', cv2.IMREAD_GRAYSCALE)

# Convert to strict binary just in case of anti-aliasing (gray pixels)
_, binary_map = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# 2. Calculate the distance map
# This assigns every pixel a number representing its distance to the nearest wall.
# Walls are 0, pixels right next to the wall are 1, center of the room is highest.
distance_map = cv2.distanceTransform(binary_map, cv2.DIST_L2, 5)

# 3. Calculate Repulsion Vectors (Gradients)
# np.gradient looks at the distance map and creates directional arrows (dx, dy) 
# pointing away from the walls and towards the open space.
grad_y, grad_x = np.gradient(distance_map)

# 4. Define your Wall Repulsion Function
def get_grid_wall_repulsion(pos_x, pos_y):
    # Convert agent's real-world coordinates to image pixel indices
    # Assuming 1 pixel = 0.1 meters
    px = int(pos_x * 10)
    py = int(pos_y * 10)
    
    # Check boundaries so the script doesn't crash if an agent walks off the map
    if px < 0 or px >= img.shape[1] or py < 0 or py >= img.shape[0]:
        return np.zeros(2)
        
    # Get the distance to the nearest wall at this specific pixel
    dist = distance_map[py, px]
    
    # If the agent is within 10 pixels (1 meter) of a wall, apply force
    if dist < 10.0 and dist > 0:
        # The gradient gives us the direction pointing away from the wall
        direction = np.array([grad_x[py, px], grad_y[py, px]])
        
        # Normalize the direction vector
        norm = np.linalg.norm(direction)
        if norm > 0:
            direction = direction / norm
            
        # Apply exponential force (stronger as they get closer)
        A = 15.0 # Force strength
        B = 2.0  # Falloff rate (pixels)
        force = direction * A * np.exp(-dist / B)
        return force
        
    return np.zeros(2)

# --- Visualization (Optional) ---
# You can plot the distance map to see the "magnetic cushion" your walls create
plt.title("Wall Distance Map (Brighter = Further from wall)")
plt.imshow(distance_map, cmap='magma')
plt.colorbar()
plt.show()