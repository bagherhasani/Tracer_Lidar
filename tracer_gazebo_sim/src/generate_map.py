#  script as generate_map.py
import numpy as np
from PIL import Image

# Create a 10x10 meter map (200x200 pixels at 0.05 m/pixel resolution)
map_data = np.ones((200, 200), dtype=np.uint8) * 255  # White background (free space)

# Add walls (black pixels)
map_data[50:70, 50:150] = 0  # Horizontal wall
map_data[100:150, 100:120] = 0  # Vertical wall

# Save as a .pgm file
image = Image.fromarray(map_data, mode='L')
image.save('simple_map.pgm')