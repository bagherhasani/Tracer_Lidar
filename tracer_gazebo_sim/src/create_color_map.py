#!/usr/bin/env python3

import cv2
import numpy as np
import os
import yaml

def create_color_map(output_dir, size=(800, 800), resolution=0.05):
    """Create a test map with colored regions."""
    # Create a white background (free space)
    map_img = np.ones((size[0], size[1], 3), dtype=np.uint8) * 255
    
    # Draw some colored regions
    # Red circle
    cv2.circle(map_img, (200, 200), 50, (0, 0, 255), -1)
    
    # Green square
    cv2.rectangle(map_img, (600, 100), (700, 200), (0, 255, 0), -1)
    
    # Blue triangle
    pts = np.array([[400, 600], [300, 700], [500, 700]], np.int32)
    cv2.fillPoly(map_img, [pts], (255, 0, 0))
    
    # Yellow rectangle
    cv2.rectangle(map_img, (100, 400), (250, 500), (0, 255, 255), -1)
    
    # Purple pentagon
    pts = np.array([[600, 400], [550, 500], [600, 600], [650, 600], [700, 500]], np.int32)
    cv2.fillPoly(map_img, [pts], (128, 0, 128))
    
    # Add some obstacles (black)
    cv2.rectangle(map_img, (300, 300), (500, 400), (0, 0, 0), -1)
    cv2.circle(map_img, (650, 350), 30, (0, 0, 0), -1)
    
    # Save the color map
    color_map_path = os.path.join(output_dir, 'color_map.png')
    cv2.imwrite(color_map_path, map_img)
    
    # Create a grayscale version for navigation (white=free, black=obstacle)
    # Convert colored regions to white (free space) and keep obstacles black
    gray_map = cv2.cvtColor(map_img, cv2.COLOR_BGR2GRAY)
    _, gray_map = cv2.threshold(gray_map, 1, 255, cv2.THRESH_BINARY)
    
    # Save the grayscale map
    gray_map_path = os.path.join(output_dir, 'color_map.pgm')
    cv2.imwrite(gray_map_path, gray_map)
    
    # Create the yaml configuration file
    yaml_content = {
        'image': 'color_map.png',
        'resolution': resolution,
        'origin': [-20.0, -20.0, 0.0],  # x, y, theta
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    # Save the yaml file
    yaml_path = os.path.join(output_dir, 'color_map.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)
    
    print(f"Created color map at {output_dir}")
    print(f"Color image: {color_map_path}")
    print(f"Grayscale image: {gray_map_path}")
    print(f"YAML config: {yaml_path}")

if __name__ == '__main__':
    # Use the maps directory from the package
    import rospkg
    rospack = rospkg.RosPack()
    maps_dir = os.path.join(rospack.get_path('tracer_gazebo_sim'), 'maps')
    
    create_color_map(maps_dir) 