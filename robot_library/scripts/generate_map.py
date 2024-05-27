from PIL import Image
import yaml

# Define the size of the image to cover 250m x 250m at 0.05m resolution
width, height = 5000, 5000

# Create a new image with a white background
# 255 represents free space in an occupancy grid (white)
image = Image.new('L', (width, height), 255)

# Save the image as a PNG file
image_filename = 'free_space_map_large.png'
image.save(image_filename)

# Define the map parameters
map_data = {
    'image': image_filename,
    'resolution': 0.05,  # Set the resolution of your map in meters/pixel
    'origin': [0.0, 0.0, 0.0],  # The origin of the map [x, y, theta]
    'negate': 0,  # No need to negate the image
    'occupied_thresh': 0.65,  # Threshold above which pixels are considered occupied
    'free_thresh': 0.196  # Threshold below which pixels are considered free
}

# Save the map parameters to a YAML file
yaml_filename = 'free_space_map_large.yaml'
with open(yaml_filename, 'w') as yaml_file:
    yaml.dump(map_data, yaml_file, default_flow_style=False)

print(f"Generated {image_filename} and {yaml_filename}")