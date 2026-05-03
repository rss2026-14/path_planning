import cv2
import numpy as np
import matplotlib.pyplot as plt

# === USER SETTINGS ===
image_path = "stata_basement.png"          # change this to your map file
output_path = "dilated_map.png"

# Option 1: choose dilation directly in pixels

# Option 2: if you know map resolution and want to match your planner
inflate_radius = 0.5        # meters
resolution = 0.0483              # meters/pixel
dilation_pixels = int(inflate_radius / resolution)

# === LOAD IMAGE ===
img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

if img is None:
    raise FileNotFoundError(f"Could not load image: {image_path}")

# === CONVERT TO BINARY OBSTACLE MAP ===
# Assumption: dark pixels = obstacles, light pixels = free space
# You can adjust the threshold if needed
_, obstacle_map = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY_INV)

# === DILATE OBSTACLES ===
kernel_size = 2 * dilation_pixels + 1
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
dilated_map = cv2.dilate(obstacle_map, kernel)

# === SAVE RESULT ===
cv2.imwrite(output_path, dilated_map)

# === DISPLAY ===
plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.title("Original Obstacle Map")
plt.imshow(obstacle_map, cmap="gray")
plt.axis("off")

plt.subplot(1, 2, 2)
plt.title("Dilated Map")
plt.imshow(dilated_map, cmap="gray")
plt.axis("off")

plt.tight_layout()
plt.show()

print(f"Saved dilated map to: {output_path}")
