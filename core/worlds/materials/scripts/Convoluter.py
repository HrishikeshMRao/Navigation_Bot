#!/usr/bin/env python3
import os
import sys

import cv2
import numpy as np

# Get the texture path from argument
if len(sys.argv) < 2:
    print("Usage: Convoluter.py <texture_path>")
    sys.exit(1)

texture_path = sys.argv[1]

# Check file exists
if not os.path.isfile(texture_path):
    print(f"File not found: {texture_path}")
    sys.exit(1)

# Load in grayscale
img = cv2.imread(texture_path, cv2.IMREAD_GRAYSCALE)

scale = 2
new_size = (int(img.shape[1] * scale), int(img.shape[0] * scale))
img = cv2.resize(img, new_size, interpolation=cv2.INTER_CUBIC)

# Threshold
_, binary = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY)

# Invert (so white lines become black)
inv = 255 - binary

# 5x5 kernel for dilation (max pooling effect)
kernel = np.ones((34, 34), np.uint8)
thinned = cv2.dilate(inv, kernel, iterations=1)
result = 255 - thinned

save_path = sys.argv[2]

# Overwrite the original image
cv2.imwrite(save_path, result)
print(f" Maze texture thinned and saved at {save_path}")
