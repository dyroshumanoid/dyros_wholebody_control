import numpy as np
from PIL import Image

# heightfield size (pixel)
width, height = 32, 32

# generate random heights(0:black, 255:white)
height_data = np.random.rand(height, width) * 255  # 0~255

# flattern the middle
for i in range(14, 18):
    for j in range(14, 18):
        height_data[i][j] = 125

height_data = height_data.astype(np.uint8)

# store image
img = Image.fromarray(height_data)
img.save("bumpy.png")