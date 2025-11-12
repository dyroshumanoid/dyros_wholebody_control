import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

w, h = 32, 32

# basic random noise
data = np.random.rand(h, w).astype(np.float32)

# Mixing multi-octave noise (more natural)
freqs = [1, 2, 4, 8]
# amplitude(1: white, 0: black)
amps = [1.0, 0.5, 0.25, 0.125]
comb = np.zeros_like(data)
for f,a in zip(freqs, amps):
    grid = np.random.rand(h//f, w//f).astype(np.float32)
    # simple upsampling(make the size h x w) with np.repeat
    # Ex: if freqs = 8 -> grid size: h/8, w/8
    grid_up = np.repeat(np.repeat(grid, f, axis=0), f, axis=1)
    grid_up = grid_up[:h, :w]
    comb += a * grid_up

# smoothing progress using gaussian filter
smooth = gaussian_filter(comb, sigma=2.0)

# convert to 0-255 uint8 after normalization
smooth -= smooth.min()
if smooth.max() != 0:
    smooth /= smooth.max()
img_u8 = (smooth * 255).astype(np.uint8)

# flattern the middle
for i in range(14, 18):
    for j in range(14, 18):
        img_u8[i][j] = 125


Image.fromarray(img_u8).save("bumpy_gaussian.png")