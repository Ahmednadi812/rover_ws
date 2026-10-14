import numpy as np
from PIL import Image

# ---------------- settings ----------------
SIZE_M = 19.0          # map is 20 m x 20 m
RES = 0.05             # meters per pixel
ORIGIN = (-10.0, -10.0, 0.0)  # real-world position of bottom-left pixel
WALL_M = 0.5           # border wall thickness in meters
N = int(SIZE_M / RES)  # 400 pixels

FREE, OCC = 254, 0

# ---------------- helpers ----------------
img = np.full((N, N), FREE, dtype=np.uint8)


def to_px(x, y):
    """World coords (m) -> (row, col) in the image.
    Image row 0 is the TOP, so y is flipped."""
    col = int(round((x - ORIGIN[0]) / RES))
    row = int(round(N - 1 - (y - ORIGIN[1]) / RES))
    return row, col


def add_box(x_min, y_min, x_max, y_max):
    """Fill a rectangle (in meters) as occupied."""
    r1, c0 = to_px(x_min, y_min)   # bottom-left
    r0, c1 = to_px(x_max, y_max)   # top-right
    img[max(r0, 0):r1 + 1, max(c0, 0):c1 + 1] = OCC


# ---------------- border walls ----------------
t = int(WALL_M / RES)  # 10 px
img[:t, :] = OCC
img[-t:, :] = OCC
img[:, :t] = OCC
img[:, -t:] = OCC

# ---------------- optional obstacles (delete or edit) ----------------
# add_box(-2, -2, 2, 2)   # example: uncomment to add an obstacle

# ---------------- save map ----------------
Image.fromarray(img, mode="L").save("map.pgm")

yaml_text = f"""image: map.pgm
mode: trinary
resolution: {RES}
origin: [{ORIGIN[0]}, {ORIGIN[1]}, {ORIGIN[2]}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open("map.yaml", "w") as f:
    f.write(yaml_text)

# ---------------- preview with 1 m grid (NOT for navigation) ----------------
prev = img.copy()
step = int(1.0 / RES)  # 20 px = 1 m
free_mask = prev == FREE
grid = np.zeros_like(prev, dtype=bool)
grid[::step, :] = True
grid[:, ::step] = True
prev[grid & free_mask] = 180
Image.fromarray(prev, mode="L").resize((800, 800), Image.NEAREST).save("map_preview.png")

# ---------------- check ----------------
a = np.array(Image.open("map.pgm"))
print("shape:", a.shape, "min:", a.min(), "max:", a.max())
print(f"map size: {a.shape[1] * RES} m x {a.shape[0] * RES} m")
print("saved: map.pgm, map.yaml, map_preview.png")
