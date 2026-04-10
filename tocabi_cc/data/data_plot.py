import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# =========================================================
# File paths
# =========================================================
lhand_file = "lhand_traj_log.txt"
rhand_file = "rhand_traj_log.txt"
dist_file  = "min_distance_log.txt"

save_dir = Path("plots")
save_dir.mkdir(exist_ok=True)

# =========================================================
# Load data
# =========================================================
lhand = np.loadtxt(lhand_file)
rhand = np.loadtxt(rhand_file)
dist  = np.loadtxt(dist_file)

# Ensure 2D shape even if there is only one row
if lhand.ndim == 1:
    lhand = lhand.reshape(1, -1)
if rhand.ndim == 1:
    rhand = rhand.reshape(1, -1)
if dist.ndim == 1:
    dist = dist.reshape(1, -1)

# =========================================================
# Parse columns
# =========================================================
# Hand log format:
# time | actual_x actual_y actual_z | desired_x desired_y desired_z
t_l = lhand[:, 0]
l_actual = lhand[:, 1:4]
l_desired = lhand[:, 4:7]

t_r = rhand[:, 0]
r_actual = rhand[:, 1:4]
r_desired = rhand[:, 4:7]

# Distance log format:
# time | dist_left dist_right
t_d = dist[:, 0]
dist_left = dist[:, 1]
dist_right = dist[:, 2]

# =========================================================
# Plot function for hand trajectory
# =========================================================
def plot_hand_trajectory(time, actual, desired, hand_name, save_path):
    labels = ["x", "y", "z"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i in range(3):
        axes[i].plot(time, actual[:, i], label="actual")
        axes[i].plot(time, desired[:, i], "--", label="desired")
        axes[i].set_ylabel(labels[i])
        axes[i].grid(True)
        axes[i].legend()

    axes[-1].set_xlabel("time [s]")
    fig.suptitle(f"{hand_name} hand trajectory", fontsize=14)
    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

# =========================================================
# Plot hand trajectories
# =========================================================
plot_hand_trajectory(
    t_l, l_actual, l_desired,
    hand_name="Left",
    save_path=save_dir / "left_hand_trajectory.png"
)

plot_hand_trajectory(
    t_r, r_actual, r_desired,
    hand_name="Right",
    save_path=save_dir / "right_hand_trajectory.png"
)

# =========================================================
# Plot minimum distances
# =========================================================
fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t_d, dist_left, label="left min distance")
ax.plot(t_d, dist_right, label="right min distance")
ax.set_xlabel("time [s]")
ax.set_ylabel("distance [m]")
ax.set_title("Minimum distance")
ax.grid(True)
ax.legend()
fig.tight_layout()
fig.savefig(save_dir / "min_distance.png", dpi=200, bbox_inches="tight")
plt.close(fig)

print("Plots saved in:", save_dir.resolve())