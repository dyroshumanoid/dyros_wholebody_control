import numpy as np
import matplotlib.pyplot as plt

# =========================
# Load data
# =========================
hand_data = np.loadtxt("hand_log.txt")
obs_data = np.loadtxt("obs_log.txt")
min_data = np.loadtxt("min_distance_log.txt")

# Ensure 2D for single-line logs
if hand_data.ndim == 1:
    hand_data = hand_data.reshape(1, -1)

if obs_data.ndim == 1:
    obs_data = obs_data.reshape(1, -1)

if min_data.ndim == 1:
    min_data = min_data.reshape(1, -1)

# =========================
# Check dimensions
# =========================
assert hand_data.shape[1] >= 7, "hand_log.txt must have at least 7 columns"
assert obs_data.shape[1] >= 7, "obs_log.txt must have at least 7 columns"
assert min_data.shape[1] >= 2, "min_distance_log.txt must have at least 2 columns"

# =========================
# Parse hand log
# =========================
t_hand = hand_data[:, 0]

lh_x = hand_data[:, 1]
lh_y = hand_data[:, 2]
lh_z = hand_data[:, 3]

rh_x = hand_data[:, 4]
rh_y = hand_data[:, 5]
rh_z = hand_data[:, 6]

# =========================
# Parse obstacle log
# =========================
t_obs = obs_data[:, 0]

obs_x = obs_data[:, 1]
obs_y = obs_data[:, 2]
obs_z = obs_data[:, 3]

obs_vx = obs_data[:, 4]
obs_vy = obs_data[:, 5]
obs_vz = obs_data[:, 6]

# =========================
# Parse minimum distance log
# =========================
t_min = min_data[:, 0]
min_dist = min_data[:, 1]

# =========================
# Plot left/right hand position
# =========================
fig1, axes1 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axes1[0].plot(t_hand, lh_x, linewidth=2, label="Left Hand")
axes1[0].plot(t_hand, rh_x, linewidth=2, label="Right Hand")
axes1[0].set_ylabel("x [m]")
axes1[0].set_title("Hand X Position")
axes1[0].grid(True)
axes1[0].legend()

axes1[1].plot(t_hand, lh_y, linewidth=2, label="Left Hand")
axes1[1].plot(t_hand, rh_y, linewidth=2, label="Right Hand")
axes1[1].set_ylabel("y [m]")
axes1[1].set_title("Hand Y Position")
axes1[1].grid(True)
axes1[1].legend()

axes1[2].plot(t_hand, lh_z, linewidth=2, label="Left Hand")
axes1[2].plot(t_hand, rh_z, linewidth=2, label="Right Hand")
axes1[2].set_ylabel("z [m]")
axes1[2].set_xlabel("time [s]")
axes1[2].set_title("Hand Z Position")
axes1[2].grid(True)
axes1[2].legend()

fig1.tight_layout()

# =========================
# Plot obstacle position
# =========================
fig2, axes2 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axes2[0].plot(t_obs, obs_x, linewidth=2)
axes2[0].set_ylabel("x [m]")
axes2[0].set_title("Obstacle X Position")
axes2[0].grid(True)

axes2[1].plot(t_obs, obs_y, linewidth=2)
axes2[1].set_ylabel("y [m]")
axes2[1].set_title("Obstacle Y Position")
axes2[1].grid(True)

axes2[2].plot(t_obs, obs_z, linewidth=2)
axes2[2].set_ylabel("z [m]")
axes2[2].set_xlabel("time [s]")
axes2[2].set_title("Obstacle Z Position")
axes2[2].grid(True)

fig2.tight_layout()

# =========================
# Plot obstacle velocity
# =========================
fig3, axes3 = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axes3[0].plot(t_obs, obs_vx, linewidth=2)
axes3[0].set_ylabel("vx [m/s]")
axes3[0].set_title("Obstacle Vx")
axes3[0].grid(True)

axes3[1].plot(t_obs, obs_vy, linewidth=2)
axes3[1].set_ylabel("vy [m/s]")
axes3[1].set_title("Obstacle Vy")
axes3[1].grid(True)

axes3[2].plot(t_obs, obs_vz, linewidth=2)
axes3[2].set_ylabel("vz [m/s]")
axes3[2].set_xlabel("time [s]")
axes3[2].set_title("Obstacle Vz")
axes3[2].grid(True)

fig3.tight_layout()

# =========================
# Plot minimum distance
# =========================
fig4, ax4 = plt.subplots(figsize=(10, 4))

ax4.plot(t_min, min_dist, linewidth=2)
ax4.set_xlabel("time [s]")
ax4.set_ylabel("distance [m]")
ax4.set_title("Minimum Distance to Obstacle")
ax4.grid(True)

fig4.tight_layout()

# =========================
# Save if needed
# =========================
# fig1.savefig("hand_position_plot.png", dpi=300, bbox_inches="tight")
# fig2.savefig("obstacle_position_plot.png", dpi=300, bbox_inches="tight")
# fig3.savefig("obstacle_velocity_plot.png", dpi=300, bbox_inches="tight")
# fig4.savefig("min_distance_plot.png", dpi=300, bbox_inches="tight")

plt.show()