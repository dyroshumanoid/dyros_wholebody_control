import numpy as np
import matplotlib.pyplot as plt

# =========================
# Load data
# =========================
data = np.loadtxt("lhand_traj_log.txt")

# Check shape
if data.ndim == 1:
    data = data.reshape(1, -1)

assert data.shape[1] == 6, f"Expected 6 columns, but got {data.shape[1]}"

# Split columns
pos_actual = data[:, 0:3]   # local_xpos: x, y, z
pos_traj   = data[:, 3:6]   # x_traj:    x, y, z

# Time axis
t = np.arange(data.shape[0])

# Labels
axis_labels = ['x', 'y', 'z']

# =========================
# Plot
# =========================
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

for i in range(3):
    axes[i].plot(t, pos_actual[:, i], label=f'actual {axis_labels[i]}', linewidth=2)
    axes[i].plot(t, pos_traj[:, i], label=f'traj {axis_labels[i]}', linewidth=2, linestyle='--')
    axes[i].set_ylabel(f'{axis_labels[i]} [m]')
    axes[i].grid(True)
    axes[i].legend()

axes[-1].set_xlabel('sample index')
fig.suptitle('Left Hand Position: Actual vs Trajectory')

plt.tight_layout()
plt.savefig("lhand_traj_comparison.png", dpi=300, bbox_inches='tight')

print("SAVE FIGURE SUCCESS")