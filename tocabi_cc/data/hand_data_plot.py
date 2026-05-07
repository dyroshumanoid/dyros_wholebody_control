import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _ensure_2d(arr):
    if arr.ndim == 1:
        return arr.reshape(1, -1)
    return arr


def _load_hand_data(hand_log_path, hand_traj_log_path):
    """
    Supported formats:
    1) hand_log:      Lx Ly Lz Rx Ry Rz
       hand_traj_log: Lx Ly Lz Rx Ry Rz
    2) hand_log:      t Lx Ly Lz Rx Ry Rz
       hand_traj_log: t Lx Ly Lz Rx Ry Rz
    """
    actual = _ensure_2d(np.loadtxt(hand_log_path))
    desired = _ensure_2d(np.loadtxt(hand_traj_log_path))

    n = min(actual.shape[0], desired.shape[0])
    actual = actual[:n]
    desired = desired[:n]

    if actual.shape[1] == 6 and desired.shape[1] == 6:
        t = np.arange(n)
        a = actual
        d = desired
    elif actual.shape[1] >= 7 and desired.shape[1] >= 7:
        # If both logs include time, use hand_log time axis.
        t = actual[:, 0]
        a = actual[:, 1:7]
        d = desired[:, 1:7]
    else:
        raise ValueError(
            "Unsupported log format. Expected 6 or 7+ columns in each file."
        )

    left_actual = a[:, 0:3]
    right_actual = a[:, 3:6]
    left_desired = d[:, 0:3]
    right_desired = d[:, 3:6]

    return t, left_actual, left_desired, right_actual, right_desired


def _calc_metrics(actual, desired):
    err = actual - desired
    rmse_xyz = np.sqrt(np.mean(err**2, axis=0))
    mae_xyz = np.mean(np.abs(err), axis=0)
    max_abs_xyz = np.max(np.abs(err), axis=0)
    norm_err = np.linalg.norm(err, axis=1)
    return {
        "rmse_xyz": rmse_xyz,
        "mae_xyz": mae_xyz,
        "max_abs_xyz": max_abs_xyz,
        "rmse_norm": np.sqrt(np.mean(norm_err**2)),
        "max_norm": np.max(norm_err),
        "err": err,
        "norm_err": norm_err,
    }


def _plot_hand_tracking(time, actual, desired, hand_name, save_path):
    axis_labels = ["x", "y", "z"]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    for i, label in enumerate(axis_labels):
        axes[i].plot(time, actual[:, i], label="actual", linewidth=1.6)
        axes[i].plot(time, desired[:, i], "--", label="desired", linewidth=1.6)
        axes[i].set_ylabel(label + " [m]")
        axes[i].grid(True, alpha=0.3)
        axes[i].legend(loc="best")

    axes[-1].set_xlabel("time [s] or sample index")
    fig.suptitle(f"{hand_name} Hand: Actual vs Desired Trajectory")
    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _plot_error(time, err, norm_err, hand_name, save_path):
    axis_labels = ["x", "y", "z"]
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

    for i, label in enumerate(axis_labels):
        axes[i].plot(time, err[:, i], linewidth=1.4)
        axes[i].axhline(0.0, linestyle="--", linewidth=1.0, color="k", alpha=0.5)
        axes[i].set_ylabel(f"e_{label} [m]")
        axes[i].grid(True, alpha=0.3)

    axes[3].plot(time, norm_err, color="tab:red", linewidth=1.6)
    axes[3].set_ylabel("||e|| [m]")
    axes[3].set_xlabel("time [s] or sample index")
    axes[3].grid(True, alpha=0.3)

    fig.suptitle(f"{hand_name} Hand: Tracking Error")
    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _write_metrics(path, left_metrics, right_metrics):
    def fmt(arr):
        return f"[{arr[0]:.6f}, {arr[1]:.6f}, {arr[2]:.6f}]"

    lines = [
        "# Trajectory Tracking Metrics",
        "",
        "[Left Hand]",
        f"RMSE xyz [m]: {fmt(left_metrics['rmse_xyz'])}",
        f"MAE  xyz [m]: {fmt(left_metrics['mae_xyz'])}",
        f"MaxAbs xyz [m]: {fmt(left_metrics['max_abs_xyz'])}",
        f"RMSE ||e|| [m]: {left_metrics['rmse_norm']:.6f}",
        f"Max  ||e|| [m]: {left_metrics['max_norm']:.6f}",
        "",
        "[Right Hand]",
        f"RMSE xyz [m]: {fmt(right_metrics['rmse_xyz'])}",
        f"MAE  xyz [m]: {fmt(right_metrics['mae_xyz'])}",
        f"MaxAbs xyz [m]: {fmt(right_metrics['max_abs_xyz'])}",
        f"RMSE ||e|| [m]: {right_metrics['rmse_norm']:.6f}",
        f"Max  ||e|| [m]: {right_metrics['max_norm']:.6f}",
        "",
    ]
    path.write_text("\n".join(lines))


def _load_min_distance(path):
    data = _ensure_2d(np.loadtxt(path))
    if data.shape[1] < 3:
        raise ValueError("min_distance_log must have at least 3 columns: t left right")
    idx = np.arange(data.shape[0])
    left = data[:, 1]
    right = data[:, 2]
    return idx, left, right


def _plot_min_distance(sample_idx, left, right, save_path):
    fig, ax = plt.subplots(1, 1, figsize=(10, 4))
    ax.plot(sample_idx, left, label="left", linewidth=1.6)
    ax.plot(sample_idx, right, label="right", linewidth=1.6)
    ax.set_xlabel("sample index")
    ax.set_ylabel("min distance [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Compare hand trajectory tracking: actual(hand_log) vs desired(hand_traj_log)."
    )
    parser.add_argument(
        "--hand-log",
        default="hand_log.txt",
        help="Path to hand_log file (actual hand positions)",
    )
    parser.add_argument(
        "--hand-traj-log",
        default="hand_traj_log.txt",
        help="Path to hand_traj_log file (desired hand trajectories)",
    )
    parser.add_argument(
        "--save-dir",
        default="plots_hand",
        help="Directory where plots and metrics are saved",
    )
    parser.add_argument(
        "--min-distance-log",
        default="min_distance_log.txt",
        help="Path to min_distance_log file (time left right)",
    )
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    time, l_act, l_des, r_act, r_des = _load_hand_data(args.hand_log, args.hand_traj_log)

    l_metrics = _calc_metrics(l_act, l_des)
    r_metrics = _calc_metrics(r_act, r_des)

    _plot_hand_tracking(time, l_act, l_des, "Left", save_dir / "left_hand_tracking.png")
    _plot_hand_tracking(time, r_act, r_des, "Right", save_dir / "right_hand_tracking.png")
    _plot_error(time, l_metrics["err"], l_metrics["norm_err"], "Left", save_dir / "left_hand_error.png")
    _plot_error(time, r_metrics["err"], r_metrics["norm_err"], "Right", save_dir / "right_hand_error.png")

    metrics_file = save_dir / "tracking_metrics.txt"
    _write_metrics(metrics_file, l_metrics, r_metrics)

    min_dist_path = Path(args.min_distance_log)
    if min_dist_path.exists():
        t_min, left_min, right_min = _load_min_distance(min_dist_path)
        _plot_min_distance(t_min, left_min, right_min, save_dir / "min_distance.png")
    else:
        print(f"min_distance_log not found, skipping: {min_dist_path}")

    print(f"Saved plots and metrics to: {save_dir.resolve()}")
    print(f"Metrics file: {metrics_file.resolve()}")


if __name__ == "__main__":
    main()