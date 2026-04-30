import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _ensure_2d(arr):
    if arr.ndim == 1:
        return arr.reshape(1, -1)
    return arr


def _load_com_data(path):
    arr = _ensure_2d(np.loadtxt(path))
    if arr.shape[1] == 3:
        t = None
        data = arr[:, 0:3]
    elif arr.shape[1] >= 4:
        t = arr[:, 0]
        data = arr[:, 1:4]
    else:
        raise ValueError("Unsupported COM log format. Expect 3 cols (x y z) or 4+ (t x y z).")
    return t, data


def _calc_metrics(actual, desired):
    err = actual - desired
    rmse_xyz = np.sqrt(np.mean(err ** 2, axis=0))
    mae_xyz = np.mean(np.abs(err), axis=0)
    max_abs_xyz = np.max(np.abs(err), axis=0)
    norm_err = np.linalg.norm(err, axis=1)
    return {
        "rmse_xyz": rmse_xyz,
        "mae_xyz": mae_xyz,
        "max_abs_xyz": max_abs_xyz,
        "rmse_norm": np.sqrt(np.mean(norm_err ** 2)),
        "max_norm": np.max(norm_err),
        "err": err,
        "norm_err": norm_err,
    }


def _plot_tracking(t, actual, desired, save_dir):
    labels = ["x", "y", "z"]
    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i in range(3):
        axs[i].plot(t, actual[:, i], label="actual", linewidth=1.5)
        axs[i].plot(t, desired[:, i], "--", label="desired", linewidth=1.5)
        axs[i].set_ylabel(f"{labels[i]} [m]")
        axs[i].grid(True, alpha=0.3)
        axs[i].legend(loc="best")
    axs[-1].set_xlabel("time [s] or sample index")
    fig.suptitle("COM: Actual vs Desired")
    fig.tight_layout()
    p = save_dir / "com_tracking.png"
    fig.savefig(p, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return p


def _plot_error(t, err, norm_err, save_dir):
    labels = ["x", "y", "z"]
    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for i in range(3):
        axs[i].plot(t, err[:, i], linewidth=1.4)
        axs[i].axhline(0.0, linestyle="--", color="k", alpha=0.5)
        axs[i].set_ylabel(f"e_{labels[i]} [m]")
        axs[i].grid(True, alpha=0.3)
    axs[3].plot(t, norm_err, color="tab:red", linewidth=1.6)
    axs[3].set_ylabel("||e|| [m]")
    axs[3].set_xlabel("time [s] or sample index")
    axs[3].grid(True, alpha=0.3)
    fig.suptitle("COM Tracking Error")
    fig.tight_layout()
    p = save_dir / "com_error.png"
    fig.savefig(p, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return p


def _write_metrics(path, metrics):
    def fmt(a):
        return f"[{a[0]:.6f}, {a[1]:.6f}, {a[2]:.6f}]"

    lines = [
        "# COM Trajectory Tracking Metrics",
        f"RMSE xyz [m]: {fmt(metrics['rmse_xyz'])}",
        f"MAE  xyz [m]: {fmt(metrics['mae_xyz'])}",
        f"MaxAbs xyz [m]: {fmt(metrics['max_abs_xyz'])}",
        f"RMSE ||e|| [m]: {metrics['rmse_norm']:.6f}",
        f"Max  ||e|| [m]: {metrics['max_norm']:.6f}",
    ]
    path.write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(description="Plot COM actual vs desired from logs.")
    parser.add_argument("--com-log", default="com_log.txt", help="Path to com_log (actual COM)")
    parser.add_argument("--com-traj-log", default="com_traj_log.txt", help="Path to com_traj_log (desired COM)")
    parser.add_argument("--save-dir", default="plots_com", help="Directory to save plots and metrics")
    parser.add_argument("--hz", type=float, default=None, help="Sampling frequency in Hz (optional). If provided and no time column present, time = np.arange(n)/hz")

    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    t_a, a = _load_com_data(args.com_log)
    t_d, d = _load_com_data(args.com_traj_log)

    n = min(a.shape[0], d.shape[0])
    a = a[:n]
    d = d[:n]

    # time axis
    if t_a is not None:
        t = t_a[:n]
    elif t_d is not None:
        t = t_d[:n]
    elif args.hz is not None:
        t = np.arange(n) / float(args.hz)
    else:
        t = np.arange(n)

    metrics = _calc_metrics(a, d)

    p1 = _plot_tracking(t, a, d, save_dir)
    p2 = _plot_error(t, metrics['err'], metrics['norm_err'], save_dir)

    metrics_file = save_dir / "com_tracking_metrics.txt"
    _write_metrics(metrics_file, metrics)

    print(f"Saved plots: {p1}, {p2}")
    print(f"Metrics: {metrics_file.resolve()}")


if __name__ == "__main__":
    main()
