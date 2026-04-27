import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _load_scalar_series(path):
    data = np.loadtxt(path)
    if np.size(data) == 0:
        raise ValueError(f"Empty log file: {path}")

    arr = np.asarray(data)
    if arr.ndim == 0:
        return arr.reshape(1)
    if arr.ndim == 1:
        return arr
    if arr.ndim == 2:
        if arr.shape[1] == 1:
            return arr[:, 0]
        raise ValueError(
            f"Expected 1 column in {path}, but got {arr.shape[1]} columns."
        )
    raise ValueError(f"Unsupported data shape in {path}: {arr.shape}")


def _resolve_time_axis(n_samples, dt, hz):
    if dt is not None and hz is not None:
        raise ValueError("Use either --dt or --hz, not both.")
    if dt is not None:
        if dt <= 0.0:
            raise ValueError("--dt must be positive.")
        return np.arange(n_samples) * dt, "time [s]"
    if hz is not None:
        if hz <= 0.0:
            raise ValueError("--hz must be positive.")
        return np.arange(n_samples) / hz, "time [s]"
    return np.arange(n_samples), "sample index"


def _calc_metrics(desired, position):
    err = desired - position
    abs_err = np.abs(err)
    return {
        "rmse": np.sqrt(np.mean(err**2)),
        "mae": np.mean(abs_err),
        "max_abs": np.max(abs_err),
        "mean_err": np.mean(err),
        "final_err": err[-1],
        "err": err,
    }


def _plot_tracking(time, desired, position, x_label, save_path):
    fig, ax = plt.subplots(figsize=(10, 4.5))
    ax.plot(time, desired, label="desired q", linewidth=1.7)
    ax.plot(time, position, label="actual q", linewidth=1.7)
    ax.set_title("Joint Position Tracking")
    ax.set_ylabel("position [rad]")
    ax.set_xlabel(x_label)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _plot_error(time, err, x_label, save_path):
    fig, ax = plt.subplots(figsize=(10, 4.0))
    ax.plot(time, err, color="tab:red", linewidth=1.5)
    ax.axhline(0.0, linestyle="--", linewidth=1.0, color="k", alpha=0.6)
    ax.set_title("Tracking Error (desired - actual)")
    ax.set_ylabel("error [rad]")
    ax.set_xlabel(x_label)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _plot_velocity_torque(time, velocity, torque, x_label, save_path):
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)

    axes[0].plot(time, velocity, color="tab:green", linewidth=1.5)
    axes[0].set_title("Joint Velocity")
    axes[0].set_ylabel("velocity [rad/s]")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time, torque, color="tab:purple", linewidth=1.5)
    axes[1].set_title("Commanded Torque")
    axes[1].set_ylabel("torque [Nm]")
    axes[1].set_xlabel(x_label)
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _write_metrics(path, metrics, n_samples):
    lines = [
        "# Joint Tracking Metrics",
        "",
        f"samples: {n_samples}",
        f"RMSE [rad]: {metrics['rmse']:.8f}",
        f"MAE [rad]: {metrics['mae']:.8f}",
        f"MaxAbs [rad]: {metrics['max_abs']:.8f}",
        f"Mean error [rad]: {metrics['mean_err']:.8f}",
        f"Final error [rad]: {metrics['final_err']:.8f}",
        "",
    ]
    path.write_text("\n".join(lines))


def main():
    parser = argparse.ArgumentParser(
        description="Compare joint tracking logs: desired position vs actual position."
    )
    parser.add_argument(
        "--desired-log",
        default="joint_desired_log.txt",
        help="Path to desired joint position log",
    )
    parser.add_argument(
        "--position-log",
        default="joint_position_log.txt",
        help="Path to measured joint position log",
    )
    parser.add_argument(
        "--velocity-log",
        default="joint_velocity_log.txt",
        help="Path to measured joint velocity log",
    )
    parser.add_argument(
        "--torque-log",
        default="torque_sum_log.txt",
        help="Path to commanded torque log",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Sampling time [s]. If omitted, x-axis uses sample index.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=None,
        help="Sampling frequency [Hz]. Alternative to --dt.",
    )
    parser.add_argument(
        "--save-dir",
        default="plots_joint",
        help="Directory where plots and metrics are saved",
    )
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    desired = _load_scalar_series(args.desired_log)
    position = _load_scalar_series(args.position_log)
    velocity = _load_scalar_series(args.velocity_log)
    torque = _load_scalar_series(args.torque_log)

    n = min(desired.size, position.size, velocity.size, torque.size)
    if n == 0:
        raise ValueError("No samples available. Check log files.")

    desired = desired[:n]
    position = position[:n]
    velocity = velocity[:n]
    torque = torque[:n]

    time, x_label = _resolve_time_axis(n, args.dt, args.hz)
    metrics = _calc_metrics(desired, position)

    _plot_tracking(time, desired, position, x_label, save_dir / "joint_tracking.png")
    _plot_error(time, metrics["err"], x_label, save_dir / "joint_error.png")
    _plot_velocity_torque(time, velocity, torque, x_label, save_dir / "joint_velocity_torque.png")

    np.savetxt(
        save_dir / "joint_compare.csv",
        np.column_stack([time, desired, position, metrics["err"], velocity, torque]),
        delimiter=",",
        header="x_axis,desired_pos,actual_pos,error,actual_vel,commanded_torque",
        comments="",
    )

    metrics_file = save_dir / "joint_tracking_metrics.txt"
    _write_metrics(metrics_file, metrics, n)

    print(f"Saved plots and metrics to: {save_dir.resolve()}")
    print(f"Metrics file: {metrics_file.resolve()}")


if __name__ == "__main__":
    main()