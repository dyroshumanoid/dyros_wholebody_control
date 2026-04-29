import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _load_matrix(path):
    data = np.loadtxt(path)
    if np.size(data) == 0:
        raise ValueError(f"Empty log file: {path}")

    arr = np.asarray(data)
    if arr.ndim == 0:
        return arr.reshape(1, 1)
    if arr.ndim == 1:
        return arr.reshape(-1, 1)
    if arr.ndim == 2:
        return arr
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


def _align_logs(desired, position, torque):
    n_samples = min(desired.shape[0], position.shape[0], torque.shape[0])
    n_joints = min(desired.shape[1], position.shape[1], torque.shape[1])

    if n_samples == 0 or n_joints == 0:
        raise ValueError("No overlapping samples/joints across logs.")

    desired = desired[:n_samples, :n_joints]
    position = position[:n_samples, :n_joints]
    torque = torque[:n_samples, :n_joints]

    return desired, position, torque


def _compute_metrics(desired, position):
    err = desired - position
    rmse = np.sqrt(np.mean(err**2, axis=0))
    mae = np.mean(np.abs(err), axis=0)
    max_abs = np.max(np.abs(err), axis=0)

    return {
        "err": err,
        "rmse": rmse,
        "mae": mae,
        "max_abs": max_abs,
        "global_rmse": float(np.sqrt(np.mean(err**2))),
        "global_mae": float(np.mean(np.abs(err))),
        "global_max_abs": float(np.max(np.abs(err))),
    }


def _plot_heatmaps(err, torque, save_path):
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)

    vmax_err = np.max(np.abs(err))
    vmax_err = 1e-8 if vmax_err == 0 else vmax_err
    im0 = axes[0].imshow(
        err,
        aspect="auto",
        origin="lower",
        cmap="coolwarm",
        vmin=-vmax_err,
        vmax=vmax_err,
    )
    axes[0].set_title("Position Error Heatmap (desired - actual)")
    axes[0].set_ylabel("sample")
    cbar0 = fig.colorbar(im0, ax=axes[0])
    cbar0.set_label("error [rad]")

    vmax_tau = np.max(np.abs(torque))
    vmax_tau = 1e-8 if vmax_tau == 0 else vmax_tau
    im1 = axes[1].imshow(
        torque,
        aspect="auto",
        origin="lower",
        cmap="viridis",
        vmin=-vmax_tau,
        vmax=vmax_tau,
    )
    axes[1].set_title("Commanded Torque Heatmap")
    axes[1].set_ylabel("sample")
    axes[1].set_xlabel("joint index")
    cbar1 = fig.colorbar(im1, ax=axes[1])
    cbar1.set_label("torque [Nm]")

    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def _parse_joint_groups(group_text):
    tokens = [t.strip() for t in group_text.split("/") if t.strip()]
    if not tokens:
        raise ValueError("--joint-groups is empty")

    groups = []
    for t in tokens:
        v = int(t)
        if v <= 0:
            raise ValueError("Each joint group size must be positive")
        groups.append(v)
    return groups


def _plot_per_joint_pages(time, x_label, desired, position, torque, save_dir, joint_groups):
    n_joints = desired.shape[1]

    start = 0
    for g in joint_groups:
        if start >= n_joints:
            break

        end = min(start + g, n_joints)
        rows = end - start

        fig, axes = plt.subplots(rows, 2, figsize=(14, 2.6 * rows), sharex="col")
        if rows == 1:
            axes = np.array([axes])

        for r, j in enumerate(range(start, end)):
            ax_pos = axes[r, 0]
            ax_tau = axes[r, 1]

            ax_pos.plot(time, desired[:, j], label="desired", linewidth=1.2)
            ax_pos.plot(time, position[:, j], label="actual", linewidth=1.2)
            ax_pos.set_ylabel(f"J{j} [rad]")
            ax_pos.grid(True, alpha=0.3)
            if r == 0:
                ax_pos.set_title("Position Tracking")
                ax_pos.legend(loc="best", fontsize=8)

            ax_tau.plot(time, torque[:, j], color="tab:purple", linewidth=1.2)
            ax_tau.set_ylabel(f"J{j} [Nm]")
            ax_tau.grid(True, alpha=0.3)
            if r == 0:
                ax_tau.set_title("Commanded Torque")

            if r == rows - 1:
                ax_pos.set_xlabel(x_label)
                ax_tau.set_xlabel(x_label)

        fig.suptitle(f"Joint Overview: J{start} to J{end - 1}", fontsize=12)
        fig.tight_layout()
        fig.savefig(save_dir / f"joint_overview_J{start:02d}_to_J{end - 1:02d}.png", dpi=200, bbox_inches="tight")
        plt.close(fig)

        start = end

    if start < n_joints:
        # Plot any remaining joints in a final page if groups do not exactly sum to n_joints.
        end = n_joints
        rows = end - start

        fig, axes = plt.subplots(rows, 2, figsize=(14, 2.6 * rows), sharex="col")
        if rows == 1:
            axes = np.array([axes])

        for r, j in enumerate(range(start, end)):
            ax_pos = axes[r, 0]
            ax_tau = axes[r, 1]

            ax_pos.plot(time, desired[:, j], label="desired", linewidth=1.2)
            ax_pos.plot(time, position[:, j], label="actual", linewidth=1.2)
            ax_pos.set_ylabel(f"J{j} [rad]")
            ax_pos.grid(True, alpha=0.3)
            if r == 0:
                ax_pos.set_title("Position Tracking")
                ax_pos.legend(loc="best", fontsize=8)

            ax_tau.plot(time, torque[:, j], color="tab:purple", linewidth=1.2)
            ax_tau.set_ylabel(f"J{j} [Nm]")
            ax_tau.grid(True, alpha=0.3)
            if r == 0:
                ax_tau.set_title("Commanded Torque")

            if r == rows - 1:
                ax_pos.set_xlabel(x_label)
                ax_tau.set_xlabel(x_label)

        fig.suptitle(f"Joint Overview: J{start} to J{end - 1}", fontsize=12)
        fig.tight_layout()
        fig.savefig(save_dir / f"joint_overview_J{start:02d}_to_J{end - 1:02d}.png", dpi=200, bbox_inches="tight")
        plt.close(fig)


def _write_metrics(path, metrics):
    rmse = metrics["rmse"]
    mae = metrics["mae"]
    max_abs = metrics["max_abs"]

    lines = [
        "# Total Joint Tracking Metrics",
        f"global_rmse_rad: {metrics['global_rmse']:.8f}",
        f"global_mae_rad: {metrics['global_mae']:.8f}",
        f"global_max_abs_rad: {metrics['global_max_abs']:.8f}",
        "",
        "joint,rmse_rad,mae_rad,max_abs_rad",
    ]

    for j in range(rmse.size):
        lines.append(f"{j},{rmse[j]:.8f},{mae[j]:.8f},{max_abs[j]:.8f}")

    path.write_text("\n".join(lines))


def _check_task_command(task_cmd, time, x_label, save_dir):
    if task_cmd.shape[1] < 12:
        raise ValueError("task_command_log must have at least 12 columns")

    task_cmd = task_cmd[:, :12]
    left_pos = task_cmd[:, 0:3]
    right_pos = task_cmd[:, 3:6]
    left_euler = task_cmd[:, 6:9]
    right_euler = task_cmd[:, 9:12]

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)

    axes[0, 0].plot(time, left_pos[:, 0], label="x", linewidth=1.2)
    axes[0, 0].plot(time, left_pos[:, 1], label="y", linewidth=1.2)
    axes[0, 0].plot(time, left_pos[:, 2], label="z", linewidth=1.2)
    axes[0, 0].set_title("Left Hand Position")
    axes[0, 0].set_ylabel("pos [m]")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(loc="best", fontsize=8)

    axes[0, 1].plot(time, right_pos[:, 0], label="x", linewidth=1.2)
    axes[0, 1].plot(time, right_pos[:, 1], label="y", linewidth=1.2)
    axes[0, 1].plot(time, right_pos[:, 2], label="z", linewidth=1.2)
    axes[0, 1].set_title("Right Hand Position")
    axes[0, 1].set_ylabel("pos [m]")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(loc="best", fontsize=8)

    axes[1, 0].plot(time, left_euler[:, 0], label="roll", linewidth=1.2)
    axes[1, 0].plot(time, left_euler[:, 1], label="pitch", linewidth=1.2)
    axes[1, 0].plot(time, left_euler[:, 2], label="yaw", linewidth=1.2)
    axes[1, 0].set_title("Left Hand Euler")
    axes[1, 0].set_ylabel("angle [rad]")
    axes[1, 0].set_xlabel(x_label)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(loc="best", fontsize=8)

    axes[1, 1].plot(time, right_euler[:, 0], label="roll", linewidth=1.2)
    axes[1, 1].plot(time, right_euler[:, 1], label="pitch", linewidth=1.2)
    axes[1, 1].plot(time, right_euler[:, 2], label="yaw", linewidth=1.2)
    axes[1, 1].set_title("Right Hand Euler")
    axes[1, 1].set_ylabel("angle [rad]")
    axes[1, 1].set_xlabel(x_label)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(loc="best", fontsize=8)

    fig.tight_layout()
    fig.savefig(save_dir / "task_command.png", dpi=200, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Check whole-joint position tracking and torque logs together."
    )
    parser.add_argument("--desired-log", default="joint_desired_log.txt", help="Desired joint position log")
    parser.add_argument("--position-log", default="joint_position_log.txt", help="Measured joint position log")
    parser.add_argument("--torque-log", default="torque_sum_log.txt", help="Commanded torque log")
    parser.add_argument(
        "--task-command-log",
        default="task_command_log.txt",
        help="Task command log (left/right hand pos + euler, 12 columns)",
    )
    parser.add_argument("--save-dir", default="plots_total", help="Directory to save outputs")
    parser.add_argument("--dt", type=float, default=None, help="Sampling period [s]")
    parser.add_argument("--hz", type=float, default=None, help="Sampling frequency [Hz]")
    parser.add_argument(
        "--joint-groups",
        default="6/6/3/8/2/8",
        help="Joint group sizes for each page, e.g. 6/6/3/8/2/8",
    )
    args = parser.parse_args()

    joint_groups = _parse_joint_groups(args.joint_groups)

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    desired = _load_matrix(args.desired_log)
    position = _load_matrix(args.position_log)
    torque = _load_matrix(args.torque_log)

    desired, position, torque = _align_logs(desired, position, torque)

    n_samples = desired.shape[0]
    n_joints = desired.shape[1]

    time, x_label = _resolve_time_axis(n_samples, args.dt, args.hz)
    metrics = _compute_metrics(desired, position)

    _plot_heatmaps(metrics["err"], torque, save_dir / "overview_heatmaps.png")
    _plot_per_joint_pages(
        time,
        x_label,
        desired,
        position,
        torque,
        save_dir,
        joint_groups,
    )

    np.save(save_dir / "position_error.npy", metrics["err"])
    _write_metrics(save_dir / "total_joint_metrics.csv", metrics)

    if args.task_command_log:
        task_cmd = _load_matrix(args.task_command_log)
        if task_cmd.shape[0] != n_samples:
            task_cmd = task_cmd[:n_samples, :]
        _check_task_command(task_cmd, time, x_label, save_dir)

    print(f"Saved outputs to: {save_dir.resolve()}")
    print(f"Samples: {n_samples}, joints: {n_joints}")


if __name__ == "__main__":
    main()
