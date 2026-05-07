#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

COMPONENTS = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]


def load_wrench_log(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Missing file: {path}")
    data = np.loadtxt(path, dtype=float)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != 12:
        raise ValueError(f"Expected 12 columns in {path}, got {data.shape[1]}")
    return data


def plot_wrench(time_s: np.ndarray, measured: np.ndarray, desired: np.ndarray, title: str) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    axes = axes.flatten()

    for i, ax in enumerate(axes):
        ax.plot(time_s, measured[:, i], label=f"{COMPONENTS[i]} meas", linewidth=1.5)
        ax.plot(time_s, desired[:, i], label=f"{COMPONENTS[i]} des", linestyle="--", linewidth=1.2)
        ax.set_ylabel(COMPONENTS[i])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)

    axes[-2].set_xlabel("Time [s]")
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])


def resolve_output_path(out_path: str, side: str) -> str:
    if "{side}" in out_path:
        return out_path.format(side=side)

    path = Path(out_path)
    if path.suffix:
        return str(path.with_name(f"{path.stem}_{side}{path.suffix}"))
    return f"{out_path}_{side}.png"


def check_tipping_constraints(wrench: np.ndarray, x_half: float, y_half: float, tol: float) -> dict:
    # Constraint rows: A * wrench <= 0
    a = np.array(
        [
            [0.0, 0.0, -y_half, -1.0, 0.0, 0.0],
            [0.0, 0.0, -y_half, +1.0, 0.0, 0.0],
            [0.0, 0.0, -x_half, 0.0, -1.0, 0.0],
            [0.0, 0.0, -x_half, 0.0, +1.0, 0.0],
        ],
        dtype=float,
    )
    vals = (a @ (wrench).T).T
    max_violation = np.max(vals, axis=1)
    violated = max_violation > tol
    return {
        "total": wrench.shape[0],
        "violated": int(np.sum(violated)),
        "max_violation": float(np.max(max_violation)) if wrench.size else 0.0,
        "mean_violation": float(np.mean(np.maximum(max_violation, 0.0))) if wrench.size else 0.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot force/torque logs (LF/RF)")
    parser.add_argument("--log", default="force_log.txt", help="Measured wrench log")
    parser.add_argument("--traj", default="force_traj_log.txt", help="Desired wrench log")
    parser.add_argument("--hz", type=float, default=2000.0, help="Log rate [Hz]")
    parser.add_argument("--out", default="plots_force/force_wrench.png", help="Output image path (default: save)")
    parser.add_argument("--show", action="store_true", help="Show plots instead of saving")
    parser.add_argument("--x-half", type=float, default=0.3, help="Half foot length X [m]")
    parser.add_argument("--y-half", type=float, default=0.26, help="Half foot width Y [m]")
    parser.add_argument("--tol", type=float, default=1e-6, help="Constraint tolerance")
    args = parser.parse_args()

    log_path = Path(args.log)
    traj_path = Path(args.traj)

    measured = load_wrench_log(log_path)
    desired = load_wrench_log(traj_path)

    n = min(measured.shape[0], desired.shape[0])
    measured = measured[:n, :]
    desired = desired[:n, :]

    time_s = np.arange(n) / args.hz

    lf_meas = -measured[:, :6]
    rf_meas = -measured[:, 6:]
    lf_des = desired[:, :6]
    rf_des = desired[:, 6:]

    lf_tip = check_tipping_constraints(lf_meas, args.x_half, args.y_half, args.tol)
    rf_tip = check_tipping_constraints(rf_meas, args.x_half, args.y_half, args.tol)
    print("Tipping constraint check (measured):")
    print(
        f"  Left:  violated {lf_tip['violated']}/{lf_tip['total']} | "
        f"max {lf_tip['max_violation']:.6f} | mean {lf_tip['mean_violation']:.6f}"
    )
    print(
        f"  Right: violated {rf_tip['violated']}/{rf_tip['total']} | "
        f"max {rf_tip['max_violation']:.6f} | mean {rf_tip['mean_violation']:.6f}"
    )

    plot_wrench(time_s, lf_meas, lf_des, "Left Foot Wrench")
    plot_wrench(time_s, rf_meas, rf_des, "Right Foot Wrench")

    if args.show:
        plt.show()
    else:
        left_out = resolve_output_path(args.out, "left")
        right_out = resolve_output_path(args.out, "right")
        plt.figure(1)
        plt.savefig(left_out, dpi=150)
        plt.figure(2)
        plt.savefig(right_out, dpi=150)


if __name__ == "__main__":
    main()
