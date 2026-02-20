#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_csv(path):
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if not rows:
            raise RuntimeError("CSV has no data rows")
        cols = reader.fieldnames or []

    data = {c: [] for c in cols}
    for r in rows:
        for c in cols:
            v = r.get(c, "")
            data[c].append(float(v) if v not in ("", None) else 0.0)
    return data


def has_all(data, names):
    return all(n in data for n in names)


def vec_cols(prefix, n=6):
    return [f"{prefix}_{i}" for i in range(n)]


def plot_vec(ax, t, data, prefix, n=6, linestyle="-", alpha=1.0, label_prefix=None):
    cols = vec_cols(prefix, n)
    if not has_all(data, cols):
        ax.text(0.5, 0.5, f"Missing: {prefix}_*", transform=ax.transAxes, ha="center")
        return
    for i, c in enumerate(cols):
        label = f"{label_prefix}{i}" if label_prefix else c
        ax.plot(t, data[c], linestyle=linestyle, alpha=alpha, label=label)


def plot_vec_compare(ax, t, data, a_prefix, b_prefix, n=6,
                     a_style="-", b_style="--",
                     a_label_prefix="a_", b_label_prefix="b_"):
    a_cols = vec_cols(a_prefix, n)
    b_cols = vec_cols(b_prefix, n)
    if not has_all(data, a_cols) or not has_all(data, b_cols):
        ax.text(0.5, 0.5, f"Missing: {a_prefix}_* or {b_prefix}_*",
                transform=ax.transAxes, ha="center")
        return
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    for i in range(n):
        color = colors[i % len(colors)] if colors else None
        ax.plot(t, data[f"{a_prefix}_{i}"], linestyle=a_style, color=color,
                alpha=0.9, label=f"{a_label_prefix}{i}")
        ax.plot(t, data[f"{b_prefix}_{i}"], linestyle=b_style, color=color,
                alpha=0.9, label=f"{b_label_prefix}{i}")


def plot_vec_indices(ax, t, data, prefix, indices, linestyle="-", alpha=1.0, label_prefix=None):
    cols = [f"{prefix}_{i}" for i in indices]
    if not has_all(data, cols):
        ax.text(0.5, 0.5, f"Missing: {prefix}_*", transform=ax.transAxes, ha="center")
        return
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    for j, i in enumerate(indices):
        c = f"{prefix}_{i}"
        color = colors[j % len(colors)] if colors else None
        label = f"{label_prefix}{i}" if label_prefix else c
        ax.plot(t, data[c], linestyle=linestyle, alpha=alpha, color=color, label=label)


def plot_vec_compare_indices(ax, t, data, a_prefix, b_prefix, indices,
                             a_style="-", b_style="--",
                             a_label_prefix="a_", b_label_prefix="b_"):
    a_cols = [f"{a_prefix}_{i}" for i in indices]
    b_cols = [f"{b_prefix}_{i}" for i in indices]
    if not has_all(data, a_cols) or not has_all(data, b_cols):
        ax.text(0.5, 0.5, f"Missing: {a_prefix}_* or {b_prefix}_*",
                transform=ax.transAxes, ha="center")
        return
    colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
    for j, i in enumerate(indices):
        color = colors[j % len(colors)] if colors else None
        ax.plot(t, data[f"{a_prefix}_{i}"], linestyle=a_style, color=color,
                alpha=0.9, label=f"{a_label_prefix}{i}")
        ax.plot(t, data[f"{b_prefix}_{i}"], linestyle=b_style, color=color,
                alpha=0.9, label=f"{b_label_prefix}{i}")


def draw_dashboard(t, data, title):
    fig, axs = plt.subplots(4, 3, figsize=(18, 13), sharex=True)
    axs = axs.ravel()

    # Row 1: eta1, eta2, joint_position
    ax = axs[0]
    plot_vec(ax, t, data, "eta1", n=3, linestyle="-", label_prefix="eta1_")
    ax.set_title("eta1")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    ax = axs[1]
    plot_vec(ax, t, data, "eta2", n=3, linestyle="-", label_prefix="eta2_")
    ax.set_title("eta2")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    ax = axs[2]
    plot_vec(ax, t, data, "joint_position", n=6, linestyle="-", label_prefix="q_")
    ax.set_title("joint_position")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    # Row 2: nu desired vs actual (1-3), nu desired vs actual (4-6), joint velocity desired vs actual
    ax = axs[3]
    plot_vec_compare_indices(ax, t, data, "nu_d", "nu", [0, 1, 2],
                             a_style="-", b_style="--",
                             a_label_prefix="nu_d_", b_label_prefix="nu_")
    ax.set_title("nu 1-3 (desired vs actual)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    ax = axs[4]
    plot_vec_compare_indices(ax, t, data, "nu_d", "nu", [3, 4, 5],
                             a_style="-", b_style="--",
                             a_label_prefix="nu_d_", b_label_prefix="nu_")
    ax.set_title("nu 4-6 (desired vs actual)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    ax = axs[5]
    if has_all(data, vec_cols("joint_velocities", 6)) and has_all(data, vec_cols("joint_velocity", 6)):
        plot_vec_compare(ax, t, data, "joint_velocities", "joint_velocity", n=6,
                         a_style="-", b_style="--",
                         a_label_prefix="dq_d_", b_label_prefix="dq_")
    else:
        if has_all(data, vec_cols("joint_velocity", 6)):
            plot_vec(ax, t, data, "joint_velocity", n=6, linestyle="-", label_prefix="dq_")
        else:
            plot_vec(ax, t, data, "joint_velocities", n=6, linestyle="-", label_prefix="dq_d_")
    ax.set_title("joint_velocity (desired vs actual)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    # Row 3: x_ee compare, q_ee compare, contact force compare
    ax = axs[6]
    x_cols = [f"x_ee_{i}" for i in range(3)]
    xd_cols = [f"x_ee_d_{i}" for i in range(3)]
    xr_cols = [f"x_ee_r_{i}" for i in range(3)]
    if has_all(data, x_cols) and has_all(data, xd_cols) and has_all(data, xr_cols):
        colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
        for i in range(3):
            color = colors[i % len(colors)] if colors else None
            ax.plot(t, data[f"x_ee_{i}"], linestyle="-", color=color, alpha=0.9, label=f"x_ee_{i}")
            ax.plot(t, data[f"x_ee_r_{i}"], linestyle="--", color=color, alpha=0.9, label=f"x_ee_r_{i}")
            ax.plot(t, data[f"x_ee_d_{i}"], linestyle=":", color=color, alpha=0.9, label=f"x_ee_d_{i}")
    else:
        ax.text(0.5, 0.5, "Missing x_ee/x_ee_r/x_ee_d", transform=ax.transAxes, ha="center")
    ax.set_title("x_ee (actual/ref/desired)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    ax = axs[7]
    q_cols = [f"q_ee_{i}" for i in range(4)]
    qd_cols = [f"q_ee_d_{i}" for i in range(4)]
    qr_cols = [f"q_ee_r_{i}" for i in range(4)]
    if has_all(data, q_cols) and has_all(data, qd_cols) and has_all(data, qr_cols):
        colors = plt.rcParams["axes.prop_cycle"].by_key().get("color", [])
        for i in range(4):
            color = colors[i % len(colors)] if colors else None
            ax.plot(t, data[f"q_ee_{i}"], linestyle="-", color=color, alpha=0.9, label=f"q_ee_{i}")
            ax.plot(t, data[f"q_ee_r_{i}"], linestyle="--", color=color, alpha=0.9, label=f"q_ee_r_{i}")
            ax.plot(t, data[f"q_ee_d_{i}"], linestyle=":", color=color, alpha=0.9, label=f"q_ee_d_{i}")
    else:
        ax.text(0.5, 0.5, "Missing q_ee/q_ee_r/q_ee_d", transform=ax.transAxes, ha="center")
    ax.set_title("q_ee (actual/ref/desired)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=4, fontsize=8)

    ax = axs[8]
    plot_vec_compare(ax, t, data, "sensor_feedback_calibrated_ontiplink", "h_e_tipframe", n=3,
                     a_style="-", b_style="--",
                     a_label_prefix="F_sensor_tip_", b_label_prefix="F_est_tip_")
    ax.set_title("Contact Force at Tip (Sensor vs Estimated)")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    # Row 4: control wrench, thruster_forces, thruster_setpoints
    ax = axs[9]
    plot_vec(ax, t, data, "control_wrench", n=6, linestyle="-", label_prefix="u_")
    ax.set_title("control_wrench")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    ax = axs[10]
    plot_vec(ax, t, data, "thruster_force", n=6, linestyle="-", label_prefix="F_thr_")
    ax.set_title("thruster_forces")
    ax.set_ylim(-110.0, 110.0)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    ax = axs[11]
    plot_vec(ax, t, data, "setpoints", n=6, linestyle="-", label_prefix="sp_")
    ax.set_title("thruster_setpoints")
    ax.set_ylim(-1.0, 1.0)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)

    for i in range(12):
        axs[i].set_xlabel("Time [s]")

    fig.suptitle(title, fontsize=12)
    plt.tight_layout()


def slice_by_time(t, data, t_start, t_end):
    idx = [i for i, ti in enumerate(t) if t_start <= ti <= t_end]
    if not idx:
        raise RuntimeError(f"No samples in [{t_start}, {t_end}]")
    t_out = [t[i] for i in idx]
    data_out = {k: [v[i] for i in idx] for k, v in data.items()}
    return t_out, data_out


def main():
    parser = argparse.ArgumentParser(description="Plot key control signals for pushing scenario")
    parser.add_argument(
        "--csv",
        type=str,
        default="src/sensorless_force_control/log/controller_data_20260220121336.csv",
        help="Path to controller CSV log",
    )
    parser.add_argument("--t-start", type=float, default=None, help="Window start time [s] from log start")
    parser.add_argument("--t-end", type=float, default=None, help="Window end time [s] from log start")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    data = load_csv(csv_path)
    if "t" not in data:
        raise RuntimeError("CSV must contain column 't'")
    t0 = data["t"][0]
    t = [x - t0 for x in data["t"]]

    draw_dashboard(t, data, f"Pushing Scenario (Overview): {csv_path.name}")

    if (args.t_start is None) ^ (args.t_end is None):
        raise RuntimeError("Please provide both --t-start and --t-end, or neither.")
    if args.t_start is not None and args.t_end is not None:
        if args.t_start >= args.t_end:
            raise RuntimeError("--t-start must be smaller than --t-end")
        tw, dw = slice_by_time(t, data, args.t_start, args.t_end)
        draw_dashboard(
            tw,
            dw,
            f"Pushing Scenario (Window {args.t_start:.2f}s - {args.t_end:.2f}s): {csv_path.name}",
        )

    plt.show()


if __name__ == "__main__":
    main()
