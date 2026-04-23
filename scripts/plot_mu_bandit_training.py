#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


plt.rcParams.update({
    "figure.figsize": (3.5, 4),
    "figure.dpi": 300,
    "font.size": 10,
    "axes.titlesize": 10,
    "axes.labelsize": 9,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "lines.linewidth": 1.4,
})


def load_rows(csv_path: Path):
    run_rows = []
    meta_rows = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["iteration"] == "arm_meta":
                meta_rows.append({
                    "arm_idx": int(row["arm_idx"]),
                    "mu_d": float(row["mu_d"]),
                })
            else:
                run_rows.append({
                    "iteration": int(row["iteration"]),
                    "arm_idx": int(row["arm_idx"]),
                    "mu_d": float(row["mu_d"]),
                    "reward": float(row["reward"]),
                    "best_arm_idx": int(row["best_arm_idx"]),
                    "best_mu_d": float(row["best_mu_d"]),
                    "reward_rms_mean": float(row["reward_rms_mean"]),
                    "thrust_std_1": float(row["thrust_std_1"]),
                    "thrust_std_2": float(row["thrust_std_2"]),
                    "thrust_std_3": float(row["thrust_std_3"]),
                    "thrust_std_4": float(row["thrust_std_4"]),
                })
    return run_rows, meta_rows


def mean_reward_per_arm(run_rows):
    grouped = {}
    for row in run_rows:
        key = (row["arm_idx"], row["mu_d"])
        grouped.setdefault(key, []).append(row["reward"])
    out = []
    for (arm_idx, mu_d), rewards in sorted(grouped.items()):
        out.append({"arm_idx": arm_idx, "mu_d": mu_d, "reward_mean": sum(rewards) / len(rewards)})
    return out


def count_per_arm(run_rows):
    grouped = {}
    for row in run_rows:
        key = (row["arm_idx"], row["mu_d"])
        grouped[key] = grouped.get(key, 0) + 1
    out = []
    for (arm_idx, mu_d), count in sorted(grouped.items()):
        out.append({"arm_idx": arm_idx, "mu_d": mu_d, "count": count})
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--save", type=Path, default=None)
    args = parser.parse_args()

    run_rows, meta_rows = load_rows(args.csv_path)
    if not run_rows:
        raise RuntimeError(f"No training rows found in {args.csv_path}")

    reward_stats = mean_reward_per_arm(run_rows)
    count_stats = count_per_arm(run_rows)

    iterations = [row["iteration"] for row in run_rows]
    rewards = [row["reward"] for row in run_rows]
    mu_d_selected = [row["mu_d"] for row in run_rows]
    best_mu_d = [row["best_mu_d"] for row in run_rows]

    fig, axes = plt.subplots(4, 1, constrained_layout=True)

    ax = axes[0]
    ax.plot(iterations, rewards, color="tab:blue")
    ax.text(0.01, 0.98, "(a)", transform=ax.transAxes, va="top", ha="left", fontsize=10, fontweight="bold")
    ax.set_xlabel("")
    ax.set_ylabel(r"$r$", labelpad=2)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.scatter(iterations, mu_d_selected, color="tab:orange", s=4, alpha=0.45, linewidths=0, label=r"selected $\mu_d$")
    ax.plot(iterations, best_mu_d, color="tab:green", linestyle="--", label=r"best $\mu_d$")
    ax.text(0.01, 0.98, "(b)", transform=ax.transAxes, va="top", ha="left", fontsize=10, fontweight="bold")
    ax.set_xlabel("")
    ax.set_ylabel(r"$\mu_d$", labelpad=2)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", frameon=False, handlelength=1.8, borderpad=0.2)

    ax = axes[2]
    ax.bar([row["mu_d"] for row in reward_stats],
           [row["reward_mean"] for row in reward_stats],
           width=0.028,
           color="tab:purple")
    ax.text(0.01, 0.98, "(c)", transform=ax.transAxes, va="top", ha="left", fontsize=10, fontweight="bold")
    ax.set_xlabel("")
    ax.set_ylabel(r"$\bar{r}$", labelpad=2)
    ax.grid(True, alpha=0.3)

    ax = axes[3]
    ax.bar([row["mu_d"] for row in count_stats],
           [row["count"] for row in count_stats],
           width=0.028,
           color="tab:red")
    ax.text(0.01, 0.98, "(d)", transform=ax.transAxes, va="top", ha="left", fontsize=10, fontweight="bold")
    ax.set_xlabel("Iteration / arm")
    ax.set_ylabel(r"$N$", labelpad=2)
    ax.grid(True, alpha=0.3)

    if args.save is not None:
        fig.savefig(args.save, bbox_inches="tight")
    else:
        plt.show()


if __name__ == "__main__":
    main()
