# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "pandas>=2.1", "matplotlib>=3.8"]
# ///
"""Plot a kestrel_sim CSV log.

    uv run tools/plot_run.py build/dev/hover.csv [--out build/figures] [--format png|svg]

Writes <name>_state.<fmt> (position, velocity, attitude, body rates) and
<name>_rotors.<fmt> (rotor speed vs command, laid out like the airframe).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

TRUTH = "#2a78d6"  # blue: simulated truth
COMMAND = "#1baf7a"  # aqua: commands and references

# (y label, columns, panel titles, smallest half-range shown). The minimum range keeps
# 1e-16 rounding noise from autoscaling into something that looks like a real signal.
STATE_ROWS = [
    ("Position [m]", ["pos_n_m", "pos_e_m", "pos_d_m"], ["North", "East", "Down"], 0.1),
    ("Velocity [m/s]", ["vel_n_mps", "vel_e_mps", "vel_d_mps"], ["North", "East", "Down"], 0.1),
    ("Attitude [deg]", ["roll_deg", "pitch_deg", "yaw_deg"], ["Roll (unwrapped)", "Pitch", "Yaw (unwrapped)"], 1.0),
    ("Body rate [rad/s]", ["omega_x_radps", "omega_y_radps", "omega_z_radps"], ["p (x)", "q (y)", "r (z)"], 0.02),
]


def set_min_range(ax: plt.Axes, values: pd.Series, min_half_range: float) -> None:
    lo, hi = float(values.min()), float(values.max())
    center, half = (lo + hi) / 2.0, max((hi - lo) / 2.0 * 1.08, min_half_range)
    ax.set_ylim(center - half, center + half)

# PX4 Quad X, placed where each motor sits when viewed from above (nose up the page).
ROTOR_PANELS = [
    (0, 0, 3, "front-left, CW"),
    (0, 1, 1, "front-right, CCW"),
    (1, 0, 2, "rear-left, CCW"),
    (1, 1, 4, "rear-right, CW"),
]


def apply_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 100,
            "savefig.dpi": 200,
            "font.size": 9,
            "axes.titlesize": 9,
            "axes.labelsize": 9,
            "axes.grid": True,
            "grid.color": "#e1e0d9",
            "grid.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "lines.linewidth": 1.6,
        }
    )


def plot_state(df: pd.DataFrame, title: str, path: Path) -> None:
    df = df.copy()
    # Unwrap angles for display so a steady spin reads as a ramp, not a sawtooth at +-180 deg.
    for column in ("roll_deg", "yaw_deg"):
        df[column] = np.rad2deg(np.unwrap(np.deg2rad(df[column])))
    fig, axes = plt.subplots(len(STATE_ROWS), 3, figsize=(10, 8), sharex=True)
    for row, (ylabel, columns, names, min_half_range) in enumerate(STATE_ROWS):
        for col, (column, name) in enumerate(zip(columns, names)):
            ax = axes[row, col]
            ax.plot(df["t_s"], df[column], color=TRUTH)
            set_min_range(ax, df[column], min_half_range)
            ax.set_title(name)
            if col == 0:
                ax.set_ylabel(ylabel)
            if row == len(STATE_ROWS) - 1:
                ax.set_xlabel("Time [s]")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def plot_rotors(df: pd.DataFrame, title: str, path: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(8, 5.5), sharex=True, sharey=True)
    for row, col, motor, where in ROTOR_PANELS:
        ax = axes[row, col]
        ax.plot(df["t_s"], df[f"rotor{motor}_cmd_radps"], color=COMMAND, label="Commanded")
        ax.plot(df["t_s"], df[f"rotor{motor}_radps"], color=TRUTH, label="Actual")
        ax.set_title(f"Motor {motor} ({where})")
        if col == 0:
            ax.set_ylabel("Rotor speed [rad/s]")
        if row == 1:
            ax.set_xlabel("Time [s]")
    axes[0, 1].legend(loc="best", frameon=False)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", type=Path, help="CSV written by kestrel_sim")
    parser.add_argument("--out", type=Path, default=Path("build/figures"), help="output directory")
    parser.add_argument("--format", choices=["png", "svg"], default="png")
    args = parser.parse_args()

    df = pd.read_csv(args.log)
    args.out.mkdir(parents=True, exist_ok=True)
    apply_style()
    name = args.log.stem
    outputs = [
        args.out / f"{name}_state.{args.format}",
        args.out / f"{name}_rotors.{args.format}",
    ]
    plot_state(df, f"{name}: vehicle state", outputs[0])
    plot_rotors(df, f"{name}: rotor speeds", outputs[1])
    for path in outputs:
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
