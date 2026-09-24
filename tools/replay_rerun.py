# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "pandas>=2.1", "rerun-sdk>=0.23"]
# ///
"""Replay a kestrel_sim CSV log in the Rerun viewer.

    uv run tools/replay_rerun.py build/dev/pitch_kick.csv          # opens the viewer
    uv run tools/replay_rerun.py build/dev/pitch_kick.csv --save run.rrd

The world frame is NED and the vehicle frame is FRD, so both use Rerun's FRD
view coordinates. Rerun stores quaternions scalar-last (x, y, z, w); kestrel
stores them scalar-first (w, x, y, z). The conversion happens in one place: log_pose().
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import rerun as rr

ARM_LENGTH_M = 0.225  # matches sim::default_quad_params()
AXIS_LENGTH_M = 0.35

# PX4 Quad X: (x forward, y right) hub positions and spin direction.
ROTORS = [
    ("motor1", (1, 1), "CCW"),
    ("motor2", (-1, -1), "CCW"),
    ("motor3", (1, -1), "CW"),
    ("motor4", (-1, 1), "CW"),
]


def log_airframe() -> None:
    """Static geometry drawn in the vehicle's own FRD frame."""
    a = ARM_LENGTH_M / np.sqrt(2.0)
    hubs = [(sx * a, sy * a, 0.0) for _, (sx, sy), _ in ROTORS]
    rr.log("world/vehicle", rr.ViewCoordinates.FRD, static=True)
    rr.log(
        "world/vehicle/arms",
        rr.LineStrips3D([[(0.0, 0.0, 0.0), hub] for hub in hubs], radii=0.008, colors=(90, 90, 90)),
        static=True,
    )
    rr.log(
        "world/vehicle/rotors",
        rr.Points3D(
            hubs,
            radii=0.06,
            colors=[(60, 60, 60) if spin == "CCW" else (140, 140, 140) for _, _, spin in ROTORS],
            labels=[f"{name} {spin}" for name, _, spin in ROTORS],
        ),
        static=True,
    )
    rr.log(
        "world/vehicle/axes",
        rr.Arrows3D(
            vectors=[(AXIS_LENGTH_M, 0, 0), (0, AXIS_LENGTH_M, 0), (0, 0, AXIS_LENGTH_M)],
            colors=[(42, 120, 214), (235, 104, 52), (27, 175, 122)],
            labels=["x nose", "y right", "z belly"],
        ),
        static=True,
    )


def log_pose(row) -> None:  # one row from DataFrame.itertuples()
    rr.log(
        "world/vehicle",
        rr.Transform3D(
            translation=(row.pos_n_m, row.pos_e_m, row.pos_d_m),
            quaternion=rr.Quaternion(xyzw=(row.q_x, row.q_y, row.q_z, row.q_w)),
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", type=Path, help="CSV written by kestrel_sim")
    parser.add_argument("--save", type=Path, help="write a .rrd file instead of opening the viewer")
    args = parser.parse_args()

    df = pd.read_csv(args.log)
    rr.init("kestrel_replay", spawn=args.save is None)
    if args.save is not None:
        rr.save(args.save)

    rr.log("world", rr.ViewCoordinates.FRD, static=True)  # NED: x north, y east, z down
    rr.log(
        "world/path",
        rr.LineStrips3D([df[["pos_n_m", "pos_e_m", "pos_d_m"]].to_numpy()], radii=0.01, colors=(42, 120, 214)),
        static=True,
    )
    log_airframe()
    for row in df.itertuples(index=False):
        rr.set_time("sim_time", duration=float(row.t_s))
        log_pose(row)
        for name in ("omega_x_radps", "omega_y_radps", "omega_z_radps"):
            rr.log(f"rates/{name}", rr.Scalars(getattr(row, name)))
    if args.save is not None:
        print(f"wrote {args.save}")


if __name__ == "__main__":
    main()
