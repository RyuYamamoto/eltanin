#!/usr/bin/env python3
# Copyright 2026 RyuYamamoto.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License

"""Plot what eltanin_navigate_on_real_map wrote.

This is a developer tool, not part of the build: nothing in CMake refers to it and no
library target depends on it. Run the example first, then point this at its output
directory.

    ./build/examples/eltanin_navigate_on_real_map /tmp/nav
    python3 examples/plot_navigation_results.py --run /tmp/nav --out /tmp/nav-plots

Requires matplotlib and numpy. What each figure shows is described in
docs/integration-design.md section 15.
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt

import numpy as np

# Cost values written by write_pgm; see include/eltanin/map/cost_values.hpp.
FREE_SPACE = 0
INSCRIBED = 253
LETHAL = 254
NO_INFORMATION = 255

# Kept distinct on purpose: the legs, the driven path and the observations overlap everywhere.
LEG_COLOURS = ("tab:blue", "tab:green", "tab:purple", "tab:brown")
DRIVEN_COLOUR = "crimson"
OBSERVED_COLOUR = "magenta"


def leg_colour(leg):
    return LEG_COLOURS[int(leg) % len(LEG_COLOURS)]


def read_meta(path):
    """meta.txt as a dict; the per-leg lines are collected under the key 'leg'."""
    meta = {"leg": []}
    with path.open() as handle:
        for line in handle:
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "leg":
                meta["leg"].append(fields[1:])
            else:
                meta[fields[0]] = fields[1] if len(fields) == 2 else fields[1:]
    return meta


def read_pgm(path):
    """Binary P5 as a numpy array, flipped so that row 0 is the bottom of the map."""
    data = path.read_bytes()
    fields = []
    offset = 0
    while len(fields) < 4:
        end = data.index(b"\n", offset)
        line = data[offset:end]
        offset = end + 1
        if line.startswith(b"#"):
            continue
        fields.extend(line.split())
    width, height = int(fields[1]), int(fields[2])
    pixels = np.frombuffer(data[offset : offset + width * height], dtype=np.uint8)
    return np.flipud(pixels.reshape(height, width))


def read_csv_columns(path):
    """CSV with a header row as a dict of float lists; non-numeric fields become nan."""
    with path.open() as handle:
        reader = csv.DictReader(handle)
        columns = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name, value in row.items():
                try:
                    columns[name].append(float(value))
                except (TypeError, ValueError):
                    columns[name].append(float("nan"))
    return {name: np.array(values) for name, values in columns.items()}


def cost_image(costmap):
    """Grey costmap: free is white, inflated darkens, lethal is black, unknown is blue-grey."""
    rgb = np.ones(costmap.shape + (3,))
    scaled = np.clip(costmap.astype(float) / INSCRIBED, 0.0, 1.0)
    for channel in range(3):
        rgb[:, :, channel] = 1.0 - 0.8 * scaled
    rgb[costmap == LETHAL] = (0.0, 0.0, 0.0)
    rgb[costmap == NO_INFORMATION] = (0.75, 0.78, 0.85)
    return rgb


def extent_of(meta, costmap):
    resolution = float(meta["resolution"])
    origin_x, origin_y = float(meta["origin_x"]), float(meta["origin_y"])
    return [
        origin_x,
        origin_x + costmap.shape[1] * resolution,
        origin_y,
        origin_y + costmap.shape[0] * resolution,
    ]


def plot_overview(run, out, meta):
    """Costmap with the planned legs, the driven trajectory and the observed cells on top."""
    costmap = read_pgm(run / "costmap.pgm")
    path = read_csv_columns(run / "path.csv")
    trajectory = read_csv_columns(run / "trajectory.csv")
    obstacles = read_csv_columns(run / "obstacles.csv")
    extent = extent_of(meta, costmap)

    figure, axes = plt.subplots(figsize=(7.0, 9.5))
    axes.imshow(cost_image(costmap), origin="lower", extent=extent, interpolation="nearest")
    for leg in np.unique(path["leg"]):
        selected = path["leg"] == leg
        axes.plot(
            path["x"][selected],
            path["y"][selected],
            color=leg_colour(leg),
            linewidth=1.6,
            label=f"planned leg {int(leg)}",
        )
    axes.plot(
        trajectory["x"],
        trajectory["y"],
        color=DRIVEN_COLOUR,
        linewidth=1.0,
        label="driven trajectory",
    )
    if obstacles["x"].size:
        axes.scatter(
            obstacles["x"],
            obstacles["y"],
            s=8.0,
            marker="s",
            color=OBSERVED_COLOUR,
            label="observed cells",
            zorder=5,
        )
    axes.scatter(
        [float(meta["start_x"])], [float(meta["start_y"])], marker="o", color="black", label="start"
    )
    axes.scatter(
        [float(meta["goal_x"])],
        [float(meta["goal_y"])],
        marker="*",
        s=120.0,
        color="black",
        label="goal",
    )
    axes.set_aspect("equal")
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_title(
        f"outcome {meta['outcome']}, final error {float(meta['final_position_error']):.4f} m"
    )
    axes.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    figure.tight_layout()
    figure.savefig(out / "overview.png", dpi=140)
    plt.close(figure)


def plot_stop(run, out, meta):
    """Zoom on the injected obstacle: where the limiter stopped and where the detour goes."""
    if "obstacle_x" not in meta:
        return
    costmap = read_pgm(run / "costmap.pgm")
    path = read_csv_columns(run / "path.csv")
    trajectory = read_csv_columns(run / "trajectory.csv")
    obstacles = read_csv_columns(run / "obstacles.csv")
    extent = extent_of(meta, costmap)
    centre = (float(meta["obstacle_x"]), float(meta["obstacle_y"]))
    half_width = float(meta["obstacle_half_width"])
    span = 3.0

    figure, axes = plt.subplots(figsize=(7.0, 7.0))
    axes.imshow(cost_image(costmap), origin="lower", extent=extent, interpolation="nearest")
    for leg in np.unique(path["leg"]):
        selected = path["leg"] == leg
        axes.plot(
            path["x"][selected],
            path["y"][selected],
            color=leg_colour(leg),
            linewidth=1.6,
            label=f"planned leg {int(leg)}",
        )
    axes.plot(trajectory["x"], trajectory["y"], color=DRIVEN_COLOUR, linewidth=1.2, label="driven")
    stopped = (trajectory["v_out"] == 0.0) & (trajectory["w_out"] == 0.0)
    if stopped.any():
        axes.scatter(
            trajectory["x"][stopped],
            trajectory["y"][stopped],
            s=60.0,
            facecolor="none",
            edgecolor="black",
            label="stopped cycles",
            zorder=6,
        )
    if obstacles["x"].size:
        axes.scatter(
            obstacles["x"],
            obstacles["y"],
            s=20.0,
            marker="s",
            color=OBSERVED_COLOUR,
            label="observed",
            zorder=5,
        )
    axes.add_patch(
        plt.Rectangle(
            (centre[0] - half_width, centre[1] - half_width),
            2.0 * half_width,
            2.0 * half_width,
            fill=False,
            edgecolor="black",
            linestyle="--",
            linewidth=1.5,
            label="unknown obstacle (truth)",
        )
    )
    axes.set_xlim(centre[0] - span, centre[0] + span)
    axes.set_ylim(centre[1] - span, centre[1] + span)
    axes.set_aspect("equal")
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_title(
        "stop clearance {:.3f} m to any obstacle, {:.3f} m to the injected one".format(
            float(meta["stop_clearance"]), float(meta["stop_obstacle_clearance"])
        )
    )
    axes.legend(loc="upper left", fontsize=8)
    figure.tight_layout()
    figure.savefig(out / "stop.png", dpi=140)
    plt.close(figure)


def plot_commands(run, out, meta):
    """Requested against limited command over time, with the collision distance below."""
    trajectory = read_csv_columns(run / "trajectory.csv")
    figure, (top, middle, bottom) = plt.subplots(3, 1, figsize=(9.0, 7.5), sharex=True)

    # The limited command hides the requested one wherever the limiter did nothing, so it goes under.
    top.plot(
        trajectory["t"], trajectory["v_in"], color="silver", linewidth=3.5, label="v requested"
    )
    top.plot(trajectory["t"], trajectory["v_out"], color="black", linewidth=1.0, label="v limited")
    top.set_ylabel("linear [m/s]")
    top.legend(loc="lower right", fontsize=8)
    top.grid(alpha=0.3)

    middle.plot(
        trajectory["t"], trajectory["w_in"], color="silver", linewidth=3.5, label="w requested"
    )
    middle.plot(
        trajectory["t"], trajectory["w_out"], color="black", linewidth=1.0, label="w limited"
    )
    middle.set_ylabel("angular [rad/s]")
    middle.legend(loc="lower right", fontsize=8)
    middle.grid(alpha=0.3)

    finite = np.isfinite(trajectory["collision_distance"])
    bottom.plot(
        trajectory["t"][finite],
        trajectory["collision_distance"][finite],
        ".",
        markersize=2.0,
        label="collision distance",
    )
    bottom.axhline(
        float(meta["collision_margin"]), color="crimson", linewidth=0.8, label="collision margin"
    )
    for leg_start in np.unique(trajectory["leg"])[1:]:
        first = trajectory["t"][trajectory["leg"] == leg_start][0]
        for axis in (top, middle, bottom):
            axis.axvline(first, color="green", linewidth=0.8, linestyle="--")
    bottom.set_ylabel("distance [m]")
    bottom.set_xlabel("t [s]")
    bottom.legend(loc="upper right", fontsize=8)
    bottom.grid(alpha=0.3)

    figure.suptitle("dashed green: replan, so a new leg starts")
    figure.tight_layout()
    figure.savefig(out / "commands.png", dpi=140)
    plt.close(figure)


def plot_traversed(run, out, meta):
    """The traversed mask over the costmap, which only lines up if both share a geometry."""
    costmap = read_pgm(run / "costmap.pgm")
    traversed = read_pgm(run / "traversed.pgm")
    if traversed.shape != costmap.shape:
        print("costmap.pgm and traversed.pgm disagree on their size", file=sys.stderr)
        return
    extent = extent_of(meta, costmap)

    figure, axes = plt.subplots(figsize=(7.0, 9.5))
    axes.imshow(cost_image(costmap), origin="lower", extent=extent, interpolation="nearest")
    mask = np.zeros(traversed.shape + (4,))
    mask[traversed == LETHAL] = (0.85, 0.0, 0.2, 1.0)
    axes.imshow(mask, origin="lower", extent=extent, interpolation="nearest")
    axes.set_aspect("equal")
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_title(f"traversed cells, {meta['colliding_poses']} of them in collision")
    figure.tight_layout()
    figure.savefig(out / "traversed.png", dpi=140)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path, help="output directory of the example")
    parser.add_argument("--out", required=True, type=Path, help="directory for the figures")
    arguments = parser.parse_args()

    if not (arguments.run / "meta.txt").is_file():
        print(f"{arguments.run} holds no meta.txt; run the example first", file=sys.stderr)
        return 1
    arguments.out.mkdir(parents=True, exist_ok=True)
    meta = read_meta(arguments.run / "meta.txt")

    plot_overview(arguments.run, arguments.out, meta)
    plot_stop(arguments.run, arguments.out, meta)
    plot_commands(arguments.run, arguments.out, meta)
    plot_traversed(arguments.run, arguments.out, meta)
    print(f"wrote the figures into {arguments.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
