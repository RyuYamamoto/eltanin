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

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import numpy as np

INSCRIBED = 253
LETHAL = 254
NO_INFORMATION = 255


def read_meta(file):
    values = {}
    for line in file.read_text().splitlines():
        key, value = line.split()
        values[key] = float(value)
    return values


def read_csv(file):
    with file.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}


def read_pgm(file):
    data = file.read_bytes()
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


def cost_image(costmap):
    rgb = np.ones(costmap.shape + (3,))
    scaled = np.clip(costmap.astype(float) / INSCRIBED, 0.0, 1.0)
    for channel in range(3):
        rgb[:, :, channel] = 1.0 - 0.8 * scaled
    rgb[costmap == LETHAL] = (0.0, 0.0, 0.0)
    rgb[costmap == NO_INFORMATION] = (0.75, 0.78, 0.85)
    return rgb


def map_extent(meta):
    return (
        meta["origin_x"],
        meta["origin_x"] + meta["size_x"] * meta["resolution"],
        meta["origin_y"],
        meta["origin_y"] + meta["size_y"] * meta["resolution"],
    )


def transform_footprint(footprint, x, y, yaw):
    cosine, sine = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[cosine, sine], [-sine, cosine]])
    return footprint @ rotation + [x, y]


def draw_map(axis, costmap, extent):
    axis.imshow(cost_image(costmap), origin="lower", extent=extent, interpolation="nearest")
    axis.set_aspect("equal")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")


def plot_result(out, meta, path, footprint, costmap, footprint_step):
    extent = map_extent(meta)
    figure, axes = plt.subplots(1, 2, figsize=(12, 6))
    draw_map(axes[0], costmap, extent)
    axes[0].plot(path["x"], path["y"], color="tab:blue", linewidth=1.5, label="Hybrid A*")

    indices = list(range(0, len(path["x"]), footprint_step))
    if indices[-1] != len(path["x"]) - 1:
        indices.append(len(path["x"]) - 1)
    bodies = []
    for index in indices:
        body = transform_footprint(
            footprint, path["x"][index], path["y"][index], path["yaw"][index]
        )
        bodies.append(plt.Polygon(body, closed=True))
    axes[0].add_collection(
        PatchCollection(bodies, facecolor="tab:orange", edgecolor="none", alpha=0.10)
    )
    axes[0].add_collection(
        PatchCollection(
            bodies, facecolor="none", edgecolor="tab:orange", linewidth=0.35, alpha=0.60
        )
    )

    axes[0].scatter([meta["start_x"]], [meta["start_y"]], color="tab:green", label="start")
    axes[0].scatter([meta["goal_x"]], [meta["goal_y"]], color="tab:red", label="goal")
    axes[0].legend()
    axes[0].set_title(f"real map, footprint at every {footprint_step} path sample(s)")

    limit = 1.0 / meta["turning_radius"]
    axes[1].plot(path["s"], path["curvature"], color="tab:purple")
    axes[1].axhline(limit, color="black", linestyle="--", label="curvature limit")
    axes[1].axhline(-limit, color="black", linestyle="--")
    axes[1].set_xlabel("arc length [m]")
    axes[1].set_ylabel("curvature [1/m]")
    axes[1].set_title("path curvature")
    axes[1].grid(True)
    axes[1].legend()

    figure.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=150)
    plt.close(figure)


def animate(out, meta, path, footprint, costmap, frame_step, fps):
    extent = map_extent(meta)
    frames = list(range(0, len(path["x"]), max(1, frame_step)))
    if frames[-1] != len(path["x"]) - 1:
        frames.append(len(path["x"]) - 1)

    figure, (whole, local) = plt.subplots(
        1, 2, figsize=(11, 7), gridspec_kw={"width_ratios": [1.0, 1.5]}
    )
    for axis in (whole, local):
        draw_map(axis, costmap, extent)
        axis.plot(path["x"], path["y"], color="tab:blue", linewidth=1.2)
    whole.set_title("Hybrid A* path")
    local.set_title("footprint")

    trail = whole.plot([], [], color="crimson", linewidth=1.2)[0]
    local_trail = local.plot([], [], color="crimson", linewidth=1.2)[0]
    whole_body = plt.Polygon(footprint, closed=True, facecolor="tab:orange", alpha=0.7)
    local_body = plt.Polygon(footprint, closed=True, facecolor="tab:orange", alpha=0.7)
    whole.add_patch(whole_body)
    local.add_patch(local_body)
    caption = figure.suptitle("")

    footprint_span = np.ptp(footprint, axis=0)
    window = max(2.0, 5.0 * float(np.max(footprint_span)))

    def draw(index):
        x, y, yaw = path["x"][index], path["y"][index], path["yaw"][index]
        body = transform_footprint(footprint, x, y, yaw)
        whole_body.set_xy(body)
        local_body.set_xy(body)
        trail.set_data(path["x"][: index + 1], path["y"][: index + 1])
        local_trail.set_data(path["x"][: index + 1], path["y"][: index + 1])
        local.set_xlim(x - 0.5 * window, x + 0.5 * window)
        local.set_ylim(y - 0.5 * window, y + 0.5 * window)
        caption.set_text(
            f"s {path['s'][index]:.2f} m   yaw {yaw:+.2f} rad   "
            f"curvature {path['curvature'][index]:+.2f} 1/m"
        )
        return whole_body, local_body, trail, local_trail, caption

    movie = matplotlib.animation.FuncAnimation(
        figure, draw, frames=frames, interval=1000.0 / fps, blit=False
    )
    movie.save(out, writer=matplotlib.animation.PillowWriter(fps=fps), dpi=80)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--out", type=Path, default=Path("hybrid_astar.png"))
    parser.add_argument("--animate", action="store_true")
    parser.add_argument("--footprint-step", type=int, default=1)
    parser.add_argument("--frame-step", type=int, default=2)
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args()
    if args.footprint_step < 1 or args.frame_step < 1 or args.fps < 1:
        parser.error("--footprint-step, --frame-step and --fps must be positive")

    meta = read_meta(args.run / "meta.txt")
    path = read_csv(args.run / "path.csv")
    footprint_data = read_csv(args.run / "footprint.csv")
    footprint = np.column_stack((footprint_data["x"], footprint_data["y"]))
    costmap = read_pgm(args.run / "costmap.pgm")

    plot_result(args.out, meta, path, footprint, costmap, args.footprint_step)
    if args.animate:
        animate(
            args.out.with_suffix(".gif"),
            meta,
            path,
            footprint,
            costmap,
            args.frame_step,
            args.fps,
        )


if __name__ == "__main__":
    main()
