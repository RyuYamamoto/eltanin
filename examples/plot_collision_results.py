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

"""Plot what eltanin_limiter_profile and eltanin_limit_on_real_map wrote.

This is a developer tool, not part of the build: nothing in CMake refers to it and
no library target depends on it. Run the C++ examples first, then point this at
their output directories.

    ./build/examples/eltanin_limiter_profile /tmp/viz
    ./build/examples/eltanin_limit_on_real_map map.yaml /tmp/viz-map
    python3 examples/plot_collision_results.py --synthetic /tmp/viz --real /tmp/viz-map

Requires matplotlib and numpy. What each figure shows is described in
docs/collision-design.md section 11.
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

import numpy as np

matplotlib.use('Agg')

ELTANIN = '#2a78d6'
NAVYU = '#eb6834'
PATH_GREEN = '#1baf7a'
BAND = '#eda100'
INK = '#0b0b0b'
MUTED = '#52514e'
SURFACE = '#fcfcfb'

# Half the length of the default footprint along x, used for the clearance annotation [m].
FOOTPRINT_HALF_LENGTH = 0.22

STYLE = {
    'figure.facecolor': SURFACE,
    'axes.facecolor': SURFACE,
    'axes.edgecolor': '#c9c8c3',
    'axes.labelcolor': MUTED,
    'text.color': INK,
    'xtick.color': MUTED,
    'ytick.color': MUTED,
    'axes.grid': True,
    'grid.color': '#e6e5e1',
    'grid.linewidth': 0.8,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'font.size': 10,
    'axes.titlesize': 11,
    'figure.dpi': 130,
}


def read_csv(path):
    """Read a CSV of floats, skipping the '#' comment lines the examples emit."""
    with open(path) as handle:
        rows = [line for line in handle if not line.startswith('#')]
    reader = csv.DictReader(rows)
    columns = {name: [] for name in reader.fieldnames or []}
    for row in reader:
        for key, value in row.items():
            columns[key].append(float(value))
    return {key: np.array(value) for key, value in columns.items()}


def read_meta(path):
    """Read the 'key value' lines of meta.txt into a dict."""
    meta = {}
    for line in Path(path).read_text().splitlines():
        key, _, value = line.partition(' ')
        try:
            meta[key] = float(value)
        except ValueError:
            meta[key] = value
    return meta


def read_pgm(path):
    """Read the binary P5 file write_pgm() produces, flipped back to origin-at-bottom."""
    data = Path(path).read_bytes()
    fields, offset = [], 0
    while len(fields) < 4:
        end = data.index(b'\n', offset)
        fields.extend(data[offset:end].split())
        offset = end + 1
    width, height = int(fields[1]), int(fields[2])
    cells = np.frombuffer(data[offset:offset + width * height], dtype=np.uint8)
    return cells.reshape(height, width)[::-1]


def plot_velocity_profile(src, dst):
    """Draw the limited speed against the gap to the wall, forward and reverse."""
    data = read_csv(src / 'velocity_profile.csv')
    # The outermost cell predicts off the map, which truncates rather than limits.
    keep = (data['gap'] >= 0.1) & (data['gap'] <= 1.35)
    gap = data['gap'][keep]
    forward = data['v_out_forward'][keep]
    reverse = data['v_out_reverse'][keep]

    fig, ax = plt.subplots(figsize=(8.0, 4.6))
    ax.axhline(0.0, color='#c9c8c3', linewidth=1.0, zorder=1)
    ax.plot(gap, data['navyu_forward'][keep], color=NAVYU, linewidth=5, alpha=0.35, zorder=2)
    ax.plot(gap, data['navyu_reverse'][keep], color=NAVYU, linewidth=2.6,
            linestyle=(0, (6, 2)), zorder=2)
    ax.plot(gap, forward, color=ELTANIN, linewidth=2, zorder=4)
    ax.plot(gap, reverse, color=ELTANIN, linewidth=2, linestyle=(0, (6, 2)), zorder=4)

    ax.annotate('forward   eltanin = navyu\n(both limit correctly)', (1.30, 0.53), va='bottom',
                color=ELTANIN, fontsize=9, fontweight='bold')
    ax.annotate('reverse   eltanin\n(mirrors forward)', (0.62, -0.20), color=ELTANIN,
                fontsize=9, fontweight='bold')
    ax.annotate('reverse   navyu  -  never limited', (1.30, -0.46), va='bottom', color=NAVYU,
                fontsize=9, fontweight='bold')

    knee = gap[forward < 0.4999]
    if knee.size:
        ax.axvspan(knee.min() - 0.025, knee.max() + 0.025, color=BAND, alpha=0.12, zorder=0)
        ax.annotate('braking-distance law binds', (knee.max() + 0.02, 0.66), ha='left',
                    va='top', color=MUTED, fontsize=9)

    for value in (0.447214, 0.316228):
        ax.annotate(f'{value:.3f}', (gap[np.argmin(np.abs(forward - value))], value),
                    textcoords='offset points', xytext=(-2, 7), ha='center', fontsize=8,
                    color=MUTED)
        ax.annotate(f'-{value:.3f}', (gap[np.argmin(np.abs(reverse + value))], -value),
                    textcoords='offset points', xytext=(26, 1), ha='center', fontsize=8,
                    color=MUTED)

    ax.set_xlabel('gap between the robot origin and the wall cell centre  [m]')
    ax.set_ylabel('commanded linear velocity  [m/s]')
    ax.set_title(r"Requested $\pm$0.5 m/s towards a wall: navyu's std::min() leaves reverse "
                 'untouched')
    ax.set_xlim(1.38, 0.07)
    ax.set_ylim(-0.62, 0.72)
    fig.tight_layout()
    fig.savefig(dst / 'velocity_profile.png')
    plt.close(fig)


def plot_heading_sweep(src, dst):
    """Draw the collision verdict over a full turn, for two obstacle distances."""
    fig, axes = plt.subplots(1, 2, figsize=(8.4, 4.3), subplot_kw={'projection': 'polar'})
    panels = [
        ('heading_sweep_diagonal.csv',
         'Circumscribed band\nobstacle 0.354 m at 45°, cost 214', ELTANIN, np.pi / 4),
        ('heading_sweep_head_on.csv',
         'Inscribed band\nobstacle 0.25 m ahead, cost 253', NAVYU, 0.0),
    ]
    for ax, (name, title, color, bearing) in zip(axes, panels):
        data = read_csv(src / name)
        yaw, hit = data['yaw'], data['collision']
        ax.set_theta_zero_location('E')
        ax.bar(yaw, hit, width=2 * np.pi / len(yaw) * 1.02, bottom=0.0, color=color,
               linewidth=0, alpha=0.9)
        ax.set_ylim(0, 1.15)
        ax.set_yticks([])
        ax.set_xticks(np.linspace(0, 2 * np.pi, 8, endpoint=False))
        ax.set_xticklabels([f'{d}°' for d in range(0, 360, 45)], fontsize=8, color=MUTED)
        ax.grid(color='#e6e5e1')
        ax.set_title(title, fontsize=10, pad=14)
        ax.annotate(f'{int(hit.sum())} / {len(hit)} headings collide', xy=(0.5, -0.16),
                    xycoords='axes fraction', ha='center', color=MUTED, fontsize=9)
        ax.plot([bearing], [1.08], marker='o', markersize=9, color=INK, zorder=5)
        ax.annotate('obstacle', xy=(bearing, 1.12), fontsize=8, color=INK, ha='left', va='bottom')

    fig.suptitle('Collision verdict over a full turn of the 0.6 m square footprint', y=0.99)
    fig.tight_layout()
    fig.savefig(dst / 'heading_sweep.png')
    plt.close(fig)


def plot_closed_loop(src, dst):
    """Draw the quantized staircase of the stopping run."""
    data = read_csv(src / 'closed_loop_forward.csv')
    keep = data['gap'] <= 1.4
    gap, v_out = data['gap'][keep], data['v_out'][keep]

    fig, ax = plt.subplots(figsize=(7.6, 3.9))
    ax.step(gap, v_out, where='post', color=ELTANIN, linewidth=2)
    ax.scatter(gap, v_out, s=30, color=ELTANIN, zorder=3, edgecolor=SURFACE, linewidth=1.2)

    labelled = set()
    for x, y in zip(gap, v_out):
        key = round(y, 4)
        if key in labelled or y > 0.4999:
            continue
        labelled.add(key)
        ax.annotate(f'{y:.3f}', (x, y), textcoords='offset points', xytext=(0, 10), ha='center',
                    fontsize=9, color=MUTED)

    stop_gap = gap[v_out == 0.0]
    if stop_gap.size:
        ax.axvline(stop_gap[0], color=NAVYU, linewidth=1.2, linestyle=(0, (4, 3)))
        ax.annotate(f'stops with a {stop_gap[0]:.3f} m gap', (stop_gap[0], 0.30),
                    textcoords='offset points', xytext=(10, 0), color=NAVYU, fontsize=9,
                    fontweight='bold')

    ax.set_xlabel('gap between the robot origin and the wall cell centre  [m]')
    ax.set_ylabel('limited velocity  [m/s]')
    ax.set_title('Closed loop through SimpleSimulator (0.1 s cycle): a quantized staircase')
    ax.set_xlim(1.42, 0.30)
    ax.set_ylim(-0.03, 0.60)
    fig.tight_layout()
    fig.savefig(dst / 'closed_loop.png')
    plt.close(fig)


def draw_map_panel(ax, src, meta):
    """Draw the map, path, trajectory and predicted poses; return the trajectory columns."""
    grid = read_pgm(src / 'crop.pgm')
    resolution, origin_x, origin_y = meta['resolution'], meta['origin_x'], meta['origin_y']
    ax.imshow(grid, origin='lower', cmap='Greys', vmin=0, vmax=254, interpolation='nearest',
              extent=[origin_x, origin_x + grid.shape[1] * resolution,
                      origin_y, origin_y + grid.shape[0] * resolution])

    path = read_csv(src / 'path.csv')
    traj = read_csv(src / 'trajectory.csv')
    pred = read_csv(src / 'predicted.csv')
    foot = read_csv(src / 'footprint.csv')

    ax.plot(path['x'], path['y'], color=PATH_GREEN, linewidth=3.0, alpha=0.75,
            label='planned path (goes straight through)')
    ax.plot(traj['x'], traj['y'], color=ELTANIN, linewidth=1.8, label='driven trajectory')

    collided = pred['has_collision']
    hits = [c for c in np.unique(pred['cycle']) if collided[pred['cycle'] == c][0] > 0.5]
    first_hit = hits[0] if hits else None
    for cycle in np.unique(pred['cycle']):
        selected = pred['cycle'] == cycle
        colliding = collided[selected][0] > 0.5
        ax.plot(pred['x'][selected], pred['y'][selected], color=NAVYU if colliding else '#9a9993',
                linewidth=2.2 if colliding else 0.8, alpha=1.0 if colliding else 0.5,
                marker='.' if colliding else None, markersize=5, zorder=7 if colliding else 4,
                label='predicted poses (collision)' if colliding and cycle == first_hit else None)

    cycles = np.unique(foot['cycle'])
    selected = foot['cycle'] == cycles[-1]
    ax.add_patch(MplPolygon(np.column_stack([foot['x'][selected], foot['y'][selected]]),
                            closed=True, facecolor=ELTANIN, alpha=0.25, edgecolor=ELTANIN,
                            linewidth=1.6, zorder=6))
    for cycle in cycles[::4]:
        selected = foot['cycle'] == cycle
        ax.add_patch(MplPolygon(np.column_stack([foot['x'][selected], foot['y'][selected]]),
                                closed=True, fill=False, edgecolor=MUTED, linewidth=0.6,
                                alpha=0.55, zorder=3))

    if 'obstacle_x' in meta:
        half = meta['obstacle_half_width']
        centre_x, centre_y = meta['obstacle_x'], meta['obstacle_y']
        ax.add_patch(MplPolygon(
            [(centre_x - half, centre_y - half), (centre_x + half, centre_y - half),
             (centre_x + half, centre_y + half), (centre_x - half, centre_y + half)],
            closed=True, facecolor=NAVYU, edgecolor=NAVYU, alpha=0.9, zorder=5))
        ax.annotate('obstacle added after planning', (centre_x, centre_y + 0.45), color=NAVYU,
                    ha='center', fontsize=9, fontweight='bold')
        pad = 2.3
        ax.set_xlim(centre_x - pad, centre_x + pad)
        ax.set_ylim(centre_y - pad, centre_y + pad)
        clearance = abs(traj['y'][-1] - centre_y) - half - FOOTPRINT_HALF_LENGTH
        stop_label = f'stop pose\nfootprint clears the obstacle by {clearance:.2f} m'
    else:
        stop_label = 'stop pose'

    ax.plot(traj['x'][-1], traj['y'][-1], marker='o', markersize=8, color=ELTANIN,
            markeredgecolor=SURFACE, markeredgewidth=1.5, zorder=8)
    ax.annotate(stop_label, (traj['x'][-1], traj['y'][-1]), textcoords='offset points',
                xytext=(14, -26), color=ELTANIN, fontsize=9, fontweight='bold')
    ax.set_aspect('equal')
    ax.set_xlabel('x  [m]')
    ax.set_ylabel('y  [m]')
    ax.set_title('Stopping in front of an obstacle the planner never saw')
    ax.legend(loc='lower left', fontsize=8.5, frameon=True, facecolor=SURFACE, framealpha=0.93,
              edgecolor='#e6e5e1')
    ax.grid(False)
    return traj


def plot_real_map(src, dst):
    """Draw the real-map run beside its velocity trace."""
    meta = read_meta(src / 'meta.txt')
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 5.0), gridspec_kw={'width_ratios': [1.25, 1]})
    traj = draw_map_panel(axes[0], src, meta)

    ax = axes[1]
    ax.plot(traj['t'], traj['v_in'], color=NAVYU, linewidth=1.8)
    ax.plot(traj['t'], traj['v_out'], color=ELTANIN, linewidth=2.0)
    ax.set_xlim(traj['t'][-1] - 2.2, traj['t'][-1] + 0.15)
    ax.set_ylim(-0.03, 0.62)
    ax.annotate('requested by PurePursuit', (traj['t'][-1] - 2.0, 0.52), color=NAVYU, fontsize=9,
                fontweight='bold')
    ax.annotate('after VelocityLimiter', (traj['t'][-1] - 2.0, 0.20), color=ELTANIN, fontsize=9,
                fontweight='bold')

    labelled = set()
    for time, value in zip(traj['t'], traj['v_out']):
        key = round(value, 4)
        if key in labelled or value > 0.4999:
            continue
        labelled.add(key)
        ax.annotate(f'{value:.3f}', (time, value), textcoords='offset points', xytext=(4, 6),
                    fontsize=8, color=MUTED)

    ax.set_xlabel('time  [s]')
    ax.set_ylabel('linear velocity  [m/s]')
    ax.set_title('The limiter cuts the command in quantized steps')
    fig.tight_layout()
    fig.savefig(dst / 'real_map.png')
    plt.close(fig)


def require(directory, names, example):
    """Exit with a usable message when the example has not been run yet."""
    missing = [name for name in names if not (directory / name).is_file()]
    if missing:
        sys.exit(f"{directory} is missing {', '.join(missing)}; run {example} first")


def main():
    """Parse the arguments and write one PNG per available input."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--synthetic', type=Path,
                        help='output directory of eltanin_limiter_profile')
    parser.add_argument('--real', type=Path,
                        help='output directory of eltanin_limit_on_real_map')
    parser.add_argument('--out', type=Path, default=Path('plots'),
                        help='directory the PNG files are written to (default: ./plots)')
    args = parser.parse_args()

    if args.synthetic is None and args.real is None:
        parser.error('pass --synthetic and/or --real')

    plt.rcParams.update(STYLE)
    args.out.mkdir(parents=True, exist_ok=True)
    written = []

    if args.synthetic is not None:
        require(args.synthetic,
                ('velocity_profile.csv', 'heading_sweep_diagonal.csv',
                 'heading_sweep_head_on.csv', 'closed_loop_forward.csv'),
                'eltanin_limiter_profile')
        plot_velocity_profile(args.synthetic, args.out)
        plot_heading_sweep(args.synthetic, args.out)
        plot_closed_loop(args.synthetic, args.out)
        written += ['velocity_profile.png', 'heading_sweep.png', 'closed_loop.png']

    if args.real is not None:
        require(args.real,
                ('meta.txt', 'crop.pgm', 'path.csv', 'trajectory.csv', 'predicted.csv',
                 'footprint.csv'),
                'eltanin_limit_on_real_map')
        plot_real_map(args.real, args.out)
        written.append('real_map.png')

    print(f"wrote {', '.join(written)} into {args.out}")


if __name__ == '__main__':
    main()
