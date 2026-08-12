#!/usr/bin/env python3
"""
plot_encoder_history.py — Reads a JSON history file produced by
encoder_monitor_node.py and renders position/RPM as two static graphs.

Usage:
    python3 plot_encoder_history.py encoder_history.json
    python3 plot_encoder_history.py encoder_history.json --save out.png
"""

import argparse
import json
import sys

import matplotlib.pyplot as plt


def load_history(path):
    with open(path) as f:
        data = json.load(f)

    position = data.get("position", [])
    rpm = data.get("rpm", [])

    pos_t = [p["t"] for p in position]
    pos_v = [p["value"] for p in position]
    rpm_t = [r["t"] for r in rpm]
    rpm_v = [r["value"] for r in rpm]

    return pos_t, pos_v, rpm_t, rpm_v


def plot_history(pos_t, pos_v, rpm_t, rpm_v, title, save_path=None):
    fig, (ax_pos, ax_rpm) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    fig.suptitle(title)

    ax_pos.plot(pos_t, pos_v, color="tab:blue")
    ax_pos.set_ylabel("Position (ticks)")
    ax_pos.grid(True, alpha=0.3)

    ax_rpm.plot(rpm_t, rpm_v, color="tab:orange")
    ax_rpm.set_ylabel("RPM")
    ax_rpm.set_xlabel("Time (s)")
    ax_rpm.grid(True, alpha=0.3)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150)
        print(f"Saved plot to {save_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot encoder history JSON.")
    parser.add_argument("json_file", help="Path to the history JSON file")
    parser.add_argument(
        "--save", metavar="PNG_PATH", default=None,
        help="Save the figure to this path instead of showing it interactively",
    )
    args = parser.parse_args()

    try:
        pos_t, pos_v, rpm_t, rpm_v = load_history(args.json_file)
    except (OSError, json.JSONDecodeError) as e:
        print(f"Error reading '{args.json_file}': {e}", file=sys.stderr)
        sys.exit(1)

    if not pos_t and not rpm_t:
        print("Warning: no data found in history file.", file=sys.stderr)

    plot_history(pos_t, pos_v, rpm_t, rpm_v, title=args.json_file, save_path=args.save)


if __name__ == "__main__":
    main()
