#!/usr/bin/env python3
"""
Regenerates media/watchdog_explainer.gif.

A scripted (not recorded) diagram animation: three monitored topics with
status dots, one of which goes stale -> dead, triggering the watchdog to
call a failover action, then recovering. Deterministic and committed
alongside its source so the GIF is never a mystery binary.

Usage:
    pip install matplotlib pillow
    python3 tools/anim/generate_explainer_gif.py
"""
import os

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

OUTPUT_PATH = os.path.join(
    os.path.dirname(__file__), "..", "..", "media", "watchdog_explainer.gif"
)

TOPICS = ["/camera/image", "/lidar/scan", "/odom"]
FAILING_TOPIC_INDEX = 1  # /lidar/scan

GREEN = "#2ecc71"
YELLOW = "#f1c40f"
RED = "#e74c3c"
GRAY = "#bdc3c7"
DARK = "#2c3e50"

# (start_frame, end_frame) phase boundaries.
PHASE_ALL_OK = (0, 10)
PHASE_STALE = (10, 20)
PHASE_DEAD = (20, 30)
PHASE_ACTION = (30, 42)
PHASE_ENGAGED = (42, 52)
PHASE_RECOVERED = (52, 64)
TOTAL_FRAMES = PHASE_RECOVERED[1]


def topic_status_and_text(frame):
    if frame < PHASE_STALE[0]:
        return GREEN, "All topics healthy"
    if frame < PHASE_DEAD[0]:
        return YELLOW, "/lidar/scan: no message for 0.5s -> STALE"
    if frame < PHASE_ACTION[0]:
        return RED, "/lidar/scan: timeout exceeded -> DEAD"
    if frame < PHASE_ENGAGED[0]:
        return RED, "Watchdog calling /safety/switch_to_safe_mode ..."
    if frame < PHASE_RECOVERED[0]:
        return RED, "Degraded mode engaged"
    return GREEN, "/lidar/scan: message received -> RECOVERED"


def build_figure():
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 6)
    ax.axis("off")

    topic_boxes = []
    topic_dots = []
    for i, name in enumerate(TOPICS):
        y = 5 - i * 1.5
        box = mpatches.FancyBboxPatch(
            (0.5, y - 0.4), 3.2, 0.8, boxstyle="round,pad=0.05", linewidth=1.2,
            edgecolor=DARK, facecolor="white"
        )
        ax.add_patch(box)
        ax.text(0.8, y, name, va="center", fontsize=10, family="monospace")
        dot = mpatches.Circle((3.4, y), 0.12, facecolor=GREEN, edgecolor=DARK, linewidth=0.8)
        ax.add_patch(dot)
        topic_boxes.append(box)
        topic_dots.append(dot)

    watchdog_box = mpatches.FancyBboxPatch(
        (5.0, 2.6), 2.4, 0.8, boxstyle="round,pad=0.05", linewidth=1.4,
        edgecolor=DARK, facecolor="#ecf0f1"
    )
    ax.add_patch(watchdog_box)
    ax.text(6.2, 3.0, "ros2_watchdog", ha="center", va="center", fontsize=10, fontweight="bold")

    action_box = mpatches.FancyBboxPatch(
        (5.0, 0.6), 2.4, 0.8, boxstyle="round,pad=0.05", linewidth=1.2,
        edgecolor=DARK, facecolor="white"
    )
    ax.add_patch(action_box)
    ax.text(
        6.2, 1.0, "call_trigger_service\n/safety/switch_to_safe_mode",
        ha="center", va="center", fontsize=7.5
    )

    (arrow,) = ax.plot([], [], color=DARK, linewidth=2)
    status_text = ax.text(5.0, 4.7, "", fontsize=10, ha="left", va="center", color=DARK)

    return fig, ax, topic_dots, action_box, arrow, status_text


def make_animation():
    fig, ax, topic_dots, action_box, arrow, status_text = build_figure()

    def update(frame):
        color, message = topic_status_and_text(frame)
        topic_dots[FAILING_TOPIC_INDEX].set_facecolor(color)
        status_text.set_text(message)

        in_action_phase = PHASE_ACTION[0] <= frame < PHASE_ENGAGED[0]
        if in_action_phase:
            progress = (frame - PHASE_ACTION[0]) / (PHASE_ACTION[1] - PHASE_ACTION[0])
            y = 5 - FAILING_TOPIC_INDEX * 1.5
            x0, y0 = 3.6, y
            x1, y1 = 5.0, 3.0
            arrow.set_data([x0, x0 + (x1 - x0) * progress], [y0, y0 + (y1 - y0) * progress])
        elif frame >= PHASE_ENGAGED[0] and frame < PHASE_RECOVERED[0]:
            arrow.set_data([3.6, 5.0], [5 - FAILING_TOPIC_INDEX * 1.5, 3.0])
        else:
            arrow.set_data([], [])

        flashing = PHASE_ENGAGED[0] <= frame < PHASE_ENGAGED[1] and frame % 4 < 2
        action_box.set_facecolor("#fdebd0" if flashing else "white")

        return [*topic_dots, action_box, arrow, status_text]

    anim = FuncAnimation(fig, update, frames=TOTAL_FRAMES, interval=180, blit=False)
    return fig, anim


def main():
    fig, anim = make_animation()
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    anim.save(OUTPUT_PATH, writer=PillowWriter(fps=8))
    print(f"wrote {os.path.abspath(OUTPUT_PATH)}")


if __name__ == "__main__":
    main()
