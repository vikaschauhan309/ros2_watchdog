"""
Shared drawing primitives for the scripted (not recorded) explainer GIFs in
tools/anim/*. Each scenario script builds a figure from these primitives and
drives it with matplotlib's FuncAnimation. Keeping this in one place means
every scenario gets the same visual language (box style, colors, caption
placement) for free.
"""
import os

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

MEDIA_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "media")

GREEN = "#2ecc71"
YELLOW = "#f1c40f"
RED = "#e74c3c"
GRAY = "#bdc3c7"
DARK = "#2c3e50"


def _box(ax, x, y, w, h, label, facecolor="white", fontsize=10, fontweight="normal"):
    box = mpatches.FancyBboxPatch(
        (x, y - h / 2), w, h, boxstyle="round,pad=0.05", linewidth=1.2,
        edgecolor=DARK, facecolor=facecolor
    )
    ax.add_patch(box)
    ax.text(x + w / 2, y, label, ha="center", va="center", fontsize=fontsize,
             fontweight=fontweight, family="monospace" if fontsize <= 10 else None)
    return box


def build_topic_column(ax, topics, x=0.5, top_y=5.0, spacing=1.5, box_w=3.2, box_h=0.8):
    """Draws one box + status dot per topic name. Returns {name: (box, dot, y)}."""
    rows = {}
    for i, name in enumerate(topics):
        y = top_y - i * spacing
        box = mpatches.FancyBboxPatch(
            (x, y - box_h / 2), box_w, box_h, boxstyle="round,pad=0.05",
            linewidth=1.2, edgecolor=DARK, facecolor="white"
        )
        ax.add_patch(box)
        ax.text(x + 0.3, y, name, va="center", fontsize=10, family="monospace")
        dot = mpatches.Circle((x + box_w - 0.2, y), 0.12, facecolor=GREEN,
                               edgecolor=DARK, linewidth=0.8)
        ax.add_patch(dot)
        rows[name] = (box, dot, y)
    return rows


def build_watchdog_box(ax, x=5.0, y=3.0, w=2.4, h=0.8):
    return _box(ax, x, y, w, h, "ros2_watchdog", facecolor="#ecf0f1", fontweight="bold")


def build_action_box(ax, label, x=5.0, y=1.0, w=2.4, h=0.8, fontsize=7.5):
    box = mpatches.FancyBboxPatch(
        (x, y - h / 2), w, h, boxstyle="round,pad=0.05", linewidth=1.2,
        edgecolor=DARK, facecolor="white"
    )
    ax.add_patch(box)
    text = ax.text(x + w / 2, y, label, ha="center", va="center", fontsize=fontsize)
    return box, text


def base_figure(figsize=(8, 4.6), xlim=(0, 11.5), ylim=(0, 6.6)):
    fig, ax = plt.subplots(figsize=figsize)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.axis("off")
    return fig, ax


def add_status_text(ax, x=5.0, y=4.7):
    return ax.text(x, y, "", fontsize=9, ha="left", va="center", color=DARK)


def add_caption_text(ax, x=0.5, y=6.2):
    """Smaller, italic line at the top of frame explaining what the scenario shows."""
    return ax.text(x, y, "", fontsize=8.5, ha="left", va="center", color="#7f8c8d",
                    style="italic")


def straight_arrow(ax):
    (arrow,) = ax.plot([], [], color=DARK, linewidth=2)
    return arrow


def save_animation(fig, update_fn, total_frames, filename, interval=180, fps=8):
    os.makedirs(MEDIA_DIR, exist_ok=True)
    output_path = os.path.join(MEDIA_DIR, filename)
    anim = FuncAnimation(fig, update_fn, frames=total_frames, interval=interval, blit=False)
    anim.save(output_path, writer=PillowWriter(fps=fps))
    print(f"wrote {os.path.abspath(output_path)}")
    plt.close(fig)
