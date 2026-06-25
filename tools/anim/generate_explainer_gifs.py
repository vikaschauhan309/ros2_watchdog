#!/usr/bin/env python3
"""
Regenerates the four scenario GIFs in media/. Each is a scripted (not
recorded) diagram animation, deterministic and committed alongside its
source so no GIF in this repo is a mystery binary.

Usage:
    pip install matplotlib pillow
    python3 tools/anim/generate_explainer_gifs.py
"""
from anim_common import (
    GREEN, RED, YELLOW,
    add_caption_text, add_status_text, base_figure, build_action_box,
    build_topic_column, build_watchdog_box, save_animation, straight_arrow,
)

# ---------------------------------------------------------------------------
# Scenario 1: single-topic failure and recovery, with two untouched topics
# present for contrast.
# ---------------------------------------------------------------------------

def scenario_failure_and_recovery():
    topics = ["/camera/image", "/lidar/scan", "/odom"]
    failing = "/lidar/scan"

    PHASE_OK, PHASE_STALE, PHASE_DEAD = (0, 10), (10, 20), (20, 30)
    PHASE_ACTION, PHASE_ENGAGED, PHASE_RECOVERED = (30, 42), (42, 52), (52, 64)
    total = PHASE_RECOVERED[1]

    fig, ax = base_figure()
    rows = build_topic_column(ax, topics)
    build_watchdog_box(ax)
    action_box, _ = build_action_box(ax, "call_trigger_service\n/safety/switch_to_safe_mode")
    arrow = straight_arrow(ax)
    status_text = add_status_text(ax)
    caption = add_caption_text(ax)
    caption.set_text(
        "/camera/image and /odom stay healthy the whole time -- only /lidar/scan is\n"
        "interrupted, to show the watchdog tracks each topic's health independently."
    )

    _, dot, y = rows[failing]

    def status(frame):
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

    def update(frame):
        color, message = status(frame)
        dot.set_facecolor(color)
        status_text.set_text(message)

        if PHASE_ACTION[0] <= frame < PHASE_ENGAGED[0]:
            progress = (frame - PHASE_ACTION[0]) / (PHASE_ACTION[1] - PHASE_ACTION[0])
            x0, y0, x1, y1 = 3.6, y, 5.0, 3.0
            arrow.set_data([x0, x0 + (x1 - x0) * progress], [y0, y0 + (y1 - y0) * progress])
        elif PHASE_ENGAGED[0] <= frame < PHASE_RECOVERED[0]:
            arrow.set_data([3.6, 5.0], [y, 3.0])
        else:
            arrow.set_data([], [])

        flashing = PHASE_ENGAGED[0] <= frame < PHASE_ENGAGED[1] and frame % 4 < 2
        action_box.set_facecolor("#fdebd0" if flashing else "white")
        return [dot, action_box, arrow, status_text]

    save_animation(fig, update, total, "01_failure_and_recovery.gif")


# ---------------------------------------------------------------------------
# Scenario 2: the same kind of failure routed through each of the four
# configured action types in turn, to show action choice is config, not code.
# ---------------------------------------------------------------------------

def scenario_action_types():
    topic = "/lidar/scan"
    actions = [
        ("log", "RCLCPP_ERROR(\n  \"/lidar/scan is DEAD\")"),
        ("publish_diagnostic", "diagnostic_msgs/DiagnosticArray\n-> /diagnostics"),
        ("publish_event", "ros2_watchdog/WatchdogEvent\n-> /watchdog/events"),
        ("call_trigger_service", "call_trigger_service\n/safety/switch_to_safe_mode"),
    ]
    OK, STALE, DEAD, ACTION, RECOVERED = 6, 4, 4, 8, 6
    cycle_len = OK + STALE + DEAD + ACTION + RECOVERED
    total = cycle_len * len(actions)

    fig, ax = base_figure()
    rows = build_topic_column(ax, [topic])
    build_watchdog_box(ax)
    action_box, action_text = build_action_box(ax, actions[0][1])
    arrow = straight_arrow(ax)
    status_text = add_status_text(ax)
    caption = add_caption_text(ax)
    caption.set_text(
        "Same stale->dead transition, four different `on_dead.action` values from\n"
        "the same YAML field -- the dispatched action is configuration, not code."
    )

    _, dot, y = rows[topic]

    def update(frame):
        cycle = frame // cycle_len
        f = frame % cycle_len
        action_name, action_label = actions[cycle % len(actions)]
        action_text.set_text(action_label)

        if f < OK:
            dot.set_facecolor(GREEN)
            status_text.set_text(f"action = {action_name}  |  all topics healthy")
            arrow.set_data([], [])
        elif f < OK + STALE:
            dot.set_facecolor(YELLOW)
            status_text.set_text(f"action = {action_name}  |  /lidar/scan: STALE")
            arrow.set_data([], [])
        elif f < OK + STALE + DEAD:
            dot.set_facecolor(RED)
            status_text.set_text(f"action = {action_name}  |  /lidar/scan: DEAD")
            arrow.set_data([], [])
        elif f < OK + STALE + DEAD + ACTION:
            dot.set_facecolor(RED)
            status_text.set_text(f"action = {action_name}  |  dispatching...")
            progress = (f - (OK + STALE + DEAD)) / ACTION
            x0, y0, x1, y1 = 3.6, y, 5.0, 3.0
            arrow.set_data([x0, x0 + (x1 - x0) * progress], [y0, y0 + (y1 - y0) * progress])
        else:
            dot.set_facecolor(GREEN)
            status_text.set_text(f"action = {action_name}  |  /lidar/scan: RECOVERED")
            arrow.set_data([], [])

        return [dot, action_box, action_text, arrow, status_text]

    save_animation(fig, update, total, "02_action_dispatch_types.gif")


# ---------------------------------------------------------------------------
# Scenario 3: two topics failing on overlapping but independent schedules,
# to show each topic's state machine is tracked separately.
# ---------------------------------------------------------------------------

def scenario_multi_topic_failure():
    topics = ["/camera/image", "/lidar/scan", "/odom"]
    total = 56

    fig, ax = base_figure()
    rows = build_topic_column(ax, topics)
    build_watchdog_box(ax)
    cam_action, cam_action_text = build_action_box(ax, "publish_event\n(/camera/image)", y=1.6)
    lidar_action, lidar_action_text = build_action_box(
        ax, "call_trigger_service\n(/lidar/scan)", y=0.5
    )
    cam_arrow = straight_arrow(ax)
    lidar_arrow = straight_arrow(ax)
    status_text = add_status_text(ax)
    caption = add_caption_text(ax)
    caption.set_text(
        "/camera/image and /lidar/scan fail on overlapping but independent\n"
        "schedules -- each topic's OK/STALE/DEAD state is tracked separately."
    )

    _, cam_dot, cam_y = rows["/camera/image"]
    _, lidar_dot, lidar_y = rows["/lidar/scan"]

    # (start, end, color, status_fragment) windows per topic.
    cam_windows = [
        (0, 8, GREEN, "OK"), (8, 16, YELLOW, "STALE"), (16, 36, RED, "DEAD"),
        (36, 56, GREEN, "RECOVERED"),
    ]
    lidar_windows = [
        (0, 20, GREEN, "OK"), (20, 28, YELLOW, "STALE"), (28, 48, RED, "DEAD"),
        (48, 56, GREEN, "RECOVERED"),
    ]

    def lookup(windows, frame):
        for start, end, color, label in windows:
            if start <= frame < end:
                return color, label, start, end
        return windows[-1][2], windows[-1][3], windows[-1][0], windows[-1][1]

    def arrow_progress(arrow, dot_y, active, action_y, window_start, window_end, frame):
        if not active:
            arrow.set_data([], [])
            return
        progress = min(1.0, (frame - window_start) / max(1, (window_end - window_start) * 0.4))
        x0, y0, x1, y1 = 3.6, dot_y, 5.0, action_y
        arrow.set_data([x0, x0 + (x1 - x0) * progress], [y0, y0 + (y1 - y0) * progress])

    def update(frame):
        cam_color, cam_label, cam_s, cam_e = lookup(cam_windows, frame)
        lidar_color, lidar_label, lidar_s, lidar_e = lookup(lidar_windows, frame)
        cam_dot.set_facecolor(cam_color)
        lidar_dot.set_facecolor(lidar_color)
        status_text.set_text(f"/camera/image: {cam_label}   |   /lidar/scan: {lidar_label}")

        cam_active = cam_label == "DEAD"
        lidar_active = lidar_label == "DEAD"
        arrow_progress(cam_arrow, cam_y, cam_active, 1.6, cam_s, cam_e, frame)
        arrow_progress(lidar_arrow, lidar_y, lidar_active, 0.5, lidar_s, lidar_e, frame)
        cam_action.set_facecolor("#fdebd0" if cam_active else "white")
        lidar_action.set_facecolor("#fdebd0" if lidar_active else "white")

        return [cam_dot, lidar_dot, cam_action, lidar_action, cam_arrow, lidar_arrow,
                status_text]

    save_animation(fig, update, total, "03_simultaneous_multi_topic_failure.gif")


# ---------------------------------------------------------------------------
# Scenario 4: a topic that never publishes a single message after startup.
# There's no "first message" to start a staleness clock from, so the
# implementation seeds the reference clock with node-start time instead of
# last-message time. That means a topic with zero messages still ages
# through the normal OK -> STALE -> DEAD timeline, just measured from when
# the watchdog came up rather than from a message that never arrived.
# ---------------------------------------------------------------------------

def scenario_never_published():
    topic = "/odom"
    PHASE_OK, PHASE_STALE, PHASE_DEAD, PHASE_ACTION, PHASE_ENGAGED, PHASE_RECOVERED = (
        (0, 10), (10, 20), (20, 30), (30, 42), (42, 52), (52, 64)
    )
    total = PHASE_RECOVERED[1]

    fig, ax = base_figure()
    rows = build_topic_column(ax, [topic])
    build_watchdog_box(ax)
    action_box, _ = build_action_box(ax, "publish_diagnostic\n-> /diagnostics")
    arrow = straight_arrow(ax)
    status_text = add_status_text(ax)
    caption = add_caption_text(ax)
    caption.set_text(
        "/odom never sends a single message -- the watchdog seeds its staleness\n"
        "clock from node-start time instead of a last-message time, so a topic\n"
        "with zero messages still ages through OK -> STALE -> DEAD on schedule."
    )

    _, dot, y = rows[topic]

    def update(frame):
        if frame < PHASE_STALE[0]:
            dot.set_facecolor(GREEN)
            status_text.set_text("/odom: 0 messages since startup, within grace period -> OK")
            arrow.set_data([], [])
        elif frame < PHASE_DEAD[0]:
            dot.set_facecolor(YELLOW)
            status_text.set_text("/odom: 0 messages, time-since-startup > staleness_timeout -> STALE")
            arrow.set_data([], [])
        elif frame < PHASE_ACTION[0]:
            dot.set_facecolor(RED)
            status_text.set_text("/odom: 0 messages, time-since-startup > dead_timeout -> DEAD")
            arrow.set_data([], [])
        elif frame < PHASE_ENGAGED[0]:
            dot.set_facecolor(RED)
            progress = (frame - PHASE_ACTION[0]) / (PHASE_ACTION[1] - PHASE_ACTION[0])
            x0, y0, x1, y1 = 3.6, y, 5.0, 3.0
            arrow.set_data([x0, x0 + (x1 - x0) * progress], [y0, y0 + (y1 - y0) * progress])
            status_text.set_text("Watchdog publishing diagnostic for /odom ...")
        elif frame < PHASE_RECOVERED[0]:
            dot.set_facecolor(RED)
            arrow.set_data([3.6, 5.0], [y, 3.0])
            status_text.set_text("Degraded mode engaged -- /odom still has 0 messages")
        else:
            dot.set_facecolor(GREEN)
            arrow.set_data([], [])
            status_text.set_text("/odom: first message ever received -> RECOVERED")

        flashing = PHASE_ENGAGED[0] <= frame < PHASE_ENGAGED[1] and frame % 4 < 2
        action_box.set_facecolor("#fdebd0" if flashing else "white")
        return [dot, action_box, arrow, status_text]

    save_animation(fig, update, total, "04_never_published_topic.gif")


def main():
    scenario_failure_and_recovery()
    scenario_action_types()
    scenario_multi_topic_failure()
    scenario_never_published()


if __name__ == "__main__":
    main()
