"""Continuous timeline construction across robot reboots.

The robot reports a microsecond timestamp (``timestamp_us``) that resets to zero
every time it reboots, while the bag's receive time (``ros_t``) keeps advancing.
Plotting against the raw robot timestamp would make a rebooted robot's curves
restart at t=0 and overlap the previous boot.

Instead we build a monotonic "display time" axis: each boot segment is shifted
forward so it begins after the previous segment, separated by the real ``ros_t``
gap that elapsed across the reboot. A vertical marker is drawn at the last
sample of each previous boot.
"""

import numpy as np


# Distinct color (not otherwise used by the telemetry/overlay plots) marking
# the last telemetry sample before a reboot.
REBOOT_LINE_STYLE = dict(color="darkviolet", linewidth=1.5, linestyle="-", alpha=0.8)

# A robot reboot resets the microsecond counter to ~0, producing a large
# backward jump. Require the timestamp to drop by more than this many seconds so
# that minor out-of-order radio frames don't get misread as reboots.
REBOOT_DROP_THRESHOLD_S = 1.0


def compute_timeline(telemetry, drop_threshold=REBOOT_DROP_THRESHOLD_S):
    """Build a monotonic display-time axis that survives robot reboots.

    :param telemetry: mapping with ``timestamp_us`` (robot clock, microseconds)
        and ``ros_t`` (bag receive time, seconds). ``ros_t`` is optional; without
        it no reboot gap can be computed and the raw robot clock is returned.
    :param drop_threshold: minimum backward jump (seconds) in the robot clock
        that counts as a reboot.
    :return: ``(display_time, reboot_times)`` where ``display_time`` is a float64
        array (seconds) aligned with the telemetry samples, and ``reboot_times``
        is a list of display-time values marking the last sample of each boot
        that preceded a reboot.
    """
    ts = np.asarray(telemetry["timestamp_us"], dtype=np.float64) * 1e-6
    n = len(ts)
    if n == 0:
        return ts, []

    if "ros_t" in telemetry:
        ros_t = np.asarray(telemetry["ros_t"], dtype=np.float64)
    else:
        ros_t = None

    # Reboot = robot clock jumps backward by more than the threshold.
    reboots = (np.where(np.diff(ts) < -drop_threshold)[0] + 1).tolist()

    seg_starts = [0] + reboots
    seg_ends = reboots + [n]

    display = np.empty(n, dtype=np.float64)
    reboot_times = []
    for k, (start, end) in enumerate(zip(seg_starts, seg_ends)):
        if k == 0:
            offset = 0.0
        else:
            prev = start - 1
            if ros_t is not None:
                gap = float(ros_t[start] - ros_t[prev])
                if gap < 0.0:
                    gap = 0.0
            else:
                gap = 0.0
            # Shift this segment so it continues just after the previous boot's
            # last display time plus the real elapsed reboot gap.
            offset = display[prev] + gap - ts[start]
            reboot_times.append(display[prev])
        display[start:end] = ts[start:end] + offset

    return display, reboot_times


def draw_reboot_lines(axs, reboot_times, label="reboot"):
    """Draw vertical reboot markers on every axes in ``axs``.

    :param axs: a single Axes, an array of Axes, or any iterable of Axes.
    :param reboot_times: display-time values at which to draw markers.
    :param label: legend label applied to the first marker on each axes.
    """
    if reboot_times is None or len(reboot_times) == 0:
        return

    if hasattr(axs, "flat"):
        axes_iter = list(axs.flat)
    elif np.ndim(axs) == 0:
        axes_iter = [axs]
    else:
        axes_iter = list(np.ravel(axs))

    for ax in axes_iter:
        for idx, rt in enumerate(reboot_times):
            ax.axvline(rt, label=label if idx == 0 else "_nolegend_",
                       **REBOOT_LINE_STYLE)
