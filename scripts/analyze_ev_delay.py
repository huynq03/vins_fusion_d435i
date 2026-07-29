#!/usr/bin/env python3
"""Estimate VINS/PX4 horizontal timing offset from a ROS1 bag.

The scan convention is:

    compare VINS-derived velocity at time t with PX4 velocity at time t + d

Therefore d > 0 means that the PX4 velocity curve occurs later (lags) the
VINS-derived velocity curve.  This script measures and plots that relationship;
it never changes EKF2 parameters.
"""

import argparse
import collections
import csv
import math
import os
import sys

import numpy as np


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Read a ROS1 bag, derive filtered VINS X/Y velocity, and scan "
            "PX4 alignment from -300 to +300 ms."
        )
    )
    parser.add_argument("bag", help="Input ROS1 .bag file")
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory (default: <bag_stem>_delay_analysis)",
    )
    parser.add_argument(
        "--vins-topic",
        default="/vins_estimator/odometry",
        help="VINS nav_msgs/Odometry topic",
    )
    parser.add_argument(
        "--bridge-topic",
        default="/mavros/odometry/out",
        help="Bridge nav_msgs/Odometry topic used for bag receipt-age checks",
    )
    parser.add_argument(
        "--px4-velocity-topic",
        default="/mavros/local_position/velocity_local",
        help="PX4 local velocity topic (TwistStamped or Odometry)",
    )
    parser.add_argument(
        "--time-source",
        choices=("header", "bag"),
        default="header",
        help="Timeline used for signal alignment (default: header)",
    )
    parser.add_argument(
        "--resample-hz",
        type=float,
        default=50.0,
        help="Uniform resampling rate (default: 50 Hz)",
    )
    parser.add_argument(
        "--smooth-ms",
        type=float,
        default=100.0,
        help="Centered moving-average width (default: 100 ms)",
    )
    parser.add_argument(
        "--max-gap-ms",
        type=float,
        default=200.0,
        help=(
            "Do not interpolate through an input gap larger than this "
            "(default: 200 ms)"
        ),
    )
    parser.add_argument(
        "--shift-min-ms",
        type=float,
        default=-300.0,
        help="Minimum scanned shift (default: -300 ms)",
    )
    parser.add_argument(
        "--shift-max-ms",
        type=float,
        default=300.0,
        help="Maximum scanned shift (default: +300 ms)",
    )
    parser.add_argument(
        "--shift-step-ms",
        type=float,
        default=5.0,
        help="Shift step (default: 5 ms)",
    )
    parser.add_argument(
        "--min-overlap-seconds",
        type=float,
        default=3.0,
        help="Minimum valid overlap required for each score",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Write numeric CSV/text only; useful if matplotlib is unavailable",
    )
    args = parser.parse_args()

    if args.resample_hz <= 0.0:
        parser.error("--resample-hz must be positive")
    if args.smooth_ms < 0.0:
        parser.error("--smooth-ms cannot be negative")
    if args.max_gap_ms <= 0.0:
        parser.error("--max-gap-ms must be positive")
    if args.shift_step_ms <= 0.0:
        parser.error("--shift-step-ms must be positive")
    if args.shift_min_ms >= args.shift_max_ms:
        parser.error("--shift-min-ms must be smaller than --shift-max-ms")
    if args.min_overlap_seconds <= 0.0:
        parser.error("--min-overlap-seconds must be positive")
    return args


def get_header_time(message):
    if not hasattr(message, "header"):
        return None
    stamp = message.header.stamp
    if stamp.secs == 0 and stamp.nsecs == 0:
        return None
    return stamp.to_sec()


def get_linear_xy(message):
    if not hasattr(message, "twist"):
        raise ValueError("message has no twist field")
    twist = message.twist
    if hasattr(twist, "twist"):
        twist = twist.twist
    if not hasattr(twist, "linear"):
        raise ValueError("message twist has no linear field")
    return float(twist.linear.x), float(twist.linear.y)


def load_bag(args):
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError(
            "Python rosbag is unavailable. Run inside the ROS Noetic "
            "environment after: source /opt/ros/noetic/setup.bash"
        ) from exc

    wanted_topics = [
        args.vins_topic,
        args.bridge_topic,
        args.px4_velocity_topic,
    ]
    vins_rows = []
    px4_rows = []
    input_receipts = collections.defaultdict(collections.deque)
    output_receipts = collections.defaultdict(collections.deque)
    vins_ages_ms = []
    bridge_ages_ms = []

    with rosbag.Bag(args.bag, "r") as bag:
        available = set(bag.get_type_and_topic_info().topics.keys())
        required = {args.vins_topic, args.px4_velocity_topic}
        missing = sorted(required - available)
        if missing:
            raise RuntimeError(
                "Bag is missing required topic(s): {}".format(", ".join(missing))
            )

        for topic, message, bag_stamp in bag.read_messages(topics=wanted_topics):
            bag_time = bag_stamp.to_sec()
            header_time = get_header_time(message)
            signal_time = header_time if args.time_source == "header" else bag_time
            if signal_time is None:
                raise RuntimeError(
                    "{} contains a zero/missing header stamp. Re-run with "
                    "--time-source bag only if receipt time is intentionally "
                    "being analysed.".format(topic)
                )

            if topic == args.vins_topic:
                position = message.pose.pose.position
                vins_rows.append(
                    (signal_time, float(position.x), float(position.y))
                )
                if header_time is not None:
                    stamp_ns = message.header.stamp.to_nsec()
                    vins_ages_ms.append((bag_time - header_time) * 1000.0)
                    input_receipts[stamp_ns].append(bag_time)

            elif topic == args.px4_velocity_topic:
                vx, vy = get_linear_xy(message)
                px4_rows.append((signal_time, vx, vy))

            elif topic == args.bridge_topic and header_time is not None:
                stamp_ns = message.header.stamp.to_nsec()
                bridge_ages_ms.append((bag_time - header_time) * 1000.0)
                output_receipts[stamp_ns].append(bag_time)

    if len(vins_rows) < 3:
        raise RuntimeError("Not enough VINS samples in the bag")
    if len(px4_rows) < 3:
        raise RuntimeError("Not enough PX4 velocity samples in the bag")

    paired_bridge_ms = []
    for stamp_ns, input_times in input_receipts.items():
        output_times = output_receipts.get(stamp_ns)
        if output_times is None:
            continue
        while input_times and output_times:
            paired_bridge_ms.append(
                (output_times.popleft() - input_times.popleft()) * 1000.0
            )

    return {
        "vins": np.asarray(vins_rows, dtype=float),
        "px4": np.asarray(px4_rows, dtype=float),
        "vins_ages_ms": np.asarray(vins_ages_ms, dtype=float),
        "bridge_ages_ms": np.asarray(bridge_ages_ms, dtype=float),
        "paired_bridge_ms": np.asarray(paired_bridge_ms, dtype=float),
    }


def sort_and_merge_duplicate_times(rows, label):
    finite_mask = np.all(np.isfinite(rows), axis=1)
    rows = rows[finite_mask]
    if len(rows) < 3:
        raise RuntimeError("{} has fewer than 3 finite samples".format(label))

    rows = rows[np.argsort(rows[:, 0], kind="stable")]
    merged_times = []
    merged_values = []
    start = 0
    while start < len(rows):
        end = start + 1
        while end < len(rows) and rows[end, 0] == rows[start, 0]:
            end += 1
        merged_times.append(rows[start, 0])
        merged_values.append(np.mean(rows[start:end, 1:], axis=0))
        start = end

    times = np.asarray(merged_times)
    values = np.asarray(merged_values)
    if len(times) < 3 or times[-1] <= times[0]:
        raise RuntimeError("{} has no usable increasing timeline".format(label))
    return times, values


def odd_window_samples(smooth_ms, sample_rate_hz):
    if smooth_ms <= 0.0:
        return 1
    samples = max(1, int(round(smooth_ms * sample_rate_hz / 1000.0)))
    if samples % 2 == 0:
        samples += 1
    return samples


def centered_moving_average(values, window):
    if window <= 1:
        return values.copy()
    half = window // 2
    kernel = np.ones(window, dtype=float) / float(window)
    if values.ndim == 1:
        padded = np.pad(values, (half, half), mode="edge")
        return np.convolve(padded, kernel, mode="valid")
    columns = []
    for column in range(values.shape[1]):
        padded = np.pad(values[:, column], (half, half), mode="edge")
        columns.append(np.convolve(padded, kernel, mode="valid"))
    return np.column_stack(columns)


def interpolation_validity(grid, source_times, max_gap_s, edge_radius):
    """Mark grid points bracketed by real samples without a long source gap."""
    right = np.searchsorted(source_times, grid, side="left")
    valid = np.zeros(len(grid), dtype=bool)

    exact = right < len(source_times)
    exact_indices = np.flatnonzero(exact)
    if len(exact_indices):
        exact_matches = np.isclose(
            source_times[right[exact_indices]],
            grid[exact_indices],
            rtol=0.0,
            atol=1.0e-9,
        )
        valid[exact_indices[exact_matches]] = True

    between = (right > 0) & (right < len(source_times)) & ~valid
    between_indices = np.flatnonzero(between)
    if len(between_indices):
        gaps = (
            source_times[right[between_indices]]
            - source_times[right[between_indices] - 1]
        )
        valid[between_indices] = gaps <= max_gap_s

    if edge_radius > 0:
        kernel = np.ones(2 * edge_radius + 1, dtype=int)
        valid = (
            np.convolve(valid.astype(int), kernel, mode="same")
            == len(kernel)
        )
    return valid


def sampled_validity(source_grid, source_valid, query_times):
    """Require both uniform-grid neighbours around each interpolated query."""
    right = np.searchsorted(source_grid, query_times, side="left")
    valid = np.zeros(len(query_times), dtype=bool)

    exact = right < len(source_grid)
    exact_indices = np.flatnonzero(exact)
    if len(exact_indices):
        exact_matches = np.isclose(
            source_grid[right[exact_indices]],
            query_times[exact_indices],
            rtol=0.0,
            atol=1.0e-9,
        )
        matched_indices = exact_indices[exact_matches]
        valid[matched_indices] = source_valid[right[matched_indices]]

    between = (right > 0) & (right < len(source_grid)) & ~valid
    between_indices = np.flatnonzero(between)
    if len(between_indices):
        valid[between_indices] = (
            source_valid[right[between_indices] - 1]
            & source_valid[right[between_indices]]
        )
    return valid


def uniform_signal(
    times,
    values,
    rate_hz,
    smooth_ms,
    max_gap_ms,
    derive_velocity,
):
    step = 1.0 / rate_hz
    if times[-1] - times[0] < 2.0 * step:
        raise RuntimeError(
            "Signal duration is too short for resampling/derivative at "
            "{:.3f} Hz".format(rate_hz)
        )
    grid = np.arange(times[0], times[-1] + step * 0.25, step)
    if len(grid) < 3:
        raise RuntimeError("Uniform signal has fewer than 3 samples")
    interpolated = np.column_stack(
        [np.interp(grid, times, values[:, axis]) for axis in range(values.shape[1])]
    )
    window = odd_window_samples(smooth_ms, rate_hz)
    if derive_velocity:
        derived = np.column_stack(
            [
                np.gradient(interpolated[:, axis], grid)
                for axis in range(interpolated.shape[1])
            ]
        )
        filtered = centered_moving_average(derived, window)
    else:
        filtered = centered_moving_average(interpolated, window)
    valid = interpolation_validity(
        grid,
        times,
        max_gap_ms / 1000.0,
        window // 2,
    )
    return grid, filtered, window, valid


def score_shift(vins_time, vins_velocity, px4_time, px4_velocity, shift_s, min_samples):
    shifted_px4_time = vins_time + shift_s
    if len(vins_time) < min_samples:
        return [float("nan")] * 3, [float("nan")] * 3, 0

    reference = vins_velocity
    observed = np.column_stack(
        [
            np.interp(shifted_px4_time, px4_time, px4_velocity[:, axis])
            for axis in range(2)
        ]
    )
    difference = observed - reference
    rmse = np.sqrt(np.mean(np.square(difference), axis=0))
    combined_rmse = math.sqrt(float(np.mean(np.square(difference))))

    correlations = []
    for axis in range(2):
        if np.std(reference[:, axis]) < 1.0e-12 or np.std(observed[:, axis]) < 1.0e-12:
            correlations.append(float("nan"))
        else:
            correlations.append(
                float(np.corrcoef(reference[:, axis], observed[:, axis])[0, 1])
            )
    finite_correlations = [
        value for value in correlations if math.isfinite(value)
    ]
    if not finite_correlations:
        combined_correlation = float("nan")
    else:
        clipped = np.clip(finite_correlations, -0.999999, 0.999999)
        combined_correlation = float(
            np.tanh(np.mean(np.arctanh(clipped)))
        )

    return (
        [float(rmse[0]), float(rmse[1]), combined_rmse],
        [correlations[0], correlations[1], combined_correlation],
        int(len(vins_time)),
    )


def best_rmse_index(values, shifts_ms):
    values = np.asarray(values)
    finite = np.isfinite(values)
    if not np.any(finite):
        raise RuntimeError("No finite delay score; test motion/overlap is insufficient")
    finite_indices = np.flatnonzero(finite)
    minimum = float(np.min(values[finite]))
    tie_tolerance = max(1.0e-9, abs(minimum) * 1.0e-3)
    near_minimum = finite_indices[values[finite_indices] <= minimum + tie_tolerance]
    selected = near_minimum[np.argmin(np.abs(shifts_ms[near_minimum]))]
    score_span = float(np.max(values[finite]) - minimum)
    ambiguous = score_span <= max(1.0e-6, abs(minimum) * 0.01)
    return int(selected), ambiguous


def best_correlation_index(values):
    values = np.asarray(values)
    finite = np.isfinite(values)
    if not np.any(finite):
        return None
    finite_indices = np.flatnonzero(finite)
    return int(finite_indices[np.argmax(values[finite])])


def describe(values):
    values = np.asarray(values)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return None
    return {
        "count": int(len(values)),
        "min": float(np.min(values)),
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def describe_line(label, values, unit="ms"):
    stats = describe(values)
    if stats is None:
        return "{}: unavailable".format(label)
    return (
        "{label}: n={count}, min/mean/median/p95/max="
        "{min:.3f}/{mean:.3f}/{median:.3f}/{p95:.3f}/{max:.3f} {unit}"
    ).format(label=label, unit=unit, **stats)


def aligned_pair(vins_time, vins_velocity, px4_time, px4_velocity, axis, shift_s):
    shifted_px4_time = vins_time + shift_s
    valid = (
        (shifted_px4_time >= px4_time[0])
        & (shifted_px4_time <= px4_time[-1])
    )
    time_values = vins_time[valid]
    reference = vins_velocity[valid, axis]
    observed = np.interp(
        shifted_px4_time[valid],
        px4_time,
        px4_velocity[:, axis],
    )
    return time_values, reference, observed


def create_plots(
    output_dir,
    vins_time,
    vins_velocity,
    px4_time,
    px4_velocity,
    shifts_ms,
    rmse,
    best_rmse_indices,
):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is required for plots. In the ROS Noetic container, "
            "install python3-matplotlib or re-run with --no-plots."
        ) from exc

    axis_names = ("X", "Y")
    for axis, axis_name in enumerate(axis_names):
        best_shift_ms = shifts_ms[best_rmse_indices[axis]]
        before_t, before_vins, before_px4 = aligned_pair(
            vins_time,
            vins_velocity,
            px4_time,
            px4_velocity,
            axis,
            0.0,
        )
        after_t, after_vins, after_px4 = aligned_pair(
            vins_time,
            vins_velocity,
            px4_time,
            px4_velocity,
            axis,
            best_shift_ms / 1000.0,
        )

        figure, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=False)
        axes[0].plot(before_t, before_vins, label="VINS d(position)/dt")
        axes[0].plot(before_t, before_px4, label="PX4 velocity, d=0 ms")
        axes[0].set_title("Velocity {} before alignment".format(axis_name))
        axes[0].set_ylabel("m/s")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(after_t, after_vins, label="VINS d(position)/dt")
        axes[1].plot(
            after_t,
            after_px4,
            label="PX4 evaluated at t + {:.1f} ms".format(best_shift_ms),
        )
        axes[1].set_title(
            "Velocity {} after RMSE alignment (d={:.1f} ms)".format(
                axis_name, best_shift_ms
            )
        )
        axes[1].set_xlabel("Seconds from common bag time origin")
        axes[1].set_ylabel("m/s")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        figure.tight_layout()
        figure.savefig(
            os.path.join(
                output_dir,
                "velocity_{}_before_after.png".format(axis_name.lower()),
            ),
            dpi=150,
        )
        plt.close(figure)

    figure, axis = plt.subplots(figsize=(11, 6))
    axis.plot(shifts_ms, rmse[:, 0], label="RMSE X")
    axis.plot(shifts_ms, rmse[:, 1], label="RMSE Y")
    axis.plot(shifts_ms, rmse[:, 2], label="RMSE combined", linewidth=2)
    axis.axvline(0.0, color="black", linestyle="--", alpha=0.4)
    for index, color in zip(best_rmse_indices, ("C0", "C1", "C2")):
        axis.axvline(shifts_ms[index], color=color, linestyle=":", alpha=0.7)
    axis.set_xlabel("Shift d (ms): compare VINS(t) with PX4(t + d)")
    axis.set_ylabel("RMSE (m/s)")
    axis.set_title("Velocity RMSE versus time shift")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(os.path.join(output_dir, "rmse_vs_time_shift.png"), dpi=150)
    plt.close(figure)


def main():
    args = parse_args()
    if not os.path.isfile(args.bag):
        raise RuntimeError("Bag does not exist: {}".format(args.bag))

    if args.output_dir is None:
        stem = os.path.splitext(os.path.abspath(args.bag))[0]
        args.output_dir = stem + "_delay_analysis"
    os.makedirs(args.output_dir, exist_ok=True)

    data = load_bag(args)
    vins_time_abs, vins_position = sort_and_merge_duplicate_times(
        data["vins"], "VINS position"
    )
    px4_time_abs, px4_velocity_raw = sort_and_merge_duplicate_times(
        data["px4"], "PX4 velocity"
    )

    common_origin = min(vins_time_abs[0], px4_time_abs[0])
    vins_time = vins_time_abs - common_origin
    px4_time = px4_time_abs - common_origin

    vins_grid, vins_velocity, smooth_window, vins_valid = uniform_signal(
        vins_time,
        vins_position,
        args.resample_hz,
        args.smooth_ms,
        args.max_gap_ms,
        derive_velocity=True,
    )
    px4_grid, px4_velocity, _, px4_valid = uniform_signal(
        px4_time,
        px4_velocity_raw,
        args.resample_hz,
        args.smooth_ms,
        args.max_gap_ms,
        derive_velocity=False,
    )

    shifts_ms = np.arange(
        args.shift_min_ms,
        args.shift_max_ms + args.shift_step_ms * 0.5,
        args.shift_step_ms,
    )
    shifts_s = shifts_ms / 1000.0
    min_samples = max(
        3, int(math.ceil(args.min_overlap_seconds * args.resample_hz))
    )

    # Use exactly the same VINS time samples for every candidate shift.  This
    # prevents endpoint transients or bag gaps from favouring a shift merely
    # because it scores an easier/shorter overlap interval.
    common_valid = (
        vins_valid
        & (vins_grid + shifts_s[0] >= px4_grid[0])
        & (vins_grid + shifts_s[-1] <= px4_grid[-1])
    )
    for shift_s in shifts_s:
        common_valid &= sampled_validity(
            px4_grid,
            px4_valid,
            vins_grid + shift_s,
        )

    evaluation_time = vins_grid[common_valid]
    evaluation_velocity = vins_velocity[common_valid]
    if len(evaluation_time) < min_samples:
        raise RuntimeError(
            "Only {} fixed-overlap samples remain after applying the full "
            "shift range and gap mask; need at least {}. Record a longer bag, "
            "reduce --shift range, or inspect dropouts.".format(
                len(evaluation_time), min_samples
            )
        )

    rmse_rows = []
    correlation_rows = []
    overlap_rows = []
    for shift_ms in shifts_ms:
        rmse_score, correlation_score, overlap = score_shift(
            evaluation_time,
            evaluation_velocity,
            px4_grid,
            px4_velocity,
            shift_ms / 1000.0,
            min_samples,
        )
        rmse_rows.append(rmse_score)
        correlation_rows.append(correlation_score)
        overlap_rows.append(overlap)

    rmse = np.asarray(rmse_rows)
    correlation = np.asarray(correlation_rows)
    overlap = np.asarray(overlap_rows)
    best_rmse_results = [
        best_rmse_index(rmse[:, axis], shifts_ms) for axis in range(3)
    ]
    best_rmse_indices = [result[0] for result in best_rmse_results]
    ambiguous_rmse = [result[1] for result in best_rmse_results]
    best_correlation_indices = [
        best_correlation_index(correlation[:, axis]) for axis in range(3)
    ]

    scan_path = os.path.join(args.output_dir, "delay_scan.csv")
    with open(scan_path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "shift_ms",
                "rmse_x_mps",
                "rmse_y_mps",
                "rmse_combined_mps",
                "corr_x",
                "corr_y",
                "corr_combined",
                "overlap_samples",
            ]
        )
        for index, shift_ms in enumerate(shifts_ms):
            writer.writerow(
                [shift_ms]
                + list(rmse[index])
                + list(correlation[index])
                + [int(overlap[index])]
            )

    summary_lines = [
        "External-vision delay scan",
        "===========================",
        "Bag: {}".format(os.path.abspath(args.bag)),
        "Timeline: {}".format(args.time_source),
        "Samples: VINS={}, PX4={}".format(len(data["vins"]), len(data["px4"])),
        "Resample: {:.3f} Hz; centered smoothing: {} samples (~{:.1f} ms)".format(
            args.resample_hz,
            smooth_window,
            smooth_window * 1000.0 / args.resample_hz,
        ),
        "Maximum interpolated source gap: {:.1f} ms".format(args.max_gap_ms),
        "Fixed overlap used for every shift: {} samples ({:.3f} s)".format(
            len(evaluation_time),
            (
                evaluation_time[-1] - evaluation_time[0]
                if len(evaluation_time) > 1
                else 0.0
            ),
        ),
        "",
        "Sign convention: score(d) compares VINS-derived velocity at t with "
        "PX4 local velocity at t+d.",
        "Positive d means the PX4 velocity curve occurs later (PX4 lags).",
        "",
    ]
    excitation_std = np.std(evaluation_velocity, axis=0)
    for axis, label in enumerate(("X", "Y", "combined")):
        rmse_index = best_rmse_indices[axis]
        corr_index = best_correlation_indices[axis]
        confidence_notes = []
        if ambiguous_rmse[axis]:
            confidence_notes.append("flat/ambiguous RMSE curve")
        if axis < 2 and excitation_std[axis] < 1.0e-3:
            confidence_notes.append("weak excitation on this axis")
        confidence_text = (
            " [" + "; ".join(confidence_notes) + "]"
            if confidence_notes
            else ""
        )
        summary_lines.append(
            "Best {} RMSE: d={:.1f} ms, RMSE={:.6f} m/s, corr={:.6f}{}".format(
                label,
                shifts_ms[rmse_index],
                rmse[rmse_index, axis],
                correlation[rmse_index, axis],
                confidence_text,
            )
        )
        if corr_index is None:
            summary_lines.append(
                "Best {} correlation: unavailable (constant/weak signal)".format(
                    label
                )
            )
        else:
            summary_lines.append(
                "Best {} correlation: d={:.1f} ms, corr={:.6f}, RMSE={:.6f} m/s".format(
                    label,
                    shifts_ms[corr_index],
                    correlation[corr_index, axis],
                    rmse[corr_index, axis],
                )
            )

    summary_lines.extend(
        [
            "",
            describe_line(
                "VINS bag-receipt minus header age",
                data["vins_ages_ms"],
            ),
            describe_line(
                "Bridge bag-receipt minus header age",
                data["bridge_ages_ms"],
            ),
            describe_line(
                "Matched bridge receipt minus VINS receipt",
                data["paired_bridge_ms"],
            ),
            "",
            "PX4 v1.14.3 parameter metadata: EKF2_EV_DELAY is milliseconds, "
            "range 0..300, and reboot-required. PX4 subtracts it from the "
            "synchronized EV sample timestamp; a positive parameter moves the "
            "EV sample earlier.",
            "Mapping caveat: let E = EV_stamp - actual_capture (positive when "
            "the EV stamp is late) and R = PX4 fused-output response lag. With "
            "this script's convention, approximately d = R - E. Only if "
            "R is independently negligible does E ~= -d and a candidate "
            "EKF2_EV_DELAY ~= max(0, -d).",
            "Therefore never copy scan d directly into EKF2_EV_DELAY. PX4 "
            "response dynamics, competing aids, filtering, and correctly "
            "stamped processing/transport latency make E non-identifiable "
            "from these two curves alone; confirm with ULog innovations and "
            "vehicle_visual_odometry timestamps.",
        ]
    )

    summary_path = os.path.join(args.output_dir, "summary.txt")
    with open(summary_path, "w") as stream:
        stream.write("\n".join(summary_lines) + "\n")

    if not args.no_plots:
        create_plots(
            args.output_dir,
            evaluation_time,
            evaluation_velocity,
            px4_grid,
            px4_velocity,
            shifts_ms,
            rmse,
            best_rmse_indices,
        )

    print("\n".join(summary_lines))
    print("")
    print("Wrote {}".format(scan_path))
    print("Wrote {}".format(summary_path))
    if not args.no_plots:
        print("Wrote velocity X/Y and RMSE PNG plots in {}".format(args.output_dir))


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        sys.exit(1)
