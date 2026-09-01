#!/usr/bin/env python3
"""Analyze front-marker VSP/YawSP jitter from a ROS1 bag on the Laptop.

The script intentionally keeps ROS message parsing separate from the analysis:
rosbags reads the copied rosbag1 file, then NumPy/SciPy/Matplotlib perform all
resampling, filtering, statistics, FFTs, and rendering on the Laptop.

The primary jitter comparison is made on a common time grid.  Samples whose
latest status is BRAKING_POSE_LOSS are kept in the output but are excluded from
the static tracking jitter statistics.  The bag's first ``--duration-sec``
seconds are analyzed, which makes an accidentally overlong capture auditable.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


AXES = ("vx", "vy", "vz", "yawsp")
UNITS = {"vx": "m/s", "vy": "m/s", "vz": "m/s", "yawsp": "rad/s"}
TAUS = (0.05, 0.10, 0.20, 0.30)
TOPIC_NAMES = {
    "pose": "/front_marker/pose",
    "position_error": "/front_marker_control/position_error",
    "yaw_error": "/front_marker_control/yaw_error",
    "cmd_vel": "/front_marker_control/cmd_vel",
    "setpoint_preview": "/front_marker_control/setpoint_preview",
    "status": "/front_marker_control/status",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="copied rosbag1 capture.bag")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="output directory; defaults to the bag's parent directory",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=60.0,
        help="analyze only this many seconds from the common data start",
    )
    parser.add_argument(
        "--max-gap-sec",
        type=float,
        default=0.15,
        help="do not interpolate across larger source-data gaps",
    )
    parser.add_argument(
        "--sample-rate-hz",
        type=float,
        default=None,
        help="common grid rate; default is the median cmd_vel rate",
    )
    return parser.parse_args()


def stamp_to_sec(stamp: Any) -> float:
    """Read ROS1 or rosbags-generated time objects."""
    sec = getattr(stamp, "sec", getattr(stamp, "secs", 0))
    nsec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", 0))
    return float(sec) + float(nsec) * 1e-9


def message_time(message: Any, bag_time_sec: float) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is None:
        return bag_time_sec
    value = stamp_to_sec(stamp)
    return value if math.isfinite(value) and value > 0.0 else bag_time_sec


def require_runtime() -> Tuple[Any, Any, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "Laptop 缺少 rosbags。请先执行: python3 -m pip install --user rosbags"
        ) from exc
    try:
        import scipy  # noqa: F401
        import matplotlib  # noqa: F401
    except ImportError as exc:
        raise RuntimeError(
            "Laptop 缺少 SciPy 或 Matplotlib；请在 Laptop 环境安装后重试。"
        ) from exc
    return AnyReader, Stores, get_typestore


def finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def vector3_message(message: Any) -> Tuple[float, float, float]:
    vector = message.vector
    return float(vector.x), float(vector.y), float(vector.z)


def pose_position_message(message: Any) -> Tuple[float, float, float]:
    position = message.pose.position
    return float(position.x), float(position.y), float(position.z)


def cmd_vel_message(message: Any) -> Tuple[float, float, float, float]:
    linear = message.twist.linear
    angular = message.twist.angular
    return float(linear.x), float(linear.y), float(linear.z), float(angular.z)


def read_bag(bag_path: Path) -> Tuple[Dict[str, List[Tuple[float, Any]]], Dict[str, Any]]:
    AnyReader, Stores, get_typestore = require_runtime()
    streams: Dict[str, List[Tuple[float, Any]]] = {
        "pose": [],
        "position_error": [],
        "yaw_error": [],
        "cmd_vel": [],
        "status": [],
    }
    info: Dict[str, Any] = {"topics_present": [], "message_counts": Counter()}
    typestore = get_typestore(Stores.ROS1_NOETIC)
    try:
        reader_context = AnyReader([bag_path], default_typestore=typestore)
    except TypeError:
        # Compatibility with older rosbags releases.
        reader_context = AnyReader([bag_path])
    with reader_context as reader:
        info["topics_present"] = sorted({connection.topic for connection in reader.connections})
        topic_to_key = {value: key for key, value in TOPIC_NAMES.items() if key != "setpoint_preview"}
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in topic_to_key
        ]
        for connection, timestamp_ns, rawdata in reader.messages(connections=connections):
            key = topic_to_key[connection.topic]
            bag_time = float(timestamp_ns) * 1e-9
            message = reader.deserialize(rawdata, connection.msgtype)
            try:
                if key == "pose":
                    value = pose_position_message(message)
                elif key == "position_error":
                    value = vector3_message(message)
                elif key == "yaw_error":
                    value = float(message.data)
                elif key == "cmd_vel":
                    value = cmd_vel_message(message)
                elif key == "status":
                    value = str(message.data)
                else:  # pragma: no cover - guarded by topic_to_key
                    continue
            except (AttributeError, TypeError, ValueError) as exc:
                raise RuntimeError(
                    f"无法解析 {connection.topic} ({connection.msgtype}): {exc}"
                ) from exc
            streams[key].append((message_time(message, bag_time), value))
            info["message_counts"][key] += 1
    info["message_counts"] = dict(info["message_counts"])
    return streams, info


def clean_numeric_records(
    records: Sequence[Tuple[float, Any]], width: int
) -> Tuple[np.ndarray, np.ndarray]:
    pairs: List[Tuple[float, Tuple[float, ...]]] = []
    for timestamp, value in records:
        time_value = finite_float(timestamp)
        try:
            values = tuple(float(item) for item in value)
        except TypeError:
            values = (float(value),)
        if time_value is None or len(values) != width or not all(math.isfinite(item) for item in values):
            continue
        pairs.append((time_value, values))
    if not pairs:
        return np.empty(0, dtype=float), np.empty((0, width), dtype=float)
    pairs.sort(key=lambda item: item[0])
    times: List[float] = []
    values: List[np.ndarray] = []
    index = 0
    while index < len(pairs):
        timestamp = pairs[index][0]
        group: List[Tuple[float, ...]] = []
        while index < len(pairs) and pairs[index][0] == timestamp:
            group.append(pairs[index][1])
            index += 1
        times.append(timestamp)
        values.append(np.mean(np.asarray(group, dtype=float), axis=0))
    return np.asarray(times), np.vstack(values)


def clean_status_records(records: Sequence[Tuple[float, Any]]) -> Tuple[np.ndarray, List[str]]:
    pairs: List[Tuple[float, str]] = []
    for timestamp, value in records:
        time_value = finite_float(timestamp)
        if time_value is not None:
            pairs.append((time_value, str(value)))
    pairs.sort(key=lambda item: item[0])
    times: List[float] = []
    statuses: List[str] = []
    for timestamp, status in pairs:
        if times and timestamp == times[-1]:
            statuses[-1] = status
        else:
            times.append(timestamp)
            statuses.append(status)
    return np.asarray(times, dtype=float), statuses


def common_window(
    position_error: Tuple[np.ndarray, np.ndarray],
    yaw_error: Tuple[np.ndarray, np.ndarray],
    cmd_vel: Tuple[np.ndarray, np.ndarray],
    duration_sec: float,
) -> Tuple[float, float]:
    if duration_sec <= 0.0 or not math.isfinite(duration_sec):
        raise ValueError("duration-sec must be finite and positive")
    starts = [position_error[0][0], yaw_error[0][0], cmd_vel[0][0]]
    ends = [position_error[0][-1], yaw_error[0][-1], cmd_vel[0][-1]]
    start = max(starts)
    end = min(min(ends), start + duration_sec)
    if end <= start:
        raise RuntimeError("position_error、yaw_error、cmd_vel 没有共同有效时间窗")
    return start, end


def infer_dt(cmd_times: np.ndarray, error_times: np.ndarray, sample_rate_hz: Optional[float]) -> float:
    if sample_rate_hz is not None:
        if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
            raise ValueError("sample-rate-hz must be finite and positive")
        return 1.0 / sample_rate_hz
    differences = np.diff(cmd_times)
    differences = differences[np.isfinite(differences) & (differences > 1e-5)]
    if differences.size < 2:
        differences = np.diff(error_times)
        differences = differences[np.isfinite(differences) & (differences > 1e-5)]
    if differences.size == 0:
        raise RuntimeError("无法从 bag 推断采样间隔")
    return float(np.median(differences))


def interpolate_with_gap(
    source_times: np.ndarray,
    source_values: np.ndarray,
    target_times: np.ndarray,
    max_gap_sec: float,
) -> np.ndarray:
    if source_values.ndim == 1:
        source_values = source_values[:, None]
    output = np.full((len(target_times), source_values.shape[1]), np.nan, dtype=float)
    if len(source_times) == 0:
        return output[:, 0] if source_values.shape[1] == 1 else output
    right = np.searchsorted(source_times, target_times, side="left")
    left = right - 1
    right = np.clip(right, 0, len(source_times) - 1)
    left = np.clip(left, 0, len(source_times) - 1)
    distances = np.minimum(
        np.abs(target_times - source_times[left]),
        np.abs(target_times - source_times[right]),
    )
    valid = distances <= max_gap_sec
    for column in range(source_values.shape[1]):
        output[valid, column] = np.interp(
            target_times[valid], source_times, source_values[:, column]
        )
    return output[:, 0] if output.shape[1] == 1 else output


def status_segments(
    target_times: np.ndarray,
    status_times: np.ndarray,
    statuses: Sequence[str],
    input_valid: np.ndarray,
) -> np.ndarray:
    segments = np.full(len(target_times), "unknown", dtype=object)
    if len(status_times):
        indexes = np.searchsorted(status_times, target_times, side="right") - 1
        valid = indexes >= 0
        for index in np.flatnonzero(valid):
            text = statuses[int(indexes[index])]
            if "BRAKING_POSE_LOSS" in text:
                segments[index] = "braking"
            elif "TRACKING" in text:
                segments[index] = "tracking"
            else:
                segments[index] = "other"
    else:
        segments[input_valid] = "tracking"
    return segments


def first_order_lowpass(values: np.ndarray, times: np.ndarray, tau: float) -> np.ndarray:
    output = np.full(len(values), np.nan, dtype=float)
    state: Optional[float] = None
    previous_time: Optional[float] = None
    for index, value in enumerate(values):
        if not math.isfinite(value):
            state = None
            previous_time = None
            continue
        if state is None or previous_time is None:
            state = float(value)
        else:
            dt = float(times[index] - previous_time)
            if not math.isfinite(dt) or dt <= 0.0:
                state = float(value)
            else:
                alpha = dt / (tau + dt)
                state += alpha * (float(value) - state)
        output[index] = state
        previous_time = float(times[index])
    return output


def contiguous_runs(mask: np.ndarray) -> List[np.ndarray]:
    indexes = np.flatnonzero(mask)
    if not len(indexes):
        return []
    split_points = np.flatnonzero(np.diff(indexes) > 1) + 1
    return list(np.split(indexes, split_points))


def longest_run(mask: np.ndarray) -> np.ndarray:
    runs = contiguous_runs(mask)
    return max(runs, key=len) if runs else np.empty(0, dtype=int)


def metric_values(values: np.ndarray, mask: np.ndarray, dt: float) -> Dict[str, float]:
    indexes = np.flatnonzero(mask & np.isfinite(values))
    selected = values[indexes]
    if len(selected) == 0:
        return {
            "n": 0,
            "duration_s": math.nan,
            "mean": math.nan,
            "std": math.nan,
            "mad": math.nan,
            "p95_abs_dev": math.nan,
            "peak_to_peak": math.nan,
            "max_jump": math.nan,
        }
    median = float(np.median(selected))
    deviations = np.abs(selected - median)
    consecutive = indexes[1:] - indexes[:-1] == 1
    jumps = np.abs(np.diff(selected))[consecutive]
    return {
        "n": int(len(selected)),
        "duration_s": float(indexes[-1] - indexes[0]) * dt,
        "mean": float(np.mean(selected)),
        "std": float(np.std(selected)),
        "mad": float(np.median(deviations)),
        "p95_abs_dev": float(np.percentile(deviations, 95)),
        "peak_to_peak": float(np.max(selected) - np.min(selected)),
        "max_jump": float(np.max(jumps)) if len(jumps) else math.nan,
    }


def xcorr_delay_sec(
    raw: np.ndarray,
    filtered: np.ndarray,
    mask: np.ndarray,
    dt: float,
) -> float:
    run = longest_run(mask & np.isfinite(raw) & np.isfinite(filtered))
    if len(run) < 16:
        return math.nan
    x = np.asarray(raw[run], dtype=float)
    y = np.asarray(filtered[run], dtype=float)
    x -= np.mean(x)
    y -= np.mean(y)
    denominator = math.sqrt(float(np.dot(x, x) * np.dot(y, y)))
    if denominator <= 1e-12:
        return math.nan
    max_lag = min(len(x) // 4, max(1, int(round(5.0 / dt))))
    best_lag = 0
    best_corr = -math.inf
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            left = x[: len(x) - lag] if lag else x
            right = y[lag:] if lag else y
        else:
            left = x[-lag:]
            right = y[: len(y) + lag]
        if len(left) < 8:
            continue
        correlation = float(np.dot(left, right)) / denominator
        if correlation > best_corr:
            best_corr = correlation
            best_lag = lag
    return float(best_lag * dt)


def interval_list(times: np.ndarray, labels: np.ndarray, label: str) -> List[Tuple[float, float]]:
    mask = labels == label
    intervals: List[Tuple[float, float]] = []
    for run in contiguous_runs(mask):
        if len(run):
            intervals.append((float(times[run[0]]), float(times[run[-1]])))
    return intervals


def compact(value: float, digits: int = 6) -> str:
    if not math.isfinite(value):
        return ""
    return f"{value:.{digits}g}"


def build_summary(
    grid: np.ndarray,
    segments: np.ndarray,
    signals: Mapping[str, np.ndarray],
    raw_by_axis: Mapping[str, np.ndarray],
    filtered_by_tau: Mapping[float, Mapping[str, np.ndarray]],
    actual_by_axis: Mapping[str, np.ndarray],
    dt: float,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    rows: List[Dict[str, Any]] = []
    delay_rows: List[Dict[str, Any]] = []
    for segment in ("tracking", "braking"):
        segment_mask = segments == segment
        for axis in AXES:
            raw = raw_by_axis[axis]
            raw_mask = segment_mask & np.isfinite(raw)
            raw_metrics = metric_values(raw, raw_mask, dt)
            actual = actual_by_axis[axis]
            actual_metrics = metric_values(actual, segment_mask & np.isfinite(actual), dt)
            rows.append(
                summary_row(
                    segment,
                    axis,
                    "pure_p",
                    None,
                    raw_metrics,
                    math.nan,
                    math.nan,
                    raw_metrics["std"],
                    "raw reconstructed P command",
                )
            )
            rows.append(
                summary_row(
                    segment,
                    axis,
                    "actual_cmd",
                    None,
                    actual_metrics,
                    math.nan,
                    math.nan,
                    raw_metrics["std"],
                    "cmd_vel after controller output shaping",
                )
            )
            if segment == "tracking":
                for tau in TAUS:
                    filtered = filtered_by_tau[tau][axis]
                    filtered_metrics = metric_values(filtered, segment_mask & np.isfinite(filtered), dt)
                    centroid = float(tau)
                    xcorr = xcorr_delay_sec(raw, filtered, segment_mask, dt)
                    row = summary_row(
                        segment,
                        axis,
                        "lpf",
                        tau,
                        filtered_metrics,
                        centroid,
                        xcorr,
                        raw_metrics["std"],
                        "first-order low-pass applied to reconstructed P command",
                    )
                    rows.append(row)
                    delay_rows.append(
                        {
                            "axis": axis,
                            "tau_s": tau,
                            "delay_centroid_s": centroid,
                            "xcorr_delay_s": xcorr,
                            "t50_s": 0.693 * tau,
                            "t63_s": tau,
                            "t90_s": 2.303 * tau,
                            "jitter_reduction_pct": row["jitter_reduction_pct"],
                        }
                    )
    return rows, delay_rows


def summary_row(
    segment: str,
    axis: str,
    signal: str,
    tau: Optional[float],
    metrics: Mapping[str, Any],
    centroid: float,
    xcorr: float,
    raw_std: float,
    reference: str,
) -> Dict[str, Any]:
    std = float(metrics["std"])
    reduction = math.nan
    if math.isfinite(raw_std) and raw_std > 1e-12 and math.isfinite(std):
        reduction = (1.0 - std / raw_std) * 100.0
    return {
        "segment": segment,
        "axis": axis,
        "unit": UNITS[axis],
        "signal": signal,
        "tau_s": tau,
        "n": metrics["n"],
        "duration_s": metrics["duration_s"],
        "mean": metrics["mean"],
        "std": std,
        "mad": metrics["mad"],
        "p95_abs_dev": metrics["p95_abs_dev"],
        "peak_to_peak": metrics["peak_to_peak"],
        "max_jump": metrics["max_jump"],
        "jitter_reduction_pct": reduction,
        "delay_centroid_s": centroid,
        "xcorr_delay_s": xcorr,
        "t50_s": 0.693 * tau if tau is not None else math.nan,
        "t63_s": tau if tau is not None else math.nan,
        "t90_s": 2.303 * tau if tau is not None else math.nan,
        "reference": reference,
    }


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fieldnames: Sequence[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            serialized = {
                field: compact(float(value)) if isinstance(value, (float, np.floating)) else value
                for field, value in row.items()
            }
            writer.writerow({field: serialized.get(field, "") for field in fieldnames})


def plot_time_domain(
    output_path: Path,
    time_rel: np.ndarray,
    raw_by_axis: Mapping[str, np.ndarray],
    filtered_by_tau: Mapping[float, Mapping[str, np.ndarray]],
    actual_by_axis: Mapping[str, np.ndarray],
    segments: np.ndarray,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {0.05: "#2f6f9f", 0.10: "#d07a2d", 0.20: "#76529b", 0.30: "#2f8f83"}
    fig, axes = plt.subplots(2, 2, figsize=(15, 8.5), sharex=True)
    handles = []
    labels = []
    for axis_name, axis in zip(AXES, axes.flat):
        axis.plot(time_rel, raw_by_axis[axis_name], color="#6b7280", linewidth=0.9, linestyle="--", label="reconstructed pure P")
        for tau in TAUS:
            axis.plot(time_rel, filtered_by_tau[tau][axis_name], color=colors[tau], linewidth=0.9, label=f"LPF τ={tau:.2f}s")
        axis.plot(time_rel, actual_by_axis[axis_name], color="#111827", linewidth=1.0, label="current cmd_vel")
        for start, end in interval_list(time_rel, segments, "braking"):
            axis.axvspan(start, end, color="#d1d5db", alpha=0.28, linewidth=0)
        axis.set_title(axis_name)
        axis.set_ylabel(UNITS[axis_name])
        axis.grid(True, color="#e5e7eb", linewidth=0.6)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
        if not handles:
            handles, labels = axis.get_legend_handles_labels()
    axes[1, 0].set_xlabel("time from analysis start (s)")
    axes[1, 1].set_xlabel("time from analysis start (s)")
    fig.suptitle("VSP / YawSP time-domain comparison", fontsize=14, y=0.98)
    fig.text(0.5, 0.945, "Gray bands mark BRAKING_POSE_LOSS; commands use m/s and rad/s", ha="center", fontsize=9, color="#4b5563")
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.915), ncol=4, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    fig.savefig(output_path, dpi=160, facecolor="white")
    plt.close(fig)


def fft_amplitude(values: np.ndarray, run: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
    if len(run) < 8:
        return np.empty(0), np.empty(0)
    data = np.asarray(values[run], dtype=float)
    data -= np.mean(data)
    spectrum = np.fft.rfft(data)
    frequency = np.fft.rfftfreq(len(data), d=dt)
    amplitude = 2.0 * np.abs(spectrum) / len(data)
    if len(amplitude):
        amplitude[0] *= 0.5
    return frequency, amplitude


def plot_spectrum(
    output_path: Path,
    raw_by_axis: Mapping[str, np.ndarray],
    filtered_by_tau: Mapping[float, Mapping[str, np.ndarray]],
    tracking_mask: np.ndarray,
    dt: float,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {0.05: "#2f6f9f", 0.10: "#d07a2d", 0.20: "#76529b", 0.30: "#2f8f83"}
    fig, axes = plt.subplots(2, 2, figsize=(14, 8.5), sharex=True)
    for axis_name, axis in zip(AXES, axes.flat):
        run = longest_run(tracking_mask & np.isfinite(raw_by_axis[axis_name]))
        frequency, amplitude = fft_amplitude(raw_by_axis[axis_name], run, dt)
        axis.plot(frequency, amplitude, color="#6b7280", linewidth=0.9, linestyle="--", label="pure P")
        for tau in TAUS:
            frequency, amplitude = fft_amplitude(filtered_by_tau[tau][axis_name], run, dt)
            axis.plot(frequency, amplitude, color=colors[tau], linewidth=0.9, label=f"τ={tau:.2f}s")
        axis.set_title(axis_name)
        axis.set_ylabel(f"amplitude ({UNITS[axis_name]})")
        axis.grid(True, color="#e5e7eb", linewidth=0.6)
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
    axes[1, 0].set_xlabel("frequency (Hz)")
    axes[1, 1].set_xlabel("frequency (Hz)")
    fig.suptitle("VSP / YawSP FFT amplitude spectrum", fontsize=14, y=0.98)
    fig.text(0.5, 0.945, "Tracking-only longest contiguous run; DC component removed", ha="center", fontsize=9, color="#4b5563")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.915), ncol=5, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    fig.savefig(output_path, dpi=160, facecolor="white")
    plt.close(fig)


def plot_tradeoff(output_path: Path, delay_rows: Sequence[Mapping[str, Any]]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"vx": "#2f6f9f", "vy": "#d07a2d", "vz": "#76529b", "yawsp": "#2f8f83"}
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    for axis_name in AXES:
        rows = [row for row in delay_rows if row["axis"] == axis_name]
        x = [float(row["tau_s"]) for row in rows]
        y = [float(row["jitter_reduction_pct"]) if math.isfinite(float(row["jitter_reduction_pct"])) else math.nan for row in rows]
        axes[0].plot(x, y, marker="o", linewidth=1.4, color=colors[axis_name], label=axis_name)
    axes[0].axhline(0.0, color="#374151", linewidth=0.8)
    axes[0].set_title("jitter reduction")
    axes[0].set_xlabel("LPF τ (s)")
    axes[0].set_ylabel("1 − std(filtered) / std(pure P) (%)")
    axes[0].grid(True, color="#e5e7eb", linewidth=0.6)
    axes[0].legend(frameon=False, ncol=2)

    unique_rows = [row for row in delay_rows if row["axis"] == AXES[0]]
    x = [float(row["tau_s"]) for row in unique_rows]
    centroid = [float(row["delay_centroid_s"]) for row in unique_rows]
    xcorr_values = []
    for tau in x:
        values = [float(row["xcorr_delay_s"]) for row in delay_rows if float(row["tau_s"]) == tau and math.isfinite(float(row["xcorr_delay_s"]))]
        xcorr_values.append(float(np.mean(values)) if values else math.nan)
    axes[1].plot(x, centroid, marker="o", linewidth=1.6, color="#111827", label="impulse centroid = τ")
    axes[1].plot(x, xcorr_values, marker="x", linewidth=1.2, color="#d07a2d", label="mean cross-correlation")
    axes[1].set_title("delay trade-off")
    axes[1].set_xlabel("LPF τ (s)")
    axes[1].set_ylabel("delay (s)")
    axes[1].grid(True, color="#e5e7eb", linewidth=0.6)
    axes[1].legend(frameon=False)
    for axis in axes:
        axis.spines["top"].set_visible(False)
        axis.spines["right"].set_visible(False)
    fig.suptitle("Filter jitter / delay trade-off", fontsize=14, y=0.98)
    fig.text(0.5, 0.935, "Primary delay is the first-order impulse-response time centroid; correlation is an empirical supplement", ha="center", fontsize=9, color="#4b5563")
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    fig.savefig(output_path, dpi=160, facecolor="white")
    plt.close(fig)


def json_number(value: Any) -> Any:
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (int, np.integer)):
        return int(value)
    return value


def build_samples(
    time_rel: np.ndarray,
    segments: np.ndarray,
    errors: np.ndarray,
    yaw_error: np.ndarray,
    raw_by_axis: Mapping[str, np.ndarray],
    filtered_by_tau: Mapping[float, Mapping[str, np.ndarray]],
    actual_by_axis: Mapping[str, np.ndarray],
) -> List[Dict[str, Any]]:
    samples: List[Dict[str, Any]] = []
    for index, timestamp in enumerate(time_rel):
        row: Dict[str, Any] = {
            "t_rel_s": float(timestamp),
            "segment": str(segments[index]),
            "ex_m": float(errors[index, 0]) if math.isfinite(errors[index, 0]) else None,
            "ey_m": float(errors[index, 1]) if math.isfinite(errors[index, 1]) else None,
            "ez_m": float(errors[index, 2]) if math.isfinite(errors[index, 2]) else None,
            "yaw_error_rad": float(yaw_error[index]) if math.isfinite(yaw_error[index]) else None,
        }
        for axis_name in AXES:
            row[f"{axis_name}_pure"] = json_number(raw_by_axis[axis_name][index])
            row[f"{axis_name}_actual"] = json_number(actual_by_axis[axis_name][index])
            for tau in TAUS:
                row[f"{axis_name}_tau_{tau:.2f}"] = json_number(filtered_by_tau[tau][axis_name][index])
        samples.append(row)
    return samples


def write_html(
    output_path: Path,
    metadata: Mapping[str, Any],
    samples: Sequence[Mapping[str, Any]],
    summary: Sequence[Mapping[str, Any]],
    delay_rows: Sequence[Mapping[str, Any]],
) -> None:
    payload = {
        "metadata": metadata,
        "samples": samples,
        "summary": [
            {key: json_number(value) for key, value in row.items()}
            for row in summary
        ],
        "delay": [
            {key: json_number(value) for key, value in row.items()}
            for row in delay_rows
        ],
    }
    serialized = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    serialized = serialized.replace("</", "<\\/")
    html = r'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="color-scheme" content="light dark">
<title>VSP / YawSP jitter report</title>
<style>
:root { color-scheme: light dark; --bg:#f8fafc; --card:#ffffff; --ink:#111827; --muted:#4b5563; --line:#d1d5db; --accent:#2f6f9f; }
@media (prefers-color-scheme: dark) { :root { --bg:#111827; --card:#1f2937; --ink:#f9fafb; --muted:#cbd5e1; --line:#4b5563; --accent:#70a7d0; } }
* { box-sizing:border-box; } body { margin:0; padding:24px; font-family:system-ui,-apple-system,Segoe UI,sans-serif; background:var(--bg); color:var(--ink); }
main { max-width:1500px; margin:auto; } h1 { margin:0 0 6px; font-size:1.8rem; } h2 { margin:28px 0 8px; font-size:1.2rem; } p { color:var(--muted); line-height:1.5; }
.card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px; margin-top:14px; }
.controls { display:flex; flex-wrap:wrap; gap:12px; align-items:end; } label { display:flex; flex-direction:column; gap:4px; font-size:.9rem; color:var(--muted); } select { padding:7px 28px 7px 8px; border:1px solid var(--line); border-radius:6px; background:var(--card); color:var(--ink); }
canvas { width:100%; height:360px; display:block; } .small canvas { height:260px; } .grid { display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:14px; } @media(max-width:800px){body{padding:12px}.grid{grid-template-columns:1fr}canvas{height:300px}}
table { width:100%; border-collapse:collapse; font-size:.86rem; } th,td { text-align:right; border-bottom:1px solid var(--line); padding:7px 6px; white-space:nowrap; } th:first-child,td:first-child { text-align:left; } th { color:var(--muted); font-weight:600; }
.notice { border-left:4px solid #d07a2d; padding-left:12px; } .mono { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:.85em; }
</style>
</head>
<body>
<main>
<h1>VSP / YawSP jitter report</h1>
<p id="scope"></p>
<div class="card notice"><strong>Interpretation:</strong> static jitter statistics use TRACKING samples only. BRAKING_POSE_LOSS samples remain visible and are reported separately. The primary filter delay is the impulse-response centroid, equal to τ for this first-order discretization.</div>
<h2>Interactive time domain</h2>
<div class="card controls">
  <label>axis<select id="axis"></select></label>
  <label>filter tau<select id="tau"></select></label>
</div>
<div class="card"><canvas id="time"></canvas></div>
<h2>Filter trade-off for selected axis</h2>
<div class="card small"><canvas id="tradeoff"></canvas></div>
<h2>Selected statistics</h2>
<div class="card"><div id="stats"></div></div>
<h2>Data-quality and run notes</h2>
<div class="card"><p id="notes"></p></div>
</main>
<script>
const REPORT = __REPORT_PAYLOAD__;
const AXES = ['vx','vy','vz','yawsp'];
const TAUS = [0.05,0.10,0.20,0.30];
const COLORS = {pure:'#6b7280',actual:'#111827','0.05':'#2f6f9f','0.10':'#d07a2d','0.20':'#76529b','0.30':'#2f8f83'};
const axisSelect = document.getElementById('axis'); const tauSelect = document.getElementById('tau');
AXES.forEach(axis => axisSelect.add(new Option(axis,axis))); TAUS.forEach(tau => tauSelect.add(new Option(`τ=${tau.toFixed(2)}s`,tau.toFixed(2))));
axisSelect.value='vx'; tauSelect.value='0.10';
document.getElementById('scope').textContent = `Analysis window: ${REPORT.metadata.analysis_duration_s.toFixed(2)} s; common grid: ${REPORT.metadata.sample_rate_hz.toFixed(2)} Hz; bag: ${REPORT.metadata.bag_name}`;
document.getElementById('notes').innerHTML = `Input topics: <span class="mono">${REPORT.metadata.topics.join(', ')}</span><br>Message counts: <span class="mono">${JSON.stringify(REPORT.metadata.message_counts)}</span><br>Observed marker from position error: mean body position ≈ (${REPORT.metadata.observed_body_position_mean_m.map(v=>v.toFixed(3)).join(', ')}) m; this is a diagnostic check against the requested 0.8 m / centered setup.<br>Caveats: ${REPORT.metadata.caveats.join(' ')}`;

function setupCanvas(canvas) { const dpr=window.devicePixelRatio||1, box=canvas.getBoundingClientRect(); canvas.width=Math.max(1,Math.round(box.width*dpr)); canvas.height=Math.max(1,Math.round(box.height*dpr)); const c=canvas.getContext('2d'); c.setTransform(dpr,0,0,dpr,0,0); return [c,box.width,box.height]; }
function extent(vals) { const finite=vals.filter(v=>Number.isFinite(v)); if(!finite.length)return[-1,1]; let lo=Math.min(...finite),hi=Math.max(...finite); if(lo===hi){const p=Math.max(.001,Math.abs(lo)*.1);return[lo-p,hi+p]} const p=(hi-lo)*.08;return[lo-p,hi+p]; }
function labelNumber(x){if(!Number.isFinite(x))return'—'; return Math.abs(x)>=1?x.toFixed(2):x.toPrecision(3)}
function lineChart(canvas, series, xTitle, yTitle, xMax, shadeRuns) {
  const [ctx,w,h]=setupCanvas(canvas); const left=58,right=18,top=24,bottom=42; ctx.clearRect(0,0,w,h); ctx.font='12px system-ui';
  const xs=series.flatMap(s=>s.x), ys=series.flatMap(s=>s.y); const [y0,y1]=extent(ys); const x0=0,x1=xMax;
  const X=x=>left+(x-x0)/(x1-x0)*(w-left-right), Y=y=>h-bottom-(y-y0)/(y1-y0)*(h-top-bottom);
  ctx.strokeStyle=getComputedStyle(document.documentElement).getPropertyValue('--line'); ctx.lineWidth=1; ctx.beginPath(); ctx.moveTo(left,top);ctx.lineTo(left,h-bottom);ctx.lineTo(w-right,h-bottom);ctx.stroke();
  if(shadeRuns){ctx.fillStyle='rgba(156,163,175,.18)';shadeRuns.forEach(run=>{ctx.fillRect(X(run[0]),top,Math.max(1,X(run[1])-X(run[0])),h-top-bottom)})}
  ctx.fillStyle=getComputedStyle(document.documentElement).getPropertyValue('--muted'); ctx.textAlign='right'; ctx.fillText(labelNumber(y1),left-7,top+4);ctx.fillText(labelNumber(y0),left-7,h-bottom+4);ctx.textAlign='center';ctx.fillText('0',left,h-bottom+18);ctx.fillText(`${x1.toFixed(1)}s`,w-right,h-bottom+18);ctx.save();ctx.translate(15,(h-top-bottom)/2+top);ctx.rotate(-Math.PI/2);ctx.fillText(yTitle,0,0);ctx.restore();ctx.fillText(xTitle,(left+w-right)/2,h-8);
  series.forEach(s=>{ctx.strokeStyle=s.color;ctx.lineWidth=s.width||1.4;ctx.setLineDash(s.dash||[]);let open=false;ctx.beginPath();s.x.forEach((x,i)=>{const y=s.y[i];if(!Number.isFinite(y)){open=false;return}if(!open){ctx.moveTo(X(x),Y(y));open=true}else ctx.lineTo(X(x),Y(y))});ctx.stroke();ctx.setLineDash([])});
  let lx=left; const ly=top-8; ctx.textAlign='left'; series.forEach(s=>{ctx.fillStyle=s.color;ctx.fillRect(lx,ly-8,18,2);ctx.fillStyle=getComputedStyle(document.documentElement).getPropertyValue('--ink');ctx.fillText(s.label,lx+23,ly);lx+=ctx.measureText(s.label).width+42});
}
function drawTime(){const axis=axisSelect.value,tau=tauSelect.value;const rows=REPORT.samples;const x=rows.map(r=>r.t_rel_s);const pure=rows.map(r=>r[`${axis}_pure`]);const filt=rows.map(r=>r[`${axis}_tau_${tau}`]);const actual=rows.map(r=>r[`${axis}_actual`]);const shade=[];let start=null;rows.forEach((r,i)=>{if(r.segment==='braking'&&start===null)start=r.t_rel_s;if(r.segment!=='braking'&&start!==null){shade.push([start,x[i-1]]);start=null}});if(start!==null)shade.push([start,x[x.length-1]]);lineChart(document.getElementById('time'),[{x,y:pure,label:'pure P',color:COLORS.pure,dash:[5,3]},{x,y:filt,label:`LPF τ=${tau}s`,color:COLORS[tau]},{x,y:actual,label:'current cmd_vel',color:COLORS.actual,width:1.6}],`time (s)`,REPORT.metadata.units[axis],x[x.length-1],shade);updateStats(axis,tau)}
function updateStats(axis,tau){const rows=REPORT.summary.filter(r=>r.axis===axis&&(r.segment==='tracking'||r.segment==='braking')&&(r.signal==='pure_p'||r.signal==='actual_cmd'||(r.signal==='lpf'&&r.segment==='tracking'&&Number(r.tau_s)===Number(tau))));const cols=['segment','signal','tau_s','n','mean','std','mad','p95_abs_dev','peak_to_peak','max_jump','jitter_reduction_pct','delay_centroid_s','xcorr_delay_s','t50_s','t63_s','t90_s'];let html='<table><thead><tr>'+cols.map(c=>`<th>${c}</th>`).join('')+'</tr></thead><tbody>';rows.forEach(r=>{html+='<tr>'+cols.map(c=>`<td>${r[c]===null||r[c]===undefined?'—':(typeof r[c]==='number'?labelNumber(r[c]):r[c])}</td>`).join('')+'</tr>'});html+='</tbody></table>';document.getElementById('stats').innerHTML=html}
function drawTradeoff(){const axis=axisSelect.value;const rows=REPORT.delay.filter(r=>r.axis===axis);const x=rows.map(r=>r.tau_s);const reduction=rows.map(r=>r.jitter_reduction_pct);const centroid=rows.map(r=>r.delay_centroid_s);lineChart(document.getElementById('tradeoff'),[{x,y:reduction,label:'jitter reduction (%)',color:'#2f6f9f'},{x,y:centroid.map(v=>v*100),label:'centroid delay (×100)',color:'#111827'}],'tau (s)','percent / delay ×100',Math.max(...x)*1.08,[])}
axisSelect.addEventListener('change',()=>{drawTime();drawTradeoff()});tauSelect.addEventListener('change',drawTime);window.addEventListener('resize',()=>{drawTime();drawTradeoff()});drawTime();drawTradeoff();
</script>
</body>
</html>
'''
    output_path.write_text(html.replace("__REPORT_PAYLOAD__", serialized), encoding="utf-8")


def main() -> int:
    args = parse_args()
    bag_path = args.bag.expanduser().resolve()
    if not bag_path.is_file():
        raise RuntimeError(f"bag 不存在: {bag_path}")
    output_dir = (args.output_dir or bag_path.parent).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.max_gap_sec <= 0.0 or not math.isfinite(args.max_gap_sec):
        raise ValueError("max-gap-sec must be finite and positive")

    streams, bag_info = read_bag(bag_path)
    pose_times, pose_values = clean_numeric_records(streams["pose"], 3)
    error_times, error_values = clean_numeric_records(streams["position_error"], 3)
    yaw_times, yaw_values = clean_numeric_records(streams["yaw_error"], 1)
    cmd_times, cmd_values = clean_numeric_records(streams["cmd_vel"], 4)
    status_times, statuses = clean_status_records(streams["status"])
    for name, times in (("position_error", error_times), ("yaw_error", yaw_times), ("cmd_vel", cmd_times)):
        if len(times) < 2:
            raise RuntimeError(f"{name} 有效消息不足，无法分析")

    start, end = common_window((error_times, error_values), (yaw_times, yaw_values), (cmd_times, cmd_values), args.duration_sec)
    all_data_times = np.concatenate((pose_times, error_times, yaw_times, cmd_times, status_times))
    bag_data_span = float(np.max(all_data_times) - np.min(all_data_times)) if len(all_data_times) else math.nan
    dt = infer_dt(cmd_times[(cmd_times >= start) & (cmd_times <= end)], error_times[(error_times >= start) & (error_times <= end)], args.sample_rate_hz)
    grid = np.arange(start, end + 0.5 * dt, dt, dtype=float)
    error_grid = interpolate_with_gap(error_times, error_values, grid, args.max_gap_sec)
    yaw_grid = interpolate_with_gap(yaw_times, yaw_values, grid, args.max_gap_sec)
    cmd_grid = interpolate_with_gap(cmd_times, cmd_values, grid, args.max_gap_sec)
    if error_grid.ndim == 1:
        error_grid = error_grid[:, None]
    if cmd_grid.ndim == 1:
        cmd_grid = cmd_grid[:, None]
    yaw_grid = np.asarray(yaw_grid, dtype=float).reshape(-1)

    raw_matrix = np.column_stack((0.50 * error_grid[:, 0], 0.60 * error_grid[:, 1], 0.60 * error_grid[:, 2], 0.40 * yaw_grid))
    raw_by_axis = {axis: raw_matrix[:, index] for index, axis in enumerate(AXES)}
    actual_by_axis = {axis: cmd_grid[:, index] for index, axis in enumerate(AXES)}
    input_valid = np.all(np.isfinite(raw_matrix), axis=1)
    segments = status_segments(grid, status_times, statuses, input_valid)
    filtered_by_tau: Dict[float, Dict[str, np.ndarray]] = {}
    for tau in TAUS:
        filtered_by_tau[tau] = {
            axis: first_order_lowpass(raw_by_axis[axis], grid, tau) for axis in AXES
        }

    summary, delay_rows = build_summary(grid, segments, {}, raw_by_axis, filtered_by_tau, actual_by_axis, dt)
    time_rel = grid - start
    tracking_mask = segments == "tracking"
    observed_body_position_mean = [
        0.8 + float(np.nanmean(error_grid[:, 0])) if np.isfinite(error_grid[:, 0]).any() else math.nan,
        float(np.nanmean(error_grid[:, 1])) if np.isfinite(error_grid[:, 1]).any() else math.nan,
        float(np.nanmean(error_grid[:, 2])) if np.isfinite(error_grid[:, 2]).any() else math.nan,
    ]
    observed_body_position_std = [
        float(np.nanstd(error_grid[:, index])) if np.isfinite(error_grid[:, index]).any() else math.nan
        for index in range(3)
    ]
    counts = Counter(str(item) for item in segments)
    if math.isfinite(bag_data_span) and bag_data_span > args.duration_sec + 0.5:
        duration_caveat = "The bag contains more than the requested duration; only the common first requested seconds are analyzed."
    elif math.isfinite(bag_data_span) and bag_data_span < args.duration_sec - 0.5:
        duration_caveat = "The bag is shorter than the requested duration; all available common data are analyzed."
    else:
        duration_caveat = "The bag duration is within the requested window; analysis uses the common valid interval across input and command topics."
    metadata: Dict[str, Any] = {
        "bag_name": bag_path.name,
        "bag_path": str(bag_path),
        "analysis_start_epoch_s": start,
        "analysis_end_epoch_s": float(grid[-1]),
        "analysis_duration_s": float(grid[-1] - grid[0]),
        "bag_data_span_s": bag_data_span,
        "sample_rate_hz": 1.0 / dt,
        "dt_s": dt,
        "duration_requested_s": args.duration_sec,
        "max_gap_sec": args.max_gap_sec,
        "topics": list(TOPIC_NAMES.values()),
        "topics_present": bag_info["topics_present"],
        "message_counts": bag_info["message_counts"],
        "segment_counts": dict(counts),
        "observed_body_position_mean_m": observed_body_position_mean,
        "observed_body_position_std_m": observed_body_position_std,
        "target_body_position_m": [0.8, 0.0, 0.0],
        "gains": {"vx_per_ex_m": 0.50, "vy_per_ey_m": 0.60, "vz_per_ez_m": 0.60, "yawsp_per_yaw_error_rad": 0.40},
        "filter_taus_s": list(TAUS),
        "actual_controller_context": {"yaw_lpf_tau_s": 0.10, "max_accel_mps2": 0.50, "velocity_deadband_enter_mps": 0.02, "velocity_deadband_exit_mps": 0.03},
        "definitions": {"p95_abs_dev": "95th percentile of absolute deviation from the signal median", "jitter_reduction_pct": "100 * (1 - std(filtered or actual) / std(reconstructed pure P))", "delay_centroid": "time centroid of the first-order filter impulse response; equals tau"},
        "caveats": [duration_caveat, "The observed marker position is diagnosed from the position-error topic and should be checked against the requested 0.8 m centered setup.", "cmd_vel is the current controller output; no real MAVROS setpoint was enabled."],
    }

    summary_fields = ["segment", "axis", "unit", "signal", "tau_s", "n", "duration_s", "mean", "std", "mad", "p95_abs_dev", "peak_to_peak", "max_jump", "jitter_reduction_pct", "delay_centroid_s", "xcorr_delay_s", "t50_s", "t63_s", "t90_s", "reference"]
    write_csv(output_dir / "jitter_summary.csv", summary, summary_fields)
    sample_rows = build_samples(time_rel, segments, error_grid, yaw_grid, raw_by_axis, filtered_by_tau, actual_by_axis)
    sample_fields = list(sample_rows[0].keys()) if sample_rows else []
    write_csv(output_dir / "filtered_samples.csv", sample_rows, sample_fields)
    (output_dir / "analysis_metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")

    plot_time_domain(output_dir / "vsp_yawsp_time_domain.png", time_rel, raw_by_axis, filtered_by_tau, actual_by_axis, segments)
    plot_spectrum(output_dir / "vsp_yawsp_spectrum.png", raw_by_axis, filtered_by_tau, tracking_mask, dt)
    plot_tradeoff(output_dir / "filter_jitter_delay_tradeoff.png", delay_rows)
    write_html(output_dir / "jitter_report.html", metadata, sample_rows, summary, delay_rows)

    print(f"analysis_dir={output_dir}")
    print(f"analysis_window_s={metadata['analysis_duration_s']:.3f}")
    print(f"sample_rate_hz={metadata['sample_rate_hz']:.3f}")
    print(f"segment_counts={dict(counts)}")
    print(f"observed_body_position_mean_m={','.join(compact(v, 5) for v in observed_body_position_mean)}")
    print("outputs=vsp_yawsp_time_domain.png,vsp_yawsp_spectrum.png,filter_jitter_delay_tradeoff.png,jitter_summary.csv,filtered_samples.csv,jitter_report.html")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
