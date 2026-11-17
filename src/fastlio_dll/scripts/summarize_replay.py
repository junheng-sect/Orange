#!/usr/bin/env python3
"""Export trajectories and summarize one DLL replay, using measurement stamps."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

import numpy as np
import rosbag


def pose_row(message):
    pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def xyz_rotation(quaternion):
    x, y, z, w = np.asarray(quaternion) / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def summarize(output, verify_observation=False):
    observation = (output / "observation.bag").is_file()
    bag_file = output / ("observation.bag" if observation else "result.bag")
    local, global_poses, transforms, candidates, legacy = {}, {}, {}, {}, {}
    local_received = []
    states = Counter()
    reasons = Counter()
    tf_publishers = Counter()
    wrong_seed = old_seed = valid_seed = invalidated = None
    final = {}
    topics = ["/fastlio/odom_world", "/dll_localization/pose",
              "/dll_localization/correction", "/dll_localization/status"]
    if observation:
        topics += ["/drone_reloc/pose", "/drone_reloc/result", "/tf"]
    with rosbag.Bag(str(bag_file)) as bag:
        topic_counts = {name: info.message_count for name, info in bag.get_type_and_topic_info().topics.items()}
        for topic, msg, received, connection in bag.read_messages(topics=topics, return_connection_header=True):
            if topic == "/tf":
                for edge in msg.transforms:
                    if (edge.header.frame_id, edge.child_frame_id) == ("drone_map", "drone_odom"):
                        publisher = connection.get("callerid", "<unknown>")
                        if isinstance(publisher, bytes):
                            publisher = publisher.decode("utf-8", errors="replace")
                        tf_publishers[publisher] += 1
                continue
            if topic == "/drone_reloc/result":
                if verify_observation:
                    result = json.loads(msg.data)
                    stamp = round(result["stamp"] * 1e9)
                    if result["confirmed"]:
                        if result["map"].endswith(".wrong"):
                            wrong_seed = stamp
                        elif wrong_seed is not None and stamp < wrong_seed:
                            old_seed = stamp
                        else:
                            valid_seed = stamp
                    else:
                        invalidated = stamp
                continue
            stamp = msg.header.stamp.to_nsec()
            if topic == "/fastlio/odom_world":
                if stamp in local:
                    raise ValueError("duplicate local odometry stamp")
                local[stamp] = pose_row(msg)
                local_received.append(received.to_nsec())
            elif topic == "/dll_localization/pose":
                global_poses[stamp] = pose_row(msg)
            elif topic == "/drone_reloc/pose":
                legacy[stamp] = pose_row(msg)
            elif topic == "/dll_localization/correction":
                t, q = msg.transform.translation, msg.transform.rotation
                transforms[stamp] = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
            elif topic == "/dll_localization/status":
                status = msg.status[0]
                fields = {item.key: item.value for item in status.values}
                candidate = int(fields["candidate_stamp_ns"])
                states[status.message] += 1
                reasons[fields["reason"]] += 1
                if candidate and candidate not in candidates:
                    candidates[candidate] = {"state": status.message, **fields}
                final = fields
    if not local:
        raise ValueError("bag contains no local odometry")
    unpaired_dll = (set(global_poses) ^ set(transforms)) | (set(global_poses) - set(local))
    if not observation and unpaired_dll:
        raise ValueError("recorded output stamps did not match the local input and correction")

    intervals = np.diff(np.asarray(list(local), dtype=np.int64)) / 1e9
    if intervals.size == 0 or np.any(intervals <= 0):
        raise ValueError("local odometry stamps are not increasing")
    received_gaps = np.diff(np.asarray(local_received, dtype=np.int64)) / 1e6
    start = min(local)
    pose_keys = ["x", "y", "z", "qx", "qy", "qz", "qw"]
    fixed = transforms[min(transforms)] if transforms else None
    fixed_rot = xyz_rotation(fixed[3:]) if fixed else None
    deltas, legacy_deltas, legacy_yaw = [], [], []
    with (output / "trajectory.csv").open("w", newline="") as out:
        columns = ["stamp_ns", "elapsed_s"]
        columns += [f"{frame}_{key}" for frame in ("odom", "dll_map") for key in pose_keys]
        columns += ["fixed_map_x", "fixed_map_y", "fixed_map_z"]
        if observation:
            columns += [f"legacy_map_{key}" for key in pose_keys]
            columns += ["dll_minus_legacy_x_m", "dll_minus_legacy_y_m",
                        "dll_minus_legacy_z_m", "dll_minus_legacy_yaw_deg"]
        writer = csv.writer(out)
        writer.writerow(columns)
        for stamp, values in local.items():
            fixed_pos = fixed_rot @ values[:3] + fixed[:3] if fixed else [None] * 3
            corrected = global_poses.get(stamp)
            if fixed and corrected:
                deltas.append(float(np.linalg.norm(np.asarray(corrected[:3]) - fixed_pos)))
            row = [stamp, (stamp - start) / 1e9, *values, *(corrected or [None] * 7), *fixed_pos]
            if observation:
                original = legacy.get(stamp)
                difference = [None] * 4
                if original and corrected:
                    delta = np.asarray(corrected[:3]) - original[:3]
                    dyaw = yaw_of(corrected[3:]) - yaw_of(original[3:])
                    angle = math.degrees(math.atan2(math.sin(dyaw), math.cos(dyaw)))
                    legacy_deltas.append(delta)
                    legacy_yaw.append(angle)
                    difference = [*delta, angle]
                row.extend([*(original or [None] * 7), *difference])
            writer.writerow(row)
    with (output / "candidates.csv").open("w", newline="") as out:
        columns = next(iter(candidates.values())).keys() if candidates else ["state", "reason"]
        writer = csv.DictWriter(out, fieldnames=columns)
        writer.writeheader()
        writer.writerows(candidates.values())

    solve_ms = [float(row["solve_ms"]) for row in candidates.values() if float(row["solve_ms"])]
    accepted = [row for row in candidates.values() if row["reason"] == "accepted"]
    def percentile(values, q):
        return float(np.percentile(values, q)) if values else None

    report = {
        "local_frames": len(local),
        "map_frames": len(global_poses),
        "local_hz": float(1 / np.median(intervals)),
        "local_mean_hz": float(1 / np.mean(intervals)),
        "local_gap_p95_ms": float(1000 * np.percentile(intervals, 95)),
        "local_gap_p99_ms": float(1000 * np.percentile(intervals, 99)),
        "local_gaps_above_150_ms": int(np.count_nonzero(intervals > 0.150)),
        "local_max_gap_ms": float(1000 * intervals.max()),
        "local_receive_gap_p95_ms": float(np.percentile(received_gaps, 95)),
        "local_receive_max_gap_ms": float(received_gaps.max()),
        "local_receive_gaps_above_150_ms": int(np.count_nonzero(received_gaps > 150)),
        "local_duration_s": (max(local) - start) / 1e9,
        "first_global_pose_s": (min(global_poses) - start) / 1e9 if global_poses else None,
        "candidate_count": len(candidates),
        "accepted_count": int(final.get("accepted_count", 0)),
        "reported_rejected_count": int(final.get("rejected_count", 0)),
        "rejection_reasons": dict(Counter(row["reason"] for row in candidates.values()
                                         if row["reason"] not in ("accepted", "confirming"))),
        "status_messages_by_state": dict(states),
        "solve_p50_ms": percentile(solve_ms, 50),
        "solve_p95_ms": percentile(solve_ms, 95),
        "accepted_distance_median_p50_m": percentile(
            [float(row["distance_median_m"]) for row in accepted], 50),
        "accepted_coverage_p50": percentile([float(row["coverage"]) for row in accepted], 50),
        "max_difference_from_fixed_correction_m": max(deltas, default=None),
        "last_difference_from_fixed_correction_m": deltas[-1] if deltas else None,
        "field_bytes": int(final["field_bytes"]) if "field_bytes" in final else None,
        "field_build_ms": float(final["field_build_ms"]) if "field_build_ms" in final else None,
        "dll_peak_rss_kib": (int((output / "dll_peak_rss_kib.txt").read_text())
                             if (output / "dll_peak_rss_kib.txt").is_file() else None),
    }
    modes = {}
    for row in candidates.values():
        ms = float(row["solve_ms"])
        if ms > 0:
            modes.setdefault(row.get("registration_mode", "unspecified"), []).append(ms)
    report["solve_ms_by_mode"] = {
        mode: {"count": len(values), "p50": percentile(values, 50),
               "p95": percentile(values, 95), "p99": percentile(values, 99),
               "max": max(values)}
        for mode, values in modes.items()
    }
    if observation:
        delta_array = np.asarray(legacy_deltas)
        report.update(
            observation=True, paired_with_legacy_frames=len(legacy_deltas),
            unpaired_dll_stamps=len(unpaired_dll),
            map_odom_tf_publishers=dict(tf_publishers), diagnostic_reasons=dict(reasons),
            topic_counts=topic_counts,
            xyz_signed_median_m=(np.median(delta_array, axis=0).tolist() if legacy_deltas else None),
            xyz_abs_p95_m=(np.percentile(np.abs(delta_array), 95, axis=0).tolist() if legacy_deltas else None),
            position_difference_max_m=(float(np.linalg.norm(delta_array, axis=1).max()) if legacy_deltas else None),
            heading_difference_abs_p95_deg=percentile(np.abs(legacy_yaw).tolist(), 95),
            heading_difference_abs_max_deg=max(map(abs, legacy_yaw), default=None),
        )
    checks = {}
    if verify_observation:
        checks = {
            "wrong_map_rejected": wrong_seed is not None and reasons["legacy_map_mismatch"] > 0,
            "old_epoch_rejected": old_seed is not None and reasons["legacy_result_before_epoch"] > 0,
            "fresh_seed_required": bool(global_poses) and valid_seed is not None and min(global_poses) > valid_seed,
            "no_pose_after_invalid": bool(global_poses) and invalidated is not None
                and max(global_poses) <= invalidated + 200_000_000 and max(local) > invalidated + 2_000_000_000,
            "legacy_is_only_map_odom_authority": set(tf_publishers) == {"/dll_replay_legacy_seed"},
            "paired_pose_recorded": len(legacy_deltas) > 100 and len(legacy_deltas) >= 0.99 * len(global_poses),
            "dll_output_stamps_consistent": not unpaired_dll,
            "local_near_10hz": 9.5 <= report["local_hz"] <= 10.5 and report["local_max_gap_ms"] < 200,
            "raw_data_recorded": topic_counts.get("/livox/lidar", 0) >= len(local)
                and topic_counts.get("/livox/imu", 0) > 10 * len(local),
            "no_flight_controller_topics": not any(name.startswith("/mavros/") for name in topic_counts),
        }
        report["checks"] = checks
    serialized = json.dumps(report, indent=2)
    (output / "summary.json").write_text(serialized + "\n")
    print(serialized)
    return bool(global_poses) and all(checks.values())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--verify-observation-replay", action="store_true")
    args = parser.parse_args()
    raise SystemExit(0 if summarize(args.output, args.verify_observation_replay) else 1)
