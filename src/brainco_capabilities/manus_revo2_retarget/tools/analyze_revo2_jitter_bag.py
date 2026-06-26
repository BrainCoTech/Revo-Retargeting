#!/usr/bin/env python3
import argparse
import json
from collections import defaultdict

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


JOINT_SUFFIXES = (
    "thumb_proximal_joint",
    "thumb_metacarpal_joint",
    "index_proximal_joint",
    "middle_proximal_joint",
    "ring_proximal_joint",
    "pinky_proximal_joint",
)
JOINT_SHORT_NAMES = (
    "thumb_prox",
    "thumb_meta",
    "index",
    "middle",
    "ring",
    "pinky",
)
DEFAULT_ERGONOMICS = (
    "ThumbMCPStretch",
    "ThumbMCPSpread",
    "ThumbPIPStretch",
    "ThumbDIPStretch",
    "IndexMCPStretch",
    "IndexPIPStretch",
    "MiddleMCPStretch",
    "RingMCPStretch",
    "PinkyMCPStretch",
)


def percentile(values, q):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return float("nan")
    return float(np.percentile(values, q))


def joint_names_for_side(side):
    return tuple(f"{side}_{suffix}" for suffix in JOINT_SUFFIXES)


def vector_from_joint_state(msg, joint_names, field="position"):
    values_by_name = getattr(msg, field)
    if len(values_by_name) < len(msg.name):
        return None
    index_by_name = {name: i for i, name in enumerate(msg.name)}
    values = []
    for name in joint_names:
        idx = index_by_name.get(name)
        if idx is None or idx >= len(values_by_name):
            return None
        values.append(float(values_by_name[idx]))
    return np.asarray(values, dtype=float)


def parse_segments(value):
    segments = []
    if not value:
        return segments
    for item in value.split(","):
        item = item.strip()
        if not item:
            continue
        parts = item.split(":")
        if len(parts) < 2:
            raise ValueError(f"bad segment '{item}', expected start:end[:name]")
        start = float(parts[0])
        end = float(parts[1])
        name = parts[2] if len(parts) >= 3 and parts[2] else f"{start:g}-{end:g}s"
        if end <= start:
            raise ValueError(f"bad segment '{item}', end must be greater than start")
        segments.append((start, end, name))
    return segments


def load_bag(args):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id=args.storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {}
    for topic, type_name in topic_types.items():
        try:
            msg_types[topic] = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            print(f"[WARN] skip {topic}: cannot load message type {type_name}: {exc}")

    joint_names = joint_names_for_side(args.side)
    target_topic = args.target_topic or f"/revo2_{args.side}/revo2_pid_controller/target_joint_states"
    actual_topic = args.actual_topic or f"/revo2_{args.side}/revo2_joint_state/joint_states"
    glove_topic = args.glove_topic or ("/manus_glove_1" if args.side == "right" else "/manus_glove_0")

    data = {
        "topic_types": topic_types,
        "stamps": defaultdict(list),
        "target": [],
        "actual": [],
        "actual_velocity": [],
        "ergonomics": defaultdict(list),
        "json_errors": defaultdict(int),
        "topics": {
            "target": target_topic,
            "actual": actual_topic,
            "glove": glove_topic,
            "raw_angles": args.raw_angles_topic,
            "raw_positions": args.raw_positions_topic,
        },
    }

    first_time = None
    last_time = None
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        stamp = stamp_ns * 1e-9
        if first_time is None:
            first_time = stamp
        last_time = stamp
        rel = stamp - first_time
        data["stamps"][topic].append(rel)

        msg_type = msg_types.get(topic)
        if msg_type is None:
            continue
        try:
            msg = deserialize_message(serialized, msg_type)
        except Exception as exc:
            print(f"[WARN] failed to deserialize {topic}: {exc}")
            continue

        if topic == target_topic:
            position = vector_from_joint_state(msg, joint_names, field="position")
            if position is not None:
                data["target"].append((rel, position))
        elif topic == actual_topic:
            position = vector_from_joint_state(msg, joint_names, field="position")
            if position is not None:
                data["actual"].append((rel, position))
            velocity = vector_from_joint_state(msg, joint_names, field="velocity")
            if velocity is not None:
                data["actual_velocity"].append((rel, velocity))
        elif topic == glove_topic:
            for ergo in getattr(msg, "ergonomics", []):
                data["ergonomics"][str(ergo.type)].append((rel, float(ergo.value)))
        elif topic in (args.raw_angles_topic, args.raw_positions_topic):
            try:
                json.loads(msg.data)
            except Exception:
                data["json_errors"][topic] += 1

    data["duration"] = 0.0 if first_time is None or last_time is None else last_time - first_time
    return data


def print_timing(data, args):
    print("\n== Topic Timing ==")
    for topic in (
        data["topics"]["raw_angles"],
        data["topics"]["raw_positions"],
        data["topics"]["glove"],
        data["topics"]["target"],
        data["topics"]["actual"],
    ):
        stamps = np.asarray(data["stamps"].get(topic, []), dtype=float)
        if stamps.size < 2:
            print(f"{topic}: no/insufficient data")
            continue
        dt = np.diff(stamps)
        median = np.median(dt)
        hz = (stamps.size - 1) / max(1e-9, stamps[-1] - stamps[0])
        print(
            f"{topic:62s} count={stamps.size:5d} hz={hz:7.2f} "
            f"dt_ms med={percentile(dt * 1000, 50):6.2f} "
            f"p95={percentile(dt * 1000, 95):6.2f} "
            f"p99={percentile(dt * 1000, 99):6.2f} "
            f"max={float(np.max(dt) * 1000):7.2f} "
            f"gaps>{args.gap_ms:g}ms={int(np.sum(dt * 1000 > args.gap_ms)):4d} "
            f"gaps>2xmed={int(np.sum(dt > 2.0 * median)):4d}"
        )


def find_gap_events(stamps, gap_ms):
    stamps = np.asarray(stamps, dtype=float)
    if stamps.size < 2:
        return []
    dt = np.diff(stamps)
    events = []
    for idx, gap in enumerate(dt):
        if gap * 1000.0 >= gap_ms:
            events.append((float(stamps[idx]), float(stamps[idx + 1]), float(gap)))
    return events


def nearest_previous_gap_time(event_t, gap_events, window_s):
    best = None
    for start, end, gap in gap_events:
        if end <= event_t and event_t - end <= window_s:
            if best is None or end > best[1]:
                best = (start, end, gap)
    return best


def print_target_jumps(data, args):
    print("\n== Target Position Jumps ==")
    target = data["target"]
    if len(target) < 2:
        print("No target data.")
        return

    raw_angle_gaps = find_gap_events(data["stamps"].get(data["topics"]["raw_angles"], []), args.gap_ms)
    raw_position_gaps = find_gap_events(
        data["stamps"].get(data["topics"]["raw_positions"], []), args.gap_ms
    )
    glove_gaps = find_gap_events(data["stamps"].get(data["topics"]["glove"], []), args.gap_ms)

    times = np.asarray([t for t, _ in target], dtype=float)
    values = np.asarray([v for _, v in target], dtype=float)
    jumps = np.abs(np.diff(values, axis=0))
    any_big = False

    for joint_index, joint_name in enumerate(JOINT_SHORT_NAMES):
        joint_jumps = jumps[:, joint_index]
        big_indices = np.flatnonzero(joint_jumps >= args.jump_rad)
        print(
            f"{joint_name:10s} range=[{np.min(values[:, joint_index]):6.3f},"
            f"{np.max(values[:, joint_index]):6.3f}] "
            f"abs_dv_p95={percentile(joint_jumps, 95):7.4f} "
            f"p99={percentile(joint_jumps, 99):7.4f} "
            f"max={float(np.max(joint_jumps)):7.4f} "
            f"jumps>{args.jump_rad:g}rad={big_indices.size}"
        )
        if big_indices.size:
            any_big = True
            top = big_indices[np.argsort(joint_jumps[big_indices])[-args.top:]][::-1]
            for idx in top:
                event_t = times[idx + 1]
                related = []
                for label, gaps in (
                    ("raw_angles", raw_angle_gaps),
                    ("raw_positions", raw_position_gaps),
                    ("manus_glove", glove_gaps),
                ):
                    gap = nearest_previous_gap_time(event_t, gaps, args.correlation_window)
                    if gap is not None:
                        related.append(
                            f"{label} gap {gap[2] * 1000:.0f}ms ended {event_t - gap[1]:.3f}s before"
                        )
                related_text = "; ".join(related) if related else "no nearby input gap"
                print(
                    f"  t={event_t:8.3f}s jump={joint_jumps[idx]:7.4f}rad "
                    f"({np.degrees(joint_jumps[idx]):5.1f}deg) "
                    f"{values[idx, joint_index]:.3f}->{values[idx + 1, joint_index]:.3f}; "
                    f"{related_text}"
                )

    if not any_big:
        print(f"No target jumps larger than {args.jump_rad:g} rad.")


def print_tracking_error(data):
    print("\n== Target vs Actual Error ==")
    target = data["target"]
    actual = data["actual"]
    if len(target) < 2 or len(actual) < 2:
        print("No/insufficient target or actual data.")
        return
    target_t = np.asarray([t for t, _ in target], dtype=float)
    target_v = np.asarray([v for _, v in target], dtype=float)
    actual_t = np.asarray([t for t, _ in actual], dtype=float)
    actual_v = np.asarray([v for _, v in actual], dtype=float)
    low = max(float(target_t[0]), float(actual_t[0]))
    high = min(float(target_t[-1]), float(actual_t[-1]))
    mask = (target_t >= low) & (target_t <= high)
    if not np.any(mask):
        print("No overlapping target/actual time range.")
        return
    for idx, name in enumerate(JOINT_SHORT_NAMES):
        actual_interp = np.interp(target_t[mask], actual_t, actual_v[:, idx])
        err = target_v[mask, idx] - actual_interp
        abs_err = np.abs(err)
        print(
            f"{name:10s} abs_err_p50={percentile(abs_err, 50):7.4f} "
            f"p95={percentile(abs_err, 95):7.4f} "
            f"max={float(np.max(abs_err)):7.4f} std={float(np.std(err)):7.4f}"
        )


def segment_array(series, start, end):
    if len(series) == 0:
        return np.empty((0, len(JOINT_SHORT_NAMES)))
    times = np.asarray([t for t, _ in series], dtype=float)
    values = np.asarray([v for _, v in series], dtype=float)
    mask = (times >= start) & (times <= end)
    return values[mask]


def print_segments(data, segments):
    if not segments:
        return
    print("\n== User Segments ==")
    for start, end, name in segments:
        print(f"\n[{name}] {start:g}-{end:g}s")
        target_values = segment_array(data["target"], start, end)
        actual_values = segment_array(data["actual"], start, end)
        velocity_values = segment_array(data["actual_velocity"], start, end)
        for label, values in (
            ("target_pos", target_values),
            ("actual_pos", actual_values),
            ("actual_vel", velocity_values),
        ):
            if values.size == 0:
                print(f"  {label}: no data")
                continue
            summary = []
            for idx, joint_name in enumerate(JOINT_SHORT_NAMES):
                joint_values = values[:, idx]
                summary.append(
                    f"{joint_name} std={np.std(joint_values):.4f} "
                    f"p2p={np.ptp(joint_values):.4f}"
                )
            print(f"  {label}: " + " | ".join(summary))


def print_recommendation(data, args):
    print("\n== Quick Read ==")
    raw_gap_counts = []
    for topic in (data["topics"]["raw_angles"], data["topics"]["raw_positions"], data["topics"]["glove"]):
        stamps = data["stamps"].get(topic, [])
        raw_gap_counts.append(len(find_gap_events(stamps, args.gap_ms)))
    target = data["target"]
    big_target_jumps = 0
    if len(target) >= 2:
        values = np.asarray([v for _, v in target], dtype=float)
        big_target_jumps = int(np.sum(np.max(np.abs(np.diff(values, axis=0)), axis=1) >= args.jump_rad))
    if max(raw_gap_counts or [0]) > 0 and big_target_jumps > 0:
        print(
            "Input timing gaps and target jumps are both present. "
            "This supports the suspicion that upstream Hex/network delivery is causing part of the jitter."
        )
    elif big_target_jumps > 0:
        print(
            "Target jumps are present but not close to the configured input gap threshold. "
            "Check retarget parameters and glove ergonomics noise."
        )
    else:
        print(
            "No large target jumps with the current threshold. "
            "If the hand still jitters, focus on actual velocity/feedback and controller tuning."
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Offline jitter analysis for Revo2 Hex/Manus MCAP rosbag recordings."
    )
    parser.add_argument("bag", help="Path to rosbag directory, e.g. /tmp/revo2_jitter_debug")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--gap-ms", type=float, default=50.0)
    parser.add_argument("--jump-rad", type=float, default=0.12)
    parser.add_argument("--top", type=int, default=5)
    parser.add_argument("--correlation-window", type=float, default=0.75)
    parser.add_argument("--target-topic", default="")
    parser.add_argument("--actual-topic", default="")
    parser.add_argument("--glove-topic", default="")
    parser.add_argument("--raw-angles-topic", default="/hex_glove/raw_angles")
    parser.add_argument("--raw-positions-topic", default="/hex_glove/raw_positions")
    parser.add_argument(
        "--segments",
        default="",
        help="Comma-separated start:end[:name] windows, e.g. '0:10:still,10:20:fist'.",
    )
    return parser


def main():
    args = build_parser().parse_args()
    segments = parse_segments(args.segments)
    data = load_bag(args)
    print(f"bag={args.bag} duration={data['duration']:.3f}s")
    print_timing(data, args)
    print_target_jumps(data, args)
    print_tracking_error(data)
    print_segments(data, segments)
    print_recommendation(data, args)


if __name__ == "__main__":
    main()
