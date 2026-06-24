#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node

from .manus_glove_io import manus_glove_from_dict

VALID_HAND_MODES = ("left", "right", "both")


@dataclass(frozen=True)
class ReplayFrame:
    t: float
    topic: str
    side: str
    msg: ManusGlove


def _side_enabled(side: str, hand_mode: str) -> bool:
    side = side.strip().lower()
    return hand_mode == "both" or side == hand_mode


def _canonical_topic(side: str) -> str:
    return "/manus_glove_0" if side == "left" else "/manus_glove_1"


def _load_frames(path: Path, hand_mode: str, topic_mode: str) -> list[ReplayFrame]:
    frames: list[ReplayFrame] = []
    with path.open("r", encoding="utf-8") as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{lineno}: invalid JSON: {exc}") from exc
            if record.get("type") != "frame":
                continue
            msg_data = record.get("msg")
            if not isinstance(msg_data, dict):
                continue
            msg = manus_glove_from_dict(msg_data)
            side = str(record.get("side") or msg.side).strip().lower()
            if not _side_enabled(side, hand_mode):
                continue
            topic = str(record.get("topic") or _canonical_topic(side))
            if topic_mode == "canonical":
                topic = _canonical_topic(side)
            frames.append(ReplayFrame(float(record.get("t", 0.0)), topic, side, msg))

    frames.sort(key=lambda frame: frame.t)
    if frames:
        t0 = frames[0].t
        frames = [ReplayFrame(frame.t - t0, frame.topic, frame.side, frame.msg) for frame in frames]
    return frames


def _lerp(a: float, b: float, alpha: float) -> float:
    return (1.0 - alpha) * float(a) + alpha * float(b)


def _interpolate_pose(out_pose, a_pose, b_pose, alpha: float) -> None:
    out_pose.position.x = _lerp(a_pose.position.x, b_pose.position.x, alpha)
    out_pose.position.y = _lerp(a_pose.position.y, b_pose.position.y, alpha)
    out_pose.position.z = _lerp(a_pose.position.z, b_pose.position.z, alpha)

    ax, ay, az, aw = float(a_pose.orientation.x), float(a_pose.orientation.y), float(a_pose.orientation.z), float(a_pose.orientation.w)
    bx, by, bz, bw = float(b_pose.orientation.x), float(b_pose.orientation.y), float(b_pose.orientation.z), float(b_pose.orientation.w)
    if ax * bx + ay * by + az * bz + aw * bw < 0.0:
        bx, by, bz, bw = -bx, -by, -bz, -bw
    qx, qy, qz, qw = _lerp(ax, bx, alpha), _lerp(ay, by, alpha), _lerp(az, bz, alpha), _lerp(aw, bw, alpha)
    norm = max((qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5, 1e-9)
    out_pose.orientation.x = qx / norm
    out_pose.orientation.y = qy / norm
    out_pose.orientation.z = qz / norm
    out_pose.orientation.w = qw / norm


def _interpolate_msg(a: ManusGlove, b: ManusGlove, alpha: float) -> ManusGlove:
    out = copy.deepcopy(a)
    b_nodes = {int(node.node_id): node for node in b.raw_nodes}
    for node in out.raw_nodes:
        other = b_nodes.get(int(node.node_id))
        if other is not None:
            _interpolate_pose(node.pose, node.pose, other.pose, alpha)

    b_ergonomics = {str(ergo.type): ergo for ergo in b.ergonomics}
    for ergo in out.ergonomics:
        other = b_ergonomics.get(str(ergo.type))
        if other is not None:
            ergo.value = _lerp(ergo.value, other.value, alpha)

    for index, pose in enumerate(out.raw_sensor):
        if index < len(b.raw_sensor):
            _interpolate_pose(pose, pose, b.raw_sensor[index], alpha)
    return out


def _transition_frames(
    start: ReplayFrame,
    end: ReplayFrame,
    start_t: float,
    duration_s: float,
    hz: float,
) -> list[ReplayFrame]:
    duration_s = max(0.0, float(duration_s))
    if duration_s <= 0.0:
        return []
    steps = max(1, int(round(duration_s * max(float(hz), 1.0))))
    out: list[ReplayFrame] = []
    topic = start.topic if start.topic == end.topic else end.topic
    side = start.side if start.side == end.side else end.side
    for step in range(1, steps + 1):
        alpha = step / steps
        out.append(
            ReplayFrame(
                start_t + duration_s * alpha,
                topic,
                side,
                _interpolate_msg(start.msg, end.msg, alpha),
            )
        )
    return out


def _load_sequence(
    paths: list[Path],
    hand_mode: str,
    topic_mode: str,
    transition_s: float,
    transition_hz: float,
    close_loop: bool,
) -> list[ReplayFrame]:
    segments = [_load_frames(path, hand_mode, topic_mode) for path in paths]
    segments = [frames for frames in segments if frames]
    if not segments:
        return []

    combined: list[ReplayFrame] = list(segments[0])
    first_frame = combined[0]
    for frames in segments[1:]:
        previous = combined[-1]
        next_first = frames[0]
        transition = _transition_frames(previous, next_first, previous.t, transition_s, transition_hz)
        combined.extend(transition)
        offset = combined[-1].t if transition else previous.t
        skip = 1 if transition else 0
        combined.extend(ReplayFrame(frame.t + offset, frame.topic, frame.side, frame.msg) for frame in frames[skip:])

    if close_loop and len(segments) > 1:
        previous = combined[-1]
        combined.extend(_transition_frames(previous, first_frame, previous.t, transition_s, transition_hz))

    return combined


class ManusReplayNode(Node):
    def __init__(self, frames: list[ReplayFrame], rate: float, loop: bool, drop_late_frames: bool):
        super().__init__("manus_replay")
        if not frames:
            raise ValueError("No frames to replay.")

        self.frames = frames
        self.rate = max(float(rate), 1e-6)
        self.loop = loop
        self.drop_late_frames = bool(drop_late_frames)
        self.index = 0
        self.start_time = time.monotonic()
        self.stop_requested = False
        self._topic_publishers: dict[str, object] = {}
        for frame in self.frames:
            if frame.topic not in self._topic_publishers:
                self._topic_publishers[frame.topic] = self.create_publisher(ManusGlove, frame.topic, 100)

        self.timer = self.create_timer(0.001, self._tick)
        duration = self.frames[-1].t if self.frames else 0.0
        sides = sorted({frame.side for frame in self.frames})
        self.get_logger().info(
            f"Replaying {len(self.frames)} MANUS frames over {duration:.3f}s at {self.rate:.3f}x "
            f"(loop={self.loop}, drop_late_frames={self.drop_late_frames}, sides={sides})"
        )

    def _tick(self) -> None:
        if self.index >= len(self.frames):
            if self.loop:
                self.index = 0
                self.start_time = time.monotonic()
            else:
                self.stop_requested = True
                return

        elapsed = (time.monotonic() - self.start_time) * self.rate
        if self.drop_late_frames:
            due_end = self.index
            while due_end < len(self.frames) and self.frames[due_end].t <= elapsed:
                due_end += 1
            if due_end <= self.index:
                return

            latest_by_topic: dict[str, ReplayFrame] = {}
            for frame in self.frames[self.index:due_end]:
                latest_by_topic[frame.topic] = frame
            for frame in latest_by_topic.values():
                self._topic_publishers[frame.topic].publish(frame.msg)
            self.index = due_end
            return

        while self.index < len(self.frames) and self.frames[self.index].t <= elapsed:
            frame = self.frames[self.index]
            self._topic_publishers[frame.topic].publish(frame.msg)
            self.index += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Replay MANUS glove JSONL recordings onto /manus_glove_* topics.")
    parser.add_argument("input", type=Path, nargs="+")
    parser.add_argument("--hand-mode", default="both", choices=sorted(VALID_HAND_MODES))
    parser.add_argument("--rate", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument(
        "--transition-s",
        type=float,
        default=0.0,
        help="Seconds of interpolated MANUS frames to insert between multiple input recordings.",
    )
    parser.add_argument("--transition-hz", type=float, default=60.0)
    parser.add_argument(
        "--drop-late-frames",
        action="store_true",
        help="If replay falls behind wall time, publish only the latest due frame per topic instead of bursting.",
    )
    parser.add_argument(
        "--topic-mode",
        choices=("recorded", "canonical"),
        default="recorded",
        help="Use recorded topics or publish left/right to /manus_glove_0/1.",
    )
    args = parser.parse_args()

    frames = _load_sequence(
        args.input,
        args.hand_mode,
        args.topic_mode,
        args.transition_s,
        args.transition_hz,
        args.loop,
    )
    rclpy.init()
    node = ManusReplayNode(frames, args.rate, args.loop, args.drop_late_frames)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
