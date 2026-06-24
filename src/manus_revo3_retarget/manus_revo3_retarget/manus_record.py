#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from typing import TextIO

import rclpy
from manus_ros2_msgs.msg import ManusGlove
from rclpy.node import Node

from .manus_glove_io import manus_glove_to_dict

VALID_HAND_MODES = ("left", "right", "both")


def _default_output_path(hand_mode: str, action: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("recordings") / hand_mode / action / f"manus_{stamp}.jsonl"


def _side_enabled(side: str, hand_mode: str) -> bool:
    side = side.strip().lower()
    return hand_mode == "both" or side == hand_mode


class ManusRecordNode(Node):
    def __init__(
        self,
        output_path: Path,
        hand_mode: str,
        duration: float | None,
        max_frames: int | None,
        action: str,
    ):
        super().__init__("manus_record")
        self.output_path = output_path
        self.hand_mode = hand_mode
        self.duration = duration
        self.max_frames = max_frames
        self.action = action
        self.start_time: float | None = None
        self.frames = 0
        self.side_counts: dict[str, int] = {}
        self.stop_requested = False

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file: TextIO = self.output_path.open("w", encoding="utf-8")
        metadata = {
            "type": "metadata",
            "version": 1,
            "created_unix": time.time(),
            "created_local": datetime.now().isoformat(timespec="seconds"),
            "hand_mode": self.hand_mode,
            "action": self.action,
            "topics": ["/manus_glove_0", "/manus_glove_1"],
        }
        self.file.write(json.dumps(metadata, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.file.flush()

        self.create_subscription(ManusGlove, "/manus_glove_0", lambda msg: self._on_msg("/manus_glove_0", msg), 100)
        self.create_subscription(ManusGlove, "/manus_glove_1", lambda msg: self._on_msg("/manus_glove_1", msg), 100)

        if self.duration is not None and self.duration > 0:
            self.create_timer(0.1, self._check_stop)

        self.get_logger().info(
            f"Recording MANUS glove data to {self.output_path} "
            f"(hand_mode={self.hand_mode}, action={self.action}, duration={self.duration or 'until stopped'})"
        )

    def _on_msg(self, topic: str, msg: ManusGlove) -> None:
        side = str(msg.side).strip().lower()
        if not _side_enabled(side, self.hand_mode):
            return

        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now
        t_rel = now - self.start_time
        frame = {
            "type": "frame",
            "t": t_rel,
            "topic": topic,
            "side": side,
            "action": self.action,
            "msg": manus_glove_to_dict(msg),
        }
        self.file.write(json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.frames += 1
        self.side_counts[side] = self.side_counts.get(side, 0) + 1
        if self.frames % 30 == 0:
            self.file.flush()

        if self.max_frames is not None and self.frames >= self.max_frames:
            self.get_logger().info(f"Reached max frames: {self.max_frames}")
            self.stop_requested = True

    def _check_stop(self) -> None:
        if self.start_time is None or self.duration is None:
            return
        if time.monotonic() - self.start_time >= self.duration:
            self.get_logger().info(f"Reached duration: {self.duration:.2f}s")
            self.stop_requested = True

    def close(self) -> None:
        self.file.flush()
        self.file.close()
        self.get_logger().info(f"Saved {self.frames} frames to {self.output_path}; side_counts={self.side_counts}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Record /manus_glove_* messages to JSONL for deterministic replay.")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--hand-mode", default="both", choices=sorted(VALID_HAND_MODES))
    parser.add_argument(
        "--action",
        required=True,
        help="Action label for this recording, e.g. open, fist, pinch_index.",
    )
    parser.add_argument("--duration", type=float, default=None, help="Seconds to record after the first accepted frame.")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()

    output = args.output or _default_output_path(args.hand_mode, args.action)

    if output.exists() and not args.overwrite:
        raise SystemExit(f"Refusing to overwrite existing file: {output} (pass --overwrite)")

    rclpy.init()
    node = ManusRecordNode(output, args.hand_mode, args.duration, args.max_frames, args.action)
    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
