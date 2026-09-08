#!/usr/bin/env python3
"""按中文提示录制 DV1 拇指姿态；只读取手套，不启动 ROS 或机器人驱动。"""

import argparse
from datetime import datetime, timezone
import importlib.util
import json
import math
from pathlib import Path
import queue
import statistics
import subprocess
import sys
import time


POSES = (
    ("neutral", "自然伸直", "手腕固定，四指自然伸直；拇指放松伸直，虎口稍微张开。"),
    ("distal_bend", "拇指末节弯曲", "拇指根部尽量保持原位，把靠近指尖的关节弯成钩状；四指放松。"),
    ("radial_open", "沿掌面张开", "先伸直拇指，再沿手掌平面向外张开虎口，停在舒适的位置。"),
    ("palmar_abduction", "离开掌面抬起", "让拇指离开手掌平面、朝掌心前方抬起，像准备握住杯子。"),
    ("index_touch", "拇指对食指", "拇指指腹轻触食指指腹。允许食指自然弯曲配合，不用捏紧；碰不到就保持最近的舒适位置。"),
    ("middle_touch", "拇指对中指", "拇指指腹轻触中指指腹。允许中指自然弯曲配合，不用捏紧；碰不到就保持最近的舒适位置。"),
    ("neutral_return", "回到自然伸直", "回到第一个姿态：四指自然伸直，拇指放松伸直、虎口稍微张开，用于检查重复性。"),
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def summarize(rows, names):
    if len(rows) < 30:
        raise ValueError(f"有效帧只有 {len(rows)}，至少需要 30 帧")
    if any(len(row) != len(names) or not all(math.isfinite(v) for v in row) for row in rows):
        raise ValueError("关节数组缺失或含无效数值")
    columns = list(zip(*rows))
    unwrapped = [[c[0] + (v-c[0]+math.pi) % (2*math.pi)-math.pi for v in c] for c in columns]
    medians = dict(zip(names, (statistics.median(c) for c in unwrapped)))
    deviations = dict(zip(names, (math.degrees(statistics.pstdev(c)) for c in unwrapped)))
    thumb_std = max(value for name, value in deviations.items() if "_thumb_" in name)
    if thumb_std > 5.0:
        raise ValueError(f"拇指仍在移动，最大标准差 {thumb_std:.2f}°；请停稳后重录")
    return {"median_rad": medians, "std_deg": deviations, "max_thumb_std_deg": thumb_std}


def load_sdk(sdk, side):
    viewer = sdk / "tools/fk_viewer.py"
    map_path = sdk / "tools/dv1_encoder_map.yaml"
    if not viewer.is_file() or not map_path.is_file():
        raise ValueError(f"SDK 缺少 DV1 映射工具：{sdk}；请使用 sdk-encoder-fk-viewer 分支")
    sys.path.insert(0, str(sdk))
    spec = importlib.util.spec_from_file_location("dv1_capture_encoder_map", viewer)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    mapper = module.EncoderMap.load(map_path, side)
    mapper.identity = True
    names = [mapper.joint_name(i) for i in range(21)]
    if len(set(names)) != 21 or any(not n.startswith(side + "_") for n in names):
        raise ValueError("SDK 关节名或手别不匹配")
    from revohuman import GloveManager
    return GloveManager, mapper, names


def capture(glove, mapper, seconds):
    rows, raw_frames = [], []
    seen = set()
    generations = set()
    invalid = 0
    subscription = glove.subscribe(queue_size=128)
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                frame = subscription.recv(timeout=min(.1, max(.001, deadline-time.monotonic())))
            except queue.Empty:
                continue
            key = (frame.generation, frame.seq)
            if key in seen:
                continue
            seen.add(key)
            generations.add(frame.generation)
            if len(frame.joints_deg) != 21 or frame.joints_valid_mask != (1 << 21)-1:
                invalid += 1
                continue
            values = [mapper.to_rad(i, frame.joints_deg[i]) for i in range(21)]
            if not all(math.isfinite(v) for v in values):
                invalid += 1
                continue
            rows.append(values)
            raw_frames.append({"seq": frame.seq, "generation": frame.generation,
                               "stamp_host_ns": frame.stamp_host_ns,
                               "stamp_device_ns": frame.stamp_device_ns,
                               "device_tick": frame.device_tick,
                               "joints_deg": list(frame.joints_deg),
                               "joints_valid_mask": frame.joints_valid_mask})
    finally:
        subscription.close()
    if len(generations) > 1:
        raise ValueError("采集期间手套发生重连，请重录当前姿态")
    if invalid > .2 * (len(rows) + invalid):
        raise ValueError(f"无效帧过多：{invalid}/{len(rows)+invalid}，请检查手套连接")
    return rows, raw_frames, invalid


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--sdk-path", type=Path, default=repo.parent / "brainco_revohuman_sdk")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--output", type=Path, help="新的结果目录；默认 artifacts/thumb_calibration/<手别_时间>")
    parser.add_argument("--list-poses", action="store_true", help="仅显示动作说明，不连接设备")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 1:
        parser.error("--seconds 至少为 1 秒")
    if args.list_poses:
        for i, (_, title, instruction) in enumerate(POSES, 1):
            print(f"{i}. {title}：{instruction}")
        return 0

    sdk = args.sdk_path.expanduser().resolve()
    GloveManager, mapper, names = load_sdk(sdk, args.side)
    output = (args.output or repo / "artifacts/thumb_calibration" /
              f"{args.side}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}").expanduser().resolve()
    if output.exists():
        parser.error(f"结果目录已经存在，避免覆盖：{output}")
    output.mkdir(parents=True)
    revision = subprocess.run(["git", "-C", str(sdk), "rev-parse", "HEAD"],
                              capture_output=True, text=True, check=False).stdout.strip()
    manifest = {"created_at_utc": utc_now(), "side": args.side, "port": args.port,
                "sdk_path": str(sdk), "sdk_commit": revision, "joint_names": names,
                "seconds_per_pose": args.seconds, "conversion": "SDK identity: wrap(deg - 180) -> radians",
                "complete": False, "poses": [], "hardware_commands_sent": False,
                "firmware_calibration_modified": False}
    write_json(output / "manifest.json", manifest)
    print(f"\n结果目录：{output}")
    print("请先关闭占用同一手套串口的 SDK 发布器或 viewer。")
    print("本脚本只读取手套，不启动 ROS/机器人驱动，也不修改固件零位。")
    print(f"每个姿态：摆好 → 按 Enter → 等待 1 秒稳定 → 采集 {args.seconds:g} 秒。")
    print("无需用力或追求最大幅度。输入 q 或按 Ctrl+C 可以退出，已录数据会保留。", flush=True)
    try:
        with GloveManager().connect(port=args.port, rate_hz=200.) as glove:
            for i, (key, title, instruction) in enumerate(POSES, 1):
                while True:
                    print(f"\n[{i}/{len(POSES)}] {title}\n{instruction}")
                    answer = input("摆好后按 Enter 开始（q 退出）：").strip().lower()
                    if answer == "q":
                        raise KeyboardInterrupt
                    if answer:
                        print("按 Enter 开始，或输入 q 退出。")
                        continue
                    print("保持不动，1 秒后开始……", flush=True)
                    time.sleep(1)
                    print(f"正在采集 {args.seconds:g} 秒……", flush=True)
                    try:
                        rows, frames, invalid = capture(glove, mapper, args.seconds)
                        stats = summarize(rows, names)
                    except ValueError as error:
                        print(f"本次未保存：{error}。仍停留在当前动作。")
                        continue
                    filename = f"{i:02d}_{key}.json"
                    result = {"pose": key, "title": title, "instruction": instruction,
                              "capture_time_utc": utc_now(), "side": args.side,
                              "samples": len(rows), "invalid_frames": invalid,
                              "joint_names": names, **stats,
                              "raw_rows_rad": rows, "raw_frames": frames}
                    write_json(output / filename, result)
                    manifest["poses"].append({"pose": key, "file": filename, "samples": len(rows),
                                              "max_thumb_std_deg": stats["max_thumb_std_deg"]})
                    write_json(output / "manifest.json", manifest)
                    print(f"已保存 {len(rows)} 帧；拇指最大标准差 {stats['max_thumb_std_deg']:.2f}°。可以放松。")
                    break
        manifest["complete"] = True
        manifest["completed_at_utc"] = utc_now()
        print(f"\n录制完成，7 个姿态均已保存。\n把这个目录发给我：\n{output}")
        return 0
    except (KeyboardInterrupt, EOFError):
        print(f"\n录制已停止，已保存 {len(manifest['poses'])} 个姿态：{output}")
        return 130
    except Exception as error:
        manifest["error"] = str(error)
        print(f"\n录制失败：{error}\n已录数据保留在：{output}", file=sys.stderr)
        return 1
    finally:
        write_json(output / "manifest.json", manifest)


if __name__ == "__main__":
    raise SystemExit(main())
