#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import rclpy
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.utilities import remove_ros_args


@dataclass(frozen=True)
class ThumbCmpParamNames:
    offset: str
    scale: str


def _target_node_name(value: str) -> str:
    name = value.strip().rstrip("/")
    if not name:
        raise ValueError("target node name cannot be empty")
    return name if name.startswith("/") else f"/{name}"


def _thumb_cmp_param_names(side: str) -> ThumbCmpParamNames:
    suffix = "_physical"
    return ThumbCmpParamNames(
        offset=f"{side}_thumb_cmp_offset_deg{suffix}",
        scale=f"{side}_thumb_cmp_scale{suffix}",
    )


class ThumbCmpDebugNode(Node):
    def __init__(self, target_node: str):
        super().__init__("thumb_cmp_debug")
        self.target_node = _target_node_name(target_node)
        self._get_client = self.create_client(GetParameters, f"{self.target_node}/get_parameters")
        self._set_client = self.create_client(SetParameters, f"{self.target_node}/set_parameters")

    def wait_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + max(float(timeout_s), 0.0)
        while rclpy.ok():
            if self._get_client.service_is_ready() and self._set_client.service_is_ready():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"Timed out waiting for {self.target_node} parameter services.")
            self._get_client.wait_for_service(timeout_sec=min(0.2, remaining))
            self._set_client.wait_for_service(timeout_sec=min(0.2, max(deadline - time.monotonic(), 0.0)))

    def _spin_future(self, future, timeout_s: float):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            raise TimeoutError(f"Timed out calling {self.target_node} parameter service.")
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(str(exc)) from exc
        return future.result()

    def get_values(self, names: ThumbCmpParamNames, timeout_s: float) -> tuple[float, float]:
        request = GetParameters.Request()
        request.names = [names.offset, names.scale]
        response = self._spin_future(self._get_client.call_async(request), timeout_s)
        values = [parameter_value_to_python(value) for value in response.values]
        return float(values[0]), float(values[1])

    def set_values(
        self,
        names: ThumbCmpParamNames,
        timeout_s: float,
        *,
        offset: float | None = None,
        scale: float | None = None,
    ) -> None:
        params = []
        if offset is not None:
            params.append(Parameter(names.offset, value=float(offset)).to_parameter_msg())
        if scale is not None:
            params.append(Parameter(names.scale, value=float(scale)).to_parameter_msg())
        if not params:
            return

        request = SetParameters.Request()
        request.parameters = params
        response = self._spin_future(self._set_client.call_async(request), timeout_s)
        failures = [result.reason or "unknown reason" for result in response.results if not result.successful]
        if failures:
            raise RuntimeError("; ".join(failures))


def _print_values(node: ThumbCmpDebugNode, names: ThumbCmpParamNames, timeout_s: float) -> tuple[float, float]:
    offset, scale = node.get_values(names, timeout_s)
    print(f"{names.offset}={offset:.3f} deg, {names.scale}={scale:.3f}")
    return offset, scale


def _interactive_loop(
    node: ThumbCmpDebugNode,
    names: ThumbCmpParamNames,
    timeout_s: float,
    offset_step: float,
    scale_step: float,
) -> None:
    print("Commands: + / - adjust offset, o <deg> set offset, s+ / s- adjust scale, s <value> set scale, show, q")
    _print_values(node, names, timeout_s)
    while rclpy.ok():
        try:
            raw = input("thumb-cmp> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not raw:
            continue
        if raw in ("q", "quit", "exit"):
            return
        if raw in ("show", "p", "print"):
            _print_values(node, names, timeout_s)
            continue

        offset, scale = node.get_values(names, timeout_s)
        new_offset = None
        new_scale = None
        parts = raw.split()
        try:
            if raw in ("+", "o+", "offset+"):
                new_offset = offset + offset_step
            elif raw in ("-", "o-", "offset-"):
                new_offset = offset - offset_step
            elif raw in ("s+", "scale+"):
                new_scale = scale + scale_step
            elif raw in ("s-", "scale-"):
                new_scale = max(0.001, scale - scale_step)
            elif len(parts) == 2 and parts[0] in ("o", "offset"):
                new_offset = float(parts[1])
            elif len(parts) == 2 and parts[0] in ("s", "scale"):
                new_scale = float(parts[1])
            else:
                print("Unknown command.")
                continue
        except ValueError:
            print("Invalid number.")
            continue

        node.set_values(names, timeout_s, offset=new_offset, scale=new_scale)
        _print_values(node, names, timeout_s)


def main() -> int:
    parser = argparse.ArgumentParser(description="Online debug tool for legacy thumb CMP offset/scale.")
    parser.add_argument("--node", default="/manus_revo3_retarget", help="Target retarget node name.")
    parser.add_argument("--side", default="right", choices=("left", "right"))
    parser.add_argument("--offset", type=float, default=None, help="Set thumb CMP offset in degrees.")
    parser.add_argument("--scale", type=float, default=None, help="Set thumb CMP scale.")
    parser.add_argument("--step", type=float, default=1.0, help="Interactive offset step in degrees.")
    parser.add_argument("--scale-step", type=float, default=0.05, help="Interactive scale step.")
    parser.add_argument("--timeout", type=float, default=5.0, help="Service timeout in seconds.")
    parser.add_argument("--show", action="store_true", help="Print current values and exit unless --interactive is set.")
    parser.add_argument("--interactive", "-i", action="store_true", help="Start an interactive adjustment shell.")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    interactive = args.interactive or (args.offset is None and args.scale is None and not args.show)
    names = _thumb_cmp_param_names(args.side)

    rclpy.init()
    node = ThumbCmpDebugNode(args.node)
    try:
        node.wait_ready(args.timeout)
        if args.show:
            _print_values(node, names, args.timeout)
        if args.offset is not None or args.scale is not None:
            node.set_values(names, args.timeout, offset=args.offset, scale=args.scale)
            _print_values(node, names, args.timeout)
        if interactive:
            _interactive_loop(node, names, args.timeout, args.step, args.scale_step)
    except Exception as exc:
        print(f"thumb_cmp_debug: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
