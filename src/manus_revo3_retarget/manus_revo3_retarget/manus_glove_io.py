from __future__ import annotations

from geometry_msgs.msg import Pose, Quaternion
from manus_ros2_msgs.msg import ManusErgonomics, ManusGlove, ManusRawNode


def _pose_to_dict(pose: Pose) -> dict[str, object]:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def _pose_from_dict(data: dict[str, object]) -> Pose:
    pose = Pose()
    position = data.get("position", {})
    orientation = data.get("orientation", {})
    if isinstance(position, dict):
        pose.position.x = float(position.get("x", 0.0))
        pose.position.y = float(position.get("y", 0.0))
        pose.position.z = float(position.get("z", 0.0))
    if isinstance(orientation, dict):
        pose.orientation.x = float(orientation.get("x", 0.0))
        pose.orientation.y = float(orientation.get("y", 0.0))
        pose.orientation.z = float(orientation.get("z", 0.0))
        pose.orientation.w = float(orientation.get("w", 1.0))
    return pose


def _quat_to_dict(quat: Quaternion) -> dict[str, float]:
    return {
        "x": float(quat.x),
        "y": float(quat.y),
        "z": float(quat.z),
        "w": float(quat.w),
    }


def _quat_from_dict(data: dict[str, object]) -> Quaternion:
    quat = Quaternion()
    quat.x = float(data.get("x", 0.0))
    quat.y = float(data.get("y", 0.0))
    quat.z = float(data.get("z", 0.0))
    quat.w = float(data.get("w", 1.0))
    return quat


def manus_glove_to_dict(msg: ManusGlove) -> dict[str, object]:
    return {
        "glove_id": int(msg.glove_id),
        "side": str(msg.side),
        "raw_node_count": int(msg.raw_node_count),
        "raw_nodes": [
            {
                "node_id": int(node.node_id),
                "parent_node_id": int(node.parent_node_id),
                "joint_type": str(node.joint_type),
                "chain_type": str(node.chain_type),
                "pose": _pose_to_dict(node.pose),
            }
            for node in msg.raw_nodes
        ],
        "ergonomics_count": int(msg.ergonomics_count),
        "ergonomics": [
            {
                "type": str(ergo.type),
                "value": float(ergo.value),
            }
            for ergo in msg.ergonomics
        ],
        "raw_sensor_orientation": _quat_to_dict(msg.raw_sensor_orientation),
        "raw_sensor_count": int(msg.raw_sensor_count),
        "raw_sensor": [_pose_to_dict(pose) for pose in msg.raw_sensor],
    }


def manus_glove_from_dict(data: dict[str, object]) -> ManusGlove:
    msg = ManusGlove()
    msg.glove_id = int(data.get("glove_id", 0))
    msg.side = str(data.get("side", ""))

    raw_nodes = data.get("raw_nodes", [])
    if isinstance(raw_nodes, list):
        for item in raw_nodes:
            if not isinstance(item, dict):
                continue
            node = ManusRawNode()
            node.node_id = int(item.get("node_id", 0))
            node.parent_node_id = int(item.get("parent_node_id", -1))
            node.joint_type = str(item.get("joint_type", ""))
            node.chain_type = str(item.get("chain_type", ""))
            pose_data = item.get("pose", {})
            if isinstance(pose_data, dict):
                node.pose = _pose_from_dict(pose_data)
            msg.raw_nodes.append(node)
    msg.raw_node_count = int(data.get("raw_node_count", len(msg.raw_nodes)))

    ergonomics = data.get("ergonomics", [])
    if isinstance(ergonomics, list):
        for item in ergonomics:
            if not isinstance(item, dict):
                continue
            ergo = ManusErgonomics()
            ergo.type = str(item.get("type", ""))
            ergo.value = float(item.get("value", 0.0))
            msg.ergonomics.append(ergo)
    msg.ergonomics_count = int(data.get("ergonomics_count", len(msg.ergonomics)))

    raw_sensor_orientation = data.get("raw_sensor_orientation", {})
    if isinstance(raw_sensor_orientation, dict):
        msg.raw_sensor_orientation = _quat_from_dict(raw_sensor_orientation)

    raw_sensor = data.get("raw_sensor", [])
    if isinstance(raw_sensor, list):
        for item in raw_sensor:
            if isinstance(item, dict):
                msg.raw_sensor.append(_pose_from_dict(item))
    msg.raw_sensor_count = int(data.get("raw_sensor_count", len(msg.raw_sensor)))
    return msg
