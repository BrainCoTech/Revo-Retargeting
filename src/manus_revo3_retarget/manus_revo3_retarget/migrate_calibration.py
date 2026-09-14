"""Convert ordered legacy Revo3 YAML layers to one effective output calibration."""
import argparse
import math
from pathlib import Path
import yaml


def parameters(document):
    result = {}
    for key in ("/**", "manus_revo3_retarget", "/manus_revo3_retarget"):
        result.update(document.get(key, {}).get("ros__parameters", {}))
    result.update(document.get("ros__parameters", {}))
    return result or dict(document)


def migrate(params):
    result = dict(params)
    for side in ("left", "right"):
        prefix = f"legacy_{side}_physical_thumb_"
        common = params.get(prefix + "joint_offset_deg", 0.0)
        for joint in ("CMP", "CMR", "MCP", "PIP", "DIP"):
            low = joint.lower()
            a1 = params.get(f"{side}_thumb_cmp_scale_physical", 1.0) if joint == "CMP" else params.get(prefix + low + "_scale", 1.0)
            b1 = common + (params.get(f"{side}_thumb_cmp_offset_deg_physical", 0.0) if joint == "CMP" else params.get(prefix + low + "_offset_deg", 0.0) if joint in ("CMR", "MCP") else 0.0)
            base = f"physical_{side}_thumb_{joint}_joint_"
            a2, b2 = params.get(base + "scale", 1.0), params.get(base + "offset_deg", 0.0)
            result[base + "scale"] = float(a1 * a2)
            result[base + "offset_deg"] = float(a2 * b1 + b2)
        for key in list(result):
            if key in {f"{side}_thumb_cmp_scale_physical", f"{side}_thumb_cmp_offset_deg_physical"} or (key.startswith(prefix) and key[len(prefix):] in {"joint_offset_deg", "cmr_offset_deg", "mcp_offset_deg", "mcp_scale", "pip_scale", "dip_scale"}):
                del result[key]
    for key in ("thumb_ik_posture_weight", "thumb_ik_smooth_weight"):
        if key in result: result[key] = (result[key] * math.radians(10)) ** 2
    if "thumb_ik_tolerance" in result:
        result["thumb_ik_normalized_tolerance"] = result.pop("thumb_ik_tolerance")
    for side in ("left", "right"):
        for joint in ("pip", "dip"):
            key = f"legacy_{side}_physical_thumb_{joint}_ik_scale"
            if key in result: result[f"{side}_thumb_ik_{joint}_weight"] = (.001 * result.pop(key)) ** 2
    for key in list(result):
        if key == "thumb_ik_max_frame_delta_deg" or "_thumb_ema_" in key or key.endswith("_thumb_reach_scale"):
            del result[key]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layers", nargs="+", type=Path, help="OLD base YAML then OLD overrides in launch order")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    merged = {}
    for path in args.layers:
        merged.update(parameters(yaml.safe_load(path.read_text())))
    args.output.write_text(yaml.safe_dump({"manus_revo3_retarget": {"ros__parameters": migrate(merged)}}, sort_keys=False))
    print("Merged affine calibration; intermediate clipping is removed. Check saturated joint ranges offline.")

if __name__ == "__main__":
    main()
