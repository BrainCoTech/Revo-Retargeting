"""Resolve resources in a source checkout or a colcon install."""
from pathlib import Path


def workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/brainco_revo3_ros2/revo3_description/package.xml").is_file():
            return parent
    return None


def package_share(name):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(name))


def description_root():
    root = workspace_root()
    return root / "src/brainco_revo3_ros2/revo3_description" if root else package_share("revo3_description")


def default_profile(model_version="may3", hand_side="right"):
    if model_version != "may3":
        raise ValueError("This package uses the workspace May3 revo3_description model only")
    if hand_side not in ("left", "right"):
        raise ValueError("hand_side must be left or right")
    relative = f"config/{hand_side}_hardware_may3.yaml"
    local = Path(__file__).resolve().parents[1] / relative
    return local if local.is_file() else package_share("manus_revo3_vector_retarget") / relative


def source_key(path, robot):
    path = Path(path).resolve()
    if path.is_relative_to(robot.description_root.resolve()):
        return "description/" + path.relative_to(robot.description_root.resolve()).as_posix()
    return "retarget_v2/" + path.relative_to(Path(__file__).resolve().parent).as_posix()


def source_path(key, robot):
    if key.startswith("description/"):
        root, relative = robot.description_root.resolve(), key[len("description/"):]
    elif key.startswith("retarget_v2/"):
        root, relative = Path(__file__).resolve().parent, key[len("retarget_v2/"):]
    else:
        raise ValueError(f"Unknown source key: {key}")
    path = (root / relative).resolve()
    if not path.is_relative_to(root):
        raise ValueError("Model asset path escapes package")
    return path
