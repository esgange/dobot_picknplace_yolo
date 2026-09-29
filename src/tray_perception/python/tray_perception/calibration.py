"""Active tray-camera selection, independent of reference-plane teaching history."""

from pathlib import Path

from item_perception_yolo.item_teach_calibration import selected_robot_camera
from item_perception_yolo.platform_teach_core import _parse_env_file, load_robot_lan1_ip


def active_robot_camera(root):
    """Use the shared explicit robot-camera choice; never reopen a teach-time file."""
    root = Path(root)
    load_robot_lan1_ip(root)
    name = _parse_env_file(root / ".env").get("ITEM_TEACH_ROBOT_CAMERA_CALIBRATION", "")
    if not name or Path(name).name != name:
        raise ValueError("Select the active robot-camera calibration in Item Teach first")
    return selected_robot_camera(root / "calibration" / name, root=root)


def camera_provenance(camera):
    return {"filename": camera.path.name, "sha256": camera.sha256}
