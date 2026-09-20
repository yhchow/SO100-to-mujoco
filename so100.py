"""Maps the lerobot SO-100 leader frame onto the MuJoCo trs_so_arm100 model.

lerobot hands us calibrated joint angles in degrees (gripper: 0-100%). MuJoCo
wants radians in its own joint frame, which differs by a per-joint direction and
zero offset depending on how your arm was assembled and where lerobot's
calibration put zero. align.py measures those two things; this module applies
them.
"""

import json
import math
import pathlib

SCENES = {
    "plain": pathlib.Path(__file__).parent / "model" / "scene.xml",
    "cubes": pathlib.Path(__file__).parent / "model" / "scene_cubes.xml",
}
MODEL_PATH = SCENES["plain"]
ALIGNMENT_PATH = pathlib.Path(__file__).parent / "alignment.json"

# MuJoCo joint name -> lerobot motor key, in kinematic order.
JOINTS = [
    ("Rotation", "shoulder_pan"),
    ("Pitch", "shoulder_lift"),
    ("Elbow", "elbow_flex"),
    ("Wrist_Pitch", "wrist_flex"),
    ("Wrist_Roll", "wrist_roll"),
    ("Jaw", "gripper"),
]
JOINT_NAMES = [name for name, _ in JOINTS]
MOTOR_KEYS = [key for _, key in JOINTS]

# The gripper is reported as 0-100% no matter what, so it is mapped onto the
# Jaw joint's range instead of being treated as an angle.
GRIPPER_JOINT = "Jaw"

# Joints that rotate freely, so an angle outside the model's limit should wrap
# rather than clamp. This is what lets Wrist_Roll carry a 180 degree offset
# (for an arm whose gripper is mounted the other way round) without pinning
# itself to the end of its range.
WRAPPING_JOINTS = {"Wrist_Roll"}

# Alignment poses. Pose A sets the zero offsets, so match it carefully; pose B
# only has to be close enough to show which way each joint turns. Both are shown
# in the viewer by align.py.
POSE_A = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
POSE_B = [1.0, -0.80, 0.80, 1.0, 1.50, 1.2]


def read_leader(teleop) -> dict[str, float]:
    """One frame from the leader arm, keyed by lerobot motor name."""
    action = teleop.get_action()
    return {key: action[f"{key}.pos"] for key in MOTOR_KEYS}


def save_alignment(alignment: dict, path: pathlib.Path = ALIGNMENT_PATH) -> None:
    path.write_text(json.dumps(alignment, indent=2) + "\n")


def load_alignment(path: pathlib.Path = ALIGNMENT_PATH) -> dict:
    if not path.exists():
        raise SystemExit(f"No alignment at {path}. Run: python align.py")
    alignment = json.loads(path.read_text())
    missing = [name for name in JOINT_NAMES if name not in alignment]
    if missing:
        raise SystemExit(f"Alignment {path} is missing joints: {missing}")
    return alignment


def to_mujoco(reading: dict[str, float], alignment: dict, ranges: dict) -> list[float]:
    """Leader reading -> MuJoCo joint angles (radians), clamped to joint limits."""
    angles = []
    for name, key in JOINTS:
        entry = alignment[name]
        low, high = ranges[name]
        if name == GRIPPER_JOINT:
            # 0-100% across the jaw's travel, flipped if the arm reports it backwards.
            fraction = reading[key] / 100.0
            if entry["sign"] < 0:
                fraction = 1.0 - fraction
            angle = low + fraction * (high - low)
        else:
            angle = entry["sign"] * math.radians(reading[key]) + entry["offset_rad"]
            if name in WRAPPING_JOINTS:
                angle = (angle + math.pi) % (2 * math.pi) - math.pi
        angles.append(min(max(angle, low), high))
    return angles


def load_model(scene: str = "plain"):
    """Load a scene and return (model, data, joint ranges in radians).

    "plain" is the bare arm; "cubes" adds three graspable blocks and the
    overhead camera.
    """
    import mujoco

    if scene not in SCENES:
        raise SystemExit(f"Unknown scene {scene!r}. Choose from: {', '.join(SCENES)}")
    model = mujoco.MjModel.from_xml_path(str(SCENES[scene]))
    data = mujoco.MjData(model)
    ranges = {name: tuple(float(v) for v in model.joint(name).range) for name in JOINT_NAMES}
    return model, data, ranges


def connect_leader(port: str, robot_id: str):
    """Connect to the SO-100 leader arm, reading joints in degrees."""
    from lerobot.teleoperators.so_leader import SO100Leader, SO100LeaderConfig

    teleop = SO100Leader(SO100LeaderConfig(port=port, id=robot_id, use_degrees=True))
    teleop.connect()
    return teleop
