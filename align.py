"""One-time alignment of the leader arm's frame to the MuJoCo model's frame.

Shows two poses in the viewer. You match the physical arm to each and press
Enter. Pose A fixes each joint's zero offset (match it carefully); pose B only
reveals which direction each joint turns, so rough is fine.

    python align.py --port COM4 --id my_leader_arm
"""

import argparse
import math
import threading
import time

import mujoco
import mujoco.viewer

import so100


def show_pose(viewer, model, data, pose, prompt):
    """Hold a pose in the viewer until the user presses Enter, then read the arm."""
    data.qpos[:len(pose)] = pose
    data.ctrl[:len(pose)] = pose
    mujoco.mj_forward(model, data)

    print(f"\n{prompt}")
    print("  Match the physical arm to the pose shown, then press Enter...")
    pressed = threading.Event()
    threading.Thread(target=lambda: (input(), pressed.set()), daemon=True).start()
    # Pace the syncs: viewer.sync() takes the viewer's lock, so calling it flat
    # out starves the render/input thread and the camera becomes unusable.
    while not pressed.is_set() and viewer.is_running():
        viewer.sync()
        time.sleep(1 / 60)
    if not viewer.is_running():
        raise SystemExit("Viewer closed, aborting alignment.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="leader arm serial port, e.g. COM4")
    parser.add_argument("--id", default="my_leader_arm", help="lerobot calibration id")
    args = parser.parse_args()

    model, data, ranges = so100.load_model()
    teleop = so100.connect_leader(args.port, args.id)

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            show_pose(viewer, model, data, so100.POSE_A, "Pose A -- sets the zero offsets. Be accurate.")
            reading_a = so100.read_leader(teleop)
            show_pose(viewer, model, data, so100.POSE_B, "Pose B -- only sets joint directions. Roughly is fine.")
            reading_b = so100.read_leader(teleop)
    finally:
        teleop.disconnect()

    alignment = {}
    print(f"\n{'joint':<12} {'sign':>5} {'offset deg':>11} {'moved':>8}")
    for i, (name, key) in enumerate(so100.JOINTS):
        expected = so100.POSE_B[i] - so100.POSE_A[i]          # radians, model frame
        measured = math.radians(reading_b[key] - reading_a[key])  # radians, arm frame
        if name == so100.GRIPPER_JOINT:
            # Gripper is a 0-100% reading, so "measured" here is just percent.
            measured = reading_b[key] - reading_a[key]
        sign = -1 if expected * measured < 0 else 1
        offset = so100.POSE_A[i] - sign * math.radians(reading_a[key])
        alignment[name] = {"sign": sign, "offset_rad": round(offset, 6)}

        moved = abs(measured if name == so100.GRIPPER_JOINT else math.degrees(measured))
        warn = "  <-- barely moved, direction may be wrong" if moved < 10 else ""
        print(f"{name:<12} {sign:>5} {math.degrees(offset):>11.1f} {moved:>8.1f}{warn}")

    so100.save_alignment(alignment)
    print(f"\nSaved {so100.ALIGNMENT_PATH}\nNow run: python teleop.py --port {args.port} --id {args.id}")


if __name__ == "__main__":
    main()
