"""Re-measure just the gripper's travel and patch it into the lerobot calibration.

The full `lerobot-calibrate` sweep also rewrites every joint's homing offset,
which would invalidate alignment.json. This touches range_min/range_max for the
gripper only, so the alignment you already did stays valid.

    python gripper_range.py --port COM4 --id my_leader_arm
"""

import argparse
import json
import pathlib
import threading
import time

MOTOR = "gripper"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="leader arm serial port, e.g. COM4")
    parser.add_argument("--id", default="my_leader_arm", help="lerobot calibration id")
    args = parser.parse_args()

    import so100

    teleop = so100.connect_leader(args.port, args.id)
    path = pathlib.Path(teleop.calibration_fpath)

    print(f"\nSqueeze the gripper trigger fully open and fully closed a few times.")
    print("Press Enter when done...\n")

    low = high = None
    done = threading.Event()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
    try:
        while not done.is_set():
            raw = teleop.bus.sync_read("Present_Position", [MOTOR], normalize=False)[MOTOR]
            low = raw if low is None else min(low, raw)
            high = raw if high is None else max(high, raw)
            print(f"\r  raw {raw:4d}   range so far {low}..{high}  ({high - low} counts)   ", end="")
            time.sleep(0.02)
    finally:
        teleop.disconnect()
    print()

    span = high - low
    if span < 200:
        raise SystemExit(f"\nOnly {span} counts of travel seen -- sweep the trigger further and retry.")

    calibration = json.loads(path.read_text())
    before = calibration[MOTOR]["range_min"], calibration[MOTOR]["range_max"]
    calibration[MOTOR]["range_min"] = low
    calibration[MOTOR]["range_max"] = high
    path.write_text(json.dumps(calibration, indent=4) + "\n")

    print(f"\n{path}")
    print(f"  gripper range {before[0]}..{before[1]} ({before[1] - before[0]} counts)"
          f"  ->  {low}..{high} ({span} counts)")
    print("\nRestart teleop.py. If the gripper now moves the wrong way, flip"
          ' "Jaw" sign in alignment.json.')


if __name__ == "__main__":
    main()
