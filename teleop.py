"""Mirror the physical SO-100 leader arm in MuJoCo.

    python teleop.py --port COM4 --id my_leader_arm
"""

import argparse
import math
import time

import mujoco
import mujoco.viewer

import panels as panels_module
import so100


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="leader arm serial port, e.g. COM4")
    parser.add_argument("--id", default="my_leader_arm", help="lerobot calibration id")
    parser.add_argument(
        "--scene", choices=["plain", "cubes"], default="plain",
        help="plain: just the arm (default). cubes: three graspable blocks in reach.",
    )
    parser.add_argument(
        "--mode", choices=["ctrl", "qpos"], default="ctrl",
        help="ctrl: drive the position actuators, arm follows with real dynamics (default). "
             "qpos: snap joints exactly, no physics.",
    )
    parser.add_argument(
        "--panels", action=argparse.BooleanOptionalAction, default=True,
        help="overlay each model camera as a picture-in-picture panel, plus a live "
             "readout of joint angles and loop rate (default: on)",
    )
    args = parser.parse_args()

    model, data, ranges = so100.load_model(args.scene)
    alignment = so100.load_alignment()
    actuators = [model.actuator(name).id for name in so100.JOINT_NAMES]
    qpos_adr = [model.joint(name).qposadr[0] for name in so100.JOINT_NAMES]

    if args.scene == "cubes" and args.mode == "qpos":
        print("Note: --mode qpos skips physics, so the cubes cannot be pushed or picked up.\n"
              "      Use --mode ctrl (the default) to grasp them.")

    overlay = panels_module.Panels(model) if args.panels else None
    if overlay is not None:
        print(f"Panels: {', '.join(overlay.cameras) or 'none in this scene'}. "
              "Press Tab / Shift-Tab to hide the viewer's own UI, [ and ] to fly the "
              "main view through each camera.")

    teleop = so100.connect_leader(args.port, args.id)
    print(f"Connected to {args.port}. Move the leader arm; close the viewer to stop.")

    read_hz = 0.0
    last_read = time.perf_counter()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            wall_start = time.perf_counter()
            next_sync = 0.0
            while viewer.is_running():
                reading = so100.read_leader(teleop)
                angles = so100.to_mujoco(reading, alignment, ranges)

                now_read = time.perf_counter()
                dt = now_read - last_read
                last_read = now_read
                if dt > 0:  # smooth it, a single interval is too jumpy to read
                    read_hz = 0.9 * read_hz + 0.1 / dt if read_hz else 1 / dt

                if args.mode == "qpos":
                    for adr, angle in zip(qpos_adr, angles):
                        data.qpos[adr] = angle
                    data.time = time.perf_counter() - wall_start
                    mujoco.mj_forward(model, data)
                else:
                    for actuator, angle in zip(actuators, angles):
                        data.ctrl[actuator] = angle
                    # Step until sim time catches up to the wall clock, but never
                    # spiral if we fall behind.
                    elapsed = time.perf_counter() - wall_start
                    for _ in range(20):
                        if data.time >= elapsed:
                            break
                        mujoco.mj_step(model, data)
                    else:
                        data.time = elapsed

                # Sync at display rate only. viewer.sync() holds the viewer's
                # lock, so calling it every serial frame (~200 Hz) starves the
                # render thread and makes the camera unresponsive.
                now = time.perf_counter()
                if now >= next_sync:
                    if overlay is not None:
                        overlay.update(viewer, data, {
                            "joints": {name: math.degrees(angle)
                                       for name, angle in zip(so100.JOINT_NAMES, angles)},
                            "gripper": reading["gripper"],
                            "read_hz": read_hz,
                        })
                    viewer.sync()
                    next_sync = now + 1 / 60
    except KeyboardInterrupt:
        pass
    finally:
        if overlay is not None:
            overlay.close()
        teleop.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()
