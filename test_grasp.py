"""Self-test: can the simulated arm actually pick up a cube?

Runs a scripted hover -> descend -> close -> lift on the cube scene and checks
the cube stays put in the gripper. No hardware, no leader arm, no calibration
needed -- useful for checking the scene still works after editing the model.

    python test_grasp.py
"""

import numpy as np
import mujoco

import so100

# A top-down grasp pose for the middle cube, found by sweeping the joint ranges
# for poses that reach cube height with the gripper pointing down AND keep the
# fingertips off the floor. See README "Cube placement".
GRASP_POSE = {"Rotation": 0.0, "Pitch": -1.573, "Elbow": 2.219, "Wrist_Pitch": 1.291, "Wrist_Roll": 0.0}
LIFT_POSE = dict(GRASP_POSE, Pitch=-2.473, Elbow=2.469)
JAW_OPEN, JAW_SHUT = 1.3, -0.174
CUBE = "cube_green"


def main():
    model, data, _ = so100.load_model("cubes")
    act = {n: model.actuator(n).id for n in so100.JOINT_NAMES}
    pads = [[i for i in range(model.ngeom)
             if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) == n][0]
            for n in ("fixed_jaw_pad_3", "moving_jaw_pad_3")]
    cube_qpos = model.joint(CUBE + "_free").qposadr[0]

    mujoco.mj_resetData(model, data)
    for body in ("cube_red", "cube_green", "cube_blue"):
        adr = model.joint(body + "_free").qposadr[0]
        data.qpos[adr:adr + 3] = model.body(body).pos
        data.qpos[adr + 3] = 1
    mujoco.mj_forward(model, data)

    grip = lambda: (data.geom_xpos[pads[0]] + data.geom_xpos[pads[1]]) / 2
    cube = lambda: data.qpos[cube_qpos:cube_qpos + 3].copy()

    def hold(pose, jaw, steps):
        for name, value in pose.items():
            data.ctrl[act[name]] = value
        data.ctrl[act["Jaw"]] = jaw
        for _ in range(steps):
            mujoco.mj_step(model, data)

    def ramp(start, end, jaw, steps):
        for i in range(steps):
            t = (i + 1) / steps
            for name in start:
                data.ctrl[act[name]] = start[name] + (end[name] - start[name]) * t
            data.ctrl[act["Jaw"]] = jaw
            mujoco.mj_step(model, data)

    hold(LIFT_POSE, JAW_OPEN, 1200)
    ramp(LIFT_POSE, GRASP_POSE, JAW_OPEN, 1500)
    hold(GRASP_POSE, JAW_SHUT, 1500)
    held_at_grasp = np.linalg.norm(grip() - cube())
    cube_low, grip_low = cube(), grip()

    ramp(GRASP_POSE, LIFT_POSE, JAW_SHUT, 2500)
    hold(LIFT_POSE, JAW_SHUT, 2000)
    held_at_top = np.linalg.norm(grip() - cube())

    risen = (cube()[2] - cube_low[2]) * 1000
    slip = (held_at_top - held_at_grasp) * 1000
    print(f"  gripper rose {(grip()[2] - grip_low[2]) * 1000:5.1f} mm")
    print(f"  cube rose    {risen:5.1f} mm")
    print(f"  slip in gripper {slip:+.1f} mm")

    if risen > 30 and abs(slip) < 8:
        print("\nPASS: the arm picks up the cube and holds it.")
        return 0
    print("\nFAIL: the cube was not lifted cleanly.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
