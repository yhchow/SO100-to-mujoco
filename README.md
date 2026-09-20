# so100-to-mujoco

Drive a simulated SO-100 arm in MuJoCo by moving a physical SO-100 **leader** arm.

A few small scripts, no framework. Servo communication is [lerobot](https://github.com/huggingface/lerobot)'s,
the arm model is [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)'s,
and this repo is only the glue between them.

```
physical leader arm  ->  lerobot SOLeader  ->  degrees  ->  alignment  ->  MuJoCo joint angles
```

---

## Why this exists

The obvious move is to reach for a simulator that already speaks SO-100. The existing
options each solve a different problem:

| Project | What it is | Why not just use it |
|---|---|---|
| [lachlanhurst/so100-mujoco-sim](https://github.com/lachlanhurst/so100-mujoco-sim) | Qt GUI around MuJoCo with record/playback | Pins an **old vendored copy of lerobot** with an incompatible calibration format, and models the arm as a *follower* — its "drive from the real robot" mode works by disabling torque so you can backdrive a follower. There is no leader path. |
| [oliverchoy/open-source-leader-arm](https://github.com/oliverchoy/open-source-leader-arm) | A leader arm streaming into MuJoCo | Different hardware: **AS5600 magnetic encoders on an ESP32**, not Feetech servos, and no lerobot. Nothing to reuse if your leader is a stock SO-100. |
| [adityakamath/so_arm_ros2](https://github.com/adityakamath/so_arm_ros2) | ROS 2 + ros2_control, Feetech *and* MuJoCo | A full ROS 2 stack with Pinocchio IK and collision checking. Correct, but heavy if all you want is to watch your arm move in a sim. |
| [lerobot](https://github.com/huggingface/lerobot) itself | The library this repo depends on | As of 0.5.2 it has **no MuJoCo target**: `lerobot.robots` are all physical arms and `lerobot.envs` are benchmark suites (libero, metaworld, robocasa). There is no built-in "teleoperate a MuJoCo arm". |

So if your leader is a stock Feetech-servo SO-100, you already calibrated it with
`lerobot-calibrate`, and you want the sim to mirror it without adopting a GUI or ROS —
that gap is what this fills.

Two things here are less common than the rest:

- **Alignment is kept separate from calibration.** lerobot's calibration describes *the
  arm*; this repo's `alignment.json` describes *how the arm's frame relates to the
  model's*. Keeping them apart means re-measuring one never invalidates the other. Most
  projects hardcode joint directions instead, which breaks on differently-assembled arms.
- **Cube placement is measured, not guessed** — see [Cube placement](#cube-placement).

---

## What you need

**Hardware**

- An SO-100 (or SO-101) **leader** arm with Feetech STS3215 servos
- Its USB serial adapter (shows up as `USB-Enhanced-SERIAL CH343`)
- No follower arm required — the sim *is* the follower

**Software** — exact versions in [`environment.yml`](environment.yml)

| Package | Why it's here |
|---|---|
| `lerobot` 0.5.2 | Talks to the servos, owns the arm's calibration |
| `mujoco` 3.13 | Simulates and renders the arm |
| `pyserial` | Serial transport; also used to list COM ports |
| `numpy` | Array maths in the mapping layer |

A discrete GPU is effectively required for the viewer — see [Hybrid graphics](#hybrid-graphics).

---

## Install

```bash
conda env create -f environment.yml
conda activate so100-mujoco
```

Already have a working lerobot environment? Skip the file entirely — `pip install mujoco`
into it is the only thing this project adds.

Then check the machine before touching the arm:

```bash
python check.py
```

It verifies packages, which GPU OpenGL is using, both model scenes, your serial ports, and
both calibration files — and prints the exact command to fix whatever is missing.

---

## Quick start

### 1. Find the port

`check.py` lists your serial ports and flags the likely one. Or directly:

```bash
python -c "from serial.tools import list_ports; [print(p.device, p.description) for p in list_ports.comports()]"
```

### 2. Calibrate the arm (lerobot's job)

Once per physical arm. Move it to the middle of every joint's travel, then sweep each
joint through its **full** range — including squeezing the gripper trigger all the way.

```bash
lerobot-calibrate --teleop.type=so100_leader --teleop.port=COM4 --teleop.id=my_leader_arm
```

Writes `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/my_leader_arm.json`
(the folder is `so_leader`, the base class name).

> A joint you barely moved gets a tiny range and will saturate — a small nudge slams it end
> to end. `check.py` flags any joint under 500 counts. For the gripper you can re-measure
> just that one joint, see [Fixing one joint](#fixing-one-joint).

### 3. Align the arm to the model (this repo's job)

lerobot's zero and the MuJoCo model's zero are not the same place, and which way each joint
turns depends on how your arm was assembled. This measures both:

```bash
python align.py --port COM4 --id my_leader_arm
```

The viewer shows two poses. Match the physical arm to each, then press Enter **in the
terminal**.

- **Pose A sets the zero offsets — match it carefully.** Its accuracy becomes your
  positional accuracy.
- **Pose B only decides which way each joint turns** — roughly is fine.

Writes `alignment.json`.

### 4. Teleop

```bash
python teleop.py --port COM4 --id my_leader_arm                  # just the arm
python teleop.py --port COM4 --id my_leader_arm --scene cubes    # with graspable cubes
```

Move the leader arm; the sim follows. Close the viewer to stop.

| Flag | Meaning |
|---|---|
| `--scene plain` | Bare arm on a ground plane (default) |
| `--scene cubes` | Three 30 mm cubes placed within reach, plus an overhead camera |
| `--mode ctrl` | Drive the position actuators, so the sim arm has real dynamics and can push and grasp things (default) |
| `--mode qpos` | Snap joints exactly — a pure kinematic mirror, no physics, cubes cannot be picked up |

### 5. Check the sim can grasp (no hardware needed)

```bash
python test_grasp.py
```

Runs a scripted hover → descend → close → lift and reports whether the cube stayed in the
gripper. Worth running after any edit to the model.

---

## Cube placement

The cubes sit where they do because of a measurement, not a guess. Placing them by eye
tends to fail in a way that looks like broken physics but isn't.

Three constraints had to hold at once:

1. **Reachable top-down.** Sweeping the four arm joints and keeping only poses whose
   gripper points downward at cube height leaves a band densest around
   `y = -0.12..-0.24`, `|x| < 0.15`. ("In front" of this model is **-y**.)
2. **Fingertips must clear the floor.** The pads extend **25 mm past the grasp midpoint**,
   so a vertical grasp aimed at a 30 mm cube's *centre* (15 mm up) drives the fingertips
   through the ground — the arm jams before it ever reaches the cube. The lowest
   achievable grasp midpoint is about 28 mm, with the fingers straddling the cube.
3. **Within the arm's strength.** Holding torque across those poses peaks at 0.93 N·m
   against a 3.5 N·m actuator limit, so strength is not the binding constraint here.

Cubes are 30 mm; the jaw opens to 77 mm and closes to 5.6 mm, so there's room to drop over
one and still close well past its faces. `condim="4"` adds torsional friction — without it
a cube spins out from between the flat pads.

---

## Cameras

Two cameras exist, ready for ACT-style recording later:

| Camera | Where | Scene |
|---|---|---|
| `overhead` | Fixed above the cube area, looking straight down | `cubes` |
| `wrist` | On the jaw, looking out along the approach axis | both |

Render either offscreen:

```python
import mujoco, so100
model, data, _ = so100.load_model("cubes")
renderer = mujoco.Renderer(model, 480, 640)
renderer.update_scene(data, camera="wrist")
image = renderer.render()
```

Nothing records yet — that comes with the dataset recorder.

---

## How it works

`so100.py` is the whole mapping layer:

```
leader reading (degrees, gripper 0-100%)
  -> sign flip and zero offset per joint     (alignment.json)
  -> wrap, for joints that rotate freely     (Wrist_Roll)
  -> clamp to the model's joint limits
  -> MuJoCo radians
```

The gripper is special-cased: lerobot reports it as 0-100% regardless of `use_degrees`, so
it's mapped across the Jaw joint's travel rather than treated as an angle.

**Calibration vs alignment** — the distinction that trips people up:

| | `lerobot-calibrate` | `align.py` |
|---|---|---|
| Describes | Your physical arm: servo zero offsets and travel limits | How lerobot's frame relates to MuJoCo's |
| Stored in | `~/.cache/huggingface/.../my_leader_arm.json` | `alignment.json` in this repo |
| Redo it when | You rebuild or re-cable the arm | You re-run calibration, or a joint mirrors backwards |

### Joint map

| MuJoCo joint | lerobot motor | Servo ID |
|---|---|---|
| `Rotation` | `shoulder_pan` | 1 |
| `Pitch` | `shoulder_lift` | 2 |
| `Elbow` | `elbow_flex` | 3 |
| `Wrist_Pitch` | `wrist_flex` | 4 |
| `Wrist_Roll` | `wrist_roll` | 5 |
| `Jaw` | `gripper` | 6 (0-100% across the jaw's travel) |

### Local changes to the Menagerie model

`model/` is Menagerie's `trs_so_arm100` with three deliberate edits, each commented in the
XML:

- `Wrist_Roll` range widened from ±2.79 to ±3.15 rad, because this arm's gripper is mounted
  180° round and its alignment carries a π offset
- a `wrist` camera added to the `Fixed_Jaw` body
- `scene_cubes.xml` added (upstream `scene.xml` is untouched)

---

## Troubleshooting

Every problem below looked like a code bug and wasn't.

### Hybrid graphics

**Symptom:** the viewer is black, or the UI renders hugely magnified and garbled.

On a laptop with both integrated and discrete GPUs, OpenGL often lands on the integrated
one, whose driver mis-renders MuJoCo's window. Offscreen rendering still works, which is
why the code can pass every test while the window stays broken.

Check which GPU you got:

```bash
python -c "import ctypes,mujoco; c=mujoco.GLContext(64,64); c.make_current(); g=ctypes.windll.opengl32; g.glGetString.restype=ctypes.c_char_p; print(g.glGetString(0x1F01).decode())"
```

If it says AMD or Intel, pin your Python to the discrete GPU: **Windows Settings → System →
Display → Graphics**, add the `python.exe` of your conda env, set **High performance**.
Applies to new processes only. `check.py` warns about this too.

### `conda activate` is not recognised

Conda hasn't been initialised for your shell. Either run `conda init powershell` once and
open a new terminal, or call the interpreter by full path. Note that `conda activate` with
no argument activates **base**, which doesn't have these packages — name the env:
`conda activate so100-mujoco`.

### The viewer is laggy and the camera barely responds

`viewer.sync()` takes the viewer's lock, so calling it in a tight loop starves the render
thread. Both scripts here pace it to 60 Hz. If you write your own loop, do the same.

### A joint mirrors backwards

Flip that joint's `sign` in `alignment.json` and restart. No need to re-run `align.py`.
It's most likely on the gripper, whose direction test is unreliable when its calibrated
range is small.

### Fixing one joint

Re-running `lerobot-calibrate` rewrites every joint's homing offset, which invalidates
`alignment.json`. To fix just the gripper's range:

```bash
python gripper_range.py --port COM4 --id my_leader_arm
```

Sweep the trigger through its full travel and press Enter. It writes only `range_min` and
`range_max` for the gripper, and refuses to write if it saw under 200 counts of travel.

### The sim arm lags behind the real one

Expected in `--mode ctrl`: the simulated arm is a physical system chasing a position
target, so it trails slightly under load. Use `--mode qpos` for an exact mirror without
physics.

---

## Files

```
so100.py          joint map, alignment load/save, leader reading -> MuJoCo radians
align.py          two-pose alignment, writes alignment.json
teleop.py         read leader -> set ctrl/qpos -> step -> sync viewer
check.py          preflight: packages, GPU, ports, calibration, alignment
gripper_range.py  re-measure just the gripper's travel
test_grasp.py     scripted pick-up self-test, no hardware needed
model/            trs_so_arm100 from MuJoCo Menagerie, plus scene_cubes.xml
alignment.json    generated by align.py -- specific to your arm, not tracked in git
alignment.example.json   the shape align.py writes, with neutral values
```

---

## Roadmap

- [x] Leader → MuJoCo teleop
- [x] Cube scene within reach, verified graspable
- [x] Overhead and wrist cameras
- [ ] Dataset recorder writing `LeRobotDataset` format, so the same data can train
      lerobot's ACT *and* a from-scratch implementation for comparison

---

## Licence

Dual-licensed under either of

- Apache License 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT License ([LICENSE-MIT](LICENSE-MIT))

at your option. Contributions are accepted under the same dual licence.

`model/` is redistributed from MuJoCo Menagerie under Apache 2.0 — its own
[LICENSE](model/LICENSE) applies there, and the modifications are listed under
[Local changes to the Menagerie model](#local-changes-to-the-menagerie-model).

---

## Credits

- [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) — the arm
- [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) — the model in
  `model/` (Apache 2.0, see `model/LICENSE`)
- [lerobot](https://github.com/huggingface/lerobot) — servo comms and calibration
