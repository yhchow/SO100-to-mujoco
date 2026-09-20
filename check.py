"""Preflight check: is this machine ready to run the teleop?

Checks the environment, the graphics stack, the serial port, and both
calibration files, and explains how to fix whatever is missing.

    python check.py
"""

import argparse
import importlib
import json
import pathlib
import sys

OK, WARN, FAIL = "  [ok]  ", "  [warn]", "  [FAIL]"


def report(status, message, fix=None):
    print(f"{status} {message}")
    if fix and status != OK:
        print(f"         -> {fix}")
    return status != FAIL


def check_packages():
    print("\nPackages")
    ok = True
    print(f"  [ok]   python {sys.version.split()[0]} at {sys.executable}")
    for module, minimum in [("lerobot", "0.5"), ("mujoco", "3."), ("serial", None), ("numpy", None)]:
        try:
            mod = importlib.import_module(module)
            version = getattr(mod, "__version__", "?")
            if minimum and not str(version).startswith(minimum):
                ok &= report(WARN, f"{module} {version} (expected {minimum}x)",
                             "see environment.yml; other versions may work but are untested")
            else:
                report(OK, f"{module} {version}")
        except ImportError:
            ok &= report(FAIL, f"{module} is not installed",
                         "conda env create -f environment.yml, then conda activate so100-mujoco")
    return ok


def check_graphics():
    print("\nGraphics")
    try:
        import ctypes
        import mujoco
        ctx = mujoco.GLContext(64, 64)
        ctx.make_current()
        gl = ctypes.windll.opengl32
        gl.glGetString.restype = ctypes.c_char_p
        renderer = gl.glGetString(0x1F01).decode()
    except Exception as exc:                      # noqa: BLE001 - diagnostics only
        return report(WARN, f"could not query OpenGL ({exc})", "the viewer may still work")
    if any(tag in renderer.upper() for tag in ("AMD", "RADEON", "INTEL")):
        return report(WARN, f"OpenGL is using {renderer}",
                      "integrated GPUs here render the viewer black/garbled. Pin python.exe to "
                      "the discrete GPU: Windows Settings > Display > Graphics > add your "
                      "python.exe > High performance. See README 'Hybrid graphics'.")
    return report(OK, f"OpenGL renderer: {renderer}")


def check_model():
    print("\nModel")
    try:
        import so100
        ok = True
        for scene in so100.SCENES:
            model, _, _ = so100.load_model(scene)
            ok &= report(OK, f"scene '{scene}' loads ({model.nbody} bodies, {model.ncam} cameras)")
        return ok
    except Exception as exc:                      # noqa: BLE001
        return report(FAIL, f"model failed to load: {exc}", "is model/ intact?")


def check_ports():
    print("\nSerial ports")
    try:
        from serial.tools import list_ports
    except ImportError:
        return report(FAIL, "pyserial missing", "conda env create -f environment.yml")
    ports = list(list_ports.comports())
    if not ports:
        return report(WARN, "no serial ports found", "plug in the leader arm and re-run")
    for p in ports:
        tag = "  <-- looks like the arm" if (p.vid == 0x1A86 or "CH34" in (p.description or "")) else ""
        report(OK, f"{p.device}: {p.description}{tag}")
    return True


def check_calibration(robot_id):
    print("\nlerobot calibration")
    try:
        from lerobot.utils.constants import HF_LEROBOT_CALIBRATION
        base = pathlib.Path(HF_LEROBOT_CALIBRATION)
    except Exception:                             # noqa: BLE001
        base = pathlib.Path.home() / ".cache/huggingface/lerobot/calibration"
    path = base / "teleoperators" / "so_leader" / f"{robot_id}.json"
    if not path.exists():
        return report(FAIL, f"no calibration for id '{robot_id}'",
                      f"lerobot-calibrate --teleop.type=so100_leader --teleop.port=COMx "
                      f"--teleop.id={robot_id}")
    data = json.loads(path.read_text())
    report(OK, f"found {path}")
    clean = True
    for joint, entry in data.items():
        span = entry["range_max"] - entry["range_min"]
        if span < 500:
            clean = False
            report(WARN, f"{joint}: only {span} counts of range",
                         "that joint was barely moved during calibration, so it will saturate. "
                         + ("Re-measure with: python gripper_range.py --port COMx"
                            if joint == "gripper" else "Re-run lerobot-calibrate."))
    if clean:
        report(OK, "all joints have a usable range")
    return True


def check_alignment():
    print("\nAlignment to the MuJoCo frame")
    import so100
    if not so100.ALIGNMENT_PATH.exists():
        return report(FAIL, "alignment.json missing",
                      "python align.py --port COMx --id my_leader_arm")
    data = json.loads(so100.ALIGNMENT_PATH.read_text())
    missing = [n for n in so100.JOINT_NAMES if n not in data]
    if missing:
        return report(FAIL, f"alignment.json is missing {missing}", "re-run align.py")
    flipped = [n for n, e in data.items() if e["sign"] < 0]
    report(OK, f"alignment.json present (joints reversed: {', '.join(flipped) or 'none'})")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", default="my_leader_arm", help="lerobot calibration id to look for")
    args = parser.parse_args()

    results = [check_packages(), check_graphics(), check_model(),
               check_ports(), check_calibration(args.id), check_alignment()]
    print()
    if all(results):
        print("Ready. Run: python teleop.py --port COMx --id " + args.id + " --scene cubes")
        return 0
    print("Some checks failed -- see the arrows above for what to do.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
