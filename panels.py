"""Camera feeds and a data readout, drawn on top of the MuJoCo viewer.

The viewer's own 3D view stays interactive; each model camera is rendered
offscreen and composited into a corner as a picture-in-picture panel, with a
text block underneath showing what the arm is doing.

Used by teleop.py, but it only needs a model and a data, so it works with any
loop that steps the simulation.
"""

import time

import mujoco

# Panels scale with the visible area, within these bounds. The model's
# offscreen buffer is 640x480, so they must stay inside that.
MIN_PANEL_WIDTH, MAX_PANEL_WIDTH = 240, 480
# Fraction of the visible width one panel may take.
PANEL_FRACTION = 3
MARGIN = 10
FONT = mujoco.mjtFontScale.mjFONTSCALE_150
GRID = mujoco.mjtGridPos.mjGRID_BOTTOMLEFT


class Panels:
    """Renders the model's cameras and composites them onto a viewer handle."""

    def __init__(self, model, cameras=None, fps=20.0):
        self.model = model
        if cameras is None:
            cameras = [
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
                for i in range(model.ncam)
            ]
        self.cameras = cameras
        self.interval = 1.0 / fps if fps > 0 else 0.0
        self._next_render = 0.0
        self._renderer = None
        self._size = (0, 0)
        self._frames = []
        self._render_ms = 0.0

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def _panel_size(self, viewport):
        """Pick a panel size that fits the visible 3D area.

        The viewport shrinks when the viewer's own UI panels are open -- at a
        large font they can take most of the window -- so a fixed size would
        cover the scene entirely.
        """
        width = int(min(MAX_PANEL_WIDTH, max(MIN_PANEL_WIDTH, viewport.width // PANEL_FRACTION)))
        width -= width % 4
        return width, int(width * 3 / 4)

    def _ensure_renderer(self, size):
        # Rebuilding costs a couple of ms, so only do it when the size really moved.
        if self._renderer is None or abs(size[0] - self._size[0]) > 16:
            if self._renderer is not None:
                self._renderer.close()
            self._renderer = mujoco.Renderer(self.model, size[1], size[0])
            self._size = size
            self._frames = []
        return self._renderer

    def update(self, viewer, data, stats=None):
        """Re-render the camera panels and push them, plus stats, to the viewer.

        Rendering is paced independently of the caller's loop: both cameras cost
        about 3 ms, which is cheap next to a physics step but not free at the
        rate a serial read loop runs at.
        """
        viewport = viewer.viewport
        size = self._panel_size(viewport)

        now = time.perf_counter()
        if self.cameras and now >= self._next_render:
            renderer = self._ensure_renderer(size)
            start = now
            frames = []
            for camera in self.cameras:
                renderer.update_scene(data, camera=camera)
                frames.append(renderer.render().copy())
            self._frames = frames
            self._render_ms = (time.perf_counter() - start) * 1000
            self._next_render = now + self.interval

        if self._frames:
            width, height = self._size
            overlays = []
            for i, image in enumerate(self._frames):
                # viewport is the 3D scene rect, not the window: its origin
                # moves when the viewer's own UI panels open. Anchor to its
                # top-right corner and stack downward from there.
                rect = mujoco.MjrRect(
                    viewport.left + viewport.width - width - MARGIN,
                    viewport.bottom + viewport.height - (i + 1) * (height + MARGIN),
                    width,
                    height,
                )
                if rect.bottom < viewport.bottom:      # ran out of room
                    break
                overlays.append((rect, image))
            if overlays:
                viewer.set_images(overlays)

        if stats is not None:
            labels, values = self._format(stats)
            viewer.set_texts([(FONT, GRID, labels, values)])

    def _format(self, stats):
        labels = []
        values = []
        for name, angle in stats.get("joints", {}).items():
            labels.append(name)
            values.append(f"{angle:+7.1f} deg")
        if "gripper" in stats:
            labels.append("gripper")
            values.append(f"{stats['gripper']:5.1f} %")
        labels.append("")
        values.append("")
        if "read_hz" in stats:
            labels.append("leader")
            values.append(f"{stats['read_hz']:5.1f} Hz")
        labels.append("panels")
        values.append(f"{self._render_ms:4.1f} ms" + (
            f" ({', '.join(self.cameras)})" if self.cameras else " (no cameras)"))
        return "\n".join(labels), "\n".join(values)
