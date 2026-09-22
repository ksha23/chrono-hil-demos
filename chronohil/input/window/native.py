"""Reading the 3D window the way Chrono intends, once PyChrono lets you.

THIS IS THE VERSION THAT NEEDS NO OPERATING SYSTEM AT ALL. Irrlicht already
delivers mouse and key events in window coordinates; the only reason Python
cannot receive them is that SWIG emits irr::IEventReceiver as an abstract class
with no constructor, so it can be named and not implemented:

    >>> irr.IEventReceiver()
    AttributeError: No constructor defined - class is abstract

One line in the interface file fixes it, and the module is already generated
with directors enabled:

    %feature("director") irr::IEventReceiver;

With that in, this file works on every platform Irrlicht supports and the
macOS sibling next door becomes dead weight. Without it, `available()` returns
False and nothing here is used.
"""

from .camera import CameraRay


def available():
    """True when this PyChrono can SUBCLASS IEventReceiver.

    Instantiating the base is the wrong test and gave the wrong answer on both
    builds. Without the director it raises AttributeError, because SWIG emitted
    an abstract class with no constructor. WITH the director it still raises,
    now RuntimeError, because OnEvent is pure virtual and directors are honest
    about that. The question is whether a subclass can be made, so ask that.
    """
    try:
        import pychrono.irrlicht as irr

        class _Probe(irr.IEventReceiver):
            def OnEvent(self, ev):
                return False

        _Probe()
    except Exception:
        return False
    return True


if available():                      # pragma: no cover - depends on the build
    import pychrono.irrlicht as irr

    class _Receiver(irr.IEventReceiver):
        """Everything the demos need, straight from Irrlicht."""

        def __init__(self):
            super().__init__()
            self.x = self.y = 0
            self.down = False
            self.right = False
            self.middle = False
            self.pressed = False
            self.keys = set()

        def OnEvent(self, ev):
            if ev.EventType == irr.EET_MOUSE_INPUT_EVENT:
                self.x, self.y = ev.MouseInput.X, ev.MouseInput.Y
                if ev.MouseInput.Event == irr.EMIE_LMOUSE_PRESSED_DOWN:
                    self.down = self.pressed = True
                elif ev.MouseInput.Event == irr.EMIE_LMOUSE_LEFT_UP:
                    self.down = False
                elif ev.MouseInput.Event == irr.EMIE_RMOUSE_PRESSED_DOWN:
                    self.right = True
                elif ev.MouseInput.Event == irr.EMIE_RMOUSE_LEFT_UP:
                    self.right = False
                elif ev.MouseInput.Event == irr.EMIE_MMOUSE_PRESSED_DOWN:
                    self.middle = True
                elif ev.MouseInput.Event == irr.EMIE_MMOUSE_LEFT_UP:
                    self.middle = False
            elif ev.EventType == irr.EET_KEY_INPUT_EVENT:
                k = int(ev.KeyInput.Key)
                self.keys.add(k) if ev.KeyInput.PressedDown else self.keys.discard(k)
            return False             # let Chrono's own GUI see it too
else:
    _Receiver = None


class NativeWindowInput(CameraRay):
    """The same surface the OS backends offer, with none of the OS calls."""

    # Irrlicht key codes, so no platform table is needed
    K = {"left": 0x25, "up": 0x26, "right": 0x27, "down": 0x28,
         "a": 0x41, "c": 0x43, "d": 0x44, "f": 0x46, "r": 0x52, "s": 0x53, "t": 0x54, "w": 0x57, "x": 0x58, "z": 0x5A,
         "lbracket": 0xDB, "rbracket": 0xDD, "esc": 0x1B, "space": 0x20}
    EDGE = {"z": "f", "x": "n", "c": "r", "t": "m",
            "rbracket": "u", "lbracket": "d", "esc": "q"}

    def __init__(self, vis, title, content_w, content_h, edges=None):
        if _Receiver is None:
            raise RuntimeError("this PyChrono cannot subclass IEventReceiver")
        self.vis, self.cw, self.ch = vis, content_w, content_h
        self.rx = _Receiver()
        vis.AddUserEventReceiver(self.rx)
        self.commands = []
        if edges is not None:
            self.EDGE = dict(edges)
        self.prev = {k: False for k in self.EDGE}
        self.prev_mouse = False
        self.last = (0.0, 0.0, 0.0)
        print("[input] reading the 3D window through Irrlicht (no OS calls)")

    # -- the same methods the macOS backend exposes --------------------------
    def _key(self, name):
        return self.K[name] in self.rx.keys

    def mouse_down(self):
        return self.rx.down

    def mouse_right_down(self):
        return self.rx.right

    def mouse_middle_down(self):
        return self.rx.middle

    def cursor_pixel(self):
        """Already window-relative, which is the whole point."""
        if 0 <= self.rx.x < self.cw and 0 <= self.rx.y < self.ch:
            return self.rx.x, self.rx.y
        return None

    # ray_through() comes from CameraRay, shared with every OS backend.

    def poll(self):
        for name, cmd in self.EDGE.items():
            now = self._key(name)
            if now and not self.prev[name]:
                self.commands.append(cmd)
            self.prev[name] = now
        steer = (-1.0 if self._key("left") else 0.0) + (1.0 if self._key("right") else 0.0)
        self.last = (steer, 1.0 if self._key("up") else 0.0,
                     1.0 if self._key("down") else 0.0)
        return self.last

    def take_commands(self):
        c, self.commands = self.commands, []
        return c

    def camera_nudge(self):
        return ((-1.0 if self._key("a") else 0.0) + (1.0 if self._key("d") else 0.0),
                (-1.0 if self._key("w") else 0.0) + (1.0 if self._key("s") else 0.0),
                (-1.0 if self._key("f") else 0.0) + (1.0 if self._key("r") else 0.0))
