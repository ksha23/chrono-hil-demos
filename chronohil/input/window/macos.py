# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of this repository and at
# http://projectchrono.org/license-chrono.txt.
# =============================================================================
"""Reading the 3D window via CoreGraphics (ctypes, no pyobjc dependency).
Queries: CGEventGetLocation, CGEventSourceButtonState, CGEventSourceKeyState,
CGWindowListCopyWindowInfo. No accessibility permission needed.
"""

import ctypes
import ctypes.util

from .camera import CameraRay

WHAT = "macOS"

# CoreGraphics constants, from CGEventTypes.h and CGWindow.h.
_HID_STATE = 1                  # kCGEventSourceStateHIDSystemState
_ON_SCREEN = 1 << 0             # kCGWindowListOptionOnScreenOnly
_NO_DESKTOP = 1 << 4            # kCGWindowListExcludeDesktopElements
_NULL_WINDOW = 0                # kCGNullWindowID
_UTF8 = 0x08000100              # kCFStringEncodingUTF8


class _Point(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]


class _Size(ctypes.Structure):
    _fields_ = [("width", ctypes.c_double), ("height", ctypes.c_double)]


class _Rect(ctypes.Structure):
    _fields_ = [("origin", _Point), ("size", _Size)]


_LIBS = None


def _frameworks():
    """(CoreGraphics, CoreFoundation), loaded once and declared once.

    EVERY SIGNATURE IS SPELLED OUT. ctypes defaults an undeclared return to
    c_int, which truncates a 64-bit pointer to 32 bits and hands back an
    address that is usually invalid and occasionally not -- the failure is a
    crash somewhere else entirely, minutes later. The three that matter most
    are CGEventCreate and the two CF getters, all of which return pointers.
    """
    global _LIBS
    if _LIBS is not None:
        return _LIBS
    cg = ctypes.cdll.LoadLibrary(ctypes.util.find_library("CoreGraphics"))
    cf = ctypes.cdll.LoadLibrary(ctypes.util.find_library("CoreFoundation"))

    cg.CGEventCreate.restype = ctypes.c_void_p
    cg.CGEventCreate.argtypes = [ctypes.c_void_p]
    cg.CGEventGetLocation.restype = _Point            # returned BY VALUE
    cg.CGEventGetLocation.argtypes = [ctypes.c_void_p]
    cg.CGEventSourceButtonState.restype = ctypes.c_bool
    cg.CGEventSourceButtonState.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
    cg.CGEventSourceKeyState.restype = ctypes.c_bool
    cg.CGEventSourceKeyState.argtypes = [ctypes.c_uint32, ctypes.c_uint16]
    cg.CGWindowListCopyWindowInfo.restype = ctypes.c_void_p
    cg.CGWindowListCopyWindowInfo.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
    cg.CGRectMakeWithDictionaryRepresentation.restype = ctypes.c_bool
    cg.CGRectMakeWithDictionaryRepresentation.argtypes = [ctypes.c_void_p,
                                                          ctypes.POINTER(_Rect)]
    cf.CFRelease.argtypes = [ctypes.c_void_p]
    cf.CFArrayGetCount.restype = ctypes.c_long
    cf.CFArrayGetCount.argtypes = [ctypes.c_void_p]
    cf.CFArrayGetValueAtIndex.restype = ctypes.c_void_p
    cf.CFArrayGetValueAtIndex.argtypes = [ctypes.c_void_p, ctypes.c_long]
    cf.CFDictionaryGetValue.restype = ctypes.c_void_p
    cf.CFDictionaryGetValue.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    cf.CFStringGetCString.restype = ctypes.c_bool
    cf.CFStringGetCString.argtypes = [ctypes.c_void_p, ctypes.c_char_p,
                                      ctypes.c_long, ctypes.c_uint32]
    # The dictionary keys are exported CFStringRef globals, not string
    # literals: CFDictionaryGetValue compares by pointer for these.
    keys = (ctypes.c_void_p.in_dll(cg, "kCGWindowName"),
            ctypes.c_void_p.in_dll(cg, "kCGWindowBounds"))
    _LIBS = (cg, cf, keys)
    return _LIBS


def unsupported():
    """A reason this machine cannot answer, or None when it can."""
    try:
        _frameworks()
    except Exception as exc:                 # no CoreGraphics is not a Mac
        return f"CoreGraphics did not load ({exc})"
    return None


def _cfstr(cf, ref):
    """A CFStringRef as a str, or None. The buffer is ours, so no release."""
    if not ref:
        return None
    buf = ctypes.create_string_buffer(512)
    if cf.CFStringGetCString(ref, buf, 512, _UTF8):
        return buf.value.decode("utf-8", "replace")
    return None


def open_input(vis, title, content_w, content_h, edges=None, quiet=False):
    """A macOS backend, or None when this session cannot answer. Never raises."""
    why = unsupported()
    if why is None:
        try:
            return MacOSWindowInput(vis, title, content_w, content_h, edges=edges)
        except Exception as exc:
            why = f"macOS said no ({exc})"
    if not quiet:
        print(f"[input] {why}")
    return None


class MacOSWindowInput(CameraRay):
    """Mouse and keyboard read from the OS, so they work ON the 3D window."""

    # macOS virtual key codes. This table has to cover every name any demo's
    # `edges` can mention, because a backend is composed rather than
    # subclassed: a demo that inherited from this class and added its own key
    # stopped doing so, the key went missing, and the first poll() raised
    # KeyError on the Mac while Linux never noticed.
    K = {"left": 123, "right": 124, "down": 125, "up": 126, "space": 49,
         "z": 6, "x": 7, "c": 8, "t": 17, "lbracket": 33, "rbracket": 30,
         # camera, deliberately on keys so the mouse stays free for grabbing
         "a": 0, "d": 2, "w": 13, "s": 1, "r": 15, "f": 3,
         "esc": 53}
    EDGE = {"z": "f", "x": "n", "c": "r", "t": "m", "rbracket": "u", "lbracket": "d",
            "esc": "q"}

    def __init__(self, vis, title, content_w, content_h, edges=None):
        self.cg, self.cf, self.keys = _frameworks()
        self.vis, self.title = vis, title
        self.cw, self.ch = content_w, content_h
        self.commands = []
        if edges is not None:
            self.EDGE = dict(edges)
        self.prev = {k: False for k in self.EDGE}
        self.prev_mouse = False
        self.last = (0.0, 0.0, 0.0)
        self._rect = None
        self._rect_age = 0
        print("[input] reading mouse and keys from the OS - click straight on the 3D window")

    # -- OS state ------------------------------------------------------------
    def _key(self, name):
        return bool(self.cg.CGEventSourceKeyState(_HID_STATE, self.K[name]))

    def mouse_down(self):
        return bool(self.cg.CGEventSourceButtonState(_HID_STATE, 0))

    def window_rect(self):
        """Screen rect of our Irrlicht window, refreshed occasionally (it can move)."""
        self._rect_age -= 1
        if self._rect is not None and self._rect_age > 0:
            return self._rect
        cg, cf, (k_name, k_bounds) = self.cg, self.cf, self.keys
        arr = cg.CGWindowListCopyWindowInfo(_ON_SCREEN | _NO_DESKTOP, _NULL_WINDOW)
        if not arr:
            return self._rect
        want = self.title.lower()
        try:
            # COPY in the name means this array is ours to release, and it is
            # rebuilt on every call. Leaking it once per physics step is a
            # leak of every on-screen window's metadata, 500 times a second.
            for i in range(cf.CFArrayGetCount(arr)):
                d = cf.CFArrayGetValueAtIndex(arr, i)
                name = _cfstr(cf, cf.CFDictionaryGetValue(d, k_name))
                if not name or want not in name.lower():
                    continue
                b = cf.CFDictionaryGetValue(d, k_bounds)
                r = _Rect()
                if b and cg.CGRectMakeWithDictionaryRepresentation(b, ctypes.byref(r)):
                    self._rect = (r.origin.x, r.origin.y, r.size.width, r.size.height)
                    self._rect_age = 60
                    return self._rect
        finally:
            cf.CFRelease(arr)
        return self._rect

    def cursor_pixel(self):
        """Cursor in window content pixels, or None when it is outside.

        CGEventGetLocation counts DOWN from the top of the main display, which
        is the same convention kCGWindowBounds uses, so the two subtract
        directly. `chrome` is the title bar: the rect covers the whole window
        and `ch` is only the part Irrlicht draws into.
        """
        r = self.window_rect()
        if r is None:
            return None
        wx, wy, _ww, wh = r
        ev = self.cg.CGEventCreate(None)
        if not ev:
            return None
        try:
            loc = self.cg.CGEventGetLocation(ev)
        finally:
            self.cf.CFRelease(ev)
        px = loc.x - wx
        py = loc.y - wy - (wh - self.ch)
        if 0 <= px < self.cw and 0 <= py < self.ch:
            return px, py
        return None

    # -- the ray Irrlicht would not give us ----------------------------------
    # ray_through() comes from CameraRay: identical on every backend, so it
    # lives in camera.py rather than three times over. See that file.

    # -- the surface the demo loops poll ------------------------------------
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

    def mouse_right_down(self):
        """The RIGHT button, for orbiting the camera by dragging.

        CoreGraphics numbers buttons 0 left, 1 right. Same global-state caveat
        as mouse_down: this is the button anywhere on the desktop, and the
        caller is expected to have checked the cursor is over our window first.
        """
        return bool(self.cg.CGEventSourceButtonState(_HID_STATE, 1))

    def mouse_middle_down(self):
        """The MIDDLE button, for panning. CoreGraphics numbers it 2."""
        return bool(self.cg.CGEventSourceButtonState(_HID_STATE, 2))

    def camera_nudge(self):
        """(orbit, zoom, rise) from A/D, W/S, R/F -- held, not edge-triggered."""
        return ((-1.0 if self._key("a") else 0.0) + (1.0 if self._key("d") else 0.0),
                (-1.0 if self._key("w") else 0.0) + (1.0 if self._key("s") else 0.0),
                (-1.0 if self._key("f") else 0.0) + (1.0 if self._key("r") else 0.0))

    def send(self, text):
        pass
