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
"""Reading the 3D window via X11 (ctypes, no python-xlib dependency).
Queries: XQueryPointer, XQueryKeymap, XQueryTree, XGetGeometry.
Wayland cannot do any of this by design -- returns None there.
"""

import ctypes
import os
import time
from ctypes import (CFUNCTYPE, POINTER, byref, c_char, c_char_p, c_int, c_long,
                    c_ubyte, c_uint, c_ulong, c_void_p)

from .camera import CameraRay

WHAT = "X11"

# Xlib ctypes bindings. All prototypes spelled out because ctypes defaults
# undeclared args to int, which truncates 64-bit XIDs.
XID = c_ulong
_ANY_PROPERTY_TYPE = 0
_SUCCESS = 0
_BUTTON2_MASK = 1 << 9          # X.h: Button2Mask, the middle button
_BUTTON3_MASK = 1 << 10         # X.h: Button3Mask, the right button
_BUTTON1_MASK = 1 << 8          # X.h: Button1Mask, the left button in the
                                # pointer mask XQueryPointer hands back

_ERRHANDLER = CFUNCTYPE(c_int, c_void_p, c_void_p)

_LIB = None                     # the CDLL, loaded at most once per process
_QUIET_ERRORS = None            # keep a ref or ctypes frees the trampoline
_SERVER = None                  # one connection, shared by every consumer


def _declare(x):
    """Prototypes. See the note on XID above for why this is not optional."""
    x.XOpenDisplay.argtypes = [c_char_p]
    x.XOpenDisplay.restype = c_void_p
    x.XDefaultRootWindow.argtypes = [c_void_p]
    x.XDefaultRootWindow.restype = XID
    x.XInternAtom.argtypes = [c_void_p, c_char_p, c_int]
    x.XInternAtom.restype = XID
    x.XFree.argtypes = [c_void_p]
    x.XFree.restype = c_int
    x.XSetErrorHandler.argtypes = [_ERRHANDLER]
    x.XSetErrorHandler.restype = c_void_p
    x.XQueryPointer.argtypes = [c_void_p, XID, POINTER(XID), POINTER(XID),
                                POINTER(c_int), POINTER(c_int), POINTER(c_int),
                                POINTER(c_int), POINTER(c_uint)]
    x.XQueryPointer.restype = c_int
    x.XQueryTree.argtypes = [c_void_p, XID, POINTER(XID), POINTER(XID),
                             POINTER(POINTER(XID)), POINTER(c_uint)]
    x.XQueryTree.restype = c_int
    x.XFetchName.argtypes = [c_void_p, XID, POINTER(c_char_p)]
    x.XFetchName.restype = c_int
    x.XGetWindowProperty.argtypes = [c_void_p, XID, XID, c_long, c_long, c_int,
                                     XID, POINTER(XID), POINTER(c_int),
                                     POINTER(c_ulong), POINTER(c_ulong),
                                     POINTER(POINTER(c_ubyte))]
    x.XGetWindowProperty.restype = c_int
    x.XGetGeometry.argtypes = [c_void_p, XID, POINTER(XID), POINTER(c_int),
                               POINTER(c_int), POINTER(c_uint), POINTER(c_uint),
                               POINTER(c_uint), POINTER(c_uint)]
    x.XGetGeometry.restype = c_int
    x.XTranslateCoordinates.argtypes = [c_void_p, XID, XID, c_int, c_int,
                                        POINTER(c_int), POINTER(c_int),
                                        POINTER(XID)]
    x.XTranslateCoordinates.restype = c_int
    x.XQueryKeymap.argtypes = [c_void_p, c_char * 32]
    x.XQueryKeymap.restype = c_int
    x.XKeysymToKeycode.argtypes = [c_void_p, XID]
    x.XKeysymToKeycode.restype = c_ubyte


def _libx11():
    global _LIB
    if _LIB is not None:
        return _LIB
    # Soname first (reuses Irrlicht's mapped copy); find_library as fallback.
    err = None
    for name in ("libX11.so.6", "libX11.so"):
        try:
            _LIB = ctypes.CDLL(name)
            break
        except OSError as exc:
            err = exc
    if _LIB is None:
        from ctypes.util import find_library
        found = find_library("X11")
        if found is None:
            raise OSError(f"libX11 is not loadable ({err})")
        _LIB = ctypes.CDLL(found)
    _declare(_LIB)
    return _LIB


def unsupported():
    """Why this session cannot be read, or None when it is worth trying.

    Checked BEFORE anything is loaded or opened, so the answer is a sentence
    about the session rather than whatever obscure way libX11 would have
    failed three calls later.
    """
    session = os.environ.get("XDG_SESSION_TYPE", "").lower()
    if session == "wayland" or (os.environ.get("WAYLAND_DISPLAY")
                                and not os.environ.get("DISPLAY")):
        return ("this is a Wayland session. No Wayland client may ask where the\n"
                "        cursor is while it is over someone else's surface, nor where\n"
                "        another window sits, so in-window picking cannot be built here")
    if not os.environ.get("DISPLAY"):
        return "DISPLAY is not set, so there is no X server to ask"
    return None


class _Server:
    """One connection to the X server, and the questions asked through it.

    OUR OWN connection, deliberately: Irrlicht has a Display of its own and
    Xlib connections are not thread-safe to share. One more file descriptor is
    the whole cost.
    """

    def __init__(self):
        global _QUIET_ERRORS
        self.x = _libx11()
        if _QUIET_ERRORS is None:
            # Xlib's DEFAULT error handler prints and then calls exit(1). This
            # backend walks a window tree that every other program on the
            # display is also changing: a window can be destroyed between
            # XQueryTree listing it and XFetchName asking its name, and that
            # BadWindow would take the demo down with it at a random moment
            # with no connection to anything the user did. Ours returns 0 and
            # the racing call simply comes back empty.
            # It is process-global -- it replaces the handler Irrlicht would
            # otherwise inherit, whose entire behaviour is to exit.
            _QUIET_ERRORS = _ERRHANDLER(lambda dpy, ev: 0)
            self.x.XSetErrorHandler(_QUIET_ERRORS)
        dpy = self.x.XOpenDisplay(None)
        if not dpy:
            raise OSError("XOpenDisplay failed for DISPLAY="
                          f"{os.environ.get('DISPLAY', '')!r} (xauth, maybe?)")
        self.dpy = c_void_p(dpy)
        self.root = self.x.XDefaultRootWindow(self.dpy)
        self.atom = {n: self.x.XInternAtom(self.dpy, n.encode(), False)
                     for n in ("_NET_WM_NAME", "_NET_CLIENT_LIST", "WM_STATE",
                               "UTF8_STRING")}

    # -- properties ----------------------------------------------------------
    def _prop(self, win, prop, want_type=_ANY_PROPERTY_TYPE, words=1024):
        """(format, bytes) of a window property, or (0, b"")."""
        kind, fmt = XID(), c_int()
        n, rest = c_ulong(), c_ulong()
        data = POINTER(c_ubyte)()
        r = self.x.XGetWindowProperty(self.dpy, win, prop, 0, words, False,
                                      want_type, byref(kind), byref(fmt),
                                      byref(n), byref(rest), byref(data))
        if r != _SUCCESS:
            return 0, b""
        # A format-32 property comes back as an array of C long, which is 64
        # bits here, NOT 32. Reading it as 4-byte words is the classic way to
        # get every second window id back as zero.
        step = {8: 1, 16: 2, 32: ctypes.sizeof(c_ulong)}.get(fmt.value, 1)
        raw = bytes(bytearray(data[i] for i in range(n.value * step))) if data else b""
        if data:
            self.x.XFree(data)      # Xlib allocates even for an empty property
        return fmt.value, raw

    def _name(self, win):
        """The window's title, _NET_WM_NAME (UTF-8) before WM_NAME (Latin-1)."""
        fmt, raw = self._prop(win, self.atom["_NET_WM_NAME"],
                              self.atom["UTF8_STRING"])
        if raw:
            return raw.split(b"\0", 1)[0].decode("utf-8", "replace")
        text = c_char_p()
        if self.x.XFetchName(self.dpy, win, byref(text)) and text.value:
            out = text.value.decode("latin-1", "replace")
            self.x.XFree(text)
            return out
        return ""

    def _children(self, win):
        root, parent = XID(), XID()
        kids = POINTER(XID)()
        n = c_uint()
        if not self.x.XQueryTree(self.dpy, win, byref(root), byref(parent),
                                 byref(kids), byref(n)):
            return []
        out = [kids[i] for i in range(n.value)]
        if kids:
            self.x.XFree(kids)
        return out

    # -- finding a window by its title ---------------------------------------
    def find(self, title):
        """The id of the window whose title contains `title`, or None.

        Substring and case-insensitive, the same test the macOS backend makes,
        because the demos hand over the whole title they set and Chrono is free
        to decorate it.
        """
        want = title.lower()
        for win in self._clients():
            if want in self._name(win).lower():
                return win
        return None

    def _clients(self):
        """Candidate top-level windows, best source first.

        _NET_CLIENT_LIST is what an EWMH window manager publishes on the root
        window: the CLIENT windows it manages, already unwrapped from their
        title-bar frames. That matters more than it sounds. Under a reparenting
        WM our window is a grandchild of the root, and if we matched the frame
        instead of the client, every cursor position would be off by the height
        of the title bar -- a bug that looks like bad picking maths rather than
        like the wrong window.

        Without a WM (a bare X server, or Xvfb in CI) there is no such property
        and no frame either, so fall back to walking the tree and preferring
        whatever carries WM_STATE, which is the ICCCM mark of a client window.
        """
        fmt, raw = self._prop(self.root, self.atom["_NET_CLIENT_LIST"])
        if raw:
            n = len(raw) // ctypes.sizeof(c_ulong)
            listed = (c_ulong * n).from_buffer_copy(raw)
            return list(listed)
        seen, out, stack = set(), [], [(self.root, 0)]
        while stack:
            win, depth = stack.pop()
            if depth > 6:            # deeper than any WM nests a frame
                continue
            for kid in self._children(win):
                if kid in seen:
                    continue
                seen.add(kid)
                out.append(kid)
                stack.append((kid, depth + 1))
        out.sort(key=lambda w: 0 if self._prop(w, self.atom["WM_STATE"])[1] else 1)
        return out

    # -- the pointer ---------------------------------------------------------
    def pointer(self, win):
        """(x, y, mask) in `win`'s pixels, or None if it could not be asked.

        The server does the subtraction, which is the nicest thing X11 does for
        this backend: no window rect has to be fetched, and the answer stays
        right the instant the window is moved.
        """
        root, child = XID(), XID()
        rx, ry, wx, wy = c_int(), c_int(), c_int(), c_int()
        mask = c_uint()
        ok = self.x.XQueryPointer(self.dpy, win, byref(root), byref(child),
                                  byref(rx), byref(ry), byref(wx), byref(wy),
                                  byref(mask))
        if not ok:
            return None
        return wx.value, wy.value, mask.value

    def size(self, win):
        """(width, height) of the window's contents, or None."""
        root = XID()
        x, y = c_int(), c_int()
        w, h, bw, depth = c_uint(), c_uint(), c_uint(), c_uint()
        if not self.x.XGetGeometry(self.dpy, win, byref(root), byref(x), byref(y),
                                   byref(w), byref(h), byref(bw), byref(depth)):
            return None
        return w.value, h.value

    def origin(self, win):
        """(x, y) of the window's top-left corner on the screen, or None."""
        ax, ay = c_int(), c_int()
        child = XID()
        if not self.x.XTranslateCoordinates(self.dpy, win, self.root, 0, 0,
                                            byref(ax), byref(ay), byref(child)):
            return None
        return ax.value, ay.value

    def keymap(self, into):
        self.x.XQueryKeymap(self.dpy, into)

    def keycode(self, keysym):
        return self.x.XKeysymToKeycode(self.dpy, keysym)


def server():
    """The process-wide connection, opened on first use."""
    global _SERVER
    if _SERVER is None:
        _SERVER = _Server()
    return _SERVER


class _Window:
    """One window of ours, and where the cursor is inside it.

    Both consumers in this file want exactly this and nothing more, so the
    caching lives here once.
    """

    # Cache X server round trips: 3 ms for pointer/keymap, 0.5 s for window
    # size, 1 s for the tree walk to find our window.
    SAMPLE_S = 0.003
    SIZE_S = 0.5
    FIND_S = 1.0

    def __init__(self, srv, title, content_w, content_h):
        self.srv, self.title = srv, title
        self.cw, self.ch = content_w, content_h
        self.win = None
        self._found_at = -1e9
        self._ptr = None
        self._sampled_at = -1e9
        self._size = (content_w, content_h)
        self._sized_at = -1e9

    def id(self):
        if self.win is not None:
            return self.win
        now = time.monotonic()
        if now - self._found_at < self.FIND_S:
            return None
        self._found_at = now
        self.win = self.srv.find(self.title)
        return self.win

    def size(self):
        """What the window IS, not what Chrono asked for.

        A tiling window manager (i3, sway's X11 half, dwm) resizes the window
        to fill its tile the moment it is mapped, and then a cursor at the
        right-hand edge reads as x = 1900 in a view the demo believes is 1280
        wide: the ray goes off into space and half the window stops picking.
        Normalising by the real size makes that case correct and is the
        identity in the ordinary one.
        """
        now = time.monotonic()
        if now - self._sized_at >= self.SIZE_S:
            self._sized_at = now
            win = self.id()
            got = self.srv.size(win) if win is not None else None
            if got and got[0] > 0 and got[1] > 0:
                self._size = got
        return self._size

    def sample(self):
        now = time.monotonic()
        if now - self._sampled_at < self.SAMPLE_S:
            return
        self._sampled_at = now
        win = self.id()
        if win is None:
            self._ptr = None
            return
        self._ptr = self.srv.pointer(win)
        if self._ptr is None:
            # Either the window is gone (BadWindow, swallowed by the quiet
            # error handler) or the cursor is on another screen of this
            # display. Both are answered by looking the window up again, and
            # FIND_S keeps that from costing anything in the second case.
            self.win = None

    def pixel(self, inside_only=True):
        """Cursor in content pixels, or None.

        inside_only=False is for a drag already in progress: a slider you keep
        pulling past the edge of the window should peg at its end, not freeze.
        """
        self.sample()
        if self._ptr is None:
            return None
        wx, wy, _ = self._ptr
        rw, rh = self.size()
        px = wx * self.cw / rw
        py = wy * self.ch / rh
        if inside_only and not (0 <= px < self.cw and 0 <= py < self.ch):
            return None
        return px, py

    def down(self):
        self.sample()
        return self._ptr is not None and bool(self._ptr[2] & _BUTTON1_MASK)

    def right_down(self):
        self.sample()
        return self._ptr is not None and bool(self._ptr[2] & _BUTTON3_MASK)

    def middle_down(self):
        self.sample()
        return self._ptr is not None and bool(self._ptr[2] & _BUTTON2_MASK)

    def rect(self):
        """(x, y, w, h) on the screen. Nothing in the demos needs it -- the
        server converts for us -- but it is what a `can you see my window?`
        check asks, so it is worth being able to print."""
        win = self.id()
        if win is None:
            return None
        at, wh = self.srv.origin(win), self.srv.size(win)
        if at is None or wh is None:
            return None
        return at[0], at[1], wh[0], wh[1]


class _Keys:
    """Which keys are down, server-wide, from one 32-byte bit vector.

    XQueryKeymap is the X counterpart of CGEventSourceKeyState: it reports the
    physical keyboard, so it answers whether the 3D window has focus or not,
    which is the whole reason this backend exists.
    """

    SAMPLE_S = _Window.SAMPLE_S

    def __init__(self, srv, names):
        self.srv = srv
        self.buf = (c_char * 32)()
        self.bits = b"\0" * 32
        self._at = -1e9
        # Keycodes are NOT constants: they are whatever the X keyboard map says
        # today, so a fixed table would name the wrong physical key on any
        # layout but the one it was written on. Asking the server for the
        # keycode that produces each keysym means "the key labelled W" drives
        # the camera on AZERTY and Dvorak too.
        self.code = {}
        for name, keysym in names.items():
            kc = srv.keycode(keysym)
            if kc:                    # 0 means this layout has no such key
                self.code[name] = kc

    def sample(self):
        now = time.monotonic()
        if now - self._at < self.SAMPLE_S:
            return
        self._at = now
        self.srv.keymap(self.buf)
        self.bits = self.buf.raw

    def down(self, name):
        self.sample()
        kc = self.code.get(name)
        if kc is None:
            # NEVER KeyError here. The macOS backend did, once: the push demo
            # stopped inheriting from it, "space" went missing from the table,
            # and the first poll() killed the interactive demo. An unmapped key
            # is a key nobody can press, which is what False means.
            return False
        return bool(self.bits[kc >> 3] & (1 << (kc & 7)))


class X11WindowInput(CameraRay):
    """Mouse and keyboard read from the X server, so they work ON the 3D window.

    The surface is the macOS backend's, method for method, because the demos
    are written against one console interface and must not be able to tell
    which of these answered.
    """

    # Keysyms, from X11/keysymdef.h. These are stable; the KEYCODES they map to
    # are not, which is why _Keys resolves them at startup rather than here.
    KEYSYM = {"left": 0xFF51, "up": 0xFF52, "right": 0xFF53, "down": 0xFF54,
              "esc": 0xFF1B, "space": 0x0020,
              "lbracket": 0x005B, "rbracket": 0x005D,
              # camera, deliberately on keys so the mouse stays free for grabbing
              "a": 0x0061, "d": 0x0064, "w": 0x0077, "s": 0x0073,
              "r": 0x0072, "f": 0x0066,
              "z": 0x007A, "x": 0x0078, "c": 0x0063, "t": 0x0074}
    EDGE = {"z": "f", "x": "n", "c": "r", "t": "m", "rbracket": "u",
            "lbracket": "d", "esc": "q"}
    K = {}          # name -> keycode, filled per instance by _Keys

    def __init__(self, vis, title, content_w, content_h, edges=None):
        srv = server()
        self.vis, self.title = vis, title
        self.cw, self.ch = content_w, content_h
        self.w = _Window(srv, title, content_w, content_h)
        self.keys = _Keys(srv, self.KEYSYM)
        self.K = self.keys.code
        self.commands = []
        if edges is not None:
            self.EDGE = dict(edges)
        self.prev = {k: False for k in self.EDGE}
        self.prev_mouse = False
        self.last = (0.0, 0.0, 0.0)
        if self.w.id() is None:
            # Not fatal: the window is found again on a later frame, and in
            # practice Chrono has already mapped it by now. Say it anyway,
            # because a silent backend that never finds its window looks
            # exactly like a broken mouse.
            print(f"[input] no X11 window titled {title!r} yet; still looking")
        missing = [k for k in self.EDGE if k not in self.K]
        if missing:
            print(f"[input] this keyboard map has no {', '.join(missing)}; "
                  "those commands are off")
        print("[input] reading mouse and keys from the X server - "
              "click straight on the 3D window")

    # -- OS state ------------------------------------------------------------
    def _key(self, name):
        return self.keys.down(name)

    def mouse_down(self):
        return self.w.down()

    def mouse_right_down(self):
        return self.w.right_down()

    def mouse_middle_down(self):
        return self.w.middle_down()

    def window_rect(self):
        return self.w.rect()

    def cursor_pixel(self):
        """Cursor in window content pixels, or None when it is outside."""
        return self.w.pixel()

    # -- the ray Irrlicht would not give us ----------------------------------
    # ray_through() comes from CameraRay, shared with every other backend.

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

    def camera_nudge(self):
        """(orbit, zoom, rise) from A/D, W/S, R/F -- held, not edge-triggered."""
        return ((-1.0 if self._key("a") else 0.0) + (1.0 if self._key("d") else 0.0),
                (-1.0 if self._key("w") else 0.0) + (1.0 if self._key("s") else 0.0),
                (-1.0 if self._key("f") else 0.0) + (1.0 if self._key("r") else 0.0))

    def send(self, text):
        pass


def open_input(vis, title, content_w, content_h, edges=None, quiet=False):
    """An X11 backend, or None when this session cannot answer. Never raises."""
    why = unsupported()
    if why is None:
        try:
            return X11WindowInput(vis, title, content_w, content_h, edges=edges)
        except Exception as exc:
            why = f"X11 said no ({exc})"
    if not quiet:
        print(f"[input] {why}")
    return None
