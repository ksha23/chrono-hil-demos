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
"""Reading the mouse over the 3D window: native (Irrlicht events) if available,
else an OS backend (macOS/Linux/Windows), else None (falls back to keyboard).
Demos never import a backend directly -- they call open_window_input().
"""

import importlib
import platform

# The whole platform map. A backend is loaded only on the platform it is for,
# so a failure inside one (no X server, say) cannot affect the others.
_BACKENDS = {"Darwin": "macos", "Linux": "linux", "Windows": "windows"}


def _backend():
    """The module for this platform, or None where there is nothing to try."""
    name = _BACKENDS.get(platform.system())
    if name is None:
        return None
    try:
        return importlib.import_module(f".{name}", __package__)
    except Exception:
        return None                  # a backend that will not even import is
                                     # a backend this machine does not have


def open_window_input(vis, title, content_w, content_h, quiet=False,
                      edges=None):
    """Best available in-window input, or None.

    `edges` remaps which key raises which command, so a demo can want
    different keys without subclassing a backend it should not know about.
    """
    from . import native
    if native.available():
        return native.NativeWindowInput(vis, title, content_w, content_h,
                                        edges=edges)

    mod = _backend()
    if mod is not None:
        try:
            # open_input returns None when the platform is there but cannot
            # answer -- a Wayland session, say -- and says why itself, because
            # only it knows. It is not supposed to raise; the guard is for the
            # ways an OS can surprise us, since a demo must never die of this.
            got = mod.open_input(vis, title, content_w, content_h, edges=edges,
                                 quiet=quiet)
            if got is not None:
                return got
        except Exception as exc:
            if not quiet:
                print(f"[input] {mod.WHAT} window input unavailable ({exc})")

    if not quiet:
        print("[input] this PyChrono cannot read its own 3D window, so clicking\n"
              "        on it does nothing. Use the input window instead.\n"
              "        One line in Chrono's SWIG interface fixes this for every\n"
              "        platform: see chronohil/input/window/native.py")
    return None
