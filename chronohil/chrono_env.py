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
"""`import pychrono` with a loader-path check, and a window-open guard."""

import os
import sys

_LOADER_VARS = ("DYLD_LIBRARY_PATH", "LD_LIBRARY_PATH", "PATH")

try:
    import pychrono as chrono
except ImportError as exc:
    if "symbol not found" in str(exc) or "undefined symbol" in str(exc):
        set_vars = [v for v in _LOADER_VARS[:2] if os.environ.get(v)]
        if set_vars:
            v = set_vars[0]
            sys.exit(
                f"PyChrono failed to load because {v} points somewhere with an\n"
                "older libChrono, so its symbols win over the packaged ones:\n"
                f"  {v}={os.environ[v]}\n\n"
                f"  unset {v} && python " + " ".join(sys.argv) + "\n\n"
                f"(original error: {exc})")
    raise


def require_window(vis, how="this demo"):
    """Stop with a message if the 3D window did not open.

    Initialize() leaves GetDevice() null on failure instead of raising, so the
    next draw call segfaults. os._exit (not sys.exit) because the null device
    also segfaults in its destructor during interpreter shutdown.
    """
    if vis.IsInitialized():
        return vis
    sys.stdout.flush()
    sys.stderr.write(
        "\nThe 3D window could not be opened, so there is nothing to drive.\n"
        "Chrono reported 'Failed to create the video driver' just above.\n\n"
        "  This needs a desktop session. Over ssh, either forward one or point\n"
        "  the process at a display that exists.\n"
        f"  For numbers without a window, {how}.\n")
    sys.stderr.flush()
    os._exit(1)


__all__ = ["chrono", "require_window"]
