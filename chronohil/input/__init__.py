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
"""Ways a person's numbers reach the simulation.

    keys    a small window of our own, portable, always available
    window  the 3D window itself, when the build allows it (see .window)
"""

from .keys import LocalInput
from .window import open_window_input

__all__ = ["LocalInput", "open_window_input"]
