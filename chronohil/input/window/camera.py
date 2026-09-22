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
"""Pixel to world ray. Platform-independent: uses only ICameraSceneNode's SWIG
bindings (getFOV, getAspectRatio). Shared by every OS backend.
"""

import math

from ...chrono_env import chrono


class CameraRay:
    """ray_through() for any backend that carries .vis, .cw and .ch.

    Mixed in rather than called, so the demos' `hasattr(console, "ray_through")`
    test -- their way of asking "can this console point at the 3D view?" --
    keeps meaning what it meant.
    """

    def ray_through(self, px, py, reach=60.0):
        """(near, far) in world coordinates through window pixel (px, py)."""
        cam = self.vis.GetActiveCamera()
        # vis.GetCameraPosition() reports (0,0,0) for a camera added with
        # AddCamera, so ask the Irrlicht node itself. Chrono passes world
        # coordinates through 1:1 once SetCameraVertical(Z) is set.
        cp, ct = cam.getAbsolutePosition(), cam.getTarget()
        eye = chrono.ChVector3d(cp.X, cp.Y, cp.Z)
        tgt = chrono.ChVector3d(ct.X, ct.Y, ct.Z)
        fwd = tgt - eye
        n = fwd.Length()
        if n < 1e-9:
            return None
        fwd = fwd / n
        world_up = chrono.ChVector3d(0, 0, 1)
        right = fwd.Cross(world_up)
        if right.Length() < 1e-6:
            right = chrono.ChVector3d(1, 0, 0)
        right = right / right.Length()
        up = right.Cross(fwd)
        # One unit in front of the eye, the view is a pane half_w wide and
        # half_h tall either side of fwd. The click is a spot on that pane.
        half_h = math.tan(cam.getFOV() * 0.5)
        half_w = half_h * cam.getAspectRatio()
        click_x = (2.0 * px / self.cw) - 1.0   # -1 left edge .. +1 right edge
        click_y = 1.0 - (2.0 * py / self.ch)   # -1 bottom .. +1 top; pixels count DOWN
        d = fwd + right * (click_x * half_w) + up * (click_y * half_h)
        d = d / d.Length()
        return eye, eye + d * reach
