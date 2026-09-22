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
"""The human's grip on the scene: press, pull, let go."""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from chronohil import (GRAB_REACH, Grabber, camera_forward, chrono,
                       pick_along_ray, pick_near_ray)


class Manipulator:
    """Whatever the person currently has hold of, and how they got it."""

    def __init__(self, system, grabbable, kinematic=False):
        self.system = system
        self.kinematic = kinematic
        self.grabbable = grabbable
        self.grabber = Grabber(system)
        self.held = False


    # -- the keyboard path ---------------------------------------------------
    def toggle(self, body):
        """Z: grab what is selected, or let go of what is held."""
        if self.held:
            self.release()
            return False
        self.grabber.grab(body, body.GetPos())
        self.held = True
        return True

    def release(self):
        if self.held:
            self.grabber.release()
            self.held = False

    # -- passthroughs the loop and the HUD want ------------------------------
    def move(self, dx, dy, dz):
        self.grabber.move(dx, dy, dz)

    @property
    def body(self):
        """The body currently held, or None. The readout names it."""
        return self.grabber.body if self.held else None

    def force(self):
        return self.grabber.force() if self.held else 0.0

    def draw_link(self):
        self.grabber.draw_link()

    # -- the mouse path ------------------------------------------------------
    def mouse_update(self, console):
        """One step of press / drag / release, straight on the 3D window."""
        if console is None or not hasattr(console, "ray_through") or self.kinematic:
            return
        down = console.mouse_down()
        at = console.cursor_pixel()
        if down and not console.prev_mouse and at is not None:
            r = console.ray_through(*at)
            if r:
                got = pick_along_ray(self.system, r[0], r[1])
                if got and got[0] not in self.grabbable:
                    if got[0].IsFixed() and got[0].GetName().startswith("panda"):
                        print(f"[mouse] {got[0].GetName()} is bolted down")
                    got = None
                if got is None:
                    d0 = r[1] - r[0]
                    got = pick_near_ray(self.grabbable, r[0], d0 / d0.Length())
                if got:
                    body, point, _normal = got
                    # SWIG __eq__ compares the C++ pointer (`is` compares proxies).
                    if self.grabber.handle is None or body != self.grabber.handle:
                        self.grabber.grab(body, point)
                        self.held = True
                        self.grabber.set_plane(point, camera_forward(console.vis))
                        print(f"[mouse] grabbed {body.GetName()}")
        elif down and self.held and at is not None:
            r = console.ray_through(*at)
            if r:
                d = (r[1] - r[0])
                d = d / d.Length()
                target = self.grabber.plane_point(r[0], d)
                if target is not None:
                    here = self.grabber.body.GetPos()
                    off = target - here
                    far = off.Length()
                    if far > GRAB_REACH:
                        target = here + off * (GRAB_REACH / far)
                    self.grabber.handle.SetPos(target)
        elif (not down) and console.prev_mouse and self.held:
            self.grabber.release(); self.held = False
            print("[mouse] released")
        console.prev_mouse = down
