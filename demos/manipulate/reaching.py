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
"""The commanded point for the arm: keys or mouse move it, IK solves the joints."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

from chronohil import (ArmChain, BeadLine, DragPlane, REACH_SPEED, ReachIK,
                       STEP, camera_forward, chrono)
from chronohil.ik import motor_angles
from chronohil.paths import FRANKA_URDF

# The Panda's "ready" pose (URDF sign convention, mirrored by motor_angles).
# Starting from the zero pose (arm straight up) makes the first move fight
# a near-singularity: 149.5 mm tracking error vs 7.9 mm from here.
PANDA_READY = motor_angles((0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785))

# 105 mm from the hand's reference frame to the grasp point (between fingertips).
# Chrono folds panda_link8 away, so this offset is applied here; the solver
# walks to panda_grasptarget independently and ReachIK refuses to start if they
# disagree by more than 1 mm.
PANDA_TIP_OFFSET = (0.0, 0.0, 0.105)

# Explicit joint chain: the URDF branches at panda_hand (fingers + grasp target),
# so letting the parser auto-discover the chain ends at a fingertip (58 mm off).
# This is also NOT the live system's joint list -- Chrono folds panda_link8 away.
PANDA_IK_JOINTS = ([f"panda_joint{i}" for i in range(1, 9)]
                   + ["panda_hand_joint", "panda_grasptarget_hand"])

GIZMO_R = 0.035        # visible marker radius
GIZMO_PICK = 0.07      # collision sphere for RayHit (2x visible, easier to click)
                       # that on the first try.
GIZMO_FAMILY = 4       # the marker's collision family; nothing collides with it.
                       # 2 and 3 are the Go2's and the arm's (chronohil.scenes).
GIZMO_OK = (0.15, 0.95, 0.45)     # green, the commanded point
GIZMO_TIP = (0.15, 0.45, 0.95)    # blue, where the hand actually is

# Workspace fence: requiring the hand to point down shrinks the usable sphere
# from the Panda's published 0.85 m to 0.65 m radially + 0.70 m ceiling.
# A UI limit, not a safety one -- past it the arm just stalls short.
REACH_MAX = 0.65       # m from the first joint's axis
REACH_TOP = 0.70       # m above the floor
REACH_MIN = 0.25       # m: inside this the target is among the arm's own links


class Reach:
    """A commanded point, the IK that chases it, and the marker you see."""

    def __init__(self, system, tip_name="panda_hand",
                 tip_offset=PANDA_TIP_OFFSET, ready=PANDA_READY,
                 urdf=FRANKA_URDF, ik_joints=PANDA_IK_JOINTS, settle=1.5):
        self.system = system
        tip = [b for b in system.GetBodies() if b.GetName() == tip_name]
        if not tip:
            raise SystemExit(f"no body named {tip_name} to command")
        self.chain = ArmChain(system, tip[0], tip_offset)
        self.ready = list(ready)[:len(self.chain)]
        self._go_to_ready(settle)
        self.ik = ReachIK(self.chain, urdf, ik_joints, rest=self.ready)
        # Copy the vector -- GetFrame2Abs().GetPos() returns a reference into
        # a temporary; storing it reads freed memory a few steps later.
        self.shoulder = chrono.ChVector3d(
            self.chain.joints[0][0].GetFrame2Abs().GetPos())
        self.clamped = False
        self.dragging = False
        self.set_target(self.chain.tip())
        self.plane = DragPlane()

        self.marker = chrono.ChBody()
        self.marker.SetFixed(True)
        self.marker.SetName("reach target")
        ball = chrono.ChVisualShapeSphere(GIZMO_R)
        ball.SetColor(chrono.ChColor(*GIZMO_OK))
        self.marker.AddVisualShape(ball)
        # Collision shape for RayHit, in a family nothing else collides with.
        # Filter goes on OTHER bodies (a ray is cast as family 0).
        # Envelope must be small or the default 3 cm inflates the pick zone.
        mat = chrono.ChContactMaterialData().CreateMaterial(system.GetContactMethod())
        self.marker.AddCollisionShape(chrono.ChCollisionShapeSphere(mat, GIZMO_PICK))
        self.marker.GetCollisionModel().SetEnvelope(0.001)
        self.marker.EnableCollision(True)
        self.marker.SetPos(self.target)
        system.AddBody(self.marker)
        # BindItem needed: body added after system has stepped is not registered
        # with the collision system otherwise; without it no ray
        # ever hit it.
        system.GetCollisionSystem().BindItem(self.marker)
        self.marker.GetCollisionModel().SetFamily(GIZMO_FAMILY)
        for b in system.GetBodies():
            if b != self.marker and b.GetCollisionModel():
                b.GetCollisionModel().DisallowCollisionsWith(GIZMO_FAMILY)

        # A second, smaller marker ON THE HAND, at the exact point being
        # commanded, and a bead line between the two. The gap between the
        # markers IS the tracking error, at true scale and in the right place:
        # nobody reads a HUD while they are aiming. Most of the time the two
        # sit inside each other, which is what success looks like.
        self.tip_marker = chrono.ChBody()
        self.tip_marker.SetFixed(True)
        self.tip_marker.EnableCollision(False)
        self.tip_marker.SetName("reach hand")
        dot = chrono.ChVisualShapeSphere(GIZMO_R * 0.55)
        dot.SetColor(chrono.ChColor(*GIZMO_TIP))
        self.tip_marker.AddVisualShape(dot)
        self.tip_marker.SetPos(self.target)
        system.AddBody(self.tip_marker)
        self.gap = BeadLine(system, n=10, radius=0.009, color=GIZMO_OK,
                            spacing=0.02, name="reach error")

    # -- setup ---------------------------------------------------------------
    def _go_to_ready(self, seconds):
        """Ramp joints to the ready pose before the window opens."""
        holders = [j[1] for j in self.chain.joints]
        start = [h.target if h else 0.0 for h in holders]
        n = max(1, int(seconds / STEP))
        for i in range(n + int(0.5 / STEP)):       # ramp, then half a second to settle
            a = min(1.0, (i + 1) / n)
            for h, q0, q1 in zip(holders, start, self.ready):
                if h is not None:
                    h.target = q0 + (q1 - q0) * a
            for h in getattr(self.system, "stance_holders", ()):
                h.update()
            self.system.DoStepDynamics(STEP)

    # -- the person's numbers ------------------------------------------------
    def command(self, steer, throttle, braking, lift, dt):
        """Move the target with keys (world axes, not camera-relative)."""
        if self.dragging:
            return                       # the mouse has it; do not fight
        v = REACH_SPEED * dt
        self.set_target(self.target + chrono.ChVector3d((throttle - braking) * v,
                                                        steer * v, lift * v))

    def set_target(self, p):
        """Clamp p to the workspace fence and store it."""
        # The ceiling first. Lowering a point that is above the shoulder brings
        # it CLOSER to the shoulder, so the sphere still holds afterwards; the
        # other order would let a capped point sit outside the fence.
        top = p.z > REACH_TOP
        if top:
            p = chrono.ChVector3d(p.x, p.y, REACH_TOP)
        d = p - self.shoulder
        n = d.Length()
        out = not (REACH_MIN <= n <= REACH_MAX)
        was, self.clamped = self.clamped, out or top
        if out and n > 1e-6:
            p = self.shoulder + d * (min(REACH_MAX, max(REACH_MIN, n)) / n)
        if self.clamped and not was:
            print(f"[reach] at the reach limit -- {REACH_MAX:.2f} m from the "
                  f"shoulder and {REACH_TOP:.2f} m up. The target stops here "
                  f"because the arm cannot get there with the hand pointing "
                  f"down, which is what it is being asked to do.")
        self.target = p

    def mouse_update(self, console):
        """Drag the marker. Returns True while the mouse belongs to this gizmo."""
        if console is None or not hasattr(console, "ray_through"):
            return False
        down = console.mouse_down()
        at = console.cursor_pixel()
        if down and not console.prev_mouse and not self.dragging and at is not None:
            r = console.ray_through(*at)
            # RayHit against the marker's model ALONE -- the target sits inside
            # the hand, and the nearest hit would be the hand.
            res = chrono.ChRayhitResult()
            if r is None or not self.system.GetCollisionSystem().RayHit(
                    r[0], r[1], self.marker.GetCollisionModel(), res):
                return False             # let the grab code have this press
            self.dragging = True
            self.plane.set(self.target, camera_forward(console.vis))
            print("[reach] target grabbed")
        elif self.dragging and down and at is not None:
            r = console.ray_through(*at)
            if r:
                d = r[1] - r[0]
                hit = self.plane.hit(r[0], d / d.Length())
                if hit is not None:
                    self.set_target(hit)
        elif self.dragging and not down:
            self.dragging = False
            print("[reach] target released")
        if self.dragging:
            console.prev_mouse = down    # this gizmo owns the button's edge now
        return self.dragging

    def recenter(self):
        """Put the target back on the hand's current position."""
        self.dragging = False
        self.set_target(self.chain.tip())

    # -- the loop ------------------------------------------------------------
    def update(self, dt):
        """One control tick. Call before DoStepDynamics, every step."""
        self.ik.step(self.target, dt)

    def draw(self):
        """Where it was asked to be, where it is, and the gap. Once per frame."""
        tip = self.chain.tip()
        self.marker.SetPos(self.target)
        self.tip_marker.SetPos(tip)
        self.gap.draw(tip, self.target)

    def status(self):
        """Three readout lines: target, hand, error/tilt."""
        p, tip = self.target, self.chain.tip()
        flag = ("  AT THE REACH LIMIT" if self.clamped
                else ("  AT A JOINT STOP" if self.ik.saturated
                      else ("  CANNOT REACH THAT POINT POINTING DOWN"
                            if self.ik.short > 0.01 else "")))
        return [f"target x {p.x:+6.3f}  y {p.y:+6.3f}  z {p.z:+6.3f}"
                f"{'   [mouse]' if self.dragging else ''}",
                f"hand   x {tip.x:+6.3f}  y {tip.y:+6.3f}  z {tip.z:+6.3f}",
                f"error  {self.ik.err * 1000:6.1f} mm   tilt {self.ik.tilt:4.1f} deg"
                f"{flag}"]
