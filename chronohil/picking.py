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
"""Turning a screen ray into a body, and a body into something you can pull."""

from .chrono_env import chrono
from .config import GRAB_MAX_SPEED, GRAB_MIN_MASS, GRAB_OMEGA, GRAB_ZETA

def pick_along_ray(system, start, end):
    """(body, world point, normal) of the first collision geometry on the
    segment, or None. Whether that body may be pulled is the caller's call."""
    res = chrono.ChRayhitResult()
    system.GetCollisionSystem().RayHit(start, end, res)
    if not res.hit:
        return None
    body = chrono.CastToChBody(res.hitModel.GetContactable())
    if body is None:
        return None
    p, nrm = res.abs_hitPoint, res.abs_hitNormal
    return (body, chrono.ChVector3d(p.x, p.y, p.z),
            chrono.ChVector3d(nrm.x, nrm.y, nrm.z))


def pick_near_ray(bodies, origin, direction, tol=0.06):
    """Pick the body whose centre lies closest along a ray, within `tol` metres.

    Geometry-free: bodies with no collision shape (Go2 limbs) are still pickable.
    A body with an empty collision mask is invisible to RayHit (measured).
    """
    best = None
    for b in bodies:
        c = b.GetPos()
        w = chrono.ChVector3d(c.x - origin.x, c.y - origin.y, c.z - origin.z)
        along = w.x * direction.x + w.y * direction.y + w.z * direction.z
        if along <= 0:
            continue                      # behind the camera
        perp = (w - direction * along).Length()
        if perp <= tol and (best is None or along < best[0]):
            best = (along, b, chrono.ChVector3d(c.x, c.y, c.z))
    if best is None:
        return None
    return best[1], best[2], chrono.ChVector3d(0, 0, 1)


class DragPlane:
    """A plane through a point, square to the camera, for 2D-to-3D picking."""

    def __init__(self):
        self.n = None
        self.d = 0.0

    def set(self, point, cam_fwd):
        """Freeze a plane through `point`, normal along the camera's view axis.

        Screen-parallel is the best-conditioned choice: a surface-normal plane
        degenerates when the surface faces the camera.
        """
        f = cam_fwd
        lf = f.Length()
        self.n = f / lf if lf > 1e-9 else chrono.ChVector3d(0, 1, 0)
        self.d = -(self.n.x * point.x + self.n.y * point.y + self.n.z * point.z)

    def hit(self, origin, direction):
        """Where a cursor ray meets the plane, or None if it is parallel."""
        if self.n is None:
            return None
        denom = (self.n.x * direction.x + self.n.y * direction.y
                 + self.n.z * direction.z)
        if abs(denom) < 1e-6:
            return None
        t = -((self.n.x * origin.x + self.n.y * origin.y + self.n.z * origin.z)
              + self.d) / denom
        if t <= 0:
            return None
        return origin + direction * t


class BeadLine:
    """A line between two moving points, drawn as a row of small spheres.

    Irrlicht visual shapes can't be resized after binding (measured: SetMutable
    has no effect), so a stretching cylinder doesn't work. Beads at fixed
    spacing instead -- the count you see IS the length. Spare beads are parked
    at z=-1000 rather than removed to avoid rebinding.
    """

    PARK = chrono.ChVector3d(0, 0, -1000.0)

    def __init__(self, system, n=12, radius=0.008, color=(1.0, 0.55, 0.1),
                 spacing=0.028, name="line"):
        self.spacing = spacing
        self.beads = []
        for i in range(n):
            b = chrono.ChBody()
            b.SetFixed(True)
            b.EnableCollision(False)
            b.SetName(f"{name} bead {i}")
            shape = chrono.ChVisualShapeSphere(radius)
            shape.SetColor(chrono.ChColor(*color))
            b.AddVisualShape(shape)
            b.SetPos(self.PARK)
            system.AddBody(b)
            self.beads.append(b)

    def draw(self, a, b):
        """Lay beads from `a` to `b`. Call once per rendered frame."""
        d = b - a
        length = d.Length()
        want = 0 if length < 1e-4 else min(len(self.beads),
                                           1 + int(length / self.spacing))
        for i, bead in enumerate(self.beads):
            if i >= want:
                bead.SetPos(self.PARK)
            else:
                t = i / max(1, want - 1) if want > 1 else 0.0
                bead.SetPos(a + d * t)

    def hide(self):
        for bead in self.beads:
            bead.SetPos(self.PARK)


def camera_forward(vis):
    """Unit view direction of the 3D window's active camera."""
    cam = vis.GetActiveCamera()
    cp, ct = cam.getAbsolutePosition(), cam.getTarget()
    f = chrono.ChVector3d(ct.X - cp.X, ct.Y - cp.Y, ct.Z - cp.Z)
    n = f.Length()
    return f / n if n > 1e-9 else chrono.ChVector3d(0, 1, 0)


class Grabber:
    """A handle body and a stiff spring to whatever is being held."""

    def __init__(self, system):
        self.system = system
        self.handle = chrono.ChBody()
        self.handle.SetFixed(True)
        self.handle.EnableCollision(False)
        self.handle.SetName("grab handle")
        marker = chrono.ChVisualShapeSphere(0.04)
        marker.SetColor(chrono.ChColor(1.0, 0.25, 0.1))
        self.handle.AddVisualShape(marker)
        system.AddBody(self.handle)
        self.spring = None
        self.body = None
        self.had_limit = False   # what the held body's speed limit was on grab
        self.plane = DragPlane()
        # 12 beads at 28 mm reach the full GRAB_REACH of 0.30 m.
        self.link = BeadLine(system, n=12, spacing=0.028, name="grab line")

    def set_plane(self, point, cam_fwd):
        """Freeze the drag plane. One plane per drag -- orbit to change depth."""
        self.plane.set(point, cam_fwd)

    def plane_point(self, origin, direction):
        """Where a cursor ray meets the drag plane, or None if it is parallel."""
        return self.plane.hit(origin, direction)

    def grab(self, body, point):
        self.release()
        self.body = body
        self.handle.SetPos(point)
        # Spring gains scale with the held body's mass (k = m*w^2) so the same
        # natural frequency works on a 134 kg chassis and a 0.154 kg calf.
        omega = getattr(self.system, "grab_omega", GRAB_OMEGA)
        m = max(body.GetMass(), GRAB_MIN_MASS)
        k = m * omega * omega
        c = 2.0 * m * omega * GRAB_ZETA
        # Speed-limit the held body to prevent tunnelling.
        self.had_limit = body.GetLimitSpeed() if hasattr(body, "GetLimitSpeed") else False
        body.SetLimitSpeed(True)
        body.SetMaxLinVel(GRAB_MAX_SPEED)
        self.spring = chrono.ChLinkTSDA()
        self.spring.Initialize(self.handle, body, False, point, point)
        self.spring.SetRestLength(0.0)
        self.spring.SetSpringCoefficient(k)
        self.spring.SetDampingCoefficient(c)
        self.system.AddLink(self.spring)

    def release(self):
        if self.spring is not None:
            self.system.RemoveLink(self.spring)
            self.spring = None
        if self.body is not None:
            # Restore the body's original speed-limit setting.
            self.body.SetLimitSpeed(self.had_limit)
        self.body = None

    def held(self):
        return self.spring is not None

    def move(self, dx, dy, dz):
        p = self.handle.GetPos()
        self.handle.SetPos(chrono.ChVector3d(p.x + dx, p.y + dy, p.z + dz))

    def force(self):
        return abs(self.spring.GetForce()) if self.spring else 0.0

    def draw_link(self):
        """Lay the marker line between the held body and the handle."""
        if self.body is None:
            self.link.hide()
            return
        self.link.draw(self.body.GetPos(), self.handle.GetPos())
