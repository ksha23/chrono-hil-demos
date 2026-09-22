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
"""IK via ikpy: command a point and a direction, not seven joints.

    ArmChain   finds the motors between base and hand in the live ChSystem,
               pairs each with its PD, and reports the actual tool point.
    ReachIK    solves for seven angles, rate-limits them into ArmChain's
               PD targets, and closes the loop on the measured tip.

The arm is never teleported -- solved angles go into joint PD setpoints.
orientation_mode="Z" pins the tool's z direction (gripper points down) and
leaves the roll free (5 constraints on 7 joints).

Chrono's own IndustrialKinematicsNdofNumerical is 300x faster but takes a full
pose, so a roll must be invented. On this redundant arm that fails: 98/190
workspace points missed. ikpy needs only position + one axis direction.
"""

import math
import warnings

import numpy as np
from ikpy.chain import Chain
from ikpy.urdf.URDF import get_chain_from_joints

from .chrono_env import chrono

# panda_joint8 is FIXED, but the Panda's URDF still writes an <axis> on it.
# Chrono ignores it; ikpy warns, once per chain built. A flaw in the file
# rather than in either library, and not something a demo should open with.
warnings.filterwarnings("ignore", message=r".*is of type: fixed, but has an "
                                          r"'axis' attribute.*")

# Chrono motor angles run opposite to the URDF convention (measured, not assumed).
# Every angle crossing between ikpy and Chrono crosses this sign.
MOTOR_SIGN = -1.0


def motor_angles(urdf_angles):
    """URDF joint values -> the numbers Chrono's motors will report. See above."""
    return [MOTOR_SIGN * q for q in urdf_angles]


def _wound(acc, err, gain, dt, cap):
    """acc += gain * err * dt, capped to prevent windup."""
    acc = acc + err * (gain * dt)
    n = acc.Length()
    return acc * (cap / n) if n > cap else acc


def _ref_frame(body):
    """The body's REFERENCE frame, not its COM frame.

    ChBodyAuxRef bodies report COM from GetPos(), not the URDF link origin.
    """
    aux = chrono.CastToChBodyAuxRef(body)
    if aux is not None:
        return aux.GetFrameRefToAbs()
    return chrono.ChFramed(body.GetPos(), body.GetRot())


def _parent_map(system):
    """child body name -> (link, parent body name), over every two-body link."""
    out = {}
    for base in system.GetLinks():
        # Skip TSDAs (the grab spring is one) so they don't shadow real joints.
        if chrono.CastToChLinkTSDA(base) is not None:
            continue
        link = chrono.CastToChLink(base)      # GetLinks() hands back ChLinkBase,
        if link is None:                      # which carries no bodies
            continue
        b1 = chrono.CastToChBody(link.GetBody1())
        b2 = chrono.CastToChBody(link.GetBody2())
        if b1 is None or b2 is None:
            continue
        out[b2.GetName()] = (base, b1.GetName())
    return out


class ArmChain:
    """Motors between a fixed base and one link, found in the running ChSystem.

    `tip_offset` is in the tip link's reference frame (e.g. panda_grasptarget).
    """

    def __init__(self, system, tip_body, tip_offset=(0.0, 0.0, 0.0)):
        self.system = system
        self.tip_body = tip_body
        self.tip_offset = tuple(tip_offset)
        holders = {}
        for h in getattr(system, "stance_holders", ()):
            m = getattr(h, "motor", None)
            if m is not None:
                holders[m.GetName()] = h

        parent = _parent_map(system)
        chain = []
        name, seen = tip_body.GetName(), set()
        while name in parent and name not in seen:
            seen.add(name)
            link, up = parent[name]
            motor = chrono.CastToChLinkMotorRotationTorque(link)
            if motor is not None:
                chain.append([motor, holders.get(link.GetName())])
            name = up
        chain.reverse()                       # base first, the order a human reads
        self.joints = chain
        if not chain:
            raise ValueError(
                f"no rotational motors between {tip_body.GetName()} and the base. "
                "A URDF parsed with ActuationType_NONE has joints but no motors.")

    def __len__(self):
        return len(self.joints)

    def names(self):
        return [j[0].GetName() for j in self.joints]

    def angles(self):
        """Where the arm ACTUALLY is, this step. Measured, not commanded."""
        return [j[0].GetMotorAngle() for j in self.joints]

    def tip(self):
        """The commanded point, in world coordinates."""
        return _ref_frame(self.tip_body).TransformPointLocalToParent(
            chrono.ChVector3d(*self.tip_offset))

    def tool_axis(self):
        """Which way the hand is pointing, in world coordinates.

        The gripper closes along the hand frame's own z, so this is the axis
        that has to point at the floor. Read off the live body: what the solver
        asked for and what an arm with mass is doing are different numbers, and
        this demo's claim is the second one.
        """
        return _ref_frame(self.tip_body).GetRotMat().GetAxisZ()


class ReachIK:
    """ikpy solver, rate-limited into joint PD targets, closed on the tip.

    Each tick:  ask = target + trim;  q* = ikpy(ask);  q += clamp(dq, QD_MAX*dt)

    trim:       integrates measured tip error to cancel gravity droop (26 mm open-loop)
    rate limit: QD_MAX prevents teleporting; the arm walks to the solved pose
    seed:       current command prevents branch-swapping (elbow-up / elbow-down)
    rejection:  every answer checked by FK; non-finite, worse, or branch-jump discarded
    """

    AXIS = (0.0, 0.0, -1.0)   # tool z points down
    IK_EVERY = 20     # physics steps between solves (25 Hz at STEP 2 ms).
                      # Median solve 3.8 ms. Measure with window open and target
                      # driven, not headless idle: 25 Hz -> 1.19 wall/sim.
    QD_MAX = 2.0      # rad/s per joint; the Panda's URDF says 2.175
    TRIM_GAIN = 1.5   # 1/s. Closes around the joint PD (12.6 rad/s, zeta 0.25).
                      # 0 -> 26 mm droop; 1.5 -> 0.16 mm; 3.0 overshoots.
    TRIM_MAX = 0.12   # m. Bounds the integrator so unreachable asks don't wind up.
    TILT_MAX = 0.5    # same bound for the axis integrator (~30 deg)
    E_MAX = 0.12      # m of error the solver sees at once -- see step()
    TOL = 1e-4        # optimizer tolerance. 3.4 ms/solve vs 16 ms at default,
                      # 2.9 mm residual the integrators absorb. 1e-3 is too loose.
    REST_PULL = 0.02  # per solve (0.5/s at 25 Hz). Null-space rest term: nudges
                      # the seed toward home so the two spare joints don't drift.
    SHORT_OK = 0.01   # m. Below this the solver is "getting there".
    JUMP_MAX = 0.6    # rad. Branch guard: rejects elbow-up/down swaps (tens of
                      # deg at once). A delay, not a veto -- see _solve().

    def __init__(self, chain, urdf, joints, rest=None):
        self.chain = chain
        # joints must be spelled out: the Panda's URDF branches at the hand,
        # and ikpy's default path follows a finger, not the tool point.
        motored = set(chain.names())
        self.ik = Chain.from_urdf_file(
            urdf, base_elements=get_chain_from_joints(urdf, joints),
            # Only joints with a Chrono motor are active; the rest are fixed frames.
            active_links_mask=[False] + [j in motored for j in joints])
        self.active = [i for i, a in enumerate(self.ik.active_links_mask) if a]
        self.q = chain.angles()               # the COMMANDED angles, our state
        # Verify ikpy and Chrono agree on the tool point. Every modelling
        # error (wrong branch, folded link, COM vs ref frame) lands here.
        here, _ = self._score(self._urdf_q(self.q))
        t = chain.tip()
        self.fk_residual = math.dist(here, (t.x, t.y, t.z))
        if self.fk_residual > 1e-3:
            raise ValueError(
                f"ikpy puts the tool point {self.fk_residual * 1000:.2f} mm "
                f"from where Chrono has it, so they are not the same arm. "
                f"ikpy turns {[self.ik.links[i].name for i in self.active]}; "
                f"Chrono has motors on {chain.names()}. Check that list, the "
                f"tool offset, and MOTOR_SIGN.")
        self.rest = list(rest) if rest else list(self.q)
        self.goal = list(self.q)              # the last solution worth walking to
        self.err = 0.0                        # |target - tip|, for the readout
        self.short = 0.0                      # m the goal pose falls from the ask
        self.tilt = 0.0                       # deg the hand is off AXIS, MEASURED
        self.saturated = False                # a joint is against a URDF stop
        self.jump = 0.0                       # rad, worst joint step asked for
        self.refused = 0                      # solves thrown away, for the record
        self.trim = chrono.ChVector3d(0, 0, 0)         # m, the droop integrator
        self.tilt_trim = chrono.ChVector3d(0, 0, 0)    # the same for the axis
        self._tick = self.IK_EVERY            # solve on the very first step

    # -- the solver ----------------------------------------------------------
    def _urdf_q(self, motor_q):
        """A full ikpy joint vector, with our seven motor angles put in it."""
        q = np.zeros(len(self.ik.links))
        for i, a in zip(self.active, motor_q):
            q[i] = MOTOR_SIGN * a             # Chrono's convention -> the URDF's
        return q

    def _seed(self):
        """Current command nudged toward rest, clamped to URDF joint limits.

        Clamping is required: Chrono doesn't enforce torque-motor limits, so
        an angle can exceed the URDF range, and ikpy raises ValueError on
        out-of-bounds seeds.
        """
        q = self._urdf_q([a + self.REST_PULL * (r - a)
                          for a, r in zip(self.q, self.rest)])
        for i in self.active:
            lo, hi = self.ik.links[i].bounds
            q[i] = min(max(q[i], lo), hi)
        return q

    def _score(self, q):
        """(tool point, tool axis) via forward kinematics."""
        t = self.ik.forward_kinematics(q)
        return t[:3, 3], t[:3, 2]

    def _solve(self, ask, axis):
        """One call to ikpy, checked three ways before it is believed."""
        p = np.array([ask.x, ask.y, ask.z])
        a = np.array([axis.x, axis.y, axis.z])
        seed = self._seed()
        sol = self.ik.inverse_kinematics(p, a, orientation_mode="Z",
                                         initial_position=seed, tol=self.TOL)

        def cost(q):
            got, got_axis = self._score(q)
            return np.linalg.norm(got - p) + 0.1 * np.linalg.norm(got_axis - a)

        # Distance from current goal to the ask (readout + guard condition).
        held, _ = self._score(self._urdf_q(self.goal))
        self.short = float(np.linalg.norm(held - p))

        # Reject: (1) non-finite, (2) worse than current, (3) branch jump
        # (but only while short < SHORT_OK -- unconditional rejection deadlocks).
        if np.all(np.isfinite(sol)):
            goal = [MOTOR_SIGN * sol[i] for i in self.active]
            self.jump = max(abs(g - q) for g, q in zip(goal, self.q))
            if cost(sol) <= cost(seed) and (self.jump <= self.JUMP_MAX
                                            or self.short > self.SHORT_OK):
                self.goal = goal
                got, _ = self._score(sol)
                self.short = float(np.linalg.norm(got - p))
                # Report if a URDF joint limit is active.
                self.saturated = any(
                    min(abs(sol[i] - self.ik.links[i].bounds[0]),
                        abs(sol[i] - self.ik.links[i].bounds[1])) < 1e-3
                    for i in self.active)
            else:
                self.refused += 1

    # -- the loop ------------------------------------------------------------
    def step(self, target, dt):
        """One control tick. Returns the tip error vector (target - tip)."""
        tip = self.chain.tip()
        e = target - tip
        self.err = e.Length()
        # WHERE THE HAND IS REALLY POINTING, read off the live body -- not the
        # axis the solver was given, nor the one its answer would produce. The
        # arm has mass, and that gap is why the second trim below exists.
        down = chrono.ChVector3d(*self.AXIS)
        z = self.chain.tool_axis()
        self.tilt = math.degrees(math.acos(max(-1.0, min(1.0, z ^ down))))

        # Integrators wind only while the solver is getting there; when the arm
        # can't reach, they unwind so it comes back when the target does.
        reaching = self.short < self.SHORT_OK
        self.trim = _wound(self.trim, e if reaching else -self.trim,
                           self.TRIM_GAIN, dt, self.TRIM_MAX)
        self.tilt_trim = _wound(self.tilt_trim,
                                (down - z) if reaching else -self.tilt_trim,
                                self.TRIM_GAIN, dt, self.TILT_MAX)

        self._tick += 1
        if self._tick >= self.IK_EVERY:
            self._tick = 0
            # Clamp the ask to E_MAX so the solver sees a small move.
            ask = target + self.trim
            d = ask - tip
            if d.Length() > self.E_MAX:
                ask = tip + d * (self.E_MAX / d.Length())
            axis = down + self.tilt_trim
            self._solve(ask, axis / axis.Length())

        step = self.QD_MAX * dt
        for i, (_motor, holder) in enumerate(self.chain.joints):
            self.q[i] += max(-step, min(step, self.goal[i] - self.q[i]))
            if holder is not None:
                # Write the PD setpoint, not a pose.
                holder.target = self.q[i]
        return e
