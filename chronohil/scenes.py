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
"""The scenes the demos put on screen."""

import os

from .chrono_env import chrono
from .controllers import (FreeDriveJoint, LockedJoint,
                          StanceHolder, joint_readers)
from .policy import Go2Policy, find_go2_policy
from .paths import FRANKA_URDF, GO2_POLICY, GO2_URDF as _GO2_URDF_DEFAULT
from .urdf import make_chrono_safe_urdf

GO2_URDF = _GO2_URDF_DEFAULT

def draggable_bodies(system):
    """Every dynamic body with visual geometry -- anything the user can see."""
    out = []
    for b in system.GetBodies():
        if b.IsFixed():
            continue
        vm = b.GetVisualModel()
        if vm is None or vm.GetNumShapes() == 0:
            continue
        out.append(b)
    return out


def ground_plane(system, size=20.0):
    """The floor. Material must match the system's contact method (NSC/SMC) or
    bodies silently fall through.
    """
    if system.GetContactMethod() == chrono.ChContactMethod_SMC:
        # The values the policy was trained against, from NeDM's rigid ground.
        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(0.9)
        mat.SetRestitution(0.01)
        mat.SetGn(60.0)
        mat.SetKn(2e5)
    else:
        mat = chrono.ChContactMaterialNSC()
        mat.SetFriction(0.8)
    g = chrono.ChBodyEasyBox(size, size, 1.0, 1000, True, True, mat)
    g.SetPos(chrono.ChVector3d(0, 0, -0.5))
    g.SetFixed(True)
    g.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 8, 8)
    system.AddBody(g)
    return g


def scene_go2(system):
    """A real Unitree Go2 from URDF, its joints holding a stance while you pull a leg."""
    import pychrono.parsers as parsers
    # Envelope/margin from the policy's training setup.
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    urdf = make_chrono_safe_urdf(GO2_URDF)
    ground = ground_plane(system)
    p = parsers.ChParserURDF(urdf)
    if "../obj/" not in open(urdf).read():
        p.EnableCollisionVisualization()   # no real visuals survived; draw the shapes
    # Without this the robot falls through the floor (no warning).
    feet = chrono.ChContactMaterialData()
    feet.mu = 0.8
    feet.cr = 0.0
    p.SetDefaultContactMaterial(feet)
    # FORCE not POSITION: position-actuated joints are constraints that can't be
    # displaced by a spring. Torque + PD gives when you pull, springs back when
    # you let go.
    p.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_FORCE)
    # Spawn above the floor: at zero joint angles the feet sit 0.42 m below
    # the base, and SMC throws the robot into the air on penetration.
    p.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(0, 0, 0.45), chrono.QUNIT))
    p.PopulateSystem(system)

    # Enable collision on all links so every part is clickable. Self-collision
    # is masked out (adjacent links overlap at joints). Costs ~11% of step budget
    # but lets 12/17 links be raycast-picked.
    ROBOT_FAMILY = 2
    _mode = globals().get("GO2_COLLIDE", "all")
    for b in system.GetBodies():
        if b is ground or b.IsFixed():
            continue
        if _mode == "feet+base" and not (b.GetName().endswith("_foot")
                                         or b.GetName() == "base"):
            continue
        b.EnableCollision(True)
        cm = b.GetCollisionModel()
        if cm:
            cm.SetFamily(ROBOT_FAMILY)
            cm.SetFamilyMask(~(1 << ROBOT_FAMILY) & 0x7FFF)   # signed short

    # Locomotion policy if available, else stance PD. The policy can step into
    # a push; the PD can only stiffen.
    it = system.GetSolver().AsIterative()
    if it:
        it.SetMaxIterations(max(600, it.GetMaxIterations()))
    ckpt = find_go2_policy() if globals().get("GO2_CONTROL", "policy") == "policy" else None
    if ckpt:
        try:
            system.stance_holders = [Go2Policy(system, p, ckpt)]
            print(f"[go2] locomotion policy: {os.path.basename(ckpt)} "
                  f"(wty-yy/go2_rl_gym, MIT; GO2_POLICY_CKPT overrides)")
            hint = "drag a leg; the policy steps to keep its feet"
        except Exception as exc:
            print(f"[go2] policy unavailable ({exc}); falling back to the stance PD")
            ckpt = None
    if not ckpt:
        # Warn loudly: the stance PD looks similar but has far lower push thresholds.
        if globals().get("GO2_CONTROL", "policy") == "policy":
            print("[go2] " + "!" * 62)
            print("[go2] NO POLICY CHECKPOINT -- running the PD stance controller.")
            print("[go2] This is NOT the locomotion policy. It holds a pose and")
            print("[go2] cannot step, so push thresholds will be far lower.")
            print(f"[go2] Expected it at: {GO2_POLICY}")
            print("[go2] Set GO2_POLICY_CKPT, or see ASSETS.md. Needs torch too.")
            print("[go2] " + "!" * 62)
        STANCE = globals().get("GO2_STANCE", {"hip": 0.0, "thigh": 0.9, "calf": -1.8})
        holders = []
        for leg in ("FL", "FR", "RL", "RR"):
            for joint, angle in STANCE.items():
                m = p.GetChMotor(f"{leg}_{joint}_joint")
                m = chrono.CastToChLinkMotorRotationTorque(m) if m else None
                if m:
                    fn = chrono.ChFunctionConst(0.0)
                    m.SetMotorFunction(fn)      # for a torque motor this IS the torque
                    holders.append(StanceHolder(m, fn, angle))
        system.stance_holders = holders   # the loop ticks these every step
        hint = "drag a leg; the joint motors fight you and pull it back"
    grabbable = draggable_bodies(system)
    base = [b for b in system.GetBodies() if b.GetName() == "base"][0]
    return grabbable, 0.9, hint, base


def scene_arm(system):
    """A Franka Emika Panda from URDF, joints held by PD controllers."""
    import pychrono.parsers as parsers
    ground_plane(system, 6.0)
    urdf = FRANKA_URDF
    if not os.path.exists(urdf):
        raise SystemExit(f"Franka URDF not found at {urdf}")
    p = parsers.ChParserURDF(urdf)
    p.SetAllJointsActuationType(parsers.ChParserURDF.ActuationType_FORCE)
    # Convex hulls instead of mesh-mesh (427 contacts, RTF 1.34 at rest).
    p.SetAllBodiesMeshCollisionType(parsers.ChParserURDF.MeshCollisionType_CONVEX_HULL)
    mat = chrono.ChContactMaterialData()
    mat.mu = 0.6
    p.SetDefaultContactMaterial(mat)
    p.SetRootInitPose(chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    p.PopulateSystem(system)

    root = p.GetRootChBody()
    if root:
        root.SetFixed(True)          # bolt the base down

    # Enable collision on all panda links (parser leaves it off) -- without it
    # the arm is invisible to raycasts. Self-collision masked off via families.
    # Bit 0 must stay set or the body loses both ground contact and raycast hits.
    ARM_FAMILY = 3
    for b in system.GetBodies():
        if not b.GetName().startswith("panda"):
            continue
        b.EnableCollision(True)
        cm = b.GetCollisionModel()
        if cm:
            cm.SetFamily(ARM_FAMILY)
            cm.SetFamilyMask(~(1 << ARM_FAMILY) & 0x7FFF)

    # PD on every actuated joint (revolute AND prismatic -- see joint_readers).
    dampers = []
    for link in system.GetLinks():
        raw = p.GetChMotor(link.GetName())
        m = (chrono.CastToChLinkMotorRotationTorque(raw)
             or chrono.CastToChLinkMotorLinearForce(raw))
        if m:
            fn = chrono.ChFunctionConst(0.0)
            m.SetMotorFunction(fn)
            pos, _ = joint_readers(m)
            prismatic = chrono.CastToChLinkMotorLinearForce(raw) is not None
            # Gripper is LOCKED (LockedJoint) because FreeDriveJoint's creep
            # would walk the fingers shut.
            cls = LockedJoint if prismatic else FreeDriveJoint
            dampers.append(cls(m, fn, pos()))
    system.stance_holders = dampers

    grabbable = draggable_bodies(system)
    hint = ("every joint holds the angle it is told to, so something can steer\n"
            "  the hand by writing those angles -- see chronohil/ik.py")
    return (grabbable, 1.1, hint, root if root else grabbable[0])
