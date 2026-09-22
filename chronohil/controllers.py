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
"""Small joint-level controllers: what holds a robot up while you pull on it."""

from .chrono_env import chrono

def joint_readers(motor):
    """(position, velocity) readers for a motor, revolute or prismatic.

    Revolute answers GetMotorAngle, prismatic GetMotorPos. Using the wrong one
    silently returns zero -- the Panda's finger joints are prismatic.
    """
    if hasattr(motor, "GetMotorAngle"):
        return motor.GetMotorAngle, motor.GetMotorAngleDt
    return motor.GetMotorPos, motor.GetMotorPosDt


class FreeDriveJoint:
    """One joint's PD controller. Write `target` to command it.

    PD torque keeps the arm upright under gravity while staying compliant:
    shove it and it gives, let go and it comes back.
    """

    KP = 400.0     # N.m per rad
    KD = 16.0      # N.m per rad/s
    TAU_MAX = 87.0  # N.m, the Panda's largest joint limit
    FOLLOW = 0.004  # per step, only while `guiding` is on
    guiding = False

    def __init__(self, motor, fn, target):
        self.motor, self.fn, self.target = motor, fn, target
        self.pos, self.vel = joint_readers(motor)

    def update(self):
        q = self.pos()
        if FreeDriveJoint.guiding:
            self.target += (q - self.target) * self.FOLLOW
        tau = self.KP * (self.target - q) - self.KD * self.vel()
        self.fn.SetConstant(max(-self.TAU_MAX, min(self.TAU_MAX, tau)))


class LockedJoint:
    """PD that holds a joint at its initial angle with no follow term.

    Used for the gripper fingers: FreeDriveJoint's creep would walk them shut.
    """

    KP = 2000.0    # N per m -- the fingers are 0.1 kg, so this is stiff
    KD = 20.0      # N per m/s
    F_MAX = 70.0   # N, the Panda gripper's own grasp force

    def __init__(self, motor, fn, target):
        self.motor, self.fn, self.target = motor, fn, target
        self.pos, self.vel = joint_readers(motor)

    def update(self):
        f = self.KP * (self.target - self.pos()) - self.KD * self.vel()
        self.fn.SetConstant(max(-self.F_MAX, min(self.F_MAX, f)))


class StanceHolder:
    """Per-joint PD stance: the fallback when torch is unavailable.

    Holds the robot's initial pose. Not a locomotion policy -- it cannot step,
    only stiffen.
    """

    # At KP 26 the robot sags and topples. Verify by checking base height
    # (z ~ 0.27 standing), not per-joint error, which reads low even when fallen.
    KP = 120.0     # N.m per rad
    KD = 4.0       # N.m per rad/s
    TAU_MAX = 60.0  # N.m

    def __init__(self, motor, fn, target):
        self.motor, self.fn, self.target = motor, fn, target

    def update(self):
        err = self.target - self.motor.GetMotorAngle()
        tau = self.KP * err - self.KD * self.motor.GetMotorAngleDt()
        self.fn.SetConstant(max(-self.TAU_MAX, min(self.TAU_MAX, tau)))
