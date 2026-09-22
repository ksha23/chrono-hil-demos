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
"""Shared library behind the demos.

    chrono_env    import pychrono with loader-path and window-open checks
    config        every tuned number, with the measurement that chose it
    paths         vendored robot asset paths
    scenes        Go2 and Franka Panda scene builders
    urdf          URDF patching for Chrono's parser
    policy        Go2 locomotion policy wrapper
    controllers   joint-level control laws (free, locked, PD, stance)
    ik            IK via ikpy: point+direction command into joint PDs
    picking       screen-ray picking and spring-based grabbing
    watchdog      blow-up detector with ring buffer
    input/        keyboard/mouse input backends
"""

from .chrono_env import chrono, require_window
from .config import (GRAB_MAX_SPEED, GRAB_MIN_MASS, GRAB_OMEGA, GRAB_REACH, GRAB_ZETA,
                     HANDLE_SPEED, REACH_SPEED, RENDER_FPS, STEP)
from .controllers import FreeDriveJoint, LockedJoint, StanceHolder
from .ik import ArmChain, ReachIK
from .picking import (BeadLine, DragPlane, Grabber, camera_forward,
                      pick_along_ray, pick_near_ray)
from .policy import Go2Policy, find_go2_policy
from .scenes import ground_plane, scene_arm, scene_go2
from .urdf import make_chrono_safe_urdf

__all__ = [
    "chrono", "require_window", "STEP", "RENDER_FPS", "HANDLE_SPEED",
    "REACH_SPEED",
    "GRAB_OMEGA", "GRAB_ZETA", "GRAB_REACH", "GRAB_MAX_SPEED", "GRAB_MIN_MASS",
    "pick_along_ray", "pick_near_ray", "Grabber",
    "BeadLine", "DragPlane", "camera_forward",
    "FreeDriveJoint", "LockedJoint", "StanceHolder",
    "ArmChain", "ReachIK",
    "Go2Policy", "find_go2_policy",
    "ground_plane", "scene_go2", "scene_arm",
    "make_chrono_safe_urdf",
]
