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
"""Where the vendored assets are, resolved once.

Every asset path used to be derived from the module's own __file__. That worked
while everything lived in one file at the repository root and broke silently the
moment the code moved into a package: the Go2 policy lookup fell through to a
fine-tuned checkpoint sitting elsewhere on the author's machine, and the demo
ran with the wrong policy without saying so.
"""

import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GO2_DIR = os.path.join(ROOT, "go2_assets")
FRANKA_DIR = os.path.join(ROOT, "franka_assets")

GO2_URDF = os.path.join(GO2_DIR, "urdf", "go2.urdf")
GO2_POLICY = os.path.join(GO2_DIR, "go2_policy.pt")
FRANKA_URDF = os.path.join(FRANKA_DIR, "panda_chrono.urdf")
