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
"""Notice the simulation going unstable, and say what was happening when it did.

A blow-up is easy to see and hard to describe afterwards: by the time anything
is on screen the state that caused it is several hundred steps in the past. This
keeps a short ring buffer so the run-up can be printed, not just the crater.
"""

import math


class Watchdog:
    """Samples the system each step and reports the first sign of trouble."""

    SPEED = 25.0        # m/s. Nothing in these scenes moves this fast honestly.
    HISTORY = 0.5       # s of run-up to keep

    def __init__(self, system, step, grabber=None):
        self.system, self.step, self.grabber = system, step, grabber
        self.ring = []
        self.n = int(self.HISTORY / step)
        self.tripped = False

    def sample(self, held=False):
        sys_ = self.system
        t = sys_.GetChTime()
        worst_v, worst_name = 0.0, ""
        nonfinite = ""
        for b in sys_.GetBodies():
            if b.IsFixed():
                continue
            p, v = b.GetPos(), b.GetPosDt()
            if not all(map(math.isfinite, (p.x, p.y, p.z, v.x, v.y, v.z))):
                nonfinite = b.GetName()
                break
            s = v.Length()
            if s > worst_v:
                worst_v, worst_name = s, b.GetName()
        f = 0.0
        try:
            f = self.grabber.force() if (self.grabber and held) else 0.0
        except Exception:
            pass
        row = (t, worst_v, worst_name, sys_.GetNumContacts(), f, held, nonfinite)
        self.ring.append(row)
        if len(self.ring) > self.n:
            self.ring.pop(0)

        if self.tripped:
            return
        if nonfinite:
            self._report(f"{nonfinite} went non-finite")
        elif worst_v > self.SPEED:
            self._report(f"{worst_name} reached {worst_v:.1f} m/s")

    def _report(self, why):
        self.tripped = True
        print("\n" + "=" * 72)
        print(f"[watchdog] UNSTABLE at t = {self.ring[-1][0]:.3f} s: {why}")
        print("=" * 72)
        print(f"{'t':>7s} {'fastest body':>18s} {'m/s':>9s} {'contacts':>9s} "
              f"{'spring N':>9s} {'held':>5s}")
        every = max(1, len(self.ring) // 12)
        for i, (t, v, n, c, f, h, nf) in enumerate(self.ring):
            if i % every and i != len(self.ring) - 1:
                continue
            print(f"{t:7.3f} {n[:18]:>18s} {v:9.2f} {c:9d} {f:9.1f} {str(h):>5s}")
        print("=" * 72 + "\n", flush=True)
