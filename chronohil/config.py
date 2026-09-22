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
"""Numbers shared across the demos, each with the measurement behind it."""

STEP = 2e-3               # s, the physics step every demo uses
RENDER_FPS = 50           # the eye does not need the step rate
HANDLE_SPEED = 1.2        # m/s at full stick, for key-driven dragging

# How fast a key-held COMMAND moves, which is a different question from how
# fast a hand can drag something. A drag handle is free to outrun the thing it
# is pulling -- the spring stretches, and that stretch IS the feedback. A
# commanded point is not: whatever the target does, the arm has to follow it,
# and how well it manages is the only feedback there is.
#
# Measured on the Panda, driving the target 0.35 m and stopping: tip error
# while moving / overshoot once the keys are released, in mm --
#
#     0.20 m/s    3.6 / 15.5        0.60 m/s    29.9 /  31.8
#     0.35 m/s    9.3 / 30.9        1.20 m/s   103.9 / 184.9
#
# HANDLE_SPEED, which is right for dragging, would put the hand 10 cm behind
# its own target and then 18 cm past it, and an arm that overshoots by more
# than the target ever moved does not look controlled. 0.35 m/s is also about
# as fast as anyone wants a 7 kg arm swinging near them.
REACH_SPEED = 0.35        # m/s at full stick, for the commanded tip target

# The grab spring, and the one number that actually matters about it.
#
# The spring is a critically damped tracker, k = m*w^2, so the acceleration it
# commands is a = F/m = w^2 * x. THE MASS CANCELS. Scaling k by mass is right and
# is why one setting works on a 0.15 kg shin and a 6.9 kg torso; what it does NOT
# do is bound how hard the spring pulls, because that is set by w and by how far
# the handle is allowed to get from the body:
#
#     a_max = GRAB_OMEGA^2 * GRAB_REACH
#
# Nobody chose that product before. At the old w=90 and reach 0.45 it was
# 3645 m/s^2, or 372 g, for ANY body grabbed -- which is why grabbing a Go2 by
# the torso threw the whole robot at 25 m/s. Clamping the FORCE instead would
# have been a hack: it reintroduces the mass dependence that scaling k removed,
# making heavy bodies sluggish and light ones violent.
#
# So the budget is chosen first and w follows. 270 m/s^2 is about 27 g, which is
# enough for a dragged Go2 calf to track the cursor through the full 0.30 m of
# reach (measured: 0.300 m, peak speed 1.67 m/s, torso undisturbed) and 13x
# gentler than what it replaced.
GRAB_OMEGA = 30.0         # rad/s, about 4.8 Hz: roughly how fast a hand tracks
GRAB_ZETA = 1.0           # critically damped
GRAB_REACH = 0.30         # m, the furthest the handle may sit from the held point
GRAB_MAX_ACCEL = GRAB_OMEGA * GRAB_OMEGA * GRAB_REACH   # 270 m/s^2, stated so a
                          # change to either number shows up in the other
# ...and the limit of that argument, which is that it assumes a FREE body.
# A foot on a driven leg is not free: what resists you is the joint controller
# and the inertia of the whole limb, not the 40 g of toe. Scaling by the
# grabbed link's own mass gives a Go2 foot a 36 N/m spring and a 10.8 N pull,
# and it moves 7 mm -- which reads as the grab not working at all.
#
# So the effective mass has a floor. Below it you are not accelerating a free
# body, you are leaning on a mechanism, and the mass of the part you happened to
# take hold of is the wrong number. Measured on a Go2 foot: 7 mm at no floor,
# 412 mm at 1 kg, and the peak speed anywhere in the robot FALLS from 3.18 m/s
# (at a 2 kg floor) to 1.18. Heavier bodies are unaffected, since the floor
# never applies to them.
GRAB_MIN_MASS = 1.0       # kg of effective inertia, at minimum

GRAB_MAX_SPEED = 2.5      # m/s: a held body cannot outrun contact detection,
                          # which is what bounds the outcome when the floor
                          # above gives a light body a stiff spring
