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
"""The smallest Chrono program that shows you something. Start here.

    python demos/first_steps/hello_chrono.py

A ball falls onto a floor.  That is all it does, and the reason it is worth
thirty seconds of your time is that EVERY other demo in this tutorial -- the
HMMWV on soil, the quadruped, the robot arm -- is this same four-part shape
with more bodies in it:

    1. a SYSTEM        owns the bodies, the constraints, and the clock
    2. some BODIES     mass, collision geometry, where they start
    3. a WINDOW        draws whatever the system currently holds
    4. a LOOP          draw, then advance time by one step, forever

Nothing here is tutorial-specific and nothing is patched: this runs on the
PyChrono from the handout's conda command.
"""

import pychrono as chrono
import pychrono.irrlicht as irr

# -----------------------------------------------------------------------------
# 1. THE SYSTEM
#
# NSC and SMC are the two contact formulations.  NSC ("non-smooth") resolves
# contact as a constraint and takes big steps; SMC treats it as a stiff spring
# and needs small ones.  Everything in this tutorial uses NSC.
# -----------------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
# Chrono does not pick an "up" for you. This tutorial is Z-up throughout, which
# is what Chrono::Vehicle assumes, so the camera below has to be told as well.
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# -----------------------------------------------------------------------------
# 2. THE BODIES
#
# The ChBodyEasy* helpers do in one line what would otherwise be four: make a
# body, give it mass and inertia for that shape, add collision geometry, and
# add something to draw.  The two True flags are "draw it" and "collide".
# -----------------------------------------------------------------------------
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.4)

ground = chrono.ChBodyEasyBox(6, 6, 0.4, 1000, True, True, mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.2))
ground.SetFixed(True)          # infinite mass: gravity and contact ignore it
sys.Add(ground)

ball = chrono.ChBodyEasySphere(0.25, 500, True, True, mat)
ball.SetPos(chrono.ChVector3d(0, 0, 2.0))
sys.Add(ball)

# -----------------------------------------------------------------------------
# 3. THE WINDOW
#
# The visual system is separate from the physics on purpose: the same ChSystem
# runs headless without one, which is how tests_reach.py and tests_parts.py
# measure this repo without a display.
# -----------------------------------------------------------------------------
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("hello, Chrono")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # match the Z-up above
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4, -4, 2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()

# -----------------------------------------------------------------------------
# 4. THE LOOP
#
# Draw, then step.  DoStepDynamics is the whole physics engine: one call
# advances every body and resolves every contact by `step` seconds.
#
# Note what is NOT here: anything about wall-clock time.  This loop runs as
# fast as the machine allows, which is right for a batch job and wrong the
# moment a person is watching.  That is Demo 1.
# -----------------------------------------------------------------------------
step = 2e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(step)
