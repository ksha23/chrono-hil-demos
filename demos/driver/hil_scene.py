# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of this repository and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Terrain for Demos 1 and 2: deformable soil (SCM) or a flat rigid patch.
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh


def _road_material(system):
    """Contact material matching the system's formulation (SMC or NSC).
    Mismatched material = no contact at all; the vehicle falls through."""
    minfo = chrono.ChContactMaterialData()
    minfo.mu = 0.9
    minfo.cr = 0.01
    minfo.Y = 2e7
    return minfo.CreateMaterial(system.GetContactMethod())


class ScenePlan:
    """Two-step build: plan first (no system needed), build after the vehicle."""

    def __init__(self, name, start, kind="scm"):
        self.name = name
        self.start = start
        self.kind = kind

    def build(self, system, vehicle=None):
        """Add terrain to system. vehicle is optional (SCM uses it for active domains)."""
        if self.kind == "scm":
            return self._build_scm(system, vehicle)
        terrain = veh.RigidTerrain(system)
        patch = terrain.AddPatch(_road_material(system), chrono.CSYSNORM, 200.0, 200.0)
        patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
        terrain.Initialize()
        return terrain

    def _build_scm(self, system, vehicle=None):
        """Deformable soil that yields under pressure and keeps the rut."""
        terrain = veh.SCMTerrain(system)
        # Softer than Chrono's own demo: 12 cm ruts you can see, not 6 cm.
        terrain.SetSoilParameters(5e5,   # Bekker Kphi
                                  0,     # Bekker Kc
                                  1.1,   # Bekker n exponent
                                  0,     # Mohr cohesive limit (Pa)
                                  30,    # Mohr friction limit (degrees)
                                  0.01,  # Janosi shear coefficient (m)
                                  2e7,   # elastic stiffness before yield (Pa/m)
                                  3e4)   # damping (Pa s/m)
        # Bulldozing piles soil along rut edges so deformation reads at a glance.
        terrain.EnableBulldozing(True)
        terrain.SetBulldozingParameters(55,    # erosion angle of friction, deg
                                        0.8,   # displaced vs pressed material
                                        1,     # erosion refinement passes
                                        3)     # concentric erosion iterations
        terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0.0, 0.15)
        # Active domains: SCM only ray-casts near each wheel (~3x fewer rays).
        if vehicle is not None:
            for axle in vehicle.GetAxles():
                for w in (0, 1):
                    terrain.AddActiveDomain(axle.GetWheel(w).GetSpindle(),
                                            chrono.ChVector3d(0, 0, 0),
                                            chrono.ChVector3d(1.0, 0.6, 1.0))
        terrain.Initialize(SCM_PATCH, SCM_PATCH, SCM_MESH)
        return terrain


# Handling tires (TMEASY) have no geometry to press into SCM -- use RIGID_MESH.
SCM_TIRE = "rigid_mesh"

SCM_PATCH = 60.0   # metres; SCM allocates nodes on demand, so patch size is free
SCM_MESH = 0.08    # metres; 0.08 runs real-time, 0.02 does not


def plan_scene(spawn_height, kind="scm"):
    """Resolve the scene to a spawn pose and a build plan, without a ChSystem.

    spawn_height   how far above the ground the plant should start
    kind           "rigid" for the flat road, "scm" for deformable soil
    """
    name = "deformable soil" if kind == "scm" else "flat terrain"
    return ScenePlan(name,
                     chrono.ChCoordsysd(chrono.ChVector3d(0, 0, spawn_height),
                                        chrono.QUNIT),
                     kind)
