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
# The thing being driven, behind a uniform interface.
# =============================================================================

import math

import pychrono as chrono
import pychrono.vehicle as veh


class VehiclePlant:
    """Wraps a Chrono::Vehicle behind the interface the simulation loop uses."""

    def __init__(self, vehicle_model, terrain, gearbox=None):
        self.model = vehicle_model
        self.vehicle = vehicle_model.GetVehicle()
        self.system = vehicle_model.GetSystem()
        self.terrain = terrain
        self.gearbox = gearbox

    def synchronize(self, t, inputs):
        self.model.Synchronize(t, inputs, self.terrain)

    def advance(self, step):
        self.model.Advance(step)

    def speed(self):
        return self.vehicle.GetSpeed()

    def status(self):
        return f"gear {self.gearbox.describe()}" if self.gearbox else ""

    def attach(self, vis):
        vis.AttachVehicle(self.vehicle)
