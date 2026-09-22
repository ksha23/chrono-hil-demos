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
# Gearbox wrapper: the transmission API lives on the vehicle, not on ChDriver.
# On the keyboard (Demo 2), Chrono's Irrlicht event receiver already binds
# the shift keys. For any other device, Gearbox below is the adapter.
# =============================================================================

import pychrono.vehicle as veh


# What Chrono's Irrlicht event receiver already binds for you, once the driver
# is attached with vis.AttachDriver(driver).  Printed at startup so nobody has
# to go looking for it in the C++ source.
KEYBOARD_HELP = """\
  driving (KEYBOARD_MODE = "cumulative")
    W/S  throttle up / down (S brakes once throttle reaches 0)
    A/D  steer left / right
    C    center steering                     R    release the pedals
    L    lock the current inputs
  driving (KEYBOARD_MODE = "held")
    W    hold to accelerate                  S    hold to brake
    A/D  hold to steer left / right          E    hold the clutch (manual only)
  camera (NOT driving controls)
    arrows  zoom and orbit the chase camera  PgUp/PgDn  raise / lower it
  gears (automatic transmission)
    Z    toggle drive mode  D <-> R          X    neutral
    T    toggle AUTO <-> MANUAL shifting
    [    shift down                          ]    shift up
  gears (manual transmission)
    [    shift down                          ]    shift up
    Q/E  clutch out / in ("cumulative" only)\
"""


class Gearbox:
    """Read and drive a vehicle's transmission, automatic or manual."""

    def __init__(self, vehicle):
        self.transmission = vehicle.GetTransmission()
        self.auto = None
        self.manual = None
        if self.transmission is not None:
            self.auto = self.transmission.asAutomatic()
            self.manual = self.transmission.asManual()

    @property
    def present(self):
        return self.transmission is not None

    # -- reading ------------------------------------------------------------

    def describe(self):
        """One short string: 'D 2/3 auto', 'N', 'R', 'M 3/6'."""
        if not self.present:
            return "--"
        gear = self.transmission.GetCurrentGear()
        top = self.transmission.GetMaxGear()
        if self.auto is not None:
            mode = self.auto.GetDriveMode()
            if mode == veh.ChAutomaticTransmission.DriveMode_NEUTRAL:
                return "N"
            if mode == veh.ChAutomaticTransmission.DriveMode_REVERSE:
                return "R"
            shifting = ("auto" if self.auto.GetShiftMode() ==
                        veh.ChAutomaticTransmission.ShiftMode_AUTOMATIC else "man")
            return f"D {gear}/{top} {shifting}"
        return f"M {gear}/{top}"

    # -- setting up ---------------------------------------------------------

    def set_manual_shifting(self, manual):
        """An automatic in AUTO mode overrides your gear on the next step."""
        if self.auto is not None:
            self.auto.SetShiftMode(
                veh.ChAutomaticTransmission.ShiftMode_MANUAL if manual
                else veh.ChAutomaticTransmission.ShiftMode_AUTOMATIC)
