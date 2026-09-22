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
# Demos 1 and 2: the clock, and a person driving.
#
#     python demos/driver/tutorial_HIL_driver.py          run it
#     python demos/driver/tutorial_HIL_driver.py --help   the switches
#
#   Demo 1  INPUT_SOURCE="data"      REALTIME=False, then True
#   Demo 2  INPUT_SOURCE="keyboard"  KEYBOARD_MODE="held"
#
# Every switch is a constant in the CONFIGURATION block at the bottom.
# =============================================================================

import math
import os
import sys
import time

# --help before anything heavy: this script has no __main__ guard, it runs on
# import, so a check placed further down never gets reached.
if "-h" in sys.argv or "--help" in sys.argv:
    print("usage: python demos/driver/tutorial_HIL_driver.py [NAME=value ...]\n"
          "\n"
          "Every switch lives in the CONFIGURATION block at the bottom of this\n"
          "file. Edit it, or set the same names on the command line:\n"
          "\n"
          "  TERRAIN=scm|rigid   REALTIME=True|False\n"
          "  INPUT_SOURCE=data|keyboard\n"
          "  KEYBOARD_MODE=cumulative|held   VEHICLE=hmmwv|sedan|uazbus|gator|audi\n"
          "  TRANSMISSION=automatic|manual   SHOW_VEHICLE_HUD=True|False\n"
          "\n"
          "  Demo 1, does the clock matter\n"
          "      INPUT_SOURCE = \"data\",  REALTIME = False  then True\n"
          "      Watch the drift column in the console run away, then not.\n"
          "\n"
          "  Demo 2, a person driving, off road\n"
          "      INPUT_SOURCE = \"keyboard\",  KEYBOARD_MODE = \"held\"\n"
          "      W/A/S/D drive, in the 3D window. The arrow keys there are\n"
          "      the chase camera, not driving controls.\n"
          "\n"
          "  Not numbered, and in the same block: VEHICLE, TRANSMISSION\n"
          "  and the SHOW_* overlay switches.")
    raise SystemExit(0)

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
from chronohil.chrono_env import chrono, require_window

import pychrono.irrlicht as irr
# veh is used at module level in the CONFIGURATION block, so this import
# must be here (not deferred).
import pychrono.vehicle as veh

import hil_gearbox
import hil_plants
import hil_scene


# =============================================================================
# VEHICLE
# =============================================================================

class JsonVehicle:
    """Wrap a JSON WheeledVehicle so it has the same GetVehicle()/GetSystem()
    interface as the model wrappers (HMMWV_Full, Sedan, ...)."""

    def __init__(self, vehicle):
        self.vehicle = vehicle

    def GetVehicle(self):
        return self.vehicle

    def GetSystem(self):
        return self.vehicle.GetSystem()

    def Synchronize(self, t, inputs, terrain):
        self.vehicle.Synchronize(t, inputs, terrain)

    def Advance(self, step):
        self.vehicle.Advance(step)


# Ride height at spawn, per model: how far above the ground the vehicle's
# reference frame has to start so that it settles onto its tires rather than
# through them.
SPAWN_HEIGHT = {"hmmwv": 1.6, "sedan": 0.5, "uazbus": 0.4, "gator": 0.4, "audi": 0.5}


def build_vehicle(name, start, transmission="automatic"):
    """Construct and initialize one of a few Chrono::Vehicle models.

    start         where to put it
    transmission  "automatic" or "manual" (TRANSMISSION)

    Returns (vehicle_model, chase_distance): vehicle_model is the wrapper
    object (what used to be called `hmmwv`); chase_distance is how far back
    the camera should sit for a vehicle of that size.
    """
    if name == "hmmwv":
        v = veh.HMMWV_Full()
        v.SetContactMethod(chrono.ChContactMethod_SMC)
        v.SetChassisCollisionType(veh.CollisionType_NONE)
        v.SetChassisFixed(False)
        v.SetInitPosition(start)
        v.SetEngineType(veh.EngineModelType_SHAFTS)
        v.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
        v.SetDriveType(veh.DrivelineTypeWV_AWD)
        v.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
        v.SetTireType(tire_model)
        v.SetTireStepSize(tire_step_size)
        v.Initialize()
        chase_dist = 6.0

    elif name == "sedan":
        v = veh.Sedan()
        v.SetContactMethod(chrono.ChContactMethod_SMC)
        v.SetChassisFixed(False)
        v.SetInitPosition(start)
        v.SetTireType(tire_model)
        v.SetTireStepSize(tire_step_size)
        v.Initialize()
        chase_dist = 6.0

    elif name == "uazbus":
        v = veh.UAZBUS()
        v.SetContactMethod(chrono.ChContactMethod_SMC)
        v.SetChassisFixed(False)
        v.SetInitPosition(start)
        v.SetTireType(tire_model)
        v.SetTireStepSize(tire_step_size)
        v.Initialize()
        chase_dist = 6.0

    elif name == "gator":
        v = veh.Gator()
        v.SetContactMethod(chrono.ChContactMethod_SMC)
        v.SetChassisFixed(False)
        v.SetInitPosition(start)
        v.SetTireType(tire_model)
        v.SetTireStepSize(tire_step_size)
        v.Initialize()
        chase_dist = 6.0

    elif name == "audi":
        # Built from JSON: only the Audi ships a manual transmission.
        v = veh.WheeledVehicle(veh.GetVehicleDataFile("audi/json/audi_Vehicle.json"))
        v.Initialize(start)
        v.SetChassisVisualizationType(chrono.VisualizationType_MESH)
        v.SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
        v.SetSteeringVisualizationType(chrono.VisualizationType_MESH)
        v.SetWheelVisualizationType(chrono.VisualizationType_MESH)

        engine = veh.ReadEngineJSON(veh.GetVehicleDataFile("audi/json/audi_EngineSimpleMap.json"))
        gearbox_json = ("audi/json/audi_ManualTransmissionShafts.json" if transmission == "manual"
                        else "audi/json/audi_AutomaticTransmissionSimpleMap.json")
        gearbox = veh.ReadTransmissionJSON(veh.GetVehicleDataFile(gearbox_json))
        v.InitializePowertrain(veh.ChPowertrainAssembly(engine, gearbox))

        for axle in v.GetAxles():
            for wheel in axle.GetWheels():
                tire = veh.ReadTireJSON(veh.GetVehicleDataFile("audi/json/audi_TMeasyTire.json"))
                tire.SetStepsize(tire_step_size)
                v.InitializeTire(tire, wheel, chrono.VisualizationType_MESH)

        return JsonVehicle(v), 6.0

    else:
        raise ValueError(f"unknown VEHICLE {name!r}")

    if transmission == "manual":
        # Wrapper classes hard-code one powertrain; requesting "manual" on them
        # silently leaves the vehicle with no transmission at all.
        print(f"[gearbox] TRANSMISSION = 'manual', but the {name} model ships only an "
              f"automatic and keeps it.\n"
              f"[gearbox] Use VEHICLE = 'audi' for a manual gearbox.")

    v.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    v.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
    v.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
    v.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    v.SetTireVisualizationType(chrono.VisualizationType_MESH)
    return v, chase_dist


# =============================================================================
# MAIN
# =============================================================================


def build_plant(plan):
    """Create the vehicle and terrain. Returns (plant, chase_distance)."""
    if VEHICLE:
        vehicle_model, chase_dist = build_vehicle(VEHICLE, plan.start, TRANSMISSION)
        system = vehicle_model.GetSystem()
        system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        # Default solver under-solves vehicle contacts (suspension judders).
        system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
        system.GetSolver().AsIterative().SetMaxIterations(150)
        terrain = plan.build(system, vehicle_model.GetVehicle())

        gearbox = hil_gearbox.Gearbox(vehicle_model.GetVehicle())
        gearbox.set_manual_shifting(START_IN_MANUAL_SHIFT)
        return hil_plants.VehiclePlant(vehicle_model, terrain, gearbox), chase_dist

    raise ValueError(f"unknown VEHICLE {VEHICLE!r}")


def main():
    # -----------------------------------------------------------------------
    # What are we driving (VEHICLE), and on what terrain?
    # -----------------------------------------------------------------------
    spawn_height = SPAWN_HEIGHT.get(VEHICLE, 0.5)
    plan = hil_scene.plan_scene(spawn_height, TERRAIN)

    plant, chase_dist = build_plant(plan)
    system = plant.system
    terrain = getattr(plant, "terrain", None)
    gearbox = getattr(plant, "gearbox", None)

    vehicle = plant.vehicle

    # -----------------------------------------------------------------------
    # Create the driver system - this is where the human plugs in
    # -----------------------------------------------------------------------
    if INPUT_SOURCE == "data":
        ### Demo 1: scripted inputs - no human in the loop ###
        # (time, steering, throttle, braking)
        data_entries = [
            (0.0, 0.0, 0.0, 0.0),
            (0.5, 0.0, 0.8, 0.0),
            (4.0, 0.4, 0.8, 0.0),
            (8.0, -0.4, 0.8, 0.0),
            (12.0, 0.0, 0.0, 0.8),
        ]
        data = veh.vector_Entry([veh.DataDriverEntry(*e) for e in data_entries])
        driver = veh.ChDataDriver(vehicle, data)   # interpolates the table by time

    elif INPUT_SOURCE == "keyboard":
    # W/A/S/D drive.  Arrow keys are the CAMERA, not the car.
        driver = veh.ChInteractiveDriver(vehicle)
        driver.SetGains(4.0, 4.0, 4.0)  # first-order lag from key to applied input

        # "held" = input follows keys currently down (like a driving game).
        # "cumulative" = each press nudges a target that stays put.
        # "held" requires build 1187+; fall back gracefully.
        if KEYBOARD_MODE == "held":
            if hasattr(veh.ChInteractiveDriver, "KeyboardMode_HELD"):
                driver.SetKeyboardMode(veh.ChInteractiveDriver.KeyboardMode_HELD)
            else:
                print('[keyboard] this PyChrono predates KeyboardMode; '
                      'using "cumulative". Update to build 1187 or later '
                      'for "held".')

        steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
        throttle_time = 1.0  # time to go from 0 to +1
        braking_time = 0.3   # time to go from 0 to +1
        driver.SetSteeringDelta(render_step_size / steering_time)
        driver.SetThrottleDelta(render_step_size / throttle_time)
        driver.SetBrakingDelta(render_step_size / braking_time)

    else:
        raise ValueError(f"unknown INPUT_SOURCE {INPUT_SOURCE!r}")

    driver.Initialize()

    # -----------------------------------------------------------------------
    # Irrlicht window (vehicle flavour: chase camera + driver HUD)
    # -----------------------------------------------------------------------
    title = f"{VEHICLE} on {plan.name} - human in the loop"
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle(title)
    vis.SetWindowSize(1280, 800)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), chase_dist, 0.5)
    vis.Initialize()
    require_window(vis, how="run tests_parts.py arm or tests_reach.py")
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    plant.attach(vis)
    if INPUT_SOURCE == "keyboard":
        vis.AttachDriver(driver)  # route Irrlicht key events to the driver

    vis.EnableStats(SHOW_VEHICLE_HUD)
    vis.SetHUDLocation(*HUD_CORNER)
    vis.ShowInfoPanel(SHOW_SIM_INFO_PANEL)
    vis.ShowProfiler(SHOW_PROFILER)

    # -----------------------------------------------------------------------
    # Real-time setup (Demo 1)
    # -----------------------------------------------------------------------
    vehicle.EnableRealtime(REALTIME)   # True: Advance() waits for wall-clock

    # -----------------------------------------------------------------------
    # Simulation loop
    # -----------------------------------------------------------------------
    render_steps = math.ceil(render_step_size / step_size)
    report_steps = math.ceil(1.0 / step_size)  # console report once per sim second

    print(f"\nVEHICLE={VEHICLE}  INPUT_SOURCE={INPUT_SOURCE}"
          f"  REALTIME={REALTIME}  step_size={step_size}")
    if INPUT_SOURCE == "keyboard" and gearbox and gearbox.present:
        ### TRANSMISSION: Chrono already binds these; this file need not ###
        print("\nkeys handled by Chrono's Irrlicht event receiver:")
        print(hil_gearbox.KEYBOARD_HELP)
    print()
    header = (f"{'sim t':>7} {'wall t':>7} {'drift':>7} {'RTF':>6} {'speed':>7} "
              f" {'steer':>6} {'thr':>5} {'brk':>5}")
    print(header + ("  status" if plant.status() else ""))

    step_number = 0
    wall_start = time.perf_counter()
    last_wall, last_sim = 0.0, 0.0  # previous report, for the measured RTF below

    while vis.Run():
        sim_time = system.GetChTime()

        # Render scene
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs (three floats) - the shape every plant here is driven through
        driver_inputs = driver.GetInputs()

        # THE BINDING: hand the three numbers to whatever is controlled. For
        # a Chrono::Vehicle this does nothing, because the vehicle reads them
        # in Synchronize; for anything else it is where they land.

        # Update modules (process inputs from other modules)
        driver.Synchronize(sim_time)
        if terrain is not None:
            terrain.Synchronize(sim_time)
        plant.synchronize(sim_time, driver_inputs)
        vis.Synchronize(sim_time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        if terrain is not None:
            terrain.Advance(step_size)
        # Vehicle.Advance steps the ChSystem internally; do not also call
        # DoStepDynamics or the clock runs at double speed.
        plant.advance(step_size)
        vis.Advance(step_size)

        # Reset the wall clock after the first iteration: render, terrain
        # init, and shader compilation all happen here, and that cost is
        # not simulation time.
        if step_number == 0:
            wall_start = time.perf_counter()

        if step_number % report_steps == 0:
            wall = time.perf_counter() - wall_start
            # RTF over the last interval: wall seconds per simulated second,
            # including drawing (vehicle.GetRTF() times physics only).
            d_wall = wall - last_wall
            d_sim = sim_time - last_sim
            rtf = (d_wall / d_sim) if d_sim > 0 else 0.0
            last_wall, last_sim = wall, sim_time
            print(f"{sim_time:7.2f} {wall:7.2f} {wall - sim_time:+7.3f} {rtf:6.2f}"
                  f" {plant.speed():7.2f}  {driver_inputs.m_steering:6.2f}"
                  f" {driver_inputs.m_throttle:5.2f} {driver_inputs.m_braking:5.2f}"
                  f"  {plant.status()}")

        step_number += 1

    # Vehicle visual system segfaults on teardown (macOS, upstream Irrlicht bug).
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


# =============================================================================
# CONFIGURATION
# =============================================================================

VEHICLE = "hmmwv"               # "hmmwv" | "sedan" | "uazbus" | "gator" | "audi"
TRANSMISSION = "automatic"      # "automatic" | "manual" (only "audi" has manual)
START_IN_MANUAL_SHIFT = False   # True = start an automatic in manual-shift mode
TERRAIN = "scm"                 # "scm" (deformable soil) | "rigid"
INPUT_SOURCE = "data"           # "data" (scripted, Demo 1) | "keyboard" (Demo 2)

# Demo 2: what a keypress means.  "cumulative" | "held"
#   "cumulative"  a keypress nudges the input by a delta and it stays there
#   "held"        the input follows the keys held down, as in a driving game
KEYBOARD_MODE = "cumulative"    # "cumulative" | "held"
REALTIME = False                # True = pace to wall clock (Demo 1: try both)

step_size = 3e-3
tire_step_size = 1e-3
render_step_size = 1.0 / 50

# TMEASY has no geometry to press into soil -- it crosses SCM leaving no ruts.
# So TERRAIN picks the tire automatically.
tire_model = (veh.TireModelType_RIGID_MESH if TERRAIN == "scm"
              else veh.TireModelType_TMEASY)

SHOW_VEHICLE_HUD = True
HUD_CORNER = (10, 10)
SHOW_SIM_INFO_PANEL = False
SHOW_PROFILER = False

# Command-line overrides: NAME=value, same names as the constants above.
# Unknown names are errors, not silent no-ops.
_OVERRIDABLE = ("VEHICLE", "TRANSMISSION", "START_IN_MANUAL_SHIFT",
                "TERRAIN", "INPUT_SOURCE", "KEYBOARD_MODE", "REALTIME",
                "SHOW_VEHICLE_HUD", "SHOW_SIM_INFO_PANEL", "SHOW_PROFILER")

for _arg in sys.argv[1:]:
    if "=" not in _arg:
        raise SystemExit(f"expected NAME=value, got {_arg!r}. "
                         f"Names: {', '.join(_OVERRIDABLE)}")
    _k, _v = _arg.split("=", 1)
    if _k not in _OVERRIDABLE:
        raise SystemExit(f"unknown setting {_k!r}. "
                         f"Names: {', '.join(_OVERRIDABLE)}")
    if isinstance(globals()[_k], bool) and _v not in ("True", "False"):
        raise SystemExit(f"{_k} is True or False, got {_v!r}")
    globals()[_k] = {"True": True, "False": False}.get(_v, _v)

# TERRAIN may have changed above, and the tire follows it.
tire_model = (veh.TireModelType_RIGID_MESH if TERRAIN == "scm"
              else veh.TireModelType_TMEASY)

main()
