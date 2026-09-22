# Demos 1 and 2: the clock, and a person driving

Both demos are this one script. Nothing is passed on the command line: every
switch is a constant in the `CONFIGURATION` block at the bottom of
`tutorial_HIL_driver.py`.

```bash
python demos/driver/tutorial_HIL_driver.py          # run whatever is configured
python demos/driver/tutorial_HIL_driver.py --help   # the switches, listed
```

Every switch in the `CONFIGURATION` block can also be set on the command line,
under the same name, so nothing here needs a file edited first:

```bash
# Demo 1, twice: watch the drift column run away, then not
python demos/driver/tutorial_HIL_driver.py REALTIME=False
python demos/driver/tutorial_HIL_driver.py REALTIME=True

# Demo 2, a person driving off road. W/A/S/D drive; arrow keys are the chase camera
python demos/driver/tutorial_HIL_driver.py INPUT_SOURCE=keyboard KEYBOARD_MODE=held REALTIME=True

# The same, on rigid flat terrain
python demos/driver/tutorial_HIL_driver.py INPUT_SOURCE=keyboard KEYBOARD_MODE=held TERRAIN=rigid REALTIME=True

# The Audi is the one with a manual gearbox
python demos/driver/tutorial_HIL_driver.py VEHICLE=audi TRANSMISSION=manual INPUT_SOURCE=keyboard
```

A misspelt name is an error, not a silent no-op.

## Demo 1: does the clock matter

```python
INPUT_SOURCE = "data"      # a scripted drive, no human
REALTIME     = False       # then rerun with True
```

No window interaction needed. Watch the `drift` column in the console: with
`False` it runs away, because the simulation is going as fast as the machine
allows. Set `REALTIME = True` and it stays near zero. That is the whole
precondition for a human in the loop, and it is one line:
`vehicle.EnableRealtime(True)`.

Only a Chrono::Vehicle has that call. Anything else does the same thing with
`chrono.ChRealtimeStepTimer().Spin(step)`, once per loop; demo 3 does.

## Demo 2: a person driving, off road

```python
INPUT_SOURCE  = "keyboard"
KEYBOARD_MODE = "held"     # needs PyChrono build 1187 or later
```

Click the 3D window so it has focus, then:

- **W/A/S/D** drive. **C** centres steering, **R** releases the pedals, **L**
  locks the current inputs.
- **The arrow keys are the chase camera**, not the car. This catches everyone,
  because demo 3's input window drives with the arrows.
- The gear keys are printed at startup, from `hil_gearbox.KEYBOARD_HELP`.

`KEYBOARD_MODE` is the point of the demo as much as the driving is. With
`"cumulative"` a press nudges the input and it stays where you left it; with
`"held"` the input follows the keys currently down, like a driving game. Same
keyboard, same vehicle, and they feel nothing alike.

## The switches that are not demos

They live in the same `CONFIGURATION` block and are named after the constant
that turns each one on.

| set this | and you get |
|---|---|
| `VEHICLE`, `SHOW_*` | `"hmmwv"`, `"sedan"`, `"uazbus"`, `"gator"` or `"audi"`, and the built-in Irrlicht overlay |
| `TRANSMISSION = "manual"` | a gearbox to row through. Only `VEHICLE = "audi"` has one; the others say so rather than silently ignoring it. |


`SHOW_VEHICLE_HUD` is the one to leave on. It is Chrono's own overlay, not
ours, and it already reports the RTF -- both the simulation's and the step's --
next to the driver inputs, which is the whole of Demo 1 in the corner of the
Demo 2 window:


## The files here

| | |
|---|---|
| `tutorial_HIL_driver.py` | the simulation loop and the `CONFIGURATION` block. One loop for both input sources. |
| `hil_plants.py` | the vehicle, behind a five-method interface the loop is written against |
| `hil_scene.py` | the terrain (deformable soil by default, or rigid) |
| `hil_gearbox.py` | `TRANSMISSION`: the gearbox, and the keys that shift it |
