# Demo 3: interacting with a mouse

Two different things a person can be to a running simulation, in one demo. In
`go2` you are a **disturbance**: you pull on a leg, and a locomotion policy
either rejects it or does not. In `arm` you are the **controller**: you move a
point, and seven joints are solved to put the robot's hand on it. The physics
never stops in either case, which is the whole point.

```bash
python demos/manipulate/main.py arm        # steer a Franka Panda by its hand
python demos/manipulate/main.py go2        # drag a leg; a locomotion policy answers
python demos/manipulate/main.py --help
```

## `arm`: commanding the hand

A green ball is the point you are asking the hand to be at. A small blue ball
is where the hand actually is, and the green beads between them are the gap.
Most of the time the blue one sits inside the green one; that is what working
looks like.

**Arrows** move the target in world X and Y, **`[`** and **`]`** move it down
and up (they toggle, so press again to stop). **C** puts the target back on the
hand. Where this build can read the 3D window, the **mouse** can drag the
target directly -- press on the green ball and it follows the cursor in a plane
facing the camera -- and pressing anywhere else grabs a LINK instead and shoves
the arm, which is the disturbance.

What to watch:

- **The gap opens and closes.** It grows while you are moving, shrinks when you
  stop, and grows a lot if you push the arm. Those three look different, and
  the beads say which is happening.
- **The gripper stays pointing down.** Wherever you take the target, the hand
  arrives at it the way it would have to arrive to pick something up. That is
  five of the arm's seven freedoms spent -- three on the point, two on the
  direction -- and it is the thing the solver is for. The readout line prints
  `tilt`, which is how far off vertical the hand really is, measured on the
  arm rather than on the solver's answer: about 2 degrees while you drive it.
- **Push a link while the target is still.** You are fighting a controller with
  a goal now, not posing a limp arm. Let go and the hand comes back -- and it
  comes back level, which is the second half of rejecting a disturbance.
- **Drive it out to the end of its reach.** The target stops, the console says
  why, and the readout line says `AT THE REACH LIMIT`. Holding the hand
  vertical costs workspace: the fence is 0.65 m and 0.70 m up, not the Panda's
  published 0.85 m, because the arm cannot get its wrist over a point out
  there and still have the fingers facing the floor.

### What it measures

`python tests_reach.py` drives the same code with no window and no human:

| what the person did | measured | the solver this replaced |
|---|---|---|
| hold 3 s, no input | **0.18 mm** | 0.57 mm |
| tracking a 0.35 m/s command | **11.68 mm** (peak 33 mm) | 9.40 mm |
| 0.5 s after the keys are released | **12.33 mm** (overshoot 32 mm) | 5.40 mm |
| 2 s after the keys are released | **1.30 mm** | 2.89 mm |
| held at the reach fence | **0.92 mm** | 20.40 mm |
| home after a 20 s tour of the workspace | **0.10 mm**, no joint more than 2.8 deg from where it started | 0.04 mm, 2.8 deg |
| forced out to the straight-arm singularity | stalls **854 mm** short, nothing above 1.06 m/s, **0.40 mm** once recentred | 14.15 mm |
| 4 s after a 25 cm shove on a link | **0.13 mm** (the link gave 127 mm, the hand was pushed 158 mm off and 10.9 deg off vertical) | 4.35 mm |
| mouse dragging the target 0.25 m | **4.80 mm** | 6.84 mm |
| **gripper off straight down** | **7.65 deg** at the 95th percentile, 1.71 median | **66.15 deg** at the 95th, 12.93 median |

Not one tolerance in `tests_reach.py` moved when the solver was swapped, which
is what makes those two columns comparable. The last row is the reason for the
swap, and the nine above it are why it is in a test file at all: **the old arm
passed every one of them while arriving at the point lying on its side.** A
distance cannot see an orientation.

One number went the wrong way and is worth knowing: settling half a second
after the keys stop is 12.33 mm against 5.40, because a pose solver has no
velocity feedforward and its integrator has to unwind after a move.

The solve is 3.8 ms at the median against the physics step's 0.14 -- 6.8 at
the mean, on a tail of asks the arm cannot meet -- so the SOLVER sets this
demo's speed, not Chrono. It runs at 25 Hz under a 500 Hz simulation for that
reason. Measured on the running demo with the 3D window open, in wall seconds
per simulated second: standing idle it holds real time at **1.028**, and with
the target driven the whole time it is **1.194** at 25 Hz against 1.362 at
50 Hz, on a floor of 1.091 that is the renderer and can't be traded away.
Headless RTF is 4.3x and tells you none of this.

Chrono ships its own IK -- `IndustrialKinematicsNdofNumerical`, compiled C++,
**300x faster per solve** -- and it was tried here and rejected. It takes a
full pose, so a wrist roll has to be invented, and on this redundant arm an
invented roll stops it converging: 98 of 190 points in the fenced workspace
missed, and the gripper 116 degrees off vertical at the 95th percentile. The
reasoning and the numbers are at the top of `chronohil/ik.py`.

### How the setpoint moves

Everything a person does to the arm ends up as one number -- `Reach.target`, a
point in world coordinates -- and there are exactly two ways in, which converge
before anything downstream sees them:

```
keys   main.py -> reach.command(steer, throttle, braking, lift, dt)
                     demo 1's own convention, reused: two pedals become one
                     axis, plus a lift axis.  World axes, NOT camera axes --
                     the camera orbits, so camera-relative keys would mean
                     something different every time you moved the view.
                          target += (throttle - braking, steer, lift) * v*dt

mouse  main.py -> reach.mouse_update(console)          <- gets first refusal
                     a press whose ray passes through the green ball owns
                     the button until it comes up; miss, and the same press
                     falls through to drag.mouse_update(), which springs a
                     LINK instead.  One button, no mode key.
                          ray -> DragPlane.hit() -> a point in a plane facing
                          the camera

both   -> Reach.set_target(p)      THE ONE CHOKE POINT, and why it exists:
                     ceiling first (z <= REACH_TOP), then the sphere about the
                     shoulder (REACH_MIN <= |p - shoulder| <= REACH_MAX), then
                     target = p.  Writing `reach.target` directly bypasses the
                     fence, which is what tests_reach.py does on purpose.
```

The green ball is not the setpoint. `draw()` does `marker.SetPos(self.target)`
once a frame, so the ball is a RENDERING of the setpoint -- which is why its
pose can simply be set. It is `SetFixed(True)` and `EnableCollision(False)`: a
request, not an object. Nothing in the scene can touch it, and it is not in
`grabbable`, so the spring can never take hold of it.

From there `update(dt)` hands `target` to the solver, every step.

### What "the mouse" actually means: a poll, not an event

There are no mouse events in this demo. PyChrono cannot hand Python an
`IEventReceiver` (see the root README), so a click is RECONSTRUCTED, every
physics step, out of three things the operating system will answer at any
moment. On macOS:

```
console.mouse_down()     CGEventSourceButtonState(HIDSystemState, 0)
                             the hardware button, globally.  Not "a click on
                             our window" -- there is no such question to ask.
console.cursor_pixel()   NSEvent.mouseLocation()            screen coords
                           - window_rect()                  where our window is
                           - (win height - content height)  the title bar
                         = a pixel inside the 3D view, or None if outside it
```

`window_rect()` is cached for 60 steps rather than asked every time, because a
window can be dragged but rarely is. X11 and Win32 answer the same three
questions differently; `camera.py` then does identical arithmetic on all of
them, which is why that file has no platform in it.

Everything else follows from having a POLLED signal instead of events:

```
1. PRESS EDGE      down and not console.prev_mouse -- last step's value is the
                   only way to tell a new press from a held one.

2. PIXEL -> RAY    console.ray_through(px, py) -> (eye, eye + dir*60).
                   Camera basis from eye/target/world-up, the pixel turned
                   into click_x, click_y in -1..+1, scaled by half_w, half_h
                   from getFOV() and getAspectRatio() -- both SWIG-wrapped, so
                   this needs no OS at all.

3. IS IT THE BALL? RayHit(from, to, marker's collision model, res) -- the
                   form that asks about ONE model, even behind others, because
                   the ball sits inside the hand and the nearest hit is the
                   hand.  The marker's collision sphere is GIZMO_PICK (0.07 m,
                   twice the ball you see -- a 3.5 cm ball three metres off is
                   about 20 pixels) with a 1 mm envelope, in GIZMO_FAMILY,
                   which every other body refuses so the arm passes through
                   it.  A miss returns False and the same press falls through
                   to the grab, which uses the NEAREST form (pick_along_ray).

4. FREEZE A PLANE  A ray is a line; the scene wants a point.  On the press,
                   DragPlane.set() fixes a plane through the current target
                   with the camera's forward as its normal -- screen-parallel,
                   the best-conditioned choice, because the cursor ray meets it
                   head on.  It is frozen at press time, which is why dragging
                   tracks the cursor ACROSS the view and does not change depth.
                   See the figure below.

5. EVERY STEP      new pixel -> new ray -> plane.hit() -> set_target(point).
   WHILE HELD      Once a frame, draw() puts the ball at the new target.

6. RELEASE         down goes False -> dragging = False.  While dragging,
                   mouse_update() writes console.prev_mouse itself, so the
                   gizmo owns the button's edge and the grab code never sees
                   it until the button comes back up.
```

A cursor gives two numbers and the scene wants three. The plane is the third,
and the angle between it and the ray is the whole of "why can I not drag this
toward me":


The right panel is the whole of it: every point along the cursor's ray sits
under the same pixel, so something has to choose which one you meant, and the
plane is what chooses. Because it is frozen at the press, the mouse slides the
point along the plane and never along the ray.

The plane's normal is `camera_forward()` -- the VIEW AXIS, eye to target --
and NOT the cursor's own ray, which is why the two are drawn at an angle to
each other. Every ray in the view is then within half a field of view of
square to the plane, which is the best-conditioned choice available.
`DragPlane.set()`'s docstring has why the obvious alternative, the normal of
the surface you clicked, degenerates. Redraw the figure with
`python3 make_figures.py`.

### What the solver adds, in four lines

`ikpy` reads the same URDF Chrono was built from and answers "what seven angles
put the tool THERE, pointing THAT WAY". What this adds is the four lines that
keep it honest:

```
ask  = target + trim      trim integrates the error measured on the real hand
q*   = ikpy(ask, tool z along world -Z, seeded where the arm already is)
q   += clamp(q* - q, QD_MAX dt)                       a rate, not a teleport
holder.target = q         -> into the joint PD's setpoint, never into SetPos
```

`chronohil/ik.py` is the whole of it, including why the trim is there (26 mm of
gravity droop the kinematics cannot see), why the seed is where it is (a solver
free to pick elbow-up or elbow-down will swap them mid-drag), and what happens
when ikpy cannot win -- which it does not announce, so every answer it gives is
checked by forward kinematics before it is used.

## `go2`: pulling on it

**On the 3D view**, where in-window input is available: press to grab, drag to
pull, release to drop. A drag moves the leg ACROSS the view and not toward you
-- the plane is frozen at the press, exactly as it is for the arm's target --
so **RIGHT-DRAG** to orbit and press again if you want a different depth.
**MIDDLE-DRAG** pans, **W/S** zoom, **R/F** tilt, **C** recentres. **ESC**
quits.

There is no depth control inside a drag, on purpose. The arm's target drag
has none either, and one gesture across both demos is worth more than the
extra freedom in one of them.

Not every build can read its own 3D window (see `chronohil/input/window/`). On
one that cannot, a small input window opens instead and the demo prints only
the keys that window actually has: **arrows** move the handle, **[ ]** move it
up and down, **Z** grabs whatever **X** has selected, **T** logs a pose,
**C** resets.

- **`go2`**: drag a leg and the policy picks a foot up and steps to keep its
  balance. That is a policy, not a PD: a stance controller can only stiffen.
- **The orange beads** from the held body to the handle are the spring. Count
  them: that is how hard you are pulling.

## Things that are easy to get wrong

- **A ray sees COLLISION geometry, never visual geometry.** A link you can see
  but whose collision is off is invisible to every click. That accounted for
  every "it will not pick" bug here.
- **The spring's gains are a choice, and one fixed stiffness is a bad one.** A
  stiffness that feels right on a 134 kg ball is a catapult on a 0.154 kg shin.
  This demo builds them from the body's mass as `k = m*w^2` and `c = 2*m*w`,
  with a floor on the mass -- `chronohil/config.py` has the measurements behind
  both. It is one reasonable rule, not the rule.
- **A Chrono motor angle is not the URDF's joint angle.** It runs the other
  way: `GetMotorAngle()` grows as the child link turns NEGATIVELY about the
  joint frame's own Z. Every number crossing between a kinematics library and
  the running simulation crosses that sign, and getting it backwards gives an
  arm that folds itself up rather than one that is subtly wrong. `MOTOR_SIGN`
  in `chronohil/ik.py` has both measurements.
- **A URDF branches, and a kinematics library will pick a branch for you.**
  The Panda's hand has three children; left to itself `ikpy` follows the first,
  which is a FINGER, and the tool point lands 58.40 mm past where the arm
  really holds it. That reads exactly like a modelling error and is not one.
- **The joints Chrono ends up with are not the joints the URDF has.** The
  parser folds `panda_link8` away -- a massless frame carrying the hand 107 mm
  out from link7 -- so the live arm attaches the hand straight to `panda_link7`.
  Read the joint names off the running system, hand them to a URDF parser, and
  those 107 mm vanish without a word. `PANDA_IK_JOINTS` in `reaching.py` is
  spelled out for both of these reasons, and `ReachIK` refuses to be built if
  the two models disagree about the tool point by as much as a millimetre.
- **A vector Chrono hands you may be a view into something that is about to
  die.** `GetFrame2Abs().GetPos()` stored for later read (0, 0, 0.333) when it
  was taken and (0, 0.0019, 0.672) a second of simulation afterwards. Copy it.
- **A visual shape cannot be restretched or recoloured after it is bound.** On
  build 1187 the geometry is baked into the Irrlicht scene node, and
  `SetMutable(True)` does not change that. The stretching spring line in this
  demo was a fixed 1.0 m stick in every frame this repo ever rendered.
  `chronohil.picking.BeadLine` is what replaced it, and it says why.

## The files here

| | |
|---|---|
| `main.py` | the simulation loop, the camera, and the console readout |
| `dragging.py` | `Manipulator`: what the person has hold of, by mouse or by key |
| `reaching.py` | `Reach`: the commanded point, the markers, and the workspace fence |

Everything else comes from `chronohil/` -- the picking, the spring, the
kinematics, the controllers and the scenes. There is no platform-specific code
in any of these three files.
