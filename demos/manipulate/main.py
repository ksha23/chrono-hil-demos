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
"""Demo 3: interact with a running simulation via mouse and keyboard.

    python demos/manipulate/main.py go2   drag a leg; a locomotion policy answers
    python demos/manipulate/main.py arm   steer a Franka's hand to a point you move
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

import pychrono.irrlicht as irr

from chronohil import (HANDLE_SPEED, RENDER_FPS, STEP, chrono,
                       require_window, scene_arm, scene_go2)
from chronohil.input import LocalInput, open_window_input
from chronohil.watchdog import Watchdog
from demos.manipulate.dragging import Manipulator
from demos.manipulate.reaching import Reach
import chronohil.scenes as scenes

SCENES = {
    "go2": scene_go2,
    "arm": scene_arm,                                   # commanded at the tip
}

# Which scenes are COMMANDED rather than pulled on. In these the keys move a
# point the hand is asked to reach, and the mouse either drags that point or
# shoves a link; everywhere else the keys move the grab handle directly.
COMMANDED = {"arm"}


def main(mode, headless_script=None, console=None):
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetSleepingAllowed(False)     # a resting body would sleep through the spring
    # Default solver under-resolves motor constraints + contacts (robot slides).
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.GetSolver().AsIterative().SetMaxIterations(200)

    grabbable, chase, hint, anchor = SCENES[mode](system)
    ap = anchor.GetPos()
    cam_t = [ap.x, ap.y, ap.z]
    # Spherical camera: azimuth, elevation, radius.
    cam_az, cam_r, cam_el = [0.55], [1.924], [0.429]
    EL_MAX = math.pi / 2 - 0.05     # short of straight up, where azimuth dies
    cam_drag = [None]        # cursor pixel at the last orbit-drag sample
    cam_pand = [None]        # ...and at the last pan-drag sample
    cam_pan = [0.0, 0.0, 0.0]   # where you have shoved the look point to
    drag = Manipulator(system, grabbable)
    reach = Reach(system) if mode in COMMANDED else None
    dog = Watchdog(system, STEP, drag.grabber)

    title = f"Demo 3: {mode} - interacting with a mouse"
    vis = irr.ChVisualSystemIrrlicht()
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # the world is Z-up
    vis.SetWindowTitle(title)
    vis.SetWindowSize(1280, 800)
    # Initialize before AttachSystem: on a headless machine, attaching first
    # segfaults inside Initialize (null video driver loads the ground texture).
    vis.Initialize()
    require_window(vis, how="run tests_reach.py or tests_parts.py arm")
    vis.AttachSystem(system)
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(chase * 1.6, -chase * 2.0, chase * 1.1),
                  chrono.ChVector3d(0, 0, 0.3))
    # RTSCamera grabs the mouse for orbit; we manage the camera ourselves.
    vis.GetActiveCamera().setInputReceiverEnabled(False)

    if console is None and headless_script is None:
        console = open_window_input(vis, title, 1280, 800) or LocalInput()
    sel = 0             # which of `grabbable` the X key has selected
    lift = 0.0          # ] / [ toggle the handle moving up / down in Z
    system.DoStepDynamics(STEP)          # the collision system must exist to raycast

    print(f"\n{hint}")
    # Print only the controls this console actually has.
    if reach is not None and hasattr(console, "ray_through"):
        print("  arrows move the GREEN TARGET in X and Y   [ ] raise / lower it\n"
              "  the hand follows it; the green line is how far behind it is\n"
              "  MOUSE: drag the target to move it, drag a LINK to shove the arm\n"
              "  C puts the target back on the hand   T logs a pose\n"
              "  camera: RIGHT-DRAG orbit   MIDDLE-DRAG pan   W/S zoom   R/F tilt   C recentre\n")
    elif reach is not None and console is not None:
        print("  arrows move the GREEN TARGET in X and Y   [ ] raise / lower it\n"
              "  the hand follows it; the green line is how far behind it is\n"
              "  C puts the target back on the hand   T logs a pose\n"
              "  no mouse on this build, so the keys are the whole control\n")
    elif hasattr(console, "ray_through"):
        print("  MOUSE on the 3D view: press to grab, drag to pull, release to drop\n"
              "  arrows move   [ ] up/down   Z grab   X select   T log   C reset\n"
              "  camera: RIGHT-DRAG orbit   MIDDLE-DRAG pan   W/S zoom   R/F tilt   C recentre\n")
    elif console is not None:
        print("  arrows move   [ ] up/down   Z grab   X select   T log   C reset\n"
              "  no mouse picking on this build, so Z grabs whatever X has selected\n")

    rt_timer = chrono.ChRealtimeStepTimer()
    render_every = max(1, int(round(1.0 / (RENDER_FPS * STEP))))
    fired = set()
    n = 0
    t0 = time.perf_counter()
    break_out = False
    while vis.Run() and not break_out:
        t = system.GetChTime()
        if headless_script is not None:
            if t > headless_script["until"]:
                break
            s, th, br = headless_script["inputs"](t)
            cmds = [c for (at, c) in headless_script["commands"] if at <= t and (at, c) not in fired]
            fired.update((at, c) for (at, c) in headless_script["commands"] if at <= t)
        else:
            s, th, br = console.poll()
            cmds = console.take_commands()

        for c in cmds:
            if c == "n":
                sel = (sel + 1) % len(grabbable)
                print(f"[select] {grabbable[sel].GetName()}")
            elif c == "f":
                if reach is not None:
                    print("[grab] the keys drive the target in this mode; "
                          "shove a link with the mouse instead")
                elif drag.held:
                    drag.release(); print("[release]")
                else:
                    body = grabbable[sel]
                    drag.toggle(body)
                    print(f"[grab] {body.GetName()}")
            elif c == "m":
                b = grabbable[sel]
                p, q = b.GetPos(), b.GetRot()
                yaw = math.degrees(math.atan2(2*(q.e0*q.e3 + q.e1*q.e2),
                                              1 - 2*(q.e2*q.e2 + q.e3*q.e3)))
                line = f"[pose] {b.GetName()}  x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f} yaw={yaw:+.1f}deg"
                print(line)
                log = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   f"poses_{mode}.log")
                with open(log, "a") as fh:
                    fh.write(f"{t:8.3f}  {line}\n")
                print(f"        -> {log}")
            elif c == "u":
                lift = +1.0 if lift <= 0.0 else 0.0
                print(f"[lift] {'up' if lift > 0 else 'off'}")
            elif c == "d":
                lift = -1.0 if lift >= 0.0 else 0.0
                print(f"[lift] {'down' if lift < 0 else 'off'}")
            elif c == "q":
                print("[quit]")
                break_out = True
            elif c == "r":
                if drag.held:
                    drag.release()
                if reach is not None:
                    reach.recenter()
                cam_pan[0] = cam_pan[1] = cam_pan[2] = 0.0
                print("[reset] camera recentred")

        # Gizmo gets first refusal; miss -> grab shoves the arm.
        if reach is None or not reach.mouse_update(console):
            drag.mouse_update(console)

        if reach is not None:
            reach.command(s, th, br, lift, STEP)
        else:
            # Move the handle in screen-relative axes, not world axes.
            fwd_x, fwd_y = -math.sin(cam_az[0]), math.cos(cam_az[0])
            rt_x, rt_y = math.cos(cam_az[0]), math.sin(cam_az[0])
            f = (th - br) * HANDLE_SPEED * STEP      # up/down arrows
            r = s * HANDLE_SPEED * STEP              # left/right arrows
            dx = fwd_x * f + rt_x * r
            dy = fwd_y * f + rt_y * r
            dz = lift * HANDLE_SPEED * STEP          # [ ] -- up is always up
            if drag.held and not (console is not None and hasattr(console, 'ray_through')
                                  and console.prev_mouse):
                drag.move(dx, dy, dz)

        if n % render_every == 0:
            a = anchor.GetPos()
            cam_t[0] += (a.x - cam_t[0]) * 0.08
            cam_t[1] += (a.y - cam_t[1]) * 0.08
            cam_t[2] += (a.z - cam_t[2]) * 0.08
            look = chrono.ChVector3d(cam_t[0] + cam_pan[0],
                                     cam_t[1] + cam_pan[1],
                                     cam_t[2] + cam_pan[2])
            if console is not None and hasattr(console, "camera_nudge"):
                orb, zoom, rise = console.camera_nudge()
                cam_az[0] += orb * 1.4 * render_every * STEP
                cam_r[0] = max(0.25, cam_r[0] * (1.0 + zoom * 1.2 * render_every * STEP))
                cam_el[0] = max(-EL_MAX, min(EL_MAX,
                                cam_el[0] + rise * 1.0 * render_every * STEP))
            # Right-drag orbits, middle-drag pans, W/S zooms.
            if console is not None and hasattr(console, "mouse_right_down"):
                rpix = console.cursor_pixel()
                if console.mouse_right_down() and rpix is not None:
                    if cam_drag[0] is not None:
                        cam_az[0] -= (rpix[0] - cam_drag[0][0]) * 0.006
                        cam_el[0] = max(-EL_MAX, min(EL_MAX, cam_el[0]
                                        + (rpix[1] - cam_drag[0][1]) * 0.006))
                    cam_drag[0] = rpix
                else:
                    cam_drag[0] = None
            if console is not None and hasattr(console, "mouse_middle_down"):
                ppix = console.cursor_pixel()
                if console.mouse_middle_down() and ppix is not None:
                    if cam_pand[0] is not None:
                        mdx = (ppix[0] - cam_pand[0][0]) * 0.0016 * chase * cam_r[0]
                        mdy = (ppix[1] - cam_pand[0][1]) * 0.0016 * chase * cam_r[0]
                        cam_pan[0] -= math.cos(cam_az[0]) * mdx
                        cam_pan[1] -= math.sin(cam_az[0]) * mdx
                        cam_pan[2] += mdy
                    cam_pand[0] = ppix
                else:
                    cam_pand[0] = None
            R = chase * cam_r[0]
            ce, se = math.cos(cam_el[0]), math.sin(cam_el[0])
            vis.UpdateCamera(chrono.ChVector3d(look.x + R * ce * math.sin(cam_az[0]),
                                               look.y - R * ce * math.cos(cam_az[0]),
                                               look.z + R * se), look)
            drag.draw_link()
            if reach is not None:
                reach.draw()
            vis.BeginScene(); vis.Render(); vis.EndScene()
            if console is not None:
                b = grabbable[sel]
                bp = b.GetPos()
                f = drag.force()
                if isinstance(console, LocalInput) and reach is not None:
                    console.draw([f"mode   {mode}      t {t:6.2f} s"] + reach.status() + [
                        f"shove  {'link %s at %.0f N' % (drag.body.GetName(), f) if drag.held else 'none'}",
                        f"in     steer {s:+.2f}  thr {th:.2f}  brk {br:.2f}  lift {lift:+.0f}",
                        "arrows move the target   [ ] up/down   C recentre   T log",
                    ])
                elif isinstance(console, LocalInput):
                    console.draw([
                        f"mode   {mode}      t {t:6.2f} s",
                        f"sel    {b.GetName()}",
                        f"state  {'HELD  spring %.0f N' % f if drag.held else 'not grabbed - press Z'}",
                        f"pos    x {bp.x:+7.3f}  y {bp.y:+7.3f}  z {bp.z:+7.3f}",
                        f"in     steer {s:+.2f}  thr {th:.2f}  brk {br:.2f}  lift {lift:+.0f}",
                        "arrows move   [ ] up/down   Z grab   X select   T log   C reset",
                    ])
                elif n % (render_every * 10) == 0 and reach is not None:
                    tp = reach.target
                    console.send(f"{t:.3f},{tp.x:.3f},{tp.y:.3f},{tp.z:.3f},"
                                 f"{reach.ik.err * 1000:.1f}mm,{reach.ik.tilt:.1f}deg")
                elif n % (render_every * 10) == 0:
                    console.send(f"{t:.3f},{bp.x:.3f},{f:.3f},{bp.z:.3f},{s:.3f},{th:.3f},{br:.3f},"
                                 f"{'HELD' if drag.held else b.GetName()[:8]}")
        if reach is not None:
            reach.update(STEP)
        dog.sample(drag.held)
        for h in getattr(system, "stance_holders", ()):
            h.update()
        system.DoStepDynamics(STEP)
        n += 1
        if headless_script is None:
            rt_timer.Spin(STEP)      # hold the loop to wall-clock speed

    if headless_script and headless_script.get("shot"):
        vis.BeginScene(); vis.Render(); vis.EndScene()
        vis.WriteImageToFile(headless_script["shot"])
    return system, grabbable


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    mode = args[0] if args else "arm"
    if "-h" in sys.argv or "--help" in sys.argv:
        print(__doc__ or "")
        print(f"usage: python demos/manipulate/main.py "
              f"[{' | '.join(SCENES)}]\n"
              "\n"
              "  go2       a Unitree Go2 held up by a trained locomotion policy.\n"
              "            Drag a leg and it steps to keep its feet.\n"
              "  arm       a Franka Panda you steer by its HAND. The arrows move\n"
              "            a green target, inverse kinematics works out the seven\n"
              "            joint angles, and the joint PDs chase them. Drag a link\n"
              "            instead and you are a disturbance it has to reject.\n"
              "\n"
              "In the 3D window: in `arm`, drag the green target to move it and\n"
              "any link to shove the arm; in `go2`, drag a leg. A drag moves\n"
              "things ACROSS the view; orbit with RIGHT-DRAG to reach a\n"
              "different depth. MIDDLE-DRAG pans, W/S zooms, C recentres,\n"
              "ESC quits.")
        raise SystemExit(0)
    if mode not in SCENES:
        raise SystemExit(
            f"usage: python demos/manipulate/main.py "
            f"[{' | '.join(SCENES)}]\n"
            "  an input window opens beside the 3D view when this build cannot\n"
            "  read the 3D window itself; either way it is one process")
    if mode == "go2":
        import os
        urdf = os.environ.get("GO2_URDF", scenes.GO2_URDF)
        if not os.path.exists(urdf):
            raise SystemExit(f"Go2 URDF not found at {urdf}\n"
                             "  see ASSETS.md; it should be vendored in go2_assets/")
        scenes.GO2_URDF = urdf
    main(mode)
