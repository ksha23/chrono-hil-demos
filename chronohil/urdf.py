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
"""Making a stock robot URDF survive Chrono's parser."""

import os
import re


def make_chrono_safe_urdf(path):
    """Two things in a stock Go2 URDF that Chrono cannot take.

    1. Links with no <inertial>.  The Go2 has eight, collision-only cylinders
       bolted to the calves, and the parser SEGFAULTS on them rather than
       complaining.  They are GIVEN a tiny inertial rather than deleted.

       Deleting them was the obvious fix and it is wrong. Those eight are the
       `calflower` links: the lower shin, and the foot hangs off the end of
       them. Drop the link and the joint that references it and the foot
       re-attaches higher up, so the robot is standing on a leg shorter than the
       one any Go2 policy was ever trained on -- which is invisible until you
       put a trained policy on it and it crouches instead of standing. A gram
       and a 1e-6 inertia keeps the chain, keeps the geometry, and is small
       enough not to matter dynamically.
    2. Collada visuals.  Chrono reads meshes with tiny_obj, so a .dae is fed to
       a Wavefront parser, fails, and then segfaults.  The visual meshes are
       dropped and the collision primitives are drawn instead -- this URDF
       describes itself in 5 boxes, 17 cylinders and 5 spheres, which is blocky
       but complete, and it is the physics we are here to push on anyway.
    """
    src = open(path).read()
    out = src
    TINY = ('<inertial><origin xyz="0 0 0" rpy="0 0 0"/>'
            '<mass value="0.001"/>'
            '<inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>'
            '</inertial>')
    drop = []
    for block in re.findall(r"<link\b.*?</link>", src, re.S):
        name = re.search(r'name="([^"]+)"', block).group(1)
        if "<inertial" not in block:
            drop.append(name)
            fixed = block.replace("</link>", TINY + "</link>")
            out = out.replace(block, fixed)
    # Collada visuals: Chrono reads every mesh with tiny_obj, so a .dae reaches a
    # Wavefront parser and segfaults. If an .obj of the same name has been
    # converted next door, point at that; otherwise drop the visual and fall back
    # to drawing the collision primitives.
    base_dir = os.path.dirname(os.path.abspath(path))
    swapped = dropped_meshes = 0
    for vis_block in re.findall(r"<visual>.*?</visual>", out, re.S):
        fn = re.search(r'filename="([^"]+)"', vis_block)
        if not fn or fn.group(1).lower().endswith(".obj"):
            continue
        stem = os.path.splitext(os.path.basename(fn.group(1)))[0]
        obj_rel = f"../obj/{stem}.obj"
        if os.path.exists(os.path.normpath(os.path.join(base_dir, obj_rel))):
            out = out.replace(vis_block, vis_block.replace(fn.group(1), obj_rel))
            swapped += 1
        else:
            out = out.replace(vis_block, "")
            dropped_meshes += 1
    safe = os.path.join(os.path.dirname(path), "_chrono_safe.urdf")
    open(safe, "w").write(out)
    if drop:
        print(f"[urdf] gave {len(drop)} inertia-less links a token inertial "
              f"so the chain survives ({drop[0]}, ...)")
    if swapped:
        print(f"[urdf] using {swapped} converted .obj visual meshes")
    if dropped_meshes:
        print(f"[urdf] dropped {dropped_meshes} meshes with no .obj beside them; "
              f"drawing collision shapes for those")
    return safe
