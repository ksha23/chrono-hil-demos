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
"""Input from a small window of our own. Portable: pygame, nothing else.

This is the fallback every platform can run. It cannot read the 3D window --
see chronohil.input.window for why that is hard and what fixes it -- so it
offers the same actions through keys instead.
"""

class LocalInput:
    """A small pygame window in this same process, so one command is enough.

    PyChrono still cannot read the Irrlicht window (SWIG directors are off, and
    getCursorControl() returns an unwrapped object), so the keys have to be read
    by something else -- but that something does not have to be another process.
    pygame and Irrlicht coexist happily in a single process.

    Same interface as the 3D-window backends, so the loop cannot tell them
    apart.
    """

    KEYS = None          # filled in on first use, so pygame is imported lazily

    def __init__(self):
        import pygame
        self.pg = pygame
        pygame.init()
        self.screen = pygame.display.set_mode((460, 190))
        pygame.display.set_caption("chrono-hil input (keep this window focused)")
        self.font = pygame.font.SysFont("menlo,dejavusansmono,monospace", 15)
        LocalInput.KEYS = {
            pygame.K_RIGHTBRACKET: "u", pygame.K_LEFTBRACKET: "d",
            pygame.K_z: "f", pygame.K_x: "n", pygame.K_c: "r", pygame.K_t: "m",
        }
        self.commands = []
        self.last = (0.0, 0.0, 0.0)
        self.status = ""
        print("[input] local window open - keep IT focused, not the 3D view")

    def poll(self):
        pg = self.pg
        for ev in pg.event.get():
            if ev.type == pg.QUIT:
                raise SystemExit
            if ev.type == pg.KEYDOWN:
                if ev.key == pg.K_ESCAPE:
                    raise SystemExit
                if ev.key in LocalInput.KEYS:
                    self.commands.append(LocalInput.KEYS[ev.key])
        k = pg.key.get_pressed()
        steer = (-1.0 if k[pg.K_LEFT] else 0.0) + (1.0 if k[pg.K_RIGHT] else 0.0)
        thr = 1.0 if k[pg.K_UP] else 0.0
        brk = 1.0 if k[pg.K_DOWN] else 0.0
        self.last = (steer, thr, brk)
        return self.last

    def take_commands(self):
        c, self.commands = self.commands, []
        return c

    def send(self, text):
        self.status = text

    def draw(self, lines):
        pg = self.pg
        self.screen.fill((18, 18, 22))
        for i, line in enumerate(lines):
            self.screen.blit(self.font.render(line, True, (220, 220, 200)), (12, 10 + i * 22))
        pg.display.flip()
