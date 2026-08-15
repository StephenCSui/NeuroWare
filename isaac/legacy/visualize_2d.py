"""Standalone live 2D visualizer for the two-robot transport demo.
Genuinely separate process -- does NOT import isaacsim, so it starts in
under a second and can't affect (or be affected by) the sim itself. Polls
a small state file the main script writes every few ticks
(two_robot_pickup_demo.py's export_vis_state()) and redraws.

Run with plain python3 (not isaacsim's python.sh) while the main script is
running live:
    python3 isaac/visualize_2d.py
"""
import os
import sys
import time

import numpy as np
import pygame

import occupancy_grid as og

STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_vis_state.npz")
CELL_PX = 10
WIDTH, HEIGHT = og.GRID_W * CELL_PX, og.GRID_H * CELL_PX

COLOR_UNKNOWN = (40, 40, 45)
COLOR_FREE = (25, 70, 30)
COLOR_OCCUPIED = (190, 40, 40)
COLOR_BG = (20, 20, 22)
COLOR_ROBOT = (80, 160, 255)
COLOR_ROBOT2 = (255, 160, 60)
COLOR_PLAN = (255, 240, 60)
COLOR_PLAN2 = (60, 240, 255)
COLOR_OBJ = (230, 230, 230)
COLOR_TEXT = (240, 240, 240)


def world_to_px(x, y):
    px = int((x - og.X_MIN) / og.CELL_SIZE * CELL_PX)
    py = int(HEIGHT - (y - og.Y_MIN) / og.CELL_SIZE * CELL_PX)  # flip so +Y is up on screen
    return px, py


def draw_grid(surf, cells):
    # cells: (GRID_H, GRID_W) int8 array, row 0 = Y_MIN
    color_lut = np.zeros((3, 3), dtype=np.uint8)
    color_lut[og.UNKNOWN] = COLOR_UNKNOWN
    color_lut[og.FREE] = COLOR_FREE
    color_lut[og.OCCUPIED] = COLOR_OCCUPIED
    rgb = color_lut[cells]  # (GRID_H, GRID_W, 3)
    rgb = np.flipud(rgb)  # flip so +Y is up on screen, matching world_to_px
    small = pygame.surfarray.make_surface(np.transpose(rgb, (1, 0, 2)))
    scaled = pygame.transform.scale(small, (WIDTH, HEIGHT))
    surf.blit(scaled, (0, 0))


def draw_robot(surf, font, x, y, yaw, color, label, carrying):
    px, py = world_to_px(x, y)
    pygame.draw.circle(surf, color, (px, py), 8)
    hx = px + int(18 * np.cos(yaw))
    hy = py - int(18 * np.sin(yaw))
    pygame.draw.line(surf, color, (px, py), (hx, hy), 3)
    text = f"{label}" + (f" [{carrying}]" if carrying else "")
    img = font.render(text, True, color)
    surf.blit(img, (px + 10, py - 10))


def draw_plan(surf, plan_xy, color):
    if plan_xy is None or len(plan_xy) < 2:
        return
    pts = [world_to_px(x, y) for x, y in plan_xy]
    pygame.draw.lines(surf, color, False, pts, 2)
    for p in pts:
        pygame.draw.circle(surf, color, p, 4)


def draw_objects(surf, font, names, xy):
    for name, (x, y) in zip(names, xy):
        px, py = world_to_px(x, y)
        pygame.draw.rect(surf, COLOR_OBJ, (px - 4, py - 4, 8, 8))
        img = font.render(str(name), True, COLOR_OBJ)
        surf.blit(img, (px + 6, py + 4))


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT + 40))
    pygame.display.set_caption("two-robot transport -- live 2D plan view")
    font = pygame.font.SysFont("monospace", 13)
    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill(COLOR_BG)
        try:
            data = np.load(STATE_PATH, allow_pickle=True)
            draw_grid(screen, data["grid"])
            draw_objects(screen, font, data["obj_names"], data["obj_xy"])
            rx, ry, ryaw = data["robot"]
            r2x, r2y, r2yaw = data["robot2"]
            draw_plan(screen, data["plan_robot"], COLOR_PLAN)
            draw_plan(screen, data["plan_robot2"], COLOR_PLAN2)
            draw_robot(screen, font, rx, ry, ryaw, COLOR_ROBOT, "Robot", str(data["carrying_robot"]))
            draw_robot(screen, font, r2x, r2y, r2yaw, COLOR_ROBOT2, "Robot2", str(data["carrying_robot2"]))
            status = "live"
        except FileNotFoundError:
            status = "waiting for state file (is the main script running?)"
        except Exception as e:
            status = f"waiting for a clean frame ({type(e).__name__})"

        img = font.render(f"status: {status}  |  yellow=Robot plan  cyan=Robot2 plan  red=occupied  green=free", True, COLOR_TEXT)
        screen.blit(img, (5, HEIGHT + 10))

        pygame.display.flip()
        clock.tick(15)

    pygame.quit()


if __name__ == "__main__":
    main()
