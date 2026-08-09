import pygame

from config import CANVAS_HEIGHT, CANVAS_WIDTH, COLORS, FPS, GRID_CELL, ROBOT_RADIUS
from sim import Simulation

OBJECT_STATE_COLOR = {
    "open":        "object_open",
    "approaching": "object_approaching",
    "anchored":    "object_anchored",
}
ROBOT_STATE_COLOR = {
    "idle":     "robot_idle",
    "moving":   "robot_moving",
    "waiting":  "robot_waiting",
    "anchored": "robot_anchored",
}


CELL_COLOR = {
    "wall":    "wall",
    "dropoff": "dropoff",
    "staging": "staging",
    "walkway": "walkway",
}


def draw_warehouse(screen, warehouse):
    for col in range(warehouse.cols):
        for row in range(warehouse.rows):
            cell = warehouse.cells[col][row]
            if cell.cell_type == "shelf":
                key = "shelf_blocked" if cell.blocked else "shelf_open"
            else:
                key = CELL_COLOR[cell.cell_type]
            rect = pygame.Rect(col * GRID_CELL, row * GRID_CELL, GRID_CELL, GRID_CELL)
            pygame.draw.rect(screen, COLORS[key], rect)


def draw(screen, font, sim):
    screen.fill(COLORS["background"])
    draw_warehouse(screen, sim.warehouse)

    for obj in sim.objects:
        x0, y0, x1, y1 = obj.bounds()
        rect = pygame.Rect(x0, y0, x1 - x0, y1 - y0)
        color = COLORS[OBJECT_STATE_COLOR[obj.state]]
        pygame.draw.rect(screen, color, rect, width=0, border_radius=4)
        pygame.draw.rect(screen, (0, 0, 0), rect, width=2, border_radius=4)

        if obj.state in ("open", "approaching"):
            for slot_idx, (sx, sy) in enumerate(obj.slots):
                filled = obj.slot_occupants[slot_idx] is not None
                slot_color = COLORS["slot_filled"] if filled else COLORS["slot_open"]
                pygame.draw.circle(screen, slot_color, (int(sx), int(sy)), 5, width=0 if filled else 1)

    for robot in sim.robots:
        color = COLORS[ROBOT_STATE_COLOR[robot.state]]
        pygame.draw.circle(screen, color, (int(robot.x), int(robot.y)), ROBOT_RADIUS)

    hud_lines = [
        f"tick {sim.tick_count}   objects {len(sim.objects)}   delivered {len(sim.completed_records)}",
        "robots: " + " ".join(
            f"{state}={sum(1 for r in sim.robots if r.state == state)}"
            for state in ("idle", "moving", "waiting", "anchored")
        ),
    ]
    for i, line in enumerate(hud_lines):
        surf = font.render(line, True, COLORS["hud_text"])
        screen.blit(surf, (10, 10 + i * 18))

    pygame.display.flip()


def main():
    pygame.init()
    screen = pygame.display.set_mode((CANVAS_WIDTH, CANVAS_HEIGHT))
    pygame.display.set_caption("NeuroWare Transport -- logic prototype")
    font = pygame.font.SysFont("monospace", 14)
    clock = pygame.time.Clock()

    sim = Simulation()
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        sim.tick(dt)
        draw(screen, font, sim)

    pygame.quit()


if __name__ == "__main__":
    main()
