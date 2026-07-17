import sys
import pygame

from config import (WINDOW_WIDTH, WINDOW_HEIGHT, FPS, COLORS, NUM_AGENTS, CELL_SIZE,
                    USE_RL_POLICY, POLICY_WEIGHTS_PATH)
from grid  import Grid
from agent import Agent


def draw_gridlines(surface):
    for x in range(0, WINDOW_WIDTH, CELL_SIZE):
        pygame.draw.line(surface, COLORS["grid"], (x, 0), (x, WINDOW_HEIGHT))
    for y in range(0, WINDOW_HEIGHT, CELL_SIZE):
        pygame.draw.line(surface, COLORS["grid"], (0, y), (WINDOW_WIDTH, y))


def draw_hud(surface, font, grid, paused):
    stats  = grid.stats()
    status = stats["status_counts"]
    lines = [
        "AUTONOMOUS FARM SWARM",
        "",
        f"day {stats['day']}/30   tick {stats['tick']}   score {stats['score']:.0f}",
        f"unmonitored: {stats['unknown_cells']}   avg moisture {stats['avg_moisture']:.1f}   avg weeds {stats['avg_weed']:.1f}",
        f"needy   -> water: {stats['needy']['water']:>3}  weed: {stats['needy']['weed']:>3}  monitor: {stats['needy']['monitor']:>3}  harvest: {stats['needy']['harvest']:>3}",
        f"status  -> growing: {status['growing']:>3}  ready: {status['ready']:>3}  harvested: {status['harvested']:>3}  dead: {status['dead']:>3}  spoiled: {status['spoiled']:>3}",
        "",
        "SPACE: pause   Q/ESC: quit   L-click: spike weeds   R-click: spike drought",
    ]
    y = 10
    for line in lines:
        s = font.render(line, True, COLORS["text"])
        surface.blit(s, (10, y))
        y += 16

    if stats["season_over"]:
        p = font.render(f"-- SEASON OVER  final score {stats['score']:.0f} --", True, (255, 220, 80))
        surface.blit(p, (WINDOW_WIDTH // 2 - p.get_width() // 2, WINDOW_HEIGHT - 24))
    elif paused:
        p = font.render("-- PAUSED --", True, (255, 80, 80))
        surface.blit(p, (WINDOW_WIDTH // 2 - p.get_width() // 2, WINDOW_HEIGHT - 24))


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("NeuroWare Autonomous Farm Swarm")
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("monospace", 13)

    policy = None
    if USE_RL_POLICY:
        from rl_policy import TaskPolicy
        policy = TaskPolicy.load(POLICY_WEIGHTS_PATH)

    grid   = Grid()
    agents = [Agent(i, cell, policy=policy) for i, cell in enumerate(grid.spawn_points(NUM_AGENTS))]

    paused = False
    dt     = 1.0 / FPS

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused

            elif event.type == pygame.MOUSEBUTTONDOWN:
                cell = grid.cell_at_px(*event.pos)
                if cell and cell.cell_type == "plant":
                    if event.button == 1:
                        cell.weed_density = 100.0
                    elif event.button == 3:
                        cell.moisture = 5.0

        if not paused:
            grid.tick(dt)
            if not grid.season_over:
                for agent in agents:
                    agent.update(dt, grid)

        # ---- Draw ----
        screen.fill(COLORS["background"])
        grid.draw(screen)
        draw_gridlines(screen)

        for agent in agents:
            agent.draw_task_link(screen)
        for agent in agents:
            agent.draw(screen, font)

        draw_hud(screen, font, grid, paused)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
