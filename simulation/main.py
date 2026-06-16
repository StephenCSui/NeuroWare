import sys
import random
import pygame

from config import (WINDOW_WIDTH, WINDOW_HEIGHT, FPS, COLORS, NUM_AGENTS,
                    CAPTAIN_A_POS, CAPTAIN_B_POS,
                    CAPTAIN_A_ZONE, CAPTAIN_B_ZONE, ITEM_TYPES)
from zone    import make_zones
from item    import Item
from captain import Captain
from agent   import Agent

GRID_SIZE = 30


def make_agents():
    agents = []
    for i in range(NUM_AGENTS):
        x = random.randint(200, WINDOW_WIDTH  - 200)
        y = random.randint(100, WINDOW_HEIGHT - 100)
        agents.append(Agent(i, x, y))
    return agents


def draw_grid(surface):
    for x in range(0, WINDOW_WIDTH, GRID_SIZE):
        pygame.draw.line(surface, COLORS["grid"], (x, 0), (x, WINDOW_HEIGHT))
    for y in range(0, WINDOW_HEIGHT, GRID_SIZE):
        pygame.draw.line(surface, COLORS["grid"], (0, y), (WINDOW_WIDTH, y))


def draw_hud(surface, font, selected_type, paused, log, pending_placement, goal_mode):
    if pending_placement:
        status = "Click registered — press 1/2/3 to place item type"
    elif goal_mode:
        status = "Goal request — press 1/2/3 for item type to deliver"
    else:
        status = "Click LOADING zone to place | 1: S (1/side)  2: M (2/side)  3: L (3/side) | G: goal"

    lines = ["WAREBOT SWARM SIM", "", status, "SPACE: pause    Q/ESC: quit"]
    y = 10
    for line in lines:
        s = font.render(line, True, COLORS["text"])
        surface.blit(s, (WINDOW_WIDTH // 2 - 340, y))
        y += 17

    # Item type swatches
    for i, tdata in ITEM_TYPES.items():
        sx = WINDOW_WIDTH // 2 + 260 + i * 38
        sy = 10
        sz = tdata["size"] - 4
        pygame.draw.rect(surface, tdata["color"], (sx, sy, sz, sz), border_radius=3)
        if i == selected_type:
            pygame.draw.rect(surface, (255, 255, 255), (sx, sy, sz, sz), 2, border_radius=3)
        r = font.render(f"x{tdata['agents_per_side']}", True, (255, 255, 200))
        surface.blit(r, (sx + 2, sy + sz + 2))

    if paused:
        p = font.render("-- PAUSED --", True, (255, 80, 80))
        surface.blit(p, (WINDOW_WIDTH // 2 - p.get_width() // 2, WINDOW_HEIGHT - 28))

    log_y = WINDOW_HEIGHT - 14 * len(log) - 6
    for entry in log:
        ls = font.render(entry, True, COLORS["text_dim"])
        surface.blit(ls, (10, log_y))
        log_y += 14


def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("NeuroWare Swarm Simulation")
    clock  = pygame.time.Clock()
    font   = pygame.font.SysFont("monospace", 13)

    zones    = make_zones()
    cap_a    = Captain(0, *CAPTAIN_A_POS, CAPTAIN_A_ZONE, "zone_a")
    cap_b    = Captain(1, *CAPTAIN_B_POS, CAPTAIN_B_ZONE, "zone_b")
    cap_a.link(cap_b)
    cap_b.link(cap_a)
    captains = [cap_a, cap_b]

    agents   = make_agents()
    items    = []
    log      = []
    MAX_LOG  = 8

    selected_type     = 0
    goal_mode         = False
    pending_placement = None
    paused            = False
    dt                = 1.0 / FPS

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

                elif event.key in (pygame.K_1, pygame.K_2, pygame.K_3):
                    key_map = {pygame.K_1: 0, pygame.K_2: 1, pygame.K_3: 2}
                    chosen  = key_map[event.key]

                    if goal_mode:
                        goal_center = zones["goal"].center()
                        if not cap_a.receive_goal_request(chosen, goal_center):
                            log.append(f"No stored {ITEM_TYPES[chosen]['label']} found")
                        else:
                            log.append(f"Goal request: {ITEM_TYPES[chosen]['label']} → GOAL zone")
                        goal_mode = False

                    elif pending_placement is not None:
                        px, py   = pending_placement
                        new_item = Item(chosen, px, py)
                        items.append(new_item)
                        aps = ITEM_TYPES[chosen]["agents_per_side"]
                        log.append(
                            f"Placed {new_item.label} ({aps} agent(s)/side) in LOADING")
                        selected_type     = chosen
                        pending_placement = None
                    else:
                        selected_type = chosen

                elif event.key == pygame.K_g:
                    goal_mode         = True
                    pending_placement = None
                    log.append("Goal request — press 1/2/3 for item type")

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                if zones["loading"].contains(mx, my):
                    pending_placement = (mx, my)
                    log.append("Click registered — press 1/2/3 to choose item type")
                else:
                    pending_placement = None

        if len(log) > MAX_LOG:
            log = log[-MAX_LOG:]

        if not paused:
            # 1. Sensing
            for agent in agents:
                agent.sense_neighbours(agents)
                agent.sense_items(items)

            # 2. Agent logic (gossip, evaluate, apply push forces to items)
            for agent in agents:
                agent.update(dt, zones, captains, items)

            # 3. Item physics (integrate forces, move items)
            for item in items:
                if not item.stored and not item.delivered:
                    item.physics_update()

            # 4. Item-item collision resolution
            active = [it for it in items if not it.delivered]
            for i in range(len(active)):
                for j in range(i + 1, len(active)):
                    active[i].resolve_collision(active[j])

            # 5. Captain logic
            for cap in captains:
                cap.update(dt, items)

        # ---- Draw ----
        screen.fill(COLORS["background"])
        draw_grid(screen)

        cap_a.draw_zone(screen)
        cap_b.draw_zone(screen)

        for zone in zones.values():
            zone.draw(screen, font)

        for agent in agents:
            agent.draw_comm_links(screen)

        for agent in agents:
            agent.draw_push_links(screen)

        cap_a.draw_captain_link(screen, cap_b)

        for item in items:
            if not item.delivered:
                item.draw(screen, font)

        for agent in agents:
            agent.draw(screen, font)

        cap_a.draw(screen, font)
        cap_b.draw(screen, font)

        draw_hud(screen, font, selected_type, paused, log, pending_placement, goal_mode)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
