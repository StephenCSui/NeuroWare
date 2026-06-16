# Environment Component Log

---

## Session 2 — NeuroWare (2026-06-16)

### Current state
**Working — all zones, items, and main loop functional**

### What was done
- Built `simulation/` directory with: `config.py`, `zone.py`, `item.py`, `main.py`
- `config.py`: single source of truth for all constants — window size, agent/item/captain parameters, zone rects, colors, item type definitions
- `zone.py`: `Zone` class with `contains(x,y)`, `center()`, `draw()` — `make_zones()` returns dict with `loading`, `storing`, `goal`
- `item.py`: `Item` class with full lifecycle: `is_available()`, `update_position()` (averages carrier positions), `release()`, `draw()` — heavy items drawn larger with double border
- `main.py`: Pygame loop, event handling, update order, draw order

### Zones
| Zone | Rect (x, y, w, h) | Purpose |
|---|---|---|
| Loading | (30, 40, 180, 160) | Items placed here by user |
| Storing | (990, 250, 180, 250) | Agents deliver items here |
| Goal | (30, 550, 180, 160) | Final delivery destination |
| Captain A zone | (0, 0, 600, 750) | Left half — Captain A's awareness zone |
| Captain B zone | (600, 0, 600, 750) | Right half — Captain B's awareness zone |

### Item types
| Key | Label | Required agents | Color |
|---|---|---|---|
| 0 | A | 1 | Red |
| 1 | B | 1 | Yellow |
| 2 | C | 1 | Green |
| 3 | A2 | 2 | Dark red |
| 4 | B2 | 2 | Dark yellow |
| 5 | C2 | 2 | Dark green |

### Controls
- Left click in loading zone → `pending_placement` state
- Press 1-6 → place item of that type at clicked position
- G → `goal_mode` → press 1-3 → dispatch goal request to captain for that item type
- Space → pause/unpause
- Q / ESC → quit

### Update order (per frame)
1. `agent.sense_neighbours(agents)` — rebuild comm graph
2. `agent.update(dt, zones, captains)` — gossip + task execution
3. `item.update_position()` — move items to average of carrier positions
4. `captain.update(dt, items)` — scan zone, process syncs, advance ping animation

### What worked
- Item placement UX: two-step (click to register position, number key to select type) — avoids accidental placement
- `update_position()` only moves item when `carriers` is non-empty — prevents teleport to (0,0)
- Draw order: zones → comm links → co-carry links → captain sync link → items → agents → captains → HUD; items and agents never occlude HUD

### What did not work
- Previous issue (now fixed): items teleported to agents immediately on ping — carrier was set before agent physically arrived. Fixed: `update_position()` only activates when `carriers` is set at pickup, not on commit.

### Key decisions
- `COMM_RANGE = 250` — increased from 160 during gossip testing to make propagation observable. Should be tuned back down once gossip is confirmed working end-to-end.
- Window: 1200×750, FPS: 30, `dt = 1/FPS` (fixed timestep, not wall-clock delta)
- Grid drawn at 30px spacing for spatial reference

### Dependencies
- All other components depend on `config.py` — change constants there only
- `main.py` owns the simulation loop and object lifetimes — items list, agents list, captains list all live here

### [WORTH EXPLORING]
- `dt` is fixed at `1/FPS` rather than measured wall-clock time — if the simulation lags, time will appear to slow down rather than skip. Consider measuring real elapsed time if frame drops become an issue.
- No collision between agents — they pass through each other. Acceptable for now but worth noting for physical realism later.
- Item IDs (`Item._next_id`) are a class-level counter and never reset within a session — fine for now, but would need resetting if items are ever cleared mid-session.
- No persistence — all state is lost on quit. If replaying scenarios becomes useful, a simple JSON dump of item placements would suffice.

---

## Session 4 — NeuroWare (2026-06-16)

### Current state
**Redesigned — push model environment, new item types, physics loop**

### What was done

**`config.py` rewritten:**
- Item types reduced from 6 to 3: `S` (agents_per_side=1), `M` (agents_per_side=2), `L` (agents_per_side=3)
- Sizes updated: S=20px, M=40px, L=58px (M and L made larger for visual clarity)
- Push physics constants added: `PUSH_FORCE=1.5`, `ITEM_DAMPING=0.78`, `PUSH_SLOT_OFFSET=24`, `SLOT_SPREAD=15`, `PUSH_SPEED_MAX=3.5`
- `DEST_THRESHOLD=30` — item considered arrived when centre is within this distance of destination
- `VISION_RANGE=200` — radius for `sense_items()` (360°, separate from `COMM_RANGE`)
- `AGENT_SEP_DIST`, `AGENT_SEP_FORCE` kept in config but separation logic removed from agent at user request

**`item.py` rewritten:**
- `claimed_slots = {"LEFT": [], "RIGHT": [], "TOP": [], "BOTTOM": []}` — per-side agent lists
- `needed_sides()`: computes required push sides from current direction to destination (>12px deadband); LEFT if dx>0, TOP if dy>0, etc.
- `slot_position(side, idx)`: computes world position for each agent slot on a given side; multi-agent sides spread by `SLOT_SPREAD`
- `apply_push(side, force)`: accumulates push force for this frame
- `physics_update()`: integrates accumulated force → velocity → position; caps at `PUSH_SPEED_MAX`; damps when no push; clamps to window bounds; resets accumulator
- `resolve_collision(other)`: AABB push-apart between any two items
- `is_available()`: False if delivered, no destination, stored but not `to_goal`, or no open slots on needed sides

**`main.py` updated:**
- Update order: sense_neighbours + sense_items → agent.update → item.physics_update (not stored/delivered) → item-item collision → captains
- `item.physics_update()` guarded: only runs when `not item.stored and not item.delivered`
- Keys 1/2/3 only (4/5/6 removed)
- `draw_push_links()` replaces `draw_co_carry_link()`

### Item types (current)
| Key | Label | Agents/side | Size | Color |
|---|---|---|---|---|
| 1 | S | 1 | 20px | Red |
| 2 | M | 2 | 40px | Yellow |
| 3 | L | 3 | 58px | Green |

### Zones (unchanged)
| Zone | Rect (x, y, w, h) | Purpose |
|---|---|---|
| Loading | (30, 40, 180, 160) | Items placed here by user |
| Storing | (990, 250, 180, 250) | Agents push items here first |
| Goal | (30, 550, 180, 160) | Final delivery destination |

### Update order (per frame)
1. `agent.sense_neighbours(agents)` — rebuild comm graph
2. `agent.sense_items(items)` — rebuild visible items list (VISION_RANGE)
3. `agent.update(dt, zones, captains, items)` — gossip + evaluate + apply push forces
4. `item.physics_update()` — integrate forces, move items (skipped if stored or delivered)
5. `item.resolve_collision(other)` — pairwise AABB push-apart for all non-delivered items
6. `captain.update(dt, items)` — scan zone, process syncs, advance ping animation

### Controls (current)
- Left click in loading zone → pending_placement
- 1/2/3 → place S/M/L item
- G → goal mode → 1/2/3 → dispatch goal for S/M/L
- Space → pause/unpause
- Q / ESC → quit

### [WORTH EXPLORING]
- Agent-to-agent collision removed at user request — they pass through each other. Fine now; add back if realistic crowd flow is needed.
- `PUSH_FORCE`, `ITEM_DAMPING`, `PUSH_SPEED_MAX` are untuned — values chosen by feel. Tuning pass needed once push behaviour is stable.
- `DEST_THRESHOLD=30` is relatively large — item is considered arrived when still 30px from destination centre. May want to tighten this once push forces are tuned.
- No item component log exists yet — `item.py` was completely rewritten and warrants one.
