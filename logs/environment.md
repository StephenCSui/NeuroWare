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

---

## Session 5 — NeuroWare (2026-07-17)

### Current state
**Working — `item.py`/`zone.py` deleted, new grid-based environment (`grid.py`) replaces them entirely, later redesigned again mid-session into a template-based layout with a real impassable barrier and weighted pathfinding. Verified via 20,000-tick headless run (re-run after each change) and a live GUI session, no errors.**

Note: this file's scope now covers `config.py`, `grid.py`, `main.py` — `item.py` and `zone.py` no longer exist. (Agent-to-agent collision, flagged above as removed at user request in the old push model, is back — see "Key decisions" below; it returned in a different form than before.)

### What was done
- `item.py` and `zone.py` deleted — the push-item-between-zones model is fully superseded by grid cells.
- New `grid.py`:
  - `Cell` — per-cell state: `cell_type` (`"plant"` | `"walkway"`), `occupied_by` (physical presence, every cell type), true state `moisture`/`weed_density` (plant cells only), known state `known_moisture`/`known_weed_density` (`None` until first observed), `last_monitored`, claim fields (`claimed_by`/`claim_task`/`claim_tick`, plant cells only)
  - `Grid._build_layout()` — generates bed rows of `PLANT_ROWS_PER_BED` plant-cell rows separated by `WALKWAY_WIDTH`-cell-wide walkway strips (minimum 2, so agents have room to pass each other under the new hard-occupancy movement model)
  - `Grid`: `neighbors()`, `plant_cells()`/`walkway_cells()`/`all_cells()`, `spawn_points()` (prefers walkway cells for agent spawn so agents don't start out blocking plant-cell work), `tick()`, `stats()` (HUD data), `draw()`
- Plant-cell true state evolves **stochastically, not linearly**: moisture decay is jittered per tick (`random.uniform(0.5, 1.5) * MOISTURE_DECAY_BASE`), weed growth is probabilistic sprout events (`WEED_SPROUT_CHANCE` per second, jumps `weed_density` by a random amount when triggered) rather than smooth continuous creep — deliberately unpredictable, at explicit user request ("allows agents to adapt more").
- `config.py` rewritten: removed all `CAPTAIN_*`, `ITEM_*`, push-physics, zone-rect constants, and `COMM_RANGE`/`VISION_RANGE` (global gossip means no local sensing radius is needed — see `agent.md`). Added grid/task/decay constants. `AGENT_SPEED` deliberately lowered (2.5 → 1.2) and `TASK_DURATIONS` lengthened, both at explicit user request to keep pacing slow/legible and leave room for a future AI layer to show measurable improvement.
- `main.py` rewritten: update loop simplified to `grid.tick(dt)` + per-agent `update(dt, grid)` — no separate sensing/gossip/item-physics/collision/captain stages (occupancy reservation inside agent movement now prevents agent-agent overlap by construction, so no separate resolution pass is needed). All click-to-place/keyboard-dispatch UI removed; replaced with a debug perturb handler (left-click sets a plant cell's `weed_density` to 100, right-click drops `moisture` to 5) that mutates **true** state only — agents still have to physically monitor the cell to find out, same as any other true-state change.

**Later in the same session, layout rebuilt again from a user-supplied reference image (`farm grid template` PNG):**
- `_build_layout()` replaced entirely: instead of a repeating bed/walkway row pattern, the grid (now 16x12 = 192 cells, `CELL_SIZE` 30→45, `NUM_AGENTS` 6→5) is built from explicit rectangular regions stamped onto a walkway base fill — `PLANT_REGION` (4x8 bed), `DEADSPACE_REGION`, `WATER_REGION` — matching the template's proportions. First attempt mis-measured the template and let deadspace cut off the only route to the water zone; corrected after the user pointed back at the image — there's a pathway strip along the very top of the image connecting the two areas without crossing the black region, which the region coordinates now preserve (2 cells wide, per a later request, so it isn't a chokepoint).
- Two new cell types: `deadspace` and `water`, alongside `plant`/`walkway`. `water` cells are the destination for the new agent refill mechanic (see `agent.md`). `deadspace` was initially just visually distinct but still walkable; per explicit user correction ("set all black cells as occupied") it's now a genuine hard barrier — `Cell.blocked = (cell_type == "deadspace")`, checked in `is_occupiable()`.
- Making deadspace impassable required real pathfinding: `Grid.shortest_path(start, target, avoid_occupied=False)` — BFS initially, later upgraded to Dijkstra (see below) once cell traversal costs stopped being uniform. `avoid_occupied=True` is a second mode used by `agent.py`'s stuck-timeout fallback, treating currently-occupied cells as temporarily impassable too (for routing around live traffic, not just permanent obstacles).
- Added `CELL_TRAVERSAL_COST = {"plant": 4}` (default 1 for everything else) and switched `shortest_path` from unweighted BFS to weighted Dijkstra, per user observation that cross-farm transit was cutting straight through the plant bed — exactly where working agents park to perform tasks — instead of using the open pathway loop around it. Weighting only discourages *using the bed as a shortcut*; a claimed destination inside it is unaffected, since entering it is unavoidable when that's the actual goal. Confirmed via direct test: a trip between two pathway points on opposite sides of the bed now enters 0 plant cells, where it previously cut diagonally through.
- `Grid.water_cells()` added alongside `plant_cells()`/`walkway_cells()`.

### What worked
- 20,000-tick headless verification (scratch script, not committed to the repo): 0 occupancy violations, 0 walkway cells ever flagged needy or claimed, 0 premature water/weed candidacy before first observation, known-state staleness measurably diverging from true state over time
- Live GUI ran without errors on the real display for the duration of testing
- Re-verified after the template layout resize: all 32 plant cells observed within the test window (vs. 157/680 before), task-type balance healthy
- Re-verified after deadspace became a hard barrier + weighted pathing landed (post the deadlock fix documented in `agent.md`): 0 blocked-cell entries, 0 occupancy violations, full coverage maintained, task throughput back to baseline

### What did not work
- Not a bug this session's first pass — see DEVLOG for the task-type-imbalance tuning gap (a scale mismatch between 680 plant cells and 6 slow agents), resolved by the template-based layout resize.
- **Layout mis-measurement**: first reading of the reference image put the deadspace region across the very top row, cutting off the only connection to the water zone. The image actually shows a thin pathway strip along the top that deadspace doesn't cover — corrected after the user flagged "black space isn't being respected" and pointed back at the image a second time. Worth remembering: read reference images for connectivity, not just proportions — a region that looks like a solid block in a rough read can have a load-bearing gap at the edge.
- A pathfinding regression is logged in detail in `agent.md` (the BFS-caching deadlock) — it's an `agent.py`/movement bug, not a `grid.py` one, but `Grid.shortest_path`'s `avoid_occupied` mode is the fix that lives in this file.

### Key decisions
- Gossip is global for v1 (any agent can act on any cell's known state regardless of position) rather than range-limited. Captains, not comms range, were the single point of failure identified this session — range-limiting is listed as a deferred feature if global sharing proves "too easy" once observable.
- Grid state is the sole source of task information — no separate pheromone/signal abstraction was built. Cell state (known vs. true) already carries the urgency signal implicitly: the longer a cell goes unaddressed, the further known and true drift apart.
- Walkways are hard-required to be ≥2 cells wide specifically to give agents room to route around each other under the new hard-occupancy movement model. Agent-agent collision is enforced again (differently than the old push-model's removed soft separation) — via cell occupancy reservation in `agent.py`'s `step_toward`, not a positional push-apart force. Confirmed working: 0 occupancy violations across the verification run, including walkway traversal.
- Deadspace is a hard barrier, not just visually distinct inert ground — a deliberate correction from the original design, which had assumed "walkable but purposeless" was close enough. It wasn't; the user wanted a genuine obstacle.
- Cell traversal cost (plant > everything else) is a cheap proxy for "avoid likely-congested areas," recommended over building real trajectory prediction/broadcasting now — see DEVLOG next-steps for the full reasoning on why the bigger version is deferred to pair with a future AI/ML layer.

### Dependencies
- `agent.py` reads `Grid.plant_cells()`, `Grid.water_cells()`, `Grid.neighbors()`, `Grid.shortest_path()`, `Grid.tick_count`, and all `Cell` fields/methods (including `.blocked`) directly — no intermediary layer, no gossip-propagation step (see `agent.md`).
- `main.py` owns `Grid` and the `Agent` list lifetimes, same pattern as before.

### [WORTH EXPLORING]
- No dedicated `grid.md` log exists yet — `grid.py` is now the largest/most central component (Cell + Grid, true/known state model, stochastic dynamics, occupancy, pathfinding). Currently folded into this file; could be split out if it keeps growing, per `LOGGING_STANDARDS.md`'s "ask before creating a new log" rule — flagging as a suggestion, not doing it unprompted.
- `shortest_path` is called fresh (not cached across agents) whenever any agent starts a new journey — fine at 192 cells / 5 agents, but worth watching if the map or swarm size grows a lot, since it's now Dijkstra with a heap rather than plain BFS.

---

## Session 5 (continued) — NeuroWare (2026-07-18)

### Current state
Working — compact 10x8 layout, full plant lifecycle/day-cycle/harvest/scoring, season halts agent activity at day 30. Verified headless (~10 full-season runs) and live on the real display.

### What was done
- Layout compacted: `GRID_COLS/ROWS` 16x12→10x8, `NUM_AGENTS` 5→2, `PLANT_REGION` 4x8 (32 cells) → 3x2 (6 cells). `DEADSPACE_REGION` and the one-way lane constants (`LANE_COL_START/END`, `LANE_TO_WATER_ROW`/`LANE_TO_FARM_ROW`) removed entirely — both existed only to manage 5-agent bridge contention that doesn't apply at this scale. `CELL_TRAVERSAL_COST` weighting also removed from `shortest_path` (back to uniform cost) for the same reason — the open compact layout doesn't have a chokepoint worth routing around. New `HARVEST_BOX = (0, 0)` region, a single new `cell_type="harvest_box"` cell.
- `Cell` gained a `status` field (plant cells only): `"growing"` → `"ready"` (at `DAYS_TO_MATURE`=28) → `"harvested"`/`"dead"`/`"spoiled"` (terminal — `Cell.tick()` now no-ops for any terminal cell, no drift, no respawn ever). New `_is_active()` helper (`cell_type=="plant" and status in ("growing","ready")`) gates `needs_water`/`needs_weeding`/`needs_monitoring` — a terminal cell can no longer show up as needing anything. New `needs_harvest()` (`status == "ready"`).
- Death conditions, checked every `Grid.tick()` after the existing true-state drift: `moisture <= 0` (100% dehydration), `weed_density >= 100`, or `moisture <= 20 and weed_density >= 80` (both ≥80%) → `status = "dead"`. Expressed directly off existing `moisture`/`weed_density` fields, no new "dehydration" field needed.
- `Grid` gained `day_count` (`tick_count // DAY_LENGTH_TICKS`, computed every tick — reuses the existing tick clock, no separate time system), `season_over` (set once `day_count >= SEASON_LENGTH_DAYS`, also flips any still-`"ready"` cell to `"spoiled"` at that moment), and `score` (float, additive-only — `+WATER_REWARD`/`+WEED_REWARD`/`+HARVEST_REWARD` applied in `agent.py`'s `_complete_task`, no penalties, per the user's own reasoning that losing a dead cell's future harvest already is the cost).
- `Grid.tick()` now no-ops entirely once `season_over` — freezes the board at its day-30 state rather than continuing to drift.
- `main.py`: per-agent `update()` calls are now skipped once `grid.season_over` (agents stop moving/acting in place) while `grid.draw()`/HUD keep running every frame — the board and final score stay visible rather than the process just ending. HUD extended with day count, score, and per-status cell counts. Also now conditionally loads a trained `TaskPolicy` (`from rl_policy import TaskPolicy`) when `config.USE_RL_POLICY` is set, passing it into each `Agent(...)` — `None` otherwise, which is the exact original behavior.
- **Balance retune (see DEVLOG for how this was caught):** `MOISTURE_DECAY_BASE` 0.9 → 0.5. The original value was tuned for a context with no death consequence; at 0.9, nearly every cell died before reaching maturity once death became real. Still a hard baseline even after the retune — see DEVLOG's "what did not work" for the full reasoning on why this was left as-is rather than tuned further.
- **Real bug fix:** every `Cell.__init__` set `last_monitored = 0`, so every plant cell became monitor-eligible on the exact same tick (`STALENESS_THRESHOLD_TICKS` later) — a synchronized burst-then-silence rhythm. Fixed by initializing `last_monitored = -random.uniform(0, STALENESS_THRESHOLD_TICKS)` instead, so eligibility (and therefore all later re-eligibility, since it derives from actual visit times) spreads out naturally instead of firing in lockstep.
- New constants: `HARVEST_BOX`, `DAY_LENGTH_TICKS`, `DAYS_TO_MATURE`, `SEASON_LENGTH_DAYS`, `HARVEST_REWARD`/`WATER_REWARD`/`WEED_REWARD`, `USE_RL_POLICY`, `POLICY_WEIGHTS_PATH`. New `TASK_DURATIONS` entries: `"harvest"` (6.0s, matches weed/water labor time) and `"deliver_harvest"` (1.0s, matches monitor/obtain_water quick-action time).
- New colors: `harvest_box`, and a resolved-cell family (`harvested`/`dead`/`spoiled`, dim/gray tones) distinct from `unknown` (never-checked) so a finished cell visually reads as "done," not "needs a look."

### What worked
- Headless full-season verification (~10 runs, scratch script extended this session): 0 occupancy/claim violations every run, all 6 cells always resolved (harvested/dead/spoiled) by season end, season-halt correct.
- Regression check with `USE_RL_POLICY=False` (default): identical behavior to before this session's RL work — the compaction/lifecycle changes and the RL addition are cleanly separable.
- Live GUI play-tested twice — once for the compact baseline (confirmed the monitoring-burst bug directly, see below), once with the trained RL policy loaded.

### What did not work
- See `agent.md` for the two real bugs caught this session (`last_monitored=0` sync burst, fixed; agent-agent swap deadlock, left unfixed by design) — the burst bug lived in this file's code (`Cell.__init__`) even though it was diagnosed from user observation of agent behavior, not a `grid.py`-side symptom.
- The `MOISTURE_DECAY_BASE` retune only partially closed the death-rate gap — see DEVLOG for the full data and the decision to leave it as an intentional hard baseline rather than keep chasing a "typically survivable" tuning.

### Key decisions
- Dropping deadspace/one-way-lanes/traversal-cost weighting together, in one pass, rather than incrementally — all three existed for the same reason (5-agent bridge contention) and none of it applies at 2 agents/6 cells. Confirmed via the user's own read of the new layout before implementation (see approved plan).
- Terminal cell states freeze completely (`Cell.tick()` no-ops) rather than continuing to track true state in the background — simpler, and there's no mechanic that would ever read a terminal cell's continuing drift anyway.
- Season-end halts *agent activity*, not *rendering* — a deliberate distinction from a simpler "just stop the main loop" implementation, per explicit user requirement that the final board state stay visible and inspectable.

### Dependencies
- `agent.py` reads `Cell.status`, `.needs_harvest()`, `Grid.score`, `.day_count`, `.season_over`, `.harvest_box_cell()` (new lookup, mirrors `.water_cells()`) directly, same no-intermediary pattern as before.
- `rl_policy.py` (new, no dedicated log — see `agent.md`) reads `Grid.plant_cells()`, `Cell.known_moisture`/`.known_weed_density`/`.status`/`.last_monitored`/`.claimed_by`, `Cell.is_claimable()`/`.needs_*()`, and `Grid.tick_count` to build RL observations/action masks — all pre-existing fields/methods, no new `grid.py` surface needed for this.
- `main.py` now also imports `rl_policy.TaskPolicy` conditionally (only when `USE_RL_POLICY` is set) — the only new cross-file dependency this session.

### [WORTH EXPLORING]
- Still no dedicated `grid.md` — `grid.py` picked up a third major subsystem this session (lifecycle/day-cycle, on top of true/known state and pathfinding). Repeating the suggestion from the prior entry: worth splitting out if it keeps growing.
- The hard baseline (frequent total crop failure under random task selection) is a deliberate, user-confirmed choice for now — but worth revisiting once the RL policy (see `agent.md`) has had more training, since a genuinely smarter policy might make the current decay/threshold tuning feel too easy rather than too hard, flipping the concern.
