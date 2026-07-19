# Agent Component Log

---

## Session 2 — NeuroWare (2026-06-16)

### Current state
**Partial — core flow working, goal delivery broken**

### What was done
- Built `Agent` class from scratch in `simulation/agent.py`
- Implemented full task state machine: `pickup` → `store` (single) / `waiting_for_co` → `co_carrying` → `waiting_at_store` (heavy)
- Implemented goal delivery state machine: `deliver_pickup` → `deliver`
- Implemented random busy state (only triggers when task is `None` and inventory is `None`)
- Implemented gossip protocol replacing previous captain-polling model:
  - `absorb_captain_info(captains)` — learns tasks from captains within `COMM_RANGE`, unconditional
  - `gossip()` — copies own `known_tasks` into every neighbour's store, unconditional
  - `cleanup_known_tasks()` — prunes stored, delivered, or fully assigned tasks
  - `evaluate_known_tasks()` — self-selects from known tasks when free
- `update()` runs gossip block before busy/idle gate so info always propagates
- `sense_neighbours(all_agents)` — rebuilds neighbour list each frame based on `COMM_RANGE`
- `draw_comm_links`, `draw_co_carry_link`, `draw` — visual state rendering

### What worked
- Single-agent pickup and store flow: reliable
- Co-carrier coordination: first agent arrives and waits (`waiting_for_co`), second arrives and both transition to `co_carrying`, proximity check during carry, both must reach storing zone before drop
- Busy state correctly blocked from triggering during carry
- Gossip: structurally correct, smoke tested — agents absorb and re-propagate without errors
- Removal of `consider_captains` / `learn_from_neighbours` was clean — no orphaned references

### What did not work
- **Goal delivery broken** — agents are not reliably acting on goal entries in `known_tasks`. `evaluate_known_tasks` checks `item.stored and not item.delivered and not item.carriers`, but the condition may not be met correctly at the point agents evaluate (item may still have `carriers` set, or `stored` may not be set yet when goal is pinged). Exact failure mode not diagnosed.
- Previous issue (now fixed): agents polled captains directly — captain was effectively assigning agents rather than agents self-selecting. Gossip protocol replaces this.
- Previous issue (now fixed): second agent for heavy items not committing — `learn_from_neighbours` only checked neighbours' `going_for` state but didn't propagate item reference reliably.

### Key decisions
- Gossip runs unconditionally (busy or not) — an agent mid-carry is still a message carrier in the mesh
- `going_for` is set on commit and cleared on `_reset()` — used by neighbours to count how many agents are already headed to an item, preventing over-subscription
- `known_tasks` format: `item_id → {"item": Item, "goal": (x,y) or None}` — `goal: None` means pickup task, `goal: (x,y)` means delivery task
- Heavy item co-carry: the agent that arrives second switches both agents to `co_carrying` immediately — avoids a third transition tick

### Dependencies
- `config.py`: `COMM_RANGE`, `AGENT_SPEED`, `AGENT_SIZE`, `BUSY_CHANCE`, `BUSY_DURATION`, `CARRY_PROXIMITY`
- `item.py`: `Item` class — `is_available()`, `assigned_agents`, `carriers`, `stored`, `delivered`
- `captain.py`: `Captain.active_pings`, `Captain.goal_pings` — read during `absorb_captain_info`
- `zone.py`: zones dict passed to `update()` for target coordinates

### [WORTH EXPLORING]
- `going_for` counting only checks `self.neighbours` — agents outside comm range are invisible, so over-subscription is still possible with larger swarms; may need a cap on `item.assigned_agents` as the authoritative count
- Wander behaviour is purely random; directed wandering (toward unexplored areas or toward captain zones) could improve gossip spread speed

---

## Session 3 — NeuroWare (2026-06-16)

### Current state
**Working — full pickup, store, goal delivery (light and heavy) functional**

### What was done

**Goal delivery bug fixes (three root causes):**
- `cleanup_known_tasks` was deleting goal tasks because `item.stored = True` triggered the prune condition — but stored is the *prerequisite* for a goal task. Fixed: goal tasks now only prune when `item.delivered`, or when another agent has fully claimed the task (`self.going_for != item_id and len(item.assigned_agents) >= item.required_agents`)
- `absorb_captain_info` skipped goal pings for items already in `known_tasks` as a pickup entry — the `if item_id not in self.known_tasks` guard blocked the upgrade. Fixed: allow overwriting when existing entry has `goal: None`
- Same skip bug existed in `gossip()` — goal upgrades never propagated to neighbours either. Fixed with same logic
- `_commit_to_goal` was setting `item.stored = False` immediately on commit, before the agent arrived at the item. This broke the second agent's ability to find and commit to the same heavy item. Fixed: `item.stored = False` now set at actual pickup in `_do_deliver_pickup`

**Heavy item goal delivery:**
- Added multi-agent goal delivery mirroring the heavy pickup flow
- `_do_deliver_pickup` now checks `item.required_agents`: single agent → carry directly; heavy → join carriers, wait or start co-deliver
- New states: `waiting_for_co_deliver` → `co_delivering` → `waiting_at_goal`
- New methods: `_do_wait_for_co_deliver`, `_do_co_deliver`, `_do_wait_at_goal`, `_deliver_and_reset`
- `_deliver_and_reset` mirrors `_drop_and_reset`: marks `item.delivered = True`, clears both carriers, resets both agents
- `update()`, `draw()`, `draw_co_carry_link` updated to handle new states
- `_do_deliver` (single agent) now also clears `item.assigned_agents` on completion

**Task prioritisation:**
- `evaluate_known_tasks` now sorts candidates before iterating
- Priority key: `(not in_progress, slots_left)` — in-progress heavy items (assigned > 0, required > 1) sort first, then by fewest slots remaining
- Effect: if a co-carrier is already waiting, nearby agents prioritise filling that slot over starting a new solo task

### What worked
- All three goal delivery bugs confirmed fixed via unit test
- Heavy goal delivery: exactly 2 agents commit when `required_agents=2`, others hold off
- Prioritisation: free agent chooses in-progress heavy item over fresh light item — confirmed by test

### What did not work
- N/A this session

### Key decisions
- `item.stored = False` moved from `_commit_to_goal` to `_do_deliver_pickup` at actual pickup moment — this is necessary so the second agent can still find the item as stored while walking to it
- For heavy delivery, `item.stored` stays `True` until both carriers have arrived (same moment `co_delivering` starts), to prevent premature cleanup of other agents' goal task entries
- Priority sort uses `not in_progress` so `False` (in-progress) sorts before `True` (fresh) — Python sort is stable so equal-priority tasks preserve insertion order

### Dependencies
- `item.py`: `Item.assigned_agents`, `Item.required_agents`, `Item.stored`, `Item.delivered`, `Item.carriers`
- `captain.py`: `Captain.goal_pings` — read during `absorb_captain_info` for goal task upgrades

### [WORTH EXPLORING]
- Cooperative task allocation (ROI problem): if a task needs 5 agents and only 1 is nearby, should that agent wait or take a lighter task? Current prioritisation always favours in-progress heavy tasks — may need a utility function (distance + expected wait time + task value) for cases with large required_agents counts. Market-based task allocation or auction mechanisms are the standard approach in swarm robotics literature.
- `going_for` counting only checks `self.neighbours` — agents outside comm range are invisible, so over-subscription is still possible with larger swarms; `item.assigned_agents` is the authoritative count and should be used as the hard cap

---

## Session 4 — NeuroWare (2026-06-16)

### Current state
**Redesigned — push model state machine implemented; end-of-session fixes applied but not yet play-tested**

### What was done

**Complete rewrite for push model:**
- Replaced carry state machine (`pickup`, `co_carrying`, `deliver`, etc.) with push state machine:
  - `seek_push_slot` — navigate to claimed slot on item's push side
  - `waiting_for_co_pusher` — hold position at slot, wait for all other slot-holders on this side
  - `pushing` — apply push force each frame, break off when axis complete or item arrives
- `push_slot` replaces `going_for`/`inventory`: `{"item": Item, "side": str, "idx": int}` — stores which slot this agent owns
- `known_tasks` redesigned: `{item_id: item}` — item shared by **reference**; destination/phase/claimed_slots all live on the item itself, so gossip is automatically current without re-sending
- `sense_items(items)` added: agents directly absorb item references within `VISION_RANGE` (360°) each frame
- `absorb_captain_info` simplified: captains only ping existence and destination; agents read `claimed_slots` directly from item ref
- `cleanup_known_tasks` updated: prunes delivered items, items stored in `to_store` phase, and items fully claimed by others (except if this agent holds a slot on that item)
- `evaluate_known_tasks`: Pass 1 sorts needed sides by claimed count descending (fill the most-populated side first — enables per-side start); Pass 2 claims a slot and enters `seek_push_slot`
- `_release_and_reset` / `_reset`: release slot on item, clear `push_slot`, return to idle

**Push execution methods:**
- `_do_seek_push_slot`: navigate to slot position at perpendicular-side-aware speed; enter `waiting_for_co_pusher` on arrival (≤2px)
- `_do_waiting_for_co_pusher`: hold at slot; if displaced >2px (item moved), re-enter `seek_push_slot`; call `_side_ready` each frame
- `_side_ready(item, side)`: checks slot count ≥ `agents_per_side`, all other holders in correct task state, AND all other holders ≤2px from their individual slot position
- `_transition_side_to_pushing(item, side)`: sets this agent and all co-pushers on same side to `pushing`; clears `item.stored` if phase is `to_goal`
- `_do_pushing(dt, zones, captains)`: arrival check first (finalize if within `DEST_THRESHOLD`); per-axis push via `item.apply_push(side, PUSH_FORCE)` if `_should_push`; break-off if `side not in item.needed_sides()`
- `_should_push(item, side)`: per-axis overshoot guard — stops pushing if item has passed destination on that axis (5px deadband)
- `_finalize_delivery(item, zones, captains)`: sets `item.destination = None` immediately as race guard; zeros velocity; marks `stored` or `delivered`; releases slot

**Speed logic in `_do_seek_push_slot` (end-of-session fix):**
- Checks perpendicular sides via `item.claimed_slots` and `self.neighbours`
- No perp agent claimed → sprint (`AGENT_SPEED * 2.0`) — solo axis, go fast
- Perp agent nearby seeking perp side → yield (`AGENT_SPEED * 0.7`) — let them anchor
- Perp claimed but not nearby → normal (`AGENT_SPEED`)

**Separation removed**: agent-to-agent collision removed at user request — agents can overlap

### What worked
- Per-side independence: LEFT-side agents start pushing while TOP-side agents are still filling — confirmed
- Diagonal rule enforced structurally via slot system — agent on LEFT side can only push rightward
- Finalization race guard: second agent hitting `_finalize_delivery` finds `item.destination = None` and exits cleanly
- `evaluate_known_tasks` side priority sort correctly sends agents to the side that already has a co-pusher
- Break-off: agent whose side is no longer in `needed_sides()` releases and resets correctly

### What did not work
- **Item pushed off map** (early bug): `_do_pushing` applied force without checking arrival first; `physics_update` ran on stored/delivered items; no boundary clamping. Fixed: arrival check moved to top of `_do_pushing`; `physics_update` guarded in `main.py`; boundary clamp added in `item.physics_update`.
- **Vertical axis no break-off** (early bug): agent kept pushing after item aligned with destination horizontally. Fixed: `elif side not in item.needed_sides(): self._release_and_reset()` in `_do_pushing`.
- **Item moved before agents in position** (early bug): `_transition_side_to_pushing` was called from `_do_seek_push_slot` on arrival and remotely switched other `seeking` agents. Fixed: all agents park in `waiting_for_co_pusher`, only per-side `_transition_side_to_pushing` fires when the side's own slots are all ready.
- **A1 pushed L-type alone on vertical axis** (end-of-session bug): `_side_ready` checked only task state — A4/A5 were displaced from slot by a moving item but still marked `waiting_for_co_pusher`. When A1 arrived, `_side_ready` returned `True`. Fixed by end-of-session patch: `_side_ready` now checks `math.hypot(a.x - tx, a.y - ty) > 2` for each other slot-holder.
- **Arrive threshold confusion**: attempted variable `arrive_dist`; user confirmed "keep at 2px, ≤2 is fine too". Pinned to 2px default.

### Key decisions
- `known_tasks = {item_id: item}` — item by reference, not a copy. All agents sharing an item in `known_tasks` are reading the same live object. This is the gossip mechanism for push state.
- Per-side push start: do not wait for ALL sides to be filled before starting. Start each side as soon as it is ready. This is a performance feature — diagonal motion can begin while one axis still fills.
- Speed is driven by swarm observable state (who has claimed perp side, who is nearby), not by distance or physics — follows the "agents talking to agents" rule.
- Captain rule: captains only set `item.destination` and `item.phase`. ALL slot decisions, timing, and speed are agent-to-agent.

### Dependencies
- `config.py`: `AGENT_SPEED`, `AGENT_SIZE`, `VISION_RANGE`, `COMM_RANGE`, `PUSH_FORCE`, `PUSH_SLOT_OFFSET`, `SLOT_SPREAD`, `DEST_THRESHOLD`, `BREAKOFF_WAIT`
- `item.py`: `Item.claimed_slots`, `Item.needed_sides()`, `Item.slot_position()`, `Item.claim_slot()`, `Item.release_slot()`, `Item.apply_push()`, `Item.is_available()`
- `zone.py`: `zones` dict passed to `update()` for finalization zone detection

### [WORTH EXPLORING]
- Speed multipliers (2.0 / 1.0 / 0.7) are currently hardcoded constants — may want to expose in config for tuning
- `_side_ready` 2px threshold is very tight and assumes agents are nearly stationary when anchored — if item is moving fast when the last agent arrives, the 2px check may cause the side to never trigger; consider a slightly larger tolerance (4-6px) if this is observed
- `evaluate_known_tasks` currently picks the first open slot on the highest-priority item — no look-ahead for whether the agent is physically close to one item vs another; a distance-weighted priority could improve allocation efficiency
- Diagonal motion is always structurally possible once each axis has at least one agent — but the agents do not communicate "I am ready to push" explicitly. This could be a bottleneck for L-type (3/side): one axis might start at 1/3 capacity while other side is at 3/3. Consider a quorum threshold before per-side start fires.

---

## Session 5 — NeuroWare (2026-07-17)

### Current state
**Working — entire push-model state machine replaced with a flat farm-task model; verified via 20,000-tick headless run, no bugs surfaced.**

### What was done
- Replaced the whole push-model state machine (`seek_push_slot`/`waiting_for_co_pusher`/`pushing`, slot claiming, side-based quorum, perpendicular-speed heuristics) with a flat three-state machine: `idle → moving_to_task → performing_task → idle`
- New movement core `step_toward(grid, target_cell)`: agents move cell-to-cell, not freely through continuous space. Reserves the next cell (`cell.occupy(agent_id)`) the moment it commits to stepping there — not on arrival — so two agents can't both start converging on the same empty cell mid-step. If every neighbouring cell is occupied, waits in place and re-evaluates next frame — deliberately simple local avoidance, not real pathfinding
- New task selection `evaluate_and_claim`: scans all plant cells for `is_claimable` + at least one active need (monitor/water/weed, evaluated against **known** state only — see `grid.md`/environment entry for the known-vs-true split), picks one cell via `random.choice` (uniform, unweighted, no scoring formula), then picks one applicable task type on that cell if multiple needs apply (also `random.choice`). Re-checks `is_claimable` immediately before committing the claim as a race guard — relies on strictly sequential per-frame agent processing in `main.py`
- `_complete_task` applies the task's effect to **true** state (water raises true moisture, weed zeros true weed_density, monitor is a no-op on true state), then calls `cell.observe(tick)` — the one and only place known state updates. All three task types count as an observation, not just monitor
- Removed entirely: `sense_neighbours`, `sense_items`, `neighbours` field, `known_tasks`/neighbour-gossip mechanism, `absorb_captain_info`, `draw_comm_links`, and every push-model method (`_try_claim_slot`, `reconsider_task` + `BREAKOFF_WAIT`, `_side_ready`, `_transition_side_to_pushing`, `_should_push`, `_do_seek_push_slot`, `_do_waiting_for_co_pusher`, `_do_pushing`, `_finalize_delivery`, `_resolve_item_collisions`)
- New fields: `current_cell`, `_moving_to_cell` (reserved destination, mid-step), `_target_cell` (task cell or wander destination — same field serves both), `_task`/`_task_type`/`_task_timer`

**Later in the same session, after the farm layout was redesigned around a user-supplied template (see `environment.md`):**
- Added a water resource flag (`self.water`, bool, starts `True`). `_complete_task` sets it `False` after a completed `water` task and `True` after a completed `obtain_water` task. `update()` checks it first, before any other task consideration — an out-of-water agent is hard-gated into `_begin_refill()` (travel to the nearest water cell, timed like any other task) and won't do monitor/weed/water until refilled. Deliberately simpler than the fetch/deliver-delegation design discussed earlier — no cross-agent handoff, single agent does its own refill trip — at explicit user direction to defer the richer version to a future AI-paired implementation.
- `step_toward` rewritten from single-step greedy ("best unoccupied neighbour toward target") to a cached shortest-path follow: computes `grid.shortest_path()` once when a new target is set, stores the remaining route in `self._path`, and advances one cell at a time, only recomputing when the target changes. This became necessary once deadspace became a real barrier (see `environment.md`) — greedy single-step movement has no way to route around a large static obstacle, it can only react to what's immediately adjacent.
- Added `self._stuck_frames` and a reroute fallback: if the next cached-path cell stays blocked by another agent for more than `STUCK_REROUTE_FRAMES`, recompute the path with `avoid_occupied=True` (treats currently-occupied cells as temporary obstacles) instead of waiting indefinitely. See "What did not work" — this fixes a real deadlock the first pathfinding version introduced.
- `_release_and_reset` now also clears `_path`/`_path_target`/`_stuck_frames` so a stale cached route can never leak into a new task.

### What worked
- 20,000-tick headless verification (scratch script, not committed): zero instances of two agents sharing a `current_cell` or `_moving_to_cell` reservation — the hard occupancy rule held under sustained load
- Zero walkway cells ever claimed; zero water/weed candidacy on cells with `known_moisture is None`
- Selection confirmed genuinely non-deterministic: sampled claim ranks (by distance from the claiming agent, among all valid candidates) spanned 5 to 678 out of ~680 candidates, with 0% of claims picking the literal nearest — not secretly greedy despite reading as "obvious" behaviour on a screen
- Re-verified after the water flag + pathfinding changes (post deadlock-fix, see below): zero water-flag violations (never watered while already empty), zero blocked-cell entries, refill trips and completed waterings roughly matched (41 vs. 42 in one run) as expected
- Re-verified after adding `CELL_TRAVERSAL_COST`-weighted pathing (see `environment.md`): a direct test between two pathway points on opposite sides of the plant bed now routes with 0 plant cells entered, vs. cutting straight through before

### What did not work
- No bugs surfaced in the initial push-model-replacement pass; only a task-type-balance tuning gap given map size vs. agent count/speed (resolved later this session via the layout redesign, see DEVLOG).
- **Real deadlock, caught via verification, not observation:** the first version of the BFS-based `step_toward` rewrite computed one shortest path and cached it, with no fallback if another agent contested the exact next cell — it would just wait on that one specific cell forever. Once the water zone's narrow bridge created real contention (multiple agents needing to cross the same 2-cell-wide corridor once the water flag made refill trips mandatory), two agents could end up permanently waiting on each other, and task throughput collapsed from a baseline of ~180 claims to 15 in the same 20,000-tick window. This is exactly what the `[WORTH EXPLORING]` note below (written *before* this happened) predicted: "wait if all neighbours blocked has no deadlock detection." Fixed with the `_stuck_frames`/reroute mechanism described above.

### Key decisions
- Occupancy (`occupied_by`, physical presence, every cell type) and task claim (`claimed_by`, plant cells only) are deliberately separate fields — an agent working a plant cell holds both simultaneously, but they're independently reasoned about. Direct response to explicit user feedback: agents must never clip into each other, but an agent standing on a plant cell it's working is always fine.
- Task selection is deliberately unweighted/flat (no urgency×distance scoring) per explicit user direction — the point is to have the crudest possible working version now so a future AI/ML layer has a clean, isolated seam to replace `random.choice` with a learned scoring function later, without restructuring anything else. This was a walk-back from an earlier, more sophisticated urgency-weighted design that was cut before implementation.
- Known vs. true cell state: task decisions read only `known_moisture`/`known_weed_density`, stale by default until a physical visit. The swarm can act on wrong information — confirmed via the divergence check in verification.
- Pathfinding correctness (routing around a static barrier) and multi-agent contention (avoiding deadlock with other agents) were treated as two separate concerns needing two separate mechanisms — a weighted-cost static path, plus a dynamic stuck-timeout fallback — rather than trying to solve both with one cleverer static algorithm. The deadlock bug above is a direct lesson in why: a "smarter" single path is still just one path, and doesn't know about agents that pick their own paths independently and simultaneously.
- Water refill is a hard gate (blocks monitor/weed too, not just watering) per explicit user direction ("need to refill") — the more literal reading, flagged to the user as a judgment call in case a softer lazy-refill was actually intended.

### Dependencies
- `grid.py`: `Cell.is_claimable`, `.claim`, `.release`, `.observe`, `.needs_water`/`.needs_weeding`/`.needs_monitoring`, `.is_occupiable`/`.occupy`/`.vacate`, `.blocked`; `Grid.plant_cells()`, `.water_cells()`, `.neighbors()`, `.shortest_path()`, `.tick_count`
- `config.py`: `AGENT_SIZE`, `AGENT_SPEED`, `BUSY_CHANCE`, `BUSY_DURATION`, `COLORS`, `TASK_DURATIONS`, `WATER_REFILL_AMOUNT`, `STUCK_REROUTE_FRAMES`

### [WORTH EXPLORING]
- ~~`step_toward`'s "wait if all neighbours blocked" fallback has no deadlock detection~~ — this happened (see "What did not work" above) and was fixed with a stuck-timeout reroute. Only stress-tested at `NUM_AGENTS=5` on the current map; worth re-checking if agent count or map size changes again.
- The race-guard in `evaluate_and_claim` relies on strictly sequential per-frame agent processing in `main.py` — would break silently if that loop is ever parallelized; currently only a code comment, no test guards against it
- Trajectory broadcasting / cooperative path reservation (agents publish an intended route in advance so others plan around it, using estimated task+travel time as path cost) was discussed and explicitly deferred — see DEVLOG Session 5 next-steps for the reasoning. `shortest_path`'s weighted-cost approach is a plain stepping stone toward this, not a replacement for it.

---

## Session 5 (continued) — NeuroWare (2026-07-18)

### Current state
Working. `evaluate_and_claim` now has two paths (RL policy / flat random baseline, togglable), harvest carrying capacity is implemented, and a known unresolved agent-agent swap deadlock exists in `step_toward` (not touched this entry — out of scope, see below).

### What was done
- Added `carrying_harvest` (bool, mirrors the existing `water` flag exactly): set `True` on a completed `harvest` task, gates all other task consideration until delivered, set `False` on a completed `deliver_harvest` task. New `_begin_deliver(grid)` mirrors `_begin_refill` — travels to the single harvest-box cell, not a plant-cell claim. Priority order in `update()`: `carrying_harvest` → `not self.water` → busy-chance → `evaluate_and_claim` (carrying checked first — strictly less flexible than needing water, can't do anything else at all while both hands full).
- `_complete_task` now sets `self.last_reward` to the exact reward constant applied (0.0 for monitor/harvest-pickup, `WATER_REWARD`/`WEED_REWARD`/`HARVEST_REWARD` otherwise) — unconditional, harmless bookkeeping on the random-baseline path, required for RL training (see below).
- `evaluate_and_claim` split into two paths on a new `self.policy` field (`None` by default): if set, delegates cell/task-type selection to `self.policy.select(self, grid)` (returns `(cell, task_type)` or `None`); otherwise falls through to the original flat `random.choice` logic completely unchanged. The race-guard/claim/target-setting code after the split is shared by both paths.
- New RL bookkeeping fields: `self.last_state`, `self.last_action` (set by `TaskPolicy.select()` as a side effect — see `rl_policy.py`, no dedicated log file yet), `self.last_reward` (set by `_complete_task`). `train.py` reads all three to assemble `(state, action, reward, next_state)` training pairs.
- `Agent.__init__` takes a new `policy=None` param, passed through from `main.py`/`train.py`.

### What worked
- Regression check: with `policy=None` (default), re-ran the existing headless verification script unchanged — identical results to before this entry's changes, confirming the random-baseline path is fully untouched by the RL addition.
- Full lifecycle verified headless (~10 runs): 0 occupancy/claim violations, harvest pickup → carry → deliver → score-increment chain worked correctly every time it was exercised.
- RL integration verified via `train.py`: 2000-episode headless training run completed with no errors, trained-greedy policy beat the random baseline (52.4 vs. 41.2 avg over 50 eval seasons — see `DEVLOG.md` for the honesty caveat on sample size/noise).

### What did not work
- **Real bug, caught by the user watching the live GUI, not fixed here (out of scope for this entry):** two agents can deadlock when their final destinations are exactly each other's current cell (a swap) — `step_toward`'s stuck-timeout reroute (`avoid_occupied=True`) deliberately still allows a path to terminate at an occupied *target* cell (you normally need to walk onto a cell someone's about to vacate), which is exactly backwards in a pure swap: neither agent's target cell will ever vacate on its own. This is a genuine livelock, not just slow pathing. Explicitly left unfixed at user direction — same pathing/negotiation problem already flagged as deferred-to-AI-layer in the prior session entry; the user didn't want a heuristic tie-breaker patch standing in for real inter-agent communication. **[POTENTIAL FIX]** if a stopgap is ever wanted before real negotiation exists: a deterministic symmetry-breaker (e.g. lower `agent_id` holds position, higher `agent_id` reroutes) would stop the literal freeze without pretending to be communication.

### Key decisions
- RL scope deliberately narrowed to *task prioritization only* — `evaluate_and_claim`'s cell/task-type choice. Movement, pathfinding, and the swap-deadlock above are explicitly not touched by `self.policy` — a different, harder problem (real-time spatial coordination vs. discrete task choice) that the user chose not to fold into the same design.
- Reward attribution is precise per-decision (`agent.last_reward`, set the instant `_complete_task` knows the exact constant applied) rather than a diff against the global `grid.score` — with two agents potentially resolving tasks in the same window, a naive global-score diff would sometimes credit one agent's outcome to the other's concurrent decision.
- Implementation is a small neural net via scikit-learn's `MLPRegressor` (already installed) trained through hand-rolled Q-learning, not a torch/gymnasium/stable-baselines3 stack — a deliberate, user-confirmed choice given the tiny state/action space (38-dim observation, 24 discrete actions); a full deep-RL stack would be a large, slow install for a problem this size.
- The random-choice baseline was kept fully intact and toggled via `config.USE_RL_POLICY`, not replaced — the entire point of building `grid.score` earlier was to have something to measure the AI against.

### Dependencies
- `rl_policy.py` (new, no dedicated log yet): `build_observation`, `build_action_mask`, `TaskPolicy.select`/`.update`/`.save`/`.load` — imported by both `agent.py` (runtime) and `train.py` (training).
- `grid.py`: unchanged by this entry — `score`, `day_count`, `season_over`, `plant_cells()`, `Cell.status`/`.is_claimable`/`.needs_*` already exposed everything needed.
- `config.py`: `USE_RL_POLICY`, `POLICY_WEIGHTS_PATH` (new), plus everything from the prior entry.

### [WORTH EXPLORING]
- The agent-agent swap deadlock (see "What did not work") is still open — flagged again here so it isn't lost between entries.
- RL policy is a first pass: only 2000 training episodes, Manhattan-distance features instead of real path cost, a fairly small/noisy eval sample (50 episodes). The ~25% score edge over baseline is real but not heavily validated — more training, reward-shaping iteration, or richer features are all credible next steps if the gap needs to be more convincing.
- `rl_policy.py`/`train.py` don't have a dedicated component log — proposed (not created) in `DEVLOG.md`'s next-steps for this entry.
- **User request (2026-07-18), not yet built:** a reward log — a per-event record of what the RL policy actually experienced (agent, episode/tick, task type, reward value, positive or negative), not just the aggregate score numbers `train.py` currently prints. Today every reward is >=0 (additive-only scoring, no penalties — see the earlier "Key decisions" on why), so in practice this would only show positive entries right now, but should be built generically since that could change. Natural home: either `TaskPolicy.update()` in `rl_policy.py` appends `(s, a, r, ...)` tuples to a list/file as they're consumed, or `train.py`'s episode loop logs each `agent.last_reward` the moment it captures one (it already has the exact value in hand there). Would help debug reward shaping and verify no reward is being mis-attributed or dropped, beyond just trusting the final score.

## Session 6 — NeuroWare (2026-07-18)

### Current state
Working. The agent-agent swap deadlock (open since Session 5) is fixed and empirically verified (0/60 stalls, down from 40/60). `Agent.water` was converted from a bool to a `water_charges` counter. A real movement bug (`_move_toward` oscillation) was introduced and fixed within this same session, caught before it reached a play-test.

### What was done
- **Swap-deadlock fix in `step_toward`:** in the blocked branch, now checks immediately (not after `STUCK_REROUTE_FRAMES`) whether the block is a mutual swap — `next_cell is target_cell` (my destination itself is what's occupied) **and** the occupying agent's own `_target_cell` is my `current_cell`. New `_find_agent(grid, agent_id)` looks the blocking agent up via `grid.agents` (list reference set by `main.py`/`train.py` after spawning, added this session — see `environment.md`). If it's a mutual swap and `_should_yield(blocker)` says so, calls `_step_aside(grid)` instead of the old reroute-and-wait path (which is provably useless for this exact case — see `grid.py`'s `shortest_path`, unchanged, `avoid_occupied` always still permits ending at the target).
- **`_should_yield(other)`** — three-tier deterministic comparison, both agents evaluate independently off shared state and always reach opposite conclusions (no message-passing needed): (1) compare `_current_action_value()` (the trained policy's own Q-value for each agent's currently-committed action) — lower yields; (2) if no Q-values available (random baseline, or an exact tie) — compare `Cell.claim_tick` on each agent's target, later claim yields; (3) final tie-break on `agent_id`, only ever reached if everything else is identical.
- **`_current_action_value()`** reads `self.policy._predict(self.committed_state)[self.committed_action]` — a *new*, longer-lived pair of fields (`committed_state`/`committed_action`), set by `TaskPolicy.select()` (`rl_policy.py`) at the same moment as the existing `last_state`/`last_action`, but deliberately **not** cleared by `train.py`'s per-tick bookkeeping. Needed because `last_action` gets consumed/cleared mid-episode during training the instant a transition is captured — reusing it directly would silently go stale and break the Q-value comparison for any agent mid-training. Cleared to `None` in `_begin_refill`/`_begin_deliver` (personal water/harvest trips aren't policy decisions, no real Q-value exists for them) and in `_release_and_reset`/`_complete_task` (task finished/abandoned).
- **`_step_aside(grid)`** — the actual deadlock-breaking move: picks any free neighboring cell that isn't the contested target, occupies it, sets it as `_moving_to_cell`, and invalidates the cached `_path` (`None`) so normal `step_toward` re-plans toward the real target once clear. If no free neighbor exists (boxed in), falls through to the existing stuck-frame/reroute safety net unchanged — this case is untouched by the fix.
- **Water resource converted from bool to counter:** `self.water` → `self.water_charges`, initialized to `config.WATER_CAPACITY` (2, up from the implicit 1). `_complete_task`'s `"water"` branch now decrements (`-= 1`) instead of setting `False`; `"obtain_water"` resets to `WATER_CAPACITY` instead of `True`. `update()`'s idle-check changed from `not self.water` to `self.water_charges <= 0`. Done specifically so watering's travel-to-a-water-cell overhead doesn't structurally bias any policy (trained or random) toward neglecting it in favor of weeding, which has no such cost.
- **`_move_toward` bug + fix (caught this session, not carried over from before):** arrival threshold was a fixed `dist < 1.5` (pixels) — fine when `AGENT_SPEED` was small (1.2), but once `TIME_SCALE` scaled it up to 4.8 (see `environment.md`), a single step could overshoot a small remaining gap and bounce back past it forever, since 4.8 > 1.5 guarantees the post-step distance can also exceed 1.5. Result: single agent frozen at the exact same position for an entire season, 100% death rate across 40 test episodes. Fixed by making the threshold `dist <= speed` — always snap once the remaining distance is within one step, regardless of what `speed` actually is. Confirmed fixed via direct x/y-position tracing (was oscillating between two values exactly `AGENT_SPEED` apart every tick) and a full 40-episode rerun (score/harvest numbers matched pre-`TIME_SCALE` figures almost exactly).

### What worked
- Deadlock fix: 0/60 stalls after (from 40/60 before), confirmed via the same headless diagnostic used to root-cause it. Also verified with a real trained policy loaded (`USE_RL_POLICY=True`, greedy) — 0/40 stalls, no crashes from the new `committed_state`/`committed_action`/Q-value-comparison code path.
- Occupancy-invariant regression check (no cell double-occupied by two different agents) — 0 violations across 40 episodes post-fix, confirming `_step_aside`'s extra `occupy()`/`vacate()` calls didn't introduce a new violation class.
- Live play-test on the real display: agent visibly stepped aside instead of freezing when converging on an adjacent cell with the other agent.

### What did not work
- See "What was done" for the `_move_toward` oscillation bug — caught headlessly (100% death rate is an unmissable signal) before any play-test, root-caused via direct per-tick x/y tracing rather than guessing.
- N/A for the deadlock fix itself — no failed attempts, root-caused precisely before writing any code (see `DEVLOG.md` for the two-stage headless diagnostic that pinned it down to 100% literal mutual swaps).

### Key decisions
- Resolution mechanism is a real decision (Q-value comparison), not an arbitrary rule, per the user's standing preference for AI-driven coordination over hand-coded heuristics — but the *detection and guarantee-of-resolution* is deliberately kept fully deterministic infrastructure, not learned, since a not-yet-converged learned negotiation policy could still fail to resolve a deadlock, which is unacceptable given how much this bug was already shown to cost (see `DEVLOG.md` — it was silently dominating ~67% of episode outcomes with pure movement-layer luck).
- `committed_state`/`committed_action` deliberately kept separate from `last_state`/`last_action` rather than reusing them — a subtle but important distinction, see "What was done." Getting this wrong would have made the Q-value arbitration silently compare against stale, unrelated actions during training specifically (live play would have been unaffected, since nothing clears `last_action` there — this asymmetry is exactly why it's easy to miss).
- Water capacity set to 2, not something larger — chosen as the smallest change that meaningfully cuts refill-trip frequency without removing the resource constraint entirely (an unlimited/very high capacity would make `obtain_water` trips rare enough to stop mattering as a decision at all).

### Dependencies
- `rl_policy.py`: `_should_yield`/`_current_action_value` call `TaskPolicy._predict` directly — a new dependency in this direction (movement code reading from the RL layer) that didn't exist before this session, when `agent.py`→`rl_policy.py` calls were limited to `evaluate_and_claim`'s `policy.select(...)`.
- `grid.py`: reads `grid.agents` (list of live `Agent` objects, set externally by `main.py`/`train.py` — see `environment.md`), `Cell.claim_tick`, `Grid.neighbors()` — all read-only, no new `grid.py`-side changes required for this fix.
- `config.py`: new `WATER_CAPACITY` import.

### [WORTH EXPLORING]
- Known limitation, not solved here: the swap-deadlock detection is specifically *pairwise mutual* (A wants B's cell and vice versa) — a 3+-agent *cyclic* deadlock (A wants B's cell, B wants C's, C wants A's) would not be caught by this check. No evidence it's needed yet (nothing runs more than 2-3 agents), but worth a note for whenever the swarm actually scales up live, not just in training smoke tests.
- The reward log requested last session (see prior entry) was built this session — see `environment.md`/`death_log.py`, a sibling log for death causes was also built and already found something real (see `DEVLOG.md`).

## Session 7 — NeuroWare (2026-07-19)

### Current state
Working. `Agent` gained a `pending_penalty` accumulator to carry team-level negative rewards (death/bleed, both defined in `grid.py`/`config.py` — see `environment.md`) into the next real task-completion reward without disturbing the existing "reward only assigned at task completion" timing. `rl_policy.py`'s observation gained a 7th per-cell feature (`est_moisture`), closing a real information gap between what the policy can see and what the reward signal is based on. Best model of the session (45.7% harvest vs. 34.5% random) reflects both changes plus the tuned bleed from `environment.md`.

### What was done
- **`Agent.pending_penalty`** (new field, `__init__`) — accumulates negative team-level rewards from `grid.py`'s per-tick death/bleed checks (see `environment.md`). Folded into `self.last_reward` inside `_complete_task()`, immediately before the existing `reward_log.record(...)` call, then reset to 0. Deliberately *not* injected by overwriting `last_reward` directly at the moment a penalty occurs — that would prematurely resolve whatever Q-transition is currently pending for an agent mid-travel, even if their current task has nothing to do with the cell that's in trouble.
- **`est_moisture` feature** (`_cell_features`, `rl_policy.py`) — `known_moisture - (_MOISTURE_DECAY_PER_TICK * ticks_since_last_monitored)`, clamped at 0, normalized to 0-1, with the same `-1.0` never-observed sentinel as `known_moisture`. `_MOISTURE_DECAY_PER_TICK` is a new module constant (`MOISTURE_DECAY_BASE / FPS`) — reuses `environment.md`'s existing decay-rate constant rather than inventing a second one, using the average of the 0.5x-1.5x per-tick jitter (which averages to 1.0x). `FEATURE_DIM` bumped from `N_CELLS*6+2+4` to `N_CELLS*7+2+4` (78→90) — confirmed via a direct `build_observation` shape check before retraining, not assumed.
- **Investigated, with real headless data, whether agents were "too cautious" about watering** (user's live-play observation) — measured across 10 episodes: only 1.8% of ticks had a *known*-needy unclaimed cell while an agent sat idle/wandering, and 28.9% of ticks had an agent actively mid-watering-task. No strong evidence of idle-avoidance. Redirected the investigation toward the actual confirmed gap instead (see below).
- **Confirmed via code, not inference, that the policy's task-selection is fundamentally blind to true cell state** — `_cell_features` only ever reads `cell.known_moisture`/`cell.last_monitored`, both of which only update when an agent physically monitors that specific cell (`grid.py`, `Cell.observe()`). The death/bleed penalties, by contrast, fire off true `cell.moisture` regardless of observation state. This is why a cell can visibly be "bleeding" (per the new yellow indicator, see `environment.md`) while the policy has no direct signal that anything's wrong beyond ordinary staleness — `est_moisture` above is the fix.
- **Discussed and resolved the movement/coordination architecture question** (no code changes) — user asked whether the RL layer should eventually own movement/pathing too, not just task priority. Landed on: pathing to a *chosen* destination is already algorithmically solved (BFS shortest-path, provably fine) and doesn't need learning; the real unsolved problem is coordination between agents when their optimal paths conflict. Considered two shapes for that: (a) centralized broadcast-and-arbitrate (agents predict/broadcast intended routes, a central authority resolves conflicts — architecturally the same as the existing Q-value-based `_should_yield` swap-fix, or classic reservation-table multi-agent pathfinding), vs. (b) fully decentralized peer-to-peer negotiation (each agent's own policy reasoning about others' broadcast intentions, no central authority). Decision: keep (a) for now, since `NUM_AGENTS=2` doesn't yet justify the added complexity/training-instability risk of (b) (multi-agent RL non-stationarity — each agent's policy is a moving target from every other agent's perspective). Revisit (b) once agent count scales up and centralized coordination visibly starts breaking down, and specifically once communication is genuinely constrained (the mining-automation discussion this session was the concrete real-world analogy that sharpened this: centralized dispatch works fine when a reliable link to a coordinator exists, decentralized negotiation becomes necessary specifically when it can't be assumed).

### What worked
- `est_moisture` produced the best harvest-rate result of the entire session (45.7% vs. 34.5% random, ~11-point edge) — see `DEVLOG.md` for the full number set. User's own live-play read ("prioritizing the yellow ones somewhat") lined up with the headless numbers, rather than contradicting them.
- The idle/water-caution investigation, even though it didn't confirm the original hypothesis, was still useful — it ruled out one explanation cleanly with real data instead of leaving it as an open guess, and pointed at a different, confirmed, more specific gap instead.

### What did not work
- No `agent.py`/`rl_policy.py`-side failures this session beyond the `DEATH_PENALTY` credit-assignment issue, which is really a `pending_penalty`-mechanism design flaw — see `DEVLOG.md` for the full failure/root-cause writeup (kept there since it's as much a reward-design story as an agent-code one).

### Key decisions
- `pending_penalty` kept as a *separate* field from `last_reward`/`committed_state`/`committed_action` rather than overloading any of them — same rationale pattern as Session 6's `committed_state`/`last_state` split: timing matters, and reusing an existing field would silently change when a value is considered "consumed."
- `est_moisture` was built as an inference from *already-known* data (last observed value + elapsed time + a known average decay rate), not as a second channel of true state — kept the partial-observability design (the whole point of "monitor" as a task, the grey "unknown" cell color, the "unmonitored" HUD stat) intentionally intact rather than quietly undermining it to fix the immediate problem.

### Dependencies
- `rl_policy.py`'s `_cell_features` now imports `MOISTURE_DECAY_BASE` and `FPS` from `config.py` (previously only used in `grid.py`) — a new cross-module dependency, but on constants, not behavior, so low risk.
- `agent.py`'s `_complete_task` now reads `self.pending_penalty`, written externally by `grid.py`'s `tick()` — same "grid writes into agent state" pattern already established by `grid.agents`/`_should_yield` in Session 6.

### [WORTH EXPLORING]
- `est_moisture`'s decay-rate estimate uses a single global average (`MOISTURE_DECAY_BASE`) — doesn't account for the fact that `DIFFICULTY_MULT`/jitter mean actual per-cell decay varies more than that single constant implies. Probably fine (the model can still learn to treat the estimate as approximate), but worth watching if a future retrain shows the model over-trusting a stale estimate.
- Decentralized peer-to-peer negotiation (see "What was done") remains unscoped beyond the high-level direction — no design doc, no broadcast-message shape decided, deliberately deferred until agent count actually increases.
