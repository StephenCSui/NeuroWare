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
