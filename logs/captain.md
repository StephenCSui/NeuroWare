# Captain Component Log

---

## Session 2 — NeuroWare (2026-06-16)

### Current state
**Working — pickup ping and captain-to-captain sync functional; goal ping fires but delivery side is broken**

### What was done
- Built `Captain` class in `simulation/captain.py`
- Two captain instances: `cap_a` (left zone, pos 200,375) and `cap_b` (right zone, pos 1000,375), linked bidirectionally
- Captain maintains:
  - `memory` — all items ever seen in zone (item_id → item)
  - `active_pings` — items currently needing pickup agents (item_id → item)
  - `goal_pings` — items awaiting delivery (item_id → (item, goal_center))
  - `pending_syncs` — delayed incoming syncs from other captain ([(delay_remaining, item)])
- `_scan_for_items(items)`: scans zone every `CAPTAIN_PING_INTERVAL` (3s), re-pings all available items each cycle (not just new ones), syncs new items to other captain
- `sync_memory(item)`: queues sync with `SYNC_DELAY = 2.0s` delay — captain-to-captain comms are not instant (simulates blockchain propagation lag)
- `_activate_sync(item)`: called when delay elapses, adds item to `active_pings` and fires visual ping
- `receive_goal_request(item_type, goal_center)`: finds stored item of requested type in memory, adds to `goal_pings`, syncs goal to other captain instantly, falls back to other captain if not found locally
- `update()`: cleans up fully assigned/stored/delivered items from pings, processes `pending_syncs` countdown, fires scan on interval, advances ping animation

### What worked
- Captain A pings items in its zone and syncs to B after delay — confirmed working
- Captain B does not immediately know A's tasks — sync delay correctly gates this
- Both captains re-ping every cycle for items still waiting (not just new arrivals) — prevents agents that come into range late from missing the ping
- Ping cleanup: items that are fully assigned or stored are removed from `active_pings` correctly
- Visual: `draw_captain_link` shows "syncing..." in yellow when `pending_syncs` is non-empty, "sync" in dim blue otherwise

### What did not work
- **Goal delivery broken** — `receive_goal_request` correctly adds to `goal_pings` and fires ping, but agents are not completing the delivery. Root cause believed to be in agent-side `evaluate_known_tasks`, not captain-side (captain's goal ping appears correct).
- Previous issue (now fixed): Captain B not pinging after sync — `sync_memory` was only updating `memory`, not `active_pings`. Fixed by queuing via `pending_syncs` → `_activate_sync`.
- Previous issue (now fixed): Captain B immediately knew A's tasks — no delay existed. Fixed with `SYNC_DELAY = 2.0s` queue.
- Previous issue (now fixed): Captain only pinged new items — `_scan_for_items` now re-pings all available items in zone every cycle.

### Key decisions
- Captains are infrastructure nodes, not agents — they do not move, do not carry, do not assign
- Captain ping is a broadcast announcement only — agents self-select, captain does not dispatch
- `active_pings` is the source of truth agents read (via gossip); captain does not track which agents are responding
- `SYNC_DELAY = 2.0s` is a deliberate design choice simulating blockchain propagation latency between captains — not a bug

### Dependencies
- `config.py`: `CAPTAIN_RADIUS`, `CAPTAIN_PING_INTERVAL`, `COMM_RANGE`, `CAPTAIN_A_ZONE`, `CAPTAIN_B_ZONE`
- `item.py`: `Item.is_available()`, `Item.stored`, `Item.delivered`
- `agent.py`: agents read `cap.active_pings` and `cap.goal_pings` directly in `absorb_captain_info`

### [POTENTIAL FIX]
- Goal delivery: verify `receive_goal_request` is being called with the correct `item_type` (0-indexed vs label mismatch possible in `main.py` key handler — `chosen % 3` maps keys 4-6 to types 0-2, which may not match what was stored)
- Confirm `item.stored = True` is set before `receive_goal_request` is called — if called while item is still in transit, `memory` lookup will find it but agents will reject it in `evaluate_known_tasks`

### [WORTH EXPLORING]
- `SYNC_DELAY` is hardcoded — could be made configurable or variable (simulating network jitter)
- Captain currently has no way to know if a task was ever completed — `memory` grows forever; may want a completed/archived dict for future debugging/analytics
- If both captains have a stored item of the requested type, `receive_goal_request` returns after the first match — no tie-breaking or load balancing between captains

---

## Session 3 — NeuroWare (2026-06-16)

### Current state
**Working — all captain functionality stable; crash on G+4/5/6 fixed**

### What was done
- Fixed infinite recursion crash in `receive_goal_request`: the fallback `self.other.receive_goal_request(...)` called back into itself endlessly when neither captain had a stored item
- Refactored into two methods: `receive_goal_request` (one-level only, tries self then other) and `_try_dispatch_goal` (does the actual memory lookup and ping, no recursion)
- Goal pings now correctly broadcast for heavy items (types 3-5) — `receive_goal_request` was already type-agnostic, the crash was the only blocker
- `main.py` updated: removed block on keys 4-6 in goal mode — all six item types can now be dispatched to goal

### What worked
- No crash when pressing G + any key with no matching stored item (returns False cleanly)
- Cross-captain fallback works: item stored in cap_b's memory, requested via cap_a → cap_b dispatches correctly

### What did not work
- N/A this session

### [WORTH EXPLORING]
- `SYNC_DELAY` is hardcoded — could be made configurable or variable (simulating network jitter)
- Captain currently has no way to know if a task was ever completed — `memory` grows forever; may want a completed/archived dict for future debugging/analytics
- If both captains have a stored item of the requested type, `_try_dispatch_goal` returns after the first match — no tie-breaking or load balancing between captains

---

## Session 4 — NeuroWare (2026-06-16)

### Current state
**Working — adapted for push model; all phase/slot guards in place**

### What was done

**Adaptations for push model:**
- `_scan_for_items`: now sets `item.destination = STORING_CENTER` and `item.phase = "to_store"` (instead of assigning carriers). Added critical guard: `if item.phase == "to_goal": continue` — prevents re-claiming an item that is already in delivery.
- `_activate_sync`: added same `if item.phase != "to_goal"` guard — prevents a sync arriving from other captain from overwriting an in-flight goal item's phase.
- `_try_dispatch_goal`: availability check now uses `not any(item.claimed_slots.values())` instead of `not item.carriers` — correct for push model.
- `active_pings` cleanup: now checks `item.is_available()` instead of `item.assigned_agents` — matches new item API.
- `STORING_CENTER` computed once from `STORING_ZONE` config rect — used as the destination set on items pinged to store.
- Captain never touches `item.claimed_slots` — that is fully agent-to-agent.

### What worked
- No re-claiming of in-flight items after guards added
- Cross-captain goal dispatch still works: item in cap_b memory, requested via cap_a → cap_b dispatches

### What did not work
- **Goals reverted to store phase** (early in session): `_scan_for_items` did not check `item.phase`, so it re-assigned `item.destination = STORING_CENTER` and `item.phase = "to_store"` on items already mid-delivery. Fixed with `if item.phase == "to_goal": continue`.
- Same regression would have occurred through `_activate_sync` for synced items — fixed with parallel guard there.

### Key decisions
- Captain's only outputs to the swarm are `item.destination` and `item.phase`. No slot assignment, no speed, no timing — all of that is agent-to-agent.
- The "to_goal" phase guard is the authoritative lock against double-assignment. Once an item is `to_goal`, no captain path can downgrade it to `to_store`.

### Dependencies
- `config.py`: `CAPTAIN_PING_INTERVAL`, `COMM_RANGE`, `CAPTAIN_A_ZONE`, `CAPTAIN_B_ZONE`, `STORING_ZONE`
- `item.py`: `Item.is_available()`, `Item.stored`, `Item.delivered`, `Item.phase`, `Item.claimed_slots`

### [WORTH EXPLORING]
- Captain has no awareness of push progress — it cannot know if a "to_store" task is stalled (all agents dropped their slots, nobody re-claimed). A watchdog timer on `active_pings` items could detect stalled tasks and re-ping.
- If `item.destination` is set by the captain but the item never gets claimed (no agents in range), the item sits with a destination but no movers indefinitely — no timeout or escalation exists.

---

## Session 5 — NeuroWare (2026-07-17)

### Current state
**Removed. `captain.py` deleted this session — no longer part of the codebase.**

### What was done
- Captains removed entirely as part of the warehouse-to-farm redesign, at explicit user direction: "remove captains completely, give agents full autonomy, as a swarm."
- Root cause for removal, identified through discussion rather than a bug report: `Item.destination` — task *existence*, not just assignment — was only ever set by `Captain._scan_for_items`. Killing the captains would have left agents structurally unable to discover any new work even though they could already sense items directly (`sense_items`). This was a hidden single point of failure that defeated the actual point of swarm robotics (functioning without a central brain), not a design preference call.
- Replacement: task existence is now derived directly from grid-cell state that agents sense themselves (see `agent.md`/environment entries this session) — no infrastructure/relay tier of any kind remains.

### Key decisions
- This is a closing entry, not a bug report. Captains functioned correctly for their scope in the warehouse model (Sessions 2–4 above) — they were removed because that scope itself (gating task existence behind a scan cycle) was the wrong shape for the autonomous-farm direction, not because of a defect in the implementation.

### Dependencies
N/A — file no longer exists.
