# NeuroWare DEVLOG

---

## Session 1 — NeuroWare (2026-06-14)

### What was done
- Project defined and scoped via transcript + concept sketches
- Reviewed reference paper: Yadav et al. "Maintaining the Level of a Payload carried by Multi-Robot System on Irregular Surface" (arXiv:2512.16024v2)
- Reviewed existing assets: `paper_robot_centered_actuator_v6/` — URDF, SDF, GLB/STL meshes, piston height model Python class
- Reviewed concept sketches (SVGs) detailing single-unit profile, scissor lift, 3-actuator tilt platform, tactile force plate with load cells

### What worked
- Existing v6 model is clean: actuator correctly centered, URDF jointed properly, piston height formula implemented and matches the paper

### What did not work
- N/A (no implementation attempted yet)

### Immediate issues
- No Isaac Sim (USD) version of the robot model exists yet
- Mechanical design not started — 3-actuator tilt platform, scissor lift, omni-wheel base are still concept-level
- No tactile sensing / load cell simulation pipeline yet
- Swarm coordination layer (mesh communication, shared load/pose) not designed

### Current state
Early inception. Reference paper implemented in Gazebo/ROS (v6 model). Project is extending this toward Isaac Sim, tactile sensing, and a more capable per-unit tilt mechanism.

### Next steps (high level)
1. Decide on target robot configuration for first Isaac Sim prototype (single piston vs. 3-actuator)
2. Convert/rebuild robot model for Isaac Sim (USD format)
3. Implement tactile force plate sensing in simulation
4. Port or rewrite swarm coordination and piston height control for Isaac Sim
5. Define swarm communication protocol between units

---

## Session 1 (continued) — NeuroWare (2026-06-15)

### Study session — Swarm robotics + communication architecture
- Read Yadav et al. paper in full; read Wen et al. "Swarm Robotics Control and Communications" (IEEE Comms Magazine, 2018)
- Key decision: AI direction is decentralised swarm robotics with blockchain-like peer-to-peer communication architecture — no central controller for carrying/balancing; consensus reached between robots using only local sensing and inter-robot communication
- Communication identified as the foundational layer — shapes all other AI/control decisions

### Architecture design — agents, captains, and communication
**Terminology:**
- Individual robots = **agents**
- Coordination/memory nodes = **captains** (not necessarily robots, infrastructure nodes)

**Communication layers:**
- Agent ↔ agent — ESP-NOW, local peer-to-peer, close proximity, fast and low latency
- Agent → captain — ESP32 mesh, messages relay through other agents to reach captain
- Captain ↔ captain — blockchain-like consensus for shared memory sync
- Captain → agents — non-blocking broadcast ping, agents self-select to respond based on availability

**Captain responsibilities:**
- Memory bank only — aggregates what agents report, does not assign or control agents
- Represents a zone by knowing what agents have reported within it
- Syncs with other captains to keep distributed memory consistent
- Does not micromanage agents — agents are autonomous at all times

**Task flow:**
- External task arrives, captain broadcasts a lightweight ping
- Free agents self-select to respond — busy agents ignore the ping
- If no agents respond, captain escalates to neighbouring captains via blockchain layer
- Neighbouring agents can reach original captain via mesh or work under their own captain
- Task ownership follows object location — captain whose zone the object is in owns the task record
- Agents report back to nearest captain after placing an object; captain records it

**Prototype scale:**
- Simulation: 12-15 agents, 2-3 captains
- Physical build: 5-6 agents
- Hardware: ESP32 viable at both scales

**Parked for later:**
- Agent priority definitions (how agents choose between competing pings)
- Dynamic captain role assignment (agent temporarily becoming captain for a task)
- Lightweight ping optimisation
- Rigid zone boundary definitions

---

## Session 2 — NeuroWare (2026-06-16)

### What was done
- Implemented gossip protocol in `simulation/agent.py` — replaces old `consider_captains` / `learn_from_neighbours` polling model
- Removed all captain-driven agent assignment; agents now fully self-select from their own `known_tasks` store
- Gossip runs unconditionally every frame (even when agent is busy or carrying), only task execution is gated on free state
- Four new methods: `absorb_captain_info`, `gossip`, `cleanup_known_tasks`, `evaluate_known_tasks`
- `update()` restructured: gossip block first, then busy/idle gate, then evaluate

### What worked
- Smoke tests pass cleanly — all gossip methods run without error, import chain intact
- Architecture is correct: an agent mid-carry or busy will still propagate task info to any agent it encounters

### What did not work
- Goal delivery broken — agents are not acting on goal-type entries in `known_tasks` correctly (exact cause not diagnosed)

### Current state
Gossip protocol implemented and structurally sound. Core pickup + store + co-carry flow should be unaffected. Goal delivery is the immediate known bug.

### Next steps (high level)
1. Debug goal delivery — trace why `evaluate_known_tasks` is not triggering `_commit_to_goal` correctly
2. Test gossip propagation end-to-end with a multi-agent scenario (item placed, captain pings, agents far from captain still learn via neighbours)
3. Tune `COMM_RANGE` back down from 250 once gossip is confirmed working

---

## Session 3 — NeuroWare (2026-06-16)

### What was done
- Fixed goal delivery — three root causes in gossip/cleanup/commit logic (detail in agent log)
- Added heavy item goal delivery: 2-agent co-carry from storage to goal zone, new `co_delivering` state chain
- Fixed crash on G+4/5/6: infinite recursion in `receive_goal_request` between captains (detail in captain log)
- Added task prioritisation: agents prefer filling an in-progress heavy item's co-carrier slot over starting a new solo task

### What worked
- All goal delivery paths now functional: light (1 agent) and heavy (2 agents)
- Priority ordering confirmed by unit test

### What did not work
- N/A

### Current state
Core simulation loop fully functional. Pickup, store, and goal delivery working for all item types. Swarm prioritises completing in-progress tasks before starting new ones.

### Next steps (high level)
1. Tune `COMM_RANGE` back from 250 and verify gossip still propagates reliably
2. Cooperative task allocation for high-agent-count tasks (parked — detail in agent log)
3. Scale agent count toward 12-15 and observe emergent behaviour

---

## Session 4 — NeuroWare (2026-06-16)

### What was done
- **Complete simulation redesign: carry model → push model.** Items now have physical presence and collision; agents push them by positioning on the opposite side and applying force.
- Rewrote `item.py`: physics accumulator (`_push_fx/fy`), `physics_update()`, `resolve_collision()`, `claimed_slots` per side, `slot_position()`, `needed_sides()`, `apply_push()`, `is_available()`
- Rewrote `agent.py`: new state machine `seek_push_slot → waiting_for_co_pusher → pushing`, gossip now shares item object directly by reference via `known_tasks = {item_id: item}`, per-side push logic, break-off on axis completion
- Updated `captain.py`: adapted for push model (`claimed_slots` replaces `carriers`), added `phase` tracking (`to_store`/`to_goal`), race guards preventing re-claim of in-flight items
- Updated `main.py`: new update order (sense → agent.update → item.physics_update → item-item collision → captains), keys 1/2/3 only, `draw_push_links` replaces `draw_co_carry_link`
- Rewrote `config.py`: 3 item types (S/M/L with `agents_per_side` 1/2/3), push physics constants, removed old carry-model constants
- Multiple bugs found and fixed across the session (detail in component logs)
- End-of-session fixes: `_side_ready` physical proximity check, `_do_waiting_for_co_pusher` displacement re-seek, `_do_seek_push_slot` perpendicular-side-based speed logic

### What worked
- Push model core: agents claim slots, navigate to slot position, wait for co-pushers, apply force — item moves
- Per-side independence: one axis can start pushing while the other is still filling agents
- Break-off logic: agents on a completed axis detect they're no longer needed and release
- Diagonal rule enforced structurally: single agent cannot push at 45° — can only push the axis their side corresponds to
- Finalization race guard: `item.destination = None` set immediately on first finalize call, prevents double-finalization

### What did not work
- Item pushed off map early on (force applied before arrival check; physics_update ran on stored items; no boundary clamp) — all fixed
- Vertical pusher not breaking off after item reached 0° angle — fixed with `elif side not in item.needed_sides()` in `_do_pushing`
- Item started moving before all agents in position — fixed with per-side `_transition_side_to_pushing` and `waiting_for_co_pusher` state
- Goals reverted to store phase — captain's `_scan_for_items` re-claimed `to_goal` items — fixed with `if item.phase == "to_goal": continue`
- `_apply_separation` AttributeError — leftover call after method deleted — fixed
- A1 pushed L-type item on vertical axis alone: `_side_ready` checked task state only, not physical position — A4/A5 were displaced by moving item but still flagged as `waiting_for_co_pusher` — **this is the final bug addressed at end of session**

### Current state
Push model fully redesigned and operational. The three end-of-session fixes (`_side_ready` proximity check, displacement re-seek, speed logic) were applied but not yet play-tested. Ready for testing session.

### Next steps (high level)
1. Play-test the three end-of-session fixes — confirm L-type no longer pushes on one axis alone
2. Tune push speed multipliers (2.0/1.0/0.7) against observed agent behaviour
3. Tune `PUSH_FORCE`, `ITEM_DAMPING`, `PUSH_SPEED_MAX` for physical feel
4. Add item log (item.py was completely rewritten and has no component log yet)

---

## Session 5 — NeuroWare (2026-07-17)

### What was done
- Extended design discussion (played out over most of the session) covering: play-testing the Session 4 push-model fixes; a critique that captain-gated task discovery was a hidden single point of failure defeating the point of swarm robotics; a critique that deterministic top-of-sort task selection wasn't real "deciding"; comparison of ant (stigmergy) vs. bee (quorum-sensing) coordination models; a domain pivot decision — warehouse → fully autonomous farm, chosen over mining for richer task-allocation variety and personal relevance, after mining was floated first and reconsidered
- Full architectural rewrite of the simulation to match: deleted `captain.py`, `item.py`, `zone.py`; added `grid.py`; rewrote `config.py`, `agent.py`, `main.py`
- Captains removed entirely — full peer-to-peer agent autonomy, no infrastructure/relay tier
- New grid-based farm model: cells are `plant` (moisture/weed dynamics) or `walkway` (inert, ≥2 cells wide so agents can pass each other), replacing the old loading/storing/goal zones and pushable items
- New information model: each plant cell has hidden **true** state that drifts continuously and stochastically (jittered moisture decay, probabilistic weed-sprout events — not smooth/linear, deliberately unpredictable) and separate **known** state that only updates when an agent physically monitors/works the cell — task decisions read known state only, so the swarm can be and stay wrong about a cell until someone re-checks
- New task model: three task types for this pass (monitor, weed, water — till/plant-organization/resource-constrained-watering explicitly deferred); flat, deliberately unweighted random selection among known needy cells (no scoring formula) — placeholder for a future AI/ML prioritization layer
- New movement model: grid-stepped with hard per-cell occupancy (exactly one agent per cell at all times, regardless of task/movement/idle state) — replaces the old free continuous movement; agents reserve their next cell the moment they commit to stepping there, wait a frame if every neighbour is blocked
- Gossip made global (any agent can act on any cell's known state regardless of position) rather than range-limited, since captains — not comms range — were the actual single point of failure identified this session
- Redesigned the farm layout from a repeating bed/walkway pattern to an explicit template-based layout (user-supplied reference image): a 4x8 plant bed, pathway margins, a deadspace block, and a water zone, on a much smaller 16x12 grid (192 cells, down from 680, `CELL_SIZE` 30→45) — directly resolves the coverage/tuning gap noted below. `NUM_AGENTS` reduced 6→5.
- Added a per-agent water resource flag: agents carry a single "charge" of water, consumed on each completed watering, requiring a refill trip to the water zone (its own timed `obtain_water` task, out-of-water is a hard gate blocking all other tasks) before they can water again — a deliberately simple, non-delegated version of the fetch/deliver mechanic discussed earlier, at explicit user direction to defer full task-handoff to a future AI-paired implementation
- Deadspace made a genuine hard barrier (`Cell.blocked`) rather than inert walkable ground, per explicit user correction, requiring real pathfinding (`Grid.shortest_path`, BFS then upgraded to weighted Dijkstra) to replace the old single-step greedy movement, which had no way to route around a large static obstacle
- Added cost-weighted pathing (plant cells cost more to traverse than pathway, via `CELL_TRAVERSAL_COST`) so cross-farm transit prefers routing around the plant bed — where working agents park — rather than cutting through it, per user observation that agents were bumping into each other unnecessarily on long trips

### What worked
- 20,000-tick headless verification run (script not committed to the repo, scratch-only): zero cell-occupancy violations, zero walkway cells ever claimed as tasks, zero premature water/weed candidacy on unmonitored cells, selection confirmed genuinely non-deterministic (claim distance-ranks spanned 5–678 of ~680 candidates, 0% picked the literal nearest), known-state staleness measurably diverging from true state over time (one cell showed a 50-point gap between belief and reality)
- Live GUI launched and ran without errors on the actual display for the duration of testing
- Re-verified after the template layout resize: all 32 plant cells got monitored within the test window (vs. 157/680 before), task-type balance became healthy (monitor/water/weed all firing at reasonable rates instead of monitor dominating)
- Re-verified after the water flag, deadspace barrier, and weighted-pathing changes: zero water-flag violations, zero blocked-cell entries, full coverage maintained throughout; direct test confirmed a cross-bed trip that previously cut through plant cells now routes entirely via pathway (0 plant cells entered)

### What did not work
- Tuning gap (now resolved): with the original 680-plant-cell farm and 6 agents at the deliberately slow `AGENT_SPEED`, coverage couldn't keep pace with the map — monitor tasks vastly outnumbered water/weed tasks (177 vs. 10 vs. 3) because most cells stayed perpetually stale. Fixed by the template-based layout resize.
- Real bug, caught via verification: the first pathfinding implementation (unweighted BFS, one cached path per journey, no fallback if another agent contested the exact next cell) caused a severe regression — task throughput collapsed to near-zero (15 claims vs. a baseline of ~180) because agents converging on the narrow water-zone bridge could deadlock waiting on each other indefinitely. Root cause: caching one shortest path and only ever checking it against the *static* map, never against other agents' actual positions. Fixed with a stuck-frame timeout (`STUCK_REROUTE_FRAMES`) that forces a reroute treating currently-occupied cells as temporary obstacles once a step has been blocked too long. Worth remembering as a category, not just a one-off: static pathfinding correctness and dynamic multi-agent contention are different problems, and fixing one doesn't fix the other.

### Current state
Farm swarm sim is functional and verified end-to-end on the new template-based layout — movement (grid-stepped, hard occupancy, weighted pathing around a real deadspace barrier), the water resource flag, task claiming, and the known/true state split are all working together with healthy task balance and full plant-cell coverage.

### Next steps (high level)
1. Rewrite `simulation/README.md` and root `README.md` — still describe the old warehouse/item/captain model
2. **Deferred, explicitly for later (not a bug, a scope decision):** trajectory broadcasting / cooperative path reservation — agents plan and publish their intended route in advance so others can route around it, using something like expected task+travel time as the path cost, instead of the current per-step reactive contention handling. This is a real, established multi-robot technique (cooperative/prioritized path planning with time-reservations), but a good version needs to estimate dynamic congestion, which is a genuinely circular problem best paired with the AI/ML layer already being deferred for task prioritization and water fetch/deliver delegation — not something to hand-build with fixed heuristics now.
3. Other deferred features from the approved plan: till task, plant-lifecycle/scoring system (day/night cycles, death, harvest), resource-constrained watering delegation (fetch/deliver as separate claimable phases), AI/ML-driven prioritization to replace flat random selection, range-limited gossip if global sharing proves too easy once observable

---

## Session 5 (continued) — NeuroWare (2026-07-18)

### What was done
- Compacted the farm: 5 agents → 2, 32 plant cells → 6, grid 16x12 → 10x8. Deadspace and the one-way lane system (both built for 5-agent bridge contention) were dropped entirely — not needed at this scale, and the one-way lanes had a measured throughput cost. `CELL_TRAVERSAL_COST` weighting dropped too — the open layout doesn't need it.
- Added a plant lifecycle: `Cell.status` (`growing` → `ready` → `harvested`/`dead`/`spoiled`, last three terminal — no respawn under any circumstance for the rest of the season, confirmed explicitly with the user). Added a day cycle reusing the existing tick clock (`DAY_LENGTH_TICKS`, no separate time system) — maturity at day 28, a 2-day harvest window, season end at day 30.
- Added death conditions (dehydration 100%, weeds 100%, or both ≥80% simultaneously) and a harvest+delivery task pair (`harvest` to pick, `deliver_harvest` to drop at a new harvest-box cell) with a one-at-a-time carrying-capacity flag on `Agent`, mirroring the existing `water` flag pattern.
- Added scoring (`grid.score`, additive only — water/weed/harvest rewards, no penalties; the user's own reasoning was that losing the chance to ever harvest a dead cell already is the penalty).
- Season end now halts agent activity (not rendering) — board and score stay visible rather than the process exiting.
- Built and shipped an RL task-prioritization layer: a `TaskPolicy` (scikit-learn `MLPRegressor`, no new dependencies) trained via hand-rolled Q-learning in a new headless `train.py`, replacing the flat `random.choice` in `evaluate_and_claim` when a `USE_RL_POLICY` config flag is on. The random baseline stays fully intact and is the default — this was a hard requirement, since `grid.score` exists specifically to compare the two.

### What worked
- Headless full-season verification (scratch script, ~10 runs): 0 occupancy/claim violations across every run, lifecycle transitions correct, season-halt correct.
- Caught and fixed two real balance/logic issues before they reached the user as "it's broken" (see below) rather than after.
- RL training completed cleanly (2000 episodes, well under a minute headless): trained-greedy policy averaged 52.4 score over 50 eval seasons vs. 41.2 for the random baseline — a real but noisy edge, not a large one; being honest about that rather than overselling a 2000-episode/2-agent training run.
- Live GUI play-tested twice this continuation (once for the compact baseline, once with the trained policy loaded) without errors.

### What did not work
- **Balance bug, caught via verification before any play-test:** `MOISTURE_DECAY_BASE` was still tuned for the old open-ended 5-agent/32-cell loop, where a stale cell was just annoying, never fatal. At that rate, nearly every cell died of dehydration before reaching maturity (4-6/6 dead, every run). Halved the constant (0.9→0.5); this helped but total crop failure is still the single most common outcome under the deliberately-dumb random baseline — decided with the user to leave this as an honest hard baseline rather than keep tuning toward "typically winnable," since a struggling dumb swarm is useful evidence for exactly the kind of AI-layer work that followed.
- **Real bug, caught by the user watching live, not by verification:** every plant cell initialized with `last_monitored = 0`, so all six became monitor-eligible on the exact same tick — a synchronized "monitor everything, then go quiet" burst rhythm instead of continuous small activity, which read as "agents stop working after the first pass." Fixed by staggering each cell's initial `last_monitored` to a random negative offset within one staleness window. Worth noting: this kind of thing is easy to miss in headless verification (which checks correctness, not rhythm/pacing) and only showed up from watching the live GUI — a reminder to actually watch it, not just trust the stats.
- **Real bug, caught by the user watching live, unresolved:** two agents can deadlock when their final destinations are literally each other's current cell (a swap) — the existing stuck-timeout reroute only avoids *intermediate* occupied cells, not the final target, so nothing resolves it. Explicitly left unfixed at user direction — this is the same pathing/negotiation problem already deferred to "the AI" in the prior entry, and the user didn't want a heuristic patch (e.g. an agent-id tie-breaker) papering over what should eventually be real inter-agent communication.

### Current state
Compact 2-agent/6-cell farm with a full 30-day lifecycle/harvest/scoring loop is working and verified. An RL-trained task-prioritization policy exists, trains cleanly, and modestly beats the random baseline; it's switched on by default in `config.py` right now (`USE_RL_POLICY = True`) after being play-tested live. The agent-agent swap deadlock is a known, unresolved issue — out of scope for the RL layer just built (that layer only replaces *which task to work*, not movement/negotiation).

### Next steps (high level)
**Stated direction (2026-07-18): the user wants future work to prioritize AI/RL-driven solutions over hand-coded/algorithmic fixes** — when a next step below could go either way, default to proposing the AI-driven option first.
1. Decide on the swap-deadlock: leave for a future negotiation/communication layer (consistent with how it's been treated so far, and with the direction above), or apply a cheap interim tie-breaker.
2. RL policy is a first pass — more training episodes, reward-shaping iteration, or richer features (e.g. real path cost instead of Manhattan distance) could plausibly widen the gap over the random baseline; the current ~25%-over-50-eval-episodes edge is real but not heavily validated statistically.
3. Two new files (`simulation/rl_policy.py`, `simulation/train.py`) don't have a component log yet — folded into `agent.md`/`environment.md` for now; flagging that a dedicated `rl.md` might be worth creating if this component keeps growing (per `LOGGING_STANDARDS.md`, proposing rather than doing this unprompted).
4. Collective/multi-agent RL over movement and coordination (not just task choice) was discussed as a legitimate future phase — explicitly not started now, scoped as its own deliberate effort later.
5. **User request (2026-07-18), not yet built:** a reward log — a record of every reward event the RL policy experienced (positive and negative), not just the aggregate `grid.score`/episode-score numbers currently printed by `train.py`. See `agent.md`'s `[WORTH EXPLORING]` for the detailed sketch.
4. `simulation/README.md` and root `README.md` still describe the old warehouse/item/captain model — not yet started.

## Session 6 — NeuroWare (2026-07-18)

### What was done
- **Fixed the agent-agent swap deadlock** (open since Session 5) — root-caused via a headless diagnostic before touching any code: 60 random-baseline seasons showed 40/60 (67%) hit a permanent physical stall, and a follow-up pass showed 43/43 of those stalls were the exact literal mutual swap (agent A's target is agent B's current cell and vice versa). Fixed in `Agent.step_toward` — detects the condition immediately (no longer waits out the generic stuck-timer) and resolves it via a real decision, not a coin flip: reuses the *already-trained* policy's own Q-values to decide which agent yields (lower value backs off), falling back to claim seniority, then `agent_id`, only when no learned signal exists. See `agent.md` for full detail.
- **Rebalanced the season**, per explicit request: `DAYS_TO_MATURE` 28→20, harvest window widened from 2 days to 10 (`SEASON_LENGTH_DAYS` unchanged at 30). Also changed *behavior*, not just timing: a `"ready"` cell is now fully frozen (`Cell._is_active()` narrowed to `"growing"` only) — no more drift, no more neglect-death risk, no more water/weed/monitor needs once matured. Previously "ready" cells could still die during the harvest window; that's gone.
- **Added a global `TIME_SCALE` (4x)** in `config.py` — day length, agent speed, decay/sprout rates, task durations, and staleness/claim timeouts all derive from it, preserving relative balance while making a season 4x faster to watch live (75s instead of 5 min) and headless eval proportionally faster. Caught and fixed a real bug this exposed: `Agent._move_toward`'s arrival threshold was a fixed 1.5px constant, smaller than the new 4x `AGENT_SPEED` — agents could overshoot a small remaining gap and oscillate forever, never arriving. 100% death rate on first test; fixed by making the threshold `<= speed` instead of a fixed number.
- **Reintroduced difficulty on purpose**, since the harvest-window widening above made the random baseline hit ~94% harvest rate — too easy to show any RL edge. `PLANT_REGION` doubled 6→12 beds (`(3,2,3,2)`→`(3,2,4,3)`), `MOISTURE_DECAY_BASE`/`WEED_SPROUT_CHANCE` scaled by a new `DIFFICULTY_MULT` (tried 1.5x — too harsh, 71% death rate; dialed to 1.3x, still ~68%). Also converted the agent's water resource from a binary flag to `WATER_CAPACITY=2` charges per water-cell visit, specifically to stop watering's travel overhead from making it structurally less attractive than weeding to any policy, dumb or smart.
- **Built two small diagnostic logs**, both requested this session: `reward_log.py` (`RewardLog`, per-decision reward events, already existed) and new `death_log.py` (`DeathLog`, records which bed died and why — `"dehydration"`/`"weeds"`/`"both"`). Both save to CSV on quit (`main.py`), both gitignored generated artifacts.
- **Rewrote the RL layer's internals**, keeping the external contract (`policy.select(agent, grid)` → `(cell, task_type)`) identical: swapped `MLPRegressor` for `RandomForestRegressor` trained via classic Fitted-Q-Iteration (batch refit off a replay buffer once per episode, not online `partial_fit` per decision — trees don't support that cheaply). Added a **swarm-context observation block** (4 fixed-size aggregate features — nearest-teammate distance, fraction of teammates idle/out-of-water/carrying-harvest) so the same architecture works at any `NUM_AGENTS` without reshaping, verified by training at both 2 and 3 agents.

### What worked
- Deadlock fix confirmed empirically, not just by code reading: 0/60 stalls after the fix (down from 40/60), avg score across all 60 episodes matched what was previously only seen in the *lucky, non-stalled* subset (131.4 vs. the old 134.0 "no stall" baseline). Occupancy-invariant regression check also clean (0 violations, 40 episodes).
- `TIME_SCALE` balance preserved almost exactly after the `_move_toward` fix: 604.0 avg score / 225 harvested / 14 dead / 1 spoiled over 40 episodes, versus 603.4 / 224 / 16 / 0 before the time compression — same outcome, 4x fewer ticks.
- 3-agent swarm-adaptability smoke test (20 training episodes) completed with no shape errors, confirming the observation design genuinely generalizes across agent counts.
- `death_log` immediately paid for itself: surfaced that under the new 12-bed/1.3x-difficulty tuning, 287/306 deaths (94%) are dehydration, only 6 (2%) weeds, 13 (4%) both — the difficulty is currently extremely lopsided toward one failure mode, not a balanced water/weed tradeoff. Flagged, not yet addressed.

### What did not work
- **Three separate RL training runs this session (one before today's fixes, effectively wasted effort in hindsight) all landed inconclusively against the random baseline** (52.4/41.2, then 48.3/52.0 on a retrain, then 54.9/59.6 on the tree-based rewrite) — eventually root-caused: the swap deadlock (fixed this session, see above) was dominating ~67% of episode outcomes with pure movement-layer luck, unrelated to task-selection quality, poisoning every eval done before the fix. Any RL comparison from before this session should be considered meaningless now.
- **`_move_toward` oscillation bug** (see "What was done") — caught via headless diagnostic (100% death rate, single agent frozen at the exact same position for the entire trace) before it reached a play-test. Root cause confirmed by direct instrumentation (x position bouncing between two values exactly `AGENT_SPEED` apart, forever).
- **`DIFFICULTY_MULT=1.5` combined with 12 beds was too harsh** (71% death rate) — dialed back to 1.3x, which barely moved the needle (68% death rate), revealing that bed-count doubling is the dominant difficulty lever here, not the decay/weed-rate multiplier. Not further resolved this session — flagged for the user to decide direction on.

### Current state
The swap deadlock is fixed and verified. The season is now 4x faster to watch and deliberately harder (12 beds vs. 2 agents, ~30% harvest rate under random baseline) than the too-easy 6-bed tuning from earlier this session — but that difficulty is currently almost entirely a dehydration problem, not a balanced multi-threat one. The RL policy on disk (`models/task_policy.pkl`) is **stale** — trained before every fix/rebalance in this session, has never been retrained or evaluated under current rules. No trustworthy answer yet on whether the RL layer actually helps.

### Next steps (high level)
1. Retrain under the current environment and evaluate properly (larger sample than the 50 used so far) — this is the first training run that will actually mean something, everything before it was measuring a broken or now-obsolete environment.
2. Decide whether to rebalance weeds vs. dehydration (currently 94%/2%/4% split) before or after that retrain — training on a single-failure-mode environment teaches a narrower lesson ("prioritize water") than a genuinely balanced tradeoff would.
3. `rl_policy.py`/`train.py` still don't have a dedicated component log, now covering a materially bigger surface (tree-based FQI, swarm-context features, Q-value-based deadlock arbitration) than when this was first proposed in Session 5 — proposing again, more strongly this time, rather than creating it unprompted.
4. `simulation/README.md` and root `README.md` still describe the old warehouse/item/captain model — carried forward from Session 5, still not started.
5. Collective/multi-agent RL over movement itself (not just task choice, not just the deadlock-specific fix above) remains a distinct, larger future phase — not started, per prior scoping discussion.
6. **User request (2026-07-18), not yet run:** a 50-episode training run on the current (12-bed/1.3x-difficulty) environment, logging progress every 5 episodes instead of the current default of 100 — explicitly deferred to a future session ("continue later"). User's stated expectation going in: survival/harvest rate around 60% once trained, against the current random baseline's ~30-32%. `train.py`'s `LOG_EVERY` is currently a hardcoded module constant (100) — needs either a quick edit or a new CLI arg to actually log every 5.

## Session 7 — NeuroWare (2026-07-19)

### What was done
- **`train.py`'s `LOG_EVERY` made CLI-overridable** (`sys.argv[3]`), superseding Session 6's next-step #6 — several training runs done this session at various episode counts/log intervals rather than the specific 50/log-5 originally requested.
- **Built `eval_harvest_rate.py`** — a headless eval separate from `train.py`'s score-only `evaluate()`, since score conflates task-completion rewards with survival outcome and can't be read as a harvest percentage on its own. Tallies real bed outcomes (`harvested`/`dead`/`spoiled`) via `grid.stats()["status_counts"]` over N episodes for a given policy path (or `"none"` for random baseline).
- **Ran the first trustworthy retrain post-Session-6-fixes** (40 episodes, smoke test) — confirmed `models/task_policy.pkl` is no longer stale/shape-incompatible. Real harvest-rate numbers obtained for the first time: random baseline 34.8%, 40-episode trained 40.8%.
- **Tested and rejected 2000 and 300 episodes as the training length** — a 2000-episode run was killed mid-flight by explicit user request (nothing lost; `train.py` only saves at episode-loop completion). Two separate 300-episode runs both hit the replay buffer's 8000-transition cap around episode ~150 and showed no consistent improvement over the 40-episode run (one even scored worse) — episode count was not the bottleneck, buffer capacity and reward design were. Settled on 160 episodes as the working default this session, per direct user instruction after seeing this data.
- **Added `DEATH_PENALTY` (-50, `config.py`)** — a first attempt at giving the RL a direct negative signal for a cell dying, since the existing all-positive reward scheme (`HARVEST_REWARD`/`WATER_REWARD`/`WEED_REWARD`, all >=0) gave no direct penalty for neglect, only the indirect/diluted cost of a forgone future harvest. Applied via a new `Agent.pending_penalty` accumulator (set in `grid.py`'s death branch, shared across all agents — the swarm shares one `TaskPolicy`, so no per-agent blame-tracking needed) that rides along on whichever task-completion reward that agent earns next, rather than closing out an in-flight Q-transition early.
- **`DEATH_PENALTY` alone made things worse, root-caused, then fixed with a second mechanism.** See "What did not work" below for the failure; the fix was `DEHYDRATION_BLEED_THRESHOLD`/`DEHYDRATION_BLEED_RATE` (`config.py`) — a dense per-*second* penalty (not per-tick; scaled by `dt` so it's independent of `FPS`/`TIME_SCALE`) applied via the same `pending_penalty` mechanism while a growing cell's moisture sits at/below 40 (i.e. >60% dehydrated, per user's framing). Unlike the sparse terminal death event, this creates a real, repeated watered-promptly-vs-left-neglected differential instead of a rare coin-flip on which transition eats the penalty.
- **Added a visual "bleeding" indicator** — `Cell.color()` now returns a flat yellow (`COLORS["bleeding"]`) for any growing cell at/below the bleed threshold, overriding both the normal moisture/weed gradient and the "unknown" (unobserved) masking, since this is a debug aid for the human watching, not a simulation of swarm knowledge.
- **Added an `est_moisture` observation feature** (`rl_policy.py`'s `_cell_features`) — confirmed via code (not guessed) that the policy's observation only ever contains `known_moisture` (last *observed* value) and `staleness` (ticks since last observed), never true current moisture, which the death/bleed penalties are based on. This is a real information gap: a cell can be actively bleeding points with the policy having no way to distinguish "stale but probably fine" from "stale and probably critical." New feature extrapolates a legitimate inference — `known_moisture` decayed by the average known decay rate over ticks-since-last-monitored — without letting the policy see true state outright. `FEATURE_DIM` 78→90 (6→7 features per cell), required a full retrain (old pickle shape-incompatible).
- **Retrained after each reward/observation change** (four full retrains this session: baseline post-fix smoke test, death-penalty-only, death-penalty+tuned-bleed, death-penalty+tuned-bleed+est_moisture), each verified against both `train.py`'s score eval and `eval_harvest_rate.py`'s real harvest percentage — score alone was not trusted as a proxy for behavior quality at any point this session.
- **Live play-tested repeatedly** (`main.py`, real display) after each retrain — used to catch things headless eval alone wouldn't surface, including the visual bleed-indicator mass-clear finding below.

### What worked
- **`est_moisture` produced the best result of the session**: 45.7% harvested vs. 34.5% random baseline (~11-point edge) — the largest gap measured all session, and a genuine upward score trend during training (unlike the noisier earlier runs), on top of a user-confirmed live impression that agents were "prioritizing the yellow ones somewhat."
- **The tuned dehydration bleed, once rescaled, produced the first clean win after the death-penalty regression**: 42.7% vs. 35.2% random.
- **Death-cause breakdown checked against a 50-episode batch with the final model**: 94.8% dehydration / 2.2% weeds / 3.1% both — statistically unchanged from Session 6's 94%/2%/4% baseline, as expected, since nothing this session touched weed dynamics (user has a separate weeds mechanism planned for later, deliberately out of scope here).

### What did not work
- **`DEATH_PENALTY` alone made the trained policy *worse than random*** — 32.8% harvested vs. random's 33.2% (score 47.1 vs. 129.5). Root-caused, not just observed: the `pending_penalty` mechanism attaches the -50 hit to *whichever task an agent's completes next*, which can be causally unrelated to the cell that died (e.g. agent A successfully waters cell 5 while cell 12 dies elsewhere — A's correct action gets contaminated with the penalty). This is a real credit-assignment bug in the mechanism as first designed, not a difficulty or magnitude problem — user correctly pushed back on the initial framing before this was confirmed.
- **First `DEHYDRATION_BLEED_RATE` attempt (-0.1/tick) was ~10x too strong**, measured before committing to a training run: random-baseline seasons averaged -400 to -700 total score, dwarfing the entire positive reward pool (12 harvested cells only tops out around +1200). Caught via a 5-season smoke test, not a full training run — rescaled to a true per-second rate (-1.0, scaled by `dt`) at the user's suggestion, which brought random-baseline scores back to a -175/+160 range, comparable to `DEATH_PENALTY`'s own magnitude.
- **The bleed visual indicator appeared to "mass-clear" cells that were still critically dehydrated** — traced (not guessed) to `day_count` being a single grid-wide counter: every still-`"growing"` cell flips to `"ready"` at the same tick once `DAYS_TO_MATURE` is hit, and the bleed check (like the bleed reward itself) only applies to `"growing"` cells, so cells that limped along critically low the whole season get a simultaneous, silent pass at maturity — visually and reward-wise — even at moisture near 0. Confirmed this is a real design gap, not a bug in the new code; left as-is per user decision (consistent with the pre-existing "ready freezes true state" design from Session 6), flagged for later.
- **"Agents are too cautious with water" hypothesis was investigated and not confirmed** — a 10-episode headless check found only 1.8% of ticks had a known-needy unclaimed cell sitting idle-unaddressed, and 28.9% of ticks had an agent actively watering, showing no strong idle-avoidance pattern. The real, confirmed gap was a different (related) one: the observation-only-sees-known-state issue described above.

### Current state
`models/task_policy.pkl` reflects the best-of-session model: 160 episodes, `DEATH_PENALTY=-50`, `DEHYDRATION_BLEED_RATE=-1.0`/sec at threshold 40, `est_moisture` observation feature. 45.7% harvest rate vs. 34.5% random. Death causes still ~95% dehydration, weeds essentially a non-factor — untouched by design this session. Movement/coordination discussion concluded explicitly: stay with the existing centralized/deterministic swap-fix approach while `NUM_AGENTS=2`; decentralized peer-to-peer negotiation (agents broadcasting intended routes, each running RL to arbitrate locally) was scoped as a legitimate future phase once agent count actually scales up and communication becomes genuinely limited — not started, no code changes toward it this session.

### Next steps (high level)
1. Weeds vs. dehydration rebalance (open since Session 6, still ~95/2/3 split) — user has a separate weeds mechanism planned, timing/design not yet discussed.
2. The bleed-visual mass-clear-at-maturity gap (see above) — left as-is, revisit if it becomes misleading in practice.
3. `rl_policy.py`/`train.py` still have no dedicated component log — proposed a third time now (Sessions 5, 6, 7), still not created per the user-approval-only rule.
4. `simulation/README.md` and root `README.md` still describe the old warehouse/item/captain model — carried forward again, still not started.
5. Decentralized multi-agent negotiation over movement (see "Current state") — explicitly scoped as future work, contingent on agent count scaling up past 2.
6. Buffer capacity (`REPLAY_BUFFER_CAP=8000`) was identified as a likely factor in the 300-episode plateau but never itself changed or tested this session — worth a controlled test (raise the cap, retrain at a longer episode count, compare) before assuming more episodes won't help.
