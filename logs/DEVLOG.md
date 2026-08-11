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

## Session 5 — NeuroWare (2026-07-22)

### What was done
- Major project pivot: from the warehouse push-model simulation (Sessions 1-4) toward cooperative multi-robot object transport — several small robots physically combining to move objects too large/heavy for one robot alone. Motivated by wanting genuine real-world economic viability (a niche standard single-robot warehouse automation doesn't cover) rather than an abstract swarm-robotics concept
- New branch `transport` created off `main`. The separate farm-swarm pivot (branch `farm`, its own divergent history) is left untouched as historical reference, not merged or continued
- Settled architecture: two-layer design — classical/deterministic control for physical coordination (positioning, coupling, movement), with a learned decision layer reserved for team-formation/task-allocation (which robots combine, for which object) once the mechanical layer is proven. Low-level physical control deliberately not a target for RL
- Isaac Sim chosen as the eventual target platform (industry relevance, real sim-to-real pathway); Isaac Lab (the RL-training layer) deliberately not needed yet since no parallel training is planned until hardware/compute allows it
- Built a logic-only pygame prototype (`transport/`) to validate the design before touching Isaac Sim: continuous-space robots, a warehouse grid for shelf/obstacle bookkeeping (mirrors `simulation/grid.py`'s Cell/Grid pattern), BFS routing, deterministic nearest-robot task assignment as a placeholder for a future RL layer
- Redesigned the warehouse layout mid-session (from an open pickup/dropoff room to a real shelf-based layout, per a reference screenshot): robots recruit at shelf slots restricted to the specific segment currently holding an object (not the whole shelf), lift it Kiva-style from underneath (not perimeter-anchored), and carry it to a dropoff zone through a deliberately tight corridor
- Established a real baseline measurement (`eval_baseline.py`, same role as `simulation/eval_harvest_rate.py` in the farm project) before any RL work: recruitment speed, straggler lag, full cycle time, throughput

### What worked
- End-to-end pipeline functions: shelf slot restocking, robot recruitment via BFS routing, a readiness gate (won't lock until every recruited robot is physically in position, not just assigned), and decentralized local-force "negotiation" for threading the group + object through the warehouse corridor without a pre-planned route
- Footprint-aware routing (BFS validated against the object's actual bounding box, not a single point) and object-object collision were both added mid-session and measurably improved delivery throughput across varied random seeds

### What did not work
- Local force-vote ("potential field") navigation has a real, recurring local-minima failure mode: a group's vote can land on a near-zero net direction along the only unblocked axis at certain positions, causing long stalls. Root-caused (not guessed) via direct tick-by-tick tracing each time it recurred. Fixed twice in different specific forms (a shelf-escape lateral-shuffle bug, then a general object-width-vs-single-point-routing mismatch), but the underlying category of failure was not eliminated — judged to be a genuine motion-planning problem, not something worth continuing to patch in pygame
- No object rotation is supported — objects can only translate, not turn to angle through a tight passage. Known and deliberately deferred, not attempted this session

### Current state
Transport prototype is functional end-to-end (recruit → lift → navigate → deliver) with a real, working warehouse/shelf layout, but the negotiation/transport phase has an accepted residual deadlock risk and no rotation support — both explicitly parked in favor of moving to Isaac Sim, where real physics/motion planning is the appropriate tool rather than further pygame force-tuning. No RL has been introduced yet; the deterministic nearest-robot assignment is the only decision-making in place so far.

### Next steps (high level)
1. Begin hands-on Isaac Sim setup (scene basics, URDF import) — starting point for proving the underneath-lift anchor mechanism in real physics
2. Once physically proven in Isaac, port the validated pygame design (recruitment, readiness gate, routing) rather than redesigning from scratch
3. RL decision layer (team/slot assignment) still not started — deliberately sequenced after the deterministic mechanical pipeline was proven, per the same lesson learned in the farm project
4. Consider a dedicated `transport.md` component log given the amount of bug/fix history already accumulating in this prototype — not created yet, pending confirmation

### Stretch goals (unscheduled)
- Space-optimization slotting: real warehouses maximize storage by fitting complementary-shaped items into a shared slot and re-slotting (relocating) items over time to improve space utilization, not just fixed one-object-per-slot placement. Current `ShelfSlot` model holds exactly one object regardless of size fit. Noted as a possible future decision problem (well suited to the RL layer) — not in scope for the current mechanical/transport-coordination work

---

## Session 6 — NeuroWare (2026-08-09)

### What was done
- Committed the Session 5 transport prototype to git on the `transport` branch (DEVLOG entry + all prototype files). Two unrelated untracked artifacts from earlier farm-branch RL training work were deliberately left out of the commit, not deleted
- Play-tested the pygame transport prototype live; directly observed the known negotiation stall (documented in Session 5) occurring specifically around the middle shelf region — confirmed as the same already-logged limitation, not a new bug
- Downloaded, installed, and did initial bring-up of Isaac Sim 6.0.0 (standalone workstation build) on this machine, ahead of porting the transport design into real physics
- Ran Isaac Sim's official compatibility checker: confirmed a hard VRAM shortfall on this machine's GPU (well under the stated minimum) while every other requirement (driver, CPU, RAM, storage, OS) passed. Also surfaced an IOMMU-related stability warning (potential memory corruption risk under CUDA on bare-metal Linux with IOMMU enabled) — not resolved this session, would require a BIOS-level change
- First interactive launch attempt, using the default renderer settings, crashed immediately after startup completed. Root cause not provable with certainty (no root access to kernel/driver logs in this environment), but strongly consistent with GPU memory exhaustion during the default renderer's initialization, matching the compatibility checker's finding
- Found and validated a low-VRAM interactive rendering path that avoids the crash entirely, and used it to build a small test scene (lit object, ground plane, camera framing) as a working starting template
- Iteratively tuned the test scene based on direct visual feedback (screenshots) rather than assumption: fixed an initial black-viewport bug, added real directional lighting (the first lighting attempt was ambient-only and produced a flat, edgeless look), added a ground plane so the background isn't empty black, and fixed initial camera framing to start close on the test object
- Moved the test script from inside the Isaac Sim install directory into the git repo (new `isaac/` folder), confirmed it still runs correctly from there — keeps the actual Isaac Sim install (tens of GB) out of git entirely while the project's own scripts stay version-controlled

### What worked
- The low-VRAM rendering path is a confirmed, working solution for this GPU — the interactive viewport stayed well within budget (roughly a quarter of total VRAM) throughout all testing, comfortable headroom for building an actual scene
- Diagnosing the black-viewport issue by reading Isaac Sim's own settings UI/source directly (rather than guessing) resolved it correctly on the first real fix — one piece of SDK documentation elsewhere was stale/misleading and would have led to the wrong conclusion if trusted over the live source

### What did not work
- Isaac Sim's default interactive renderer is not usable on this hardware — confirmed by direct crash, not just the compatibility checker's static warning
- IOMMU stability warning surfaced but intentionally not addressed this session (requires BIOS change + reboot, no instability actually observed yet to justify it)

### Current state
Isaac Sim 6.0.0 is installed and confirmed runnable on this hardware using the low-VRAM rendering path, with a working minimal test scene as a starting template. The default/full-quality renderer does not work on this GPU and should not be used going forward. No actual project content (robot bodies, the coupling mechanism, a real warehouse scene) has been built yet — this session was entirely Isaac Sim bring-up.

### Next steps (high level)
1. Begin building the actual coupling-mechanism test scene (robot bodies, physics, underneath-lift joint), starting from the validated low-VRAM test scene rather than from scratch
2. Revisit the IOMMU warning if real instability shows up later — not urgent otherwise
3. Once physically proven in Isaac, port the validated pygame transport design (recruitment, readiness gate, routing) rather than redesigning from scratch — carried over from Session 5
4. RL decision layer still not started — deliberately sequenced after the mechanical pipeline is proven, per the same lesson learned in the farm project
5. `transport.md` (and/or an Isaac-specific component log, given how much setup/render-configuration history already exists) still proposed, not created — pending confirmation

---

## Session 6 (continued) — NeuroWare (2026-08-09)

### What was done
- Found and reused an existing reference robot model (`gitignore/paper_robot_centered_actuator_v6`, predates this project's visible session history) instead of building a placeholder from scratch — a centered piston lift, a ball-joint contact plate for conforming to angled surfaces, and skid-steer wheels, built to match a reference paper's described mechanism rather than sourced from the paper itself. Fits the underneath-lift coupling concept well
- Imported the robot into Isaac Sim (URDF → USD → PhysX articulation) and let it settle under real gravity onto a ground plane, using the same low-VRAM rendering path validated earlier this session — confirmed the whole import pipeline works end to end on this hardware, staying well within the VRAM budget
- Began wiring up real joint-level control (drive gains, velocity targets for the wheels, with position control for the piston lift planned next) — user is hand-writing this control code directly as a learning exercise, with iterative debugging support rather than the code being written for them
- Got the wheels physically responding to velocity commands under real physics — confirmed by direct observation after fixing a couple of API usage mistakes
- Started into keyboard-driven manual control as the next step; not completed yet, deliberately stopped here
- Worked through and settled the design approach for the coupling mechanism itself: a two-phase plan (rigid physics joint first, to prove the multi-robot coordination/negotiation logic without depending on real holding physics; real contact/friction-based holding second, to actually test whether an object stays on/between the robots during motion) plus a refinement that deliberately-unrealistic object shapes are valid as pure navigation stress tests for phase one, while realistic-but-awkward shapes are reserved for phase two

### What worked
- The existing reference robot model imported cleanly with working physics (rigid bodies, joints) despite some benign warnings (zero-mass intermediate ball-joint links with no geometry of their own, missing joint gain values from the source URDF) — neither was a real blocker
- Manually setting joint drive gains and velocity targets got the wheels genuinely spinning/driving under real physics once the control code's API usage was corrected

### What did not work
- Initial hand-written control code had two parameter-naming mistakes when calling the joint-control API — caught by actually running it and reading the real error each time, not by guessing, and fixed both times
- Keyboard control not yet implemented

### Current state
A drivable robot now exists in Isaac Sim under real physics (imported from the reference model, wheels respond to hardcoded velocity targets), but it is not yet keyboard-controllable, the piston lift and ball joint are untested, and no coupling mechanism (joint-based or friction-based) has been built yet. This session was entirely about getting a controllable robot into the scene — no actual transport/coupling logic has been ported over yet.

### Next steps (high level)
1. Finish keyboard-driven manual control of the wheels
2. Test the piston (position control) and ball joint, confirm the lift mechanism behaves as expected
3. Build the phase-one coupling mechanism (rigid joint between the robot's contact plate and a test object) once the robot is reliably drivable
4. Phase two (real contact/friction-based holding) once phase one proves the coordination logic works
5. Carried over from earlier: RL decision layer still not started; `transport.md` / an Isaac-specific component log still proposed, not created — pending confirmation

---

## Session 7 — NeuroWare (2026-08-10)

### What was done
- Finished keyboard-driven manual control of the wheels (held-key tracking plus a per-frame differential velocity mapping for forward/back and skid-steer turning), hand-written by the user with debugging support rather than written for them
- Root-caused and fixed a real control bug: wheel joint indices were being resolved against the wrong index space (the full joint list, which includes non-driven fixed joints, instead of the drivable DOF list), silently misdirecting velocity commands to the wrong wheels — and in one case onto the piston joint instead of a wheel. Found by printing the robot's actual joint/DOF name lists at runtime rather than guessing, not by reasoning about geometry
- Applied damping to the previously fully-undriven piston and ball-joint stack (no drive at all existed for these before) so they stop flopping freely under gravity, as a secondary stability improvement once the real turning bug above was already fixed
- Established that the robot's piston lift mechanism points upward from the chassis, meaning any payload needs to be elevated with ground clearance for the robot to drive underneath and reach it — full warehouse shelving is not needed for this, just a simple raised static platform
- Built a shelf/stand prototype directly in the Isaac Sim GUI (Create menu) rather than guessing dimensions in code, positioned visually against the live robot, and saved it to a standalone USD file
- Fixed the saved shelf's structure: all pieces had been created as flat siblings of the stage root instead of grouped, so they were reparented under a single new Xform (world positions preserved) via a script-driven fix
- Added static collision (no rigid body — same treatment as the ground plane) to every piece of the shelf, confirmed as the intended behavior (fixed in place, not affected by gravity) before applying

### What worked
- The DOF-index fix fully resolved the erratic/scrambled turning that had been reported during control testing — confirmed as the actual root cause once the real joint/DOF name lists were printed and compared
- Prototyping the shelf visually in the GUI before writing any code was an effective way to dial in placement and size against the robot without guessing numbers blind
- Reparenting the shelf pieces via the `MovePrim` command with world-transform preservation correctly regrouped everything without shifting any positions

### What did not work
- Initial suspicion that bad inertia on the piston/ball-joint stack was causing the erratic turning was a plausible-looking but incorrect theory — the real cause was the joint/DOF indexing bug. The damping fix was still worth keeping as a legitimate secondary improvement, just wasn't the actual fix for the reported symptom
- First attempt to save the manually-built shelf failed — the save dialog defaulted to an Omniverse Nucleus path that isn't reachable in this setup rather than a local disk path. Fixed by explicitly choosing a local path on retry

### Current state
The robot is now fully keyboard-drivable with correct per-wheel differential control and no known control bugs. A static shelf/stand object exists as its own saved USD file (grouped, collision-enabled) but is not yet loaded into the same scene as the robot. No dynamic payload object and no coupling mechanism exist yet.

### Next steps (high level)
1. Bring the shelf into the main robot scene (reference the saved file or transcribe its layout into the script)
2. Empirically confirm the correct shelf height by driving the robot underneath, extending the piston fully, and reading the contact plate's actual world position, rather than calculating it by hand
3. Add a dynamic payload object (rigid body + collision) resting on the shelf
4. Build the phase-one coupling mechanism (rigid joint) once robot, shelf, and payload coexist in one scene
5. Carried over: piston/ball joint still untested for actual lift motion; phase two (friction-based holding); RL decision layer; `transport.md` / an Isaac-specific component log still proposed, not created — pending confirmation

---

## Session 7 (continued) — NeuroWare (2026-08-10)

### What was done
- Confirmed there's no boolean/extrude mesh-modeling tooling available in this Isaac Sim install — custom shapes are composed from separate simple primitives instead, which is sufficient for this project's needs (no cutting/CSG required for a shelf or payload)
- Built a set of payload objects (a cylinder and three cubes) on the shelf using the same hand-in-GUI workflow as the shelf itself, then applied real dynamic rigid-body physics to them via script (collision plus gravity/dynamics, unlike the shelf's static-only treatment), confirmed applied by directly reading the saved file's physics data back
- Merged the hand-built shelf-and-payload layout into the main project script, replacing its previous from-scratch scene setup (which re-imported the robot fresh every run). The script now opens the saved layout directly, which already contains the robot, and adds the keyboard-driven wheel control on top — robot, shelf, and payload all now load together in one place and one script, confirmed running cleanly with no errors

### What worked
- The static-vs-dynamic physics split (collision-only for the shelf, collision plus rigid body for the payload) behaved exactly as intended once correctly applied and verified against the saved file's actual data
- Directly inspecting the saved USD file's real contents (rather than trusting the GUI's apparent state or console output alone) was repeatedly what actually resolved confusion about what had or hadn't been saved

### What did not work
- Misdiagnosed a live, actively-in-use interactive session as hung — based on an external log file showing no new output for several minutes — and force-killed it. The real cause was output buffering (the log looked silent while the session was actually running fine and being used), not an actual hang; this risked losing whatever unsaved work existed in that session at the time. Root-caused afterward; going forward, output is captured unbuffered so silence in the log can be trusted, and a live interactive session is not force-killed on log silence alone without checking first
- Repeated confusion between what was saved to disk versus what only existed in the still-open GUI session, leading to at least one wasted round of trying to add physics to objects that, at that exact moment, genuinely weren't in the saved file yet. Resolved each time by re-checking the actual file directly rather than assuming a save had completed

### Current state
The main project script now opens a hand-built environment (static-collision shelf, four dynamic payload objects) together with the already-embedded, keyboard-drivable robot, confirmed running without errors — the robot can be driven up to the shelf and payload to observe real collision behavior. No coupling mechanism exists yet; this is purely the physical test environment it will be built on top of. The shelf/payload USD file itself lives outside the git repo (not version-controlled) and was built entirely by hand plus one-off physics-fixup scripts, not from a reusable code path.

### Next steps (high level)
1. Empirically confirm the shelf height/fit is actually correct for the robot's piston reach (still not measured — the current shelf was positioned by eye in the GUI, not validated against the piston's extension range)
2. Build the phase-one coupling mechanism (rigid joint) now that robot, shelf, and payload coexist in one scene
3. Test the piston (position control) and ball joint — still untested
4. Decide whether the hand-built shelf/payload layout should be brought into the repo (e.g. as a tracked asset) for reproducibility, since it currently only exists on disk outside git
5. Carried over: phase two (friction-based holding); RL decision layer; `transport.md` / an Isaac-specific component log still proposed, not created — pending confirmation

---

## Session 8 — NeuroWare (2026-08-11)

### What was done
- Made the piston hold position under load ("float") instead of just damping, and remapped the F key from piston-down to a one-shot "frame the whole scene" camera action
- Root-caused and fixed why the platform couldn't reach down far enough to slide under shelf objects — the real fix required editing the robot's physics joint offsets (not the visual mesh), in the shared reference robot asset outside this repo (see note below)
- Root-caused and fixed a robot turning stall that appeared after an earlier chassis mass increase — a PhysX solver-convergence issue, fixed via articulation solver iteration counts, not by increasing drive torque
- Added, then fully removed, a set of invisible "riser" objects that were meant to lift payload objects off the shelf surface for clearance — they ended up being the direct cause of the robot's lift mechanism jamming against the shelf, confirmed by the user's own live testing
- Found and fixed the real cause of "the robot can't pick anything up": the piston's drive force was capped far below what's needed to lift real payload masses. Also found and fixed a second, unrelated cause: a leftover collision proxy on the piston rod (sized for old, pre-fix geometry) was physically colliding with the shelf structure
- Corrected the robot's spawn position/orientation twice — first to face the shelf at all, then to approach from the shelf's short side rather than its long side, based on the shelf's actual corner-leg geometry rather than assumption
- Built a new autonomous single-robot pick-up script (`isaac/pickup_demo.py`) — drives straight to a shelf object and lifts it with no steering needed, triggered by pressing P, then hands off to the same manual keyboard controls as the main script
- Fixed harsh unlit-black shading in the low-VRAM viewport by disabling shadow casting on the scene's directional light and increasing ambient fill lighting
- At the user's request, simplified the shelf (removed the full-width solid floor panels down to just corner legs and thin rails) and retuned the payload objects' size/mass and the piston's lift-force cap so that 3 of the 4 objects are liftable by one robot and one deliberately is not (requires two robots) — the force value needed was found empirically, not by calculation (see below)

### What worked
- The float/position-hold piston, the turning fix, and the pick-up sequence were all confirmed via direct headless verification before being shown live, and then confirmed again live by the user
- Diagnosing collision issues by directly measuring USD collision-geometry bounding boxes (rather than guessing from the visual scene) correctly found both the riser jam and the leftover-collision-proxy jam
- Removing the shelf's solid floor panels (user's own change, done live in the GUI) left no orphaned collision or dangling references — verified directly afterward

### What did not work
- Static physics (mass × gravity) badly underestimated the real force needed to lift objects in practice — a value that should have worked by the formula left a 9.8kg object barely able to lift at all. The working force value was found by direct empirical testing (try a value, measure whether the object actually lifts) rather than trusted from calculation. This is worth remembering for any future force/weight tuning in this project — don't trust the static formula alone
- The robot's first two spawn placements were both wrong for reaching the shelf object — first facing directly away from any clear approach path, then facing the shelf's long side instead of its short side. Both were corrected based on directly measured shelf corner-leg positions once the mistake was pointed out

### Current state
A single robot can now reliably drive to a shelf object and pick it up on its own (autonomous, P-triggered), with the lift force intentionally capped so 3 of the 4 current payload objects are liftable solo and the heaviest (18kg) is not. No multi-robot coupling or coordination exists yet — this session was entirely about proving one robot's mechanics work correctly, which was the standing precondition from Session 7. The functional fixes to the robot's own joint geometry live in `gitignore/paper_robot_centered_actuator_v6/...` (the shared reference robot asset), which is excluded from git via `.gitignore` — those fixes are not currently version-controlled anywhere.

### Next steps (high level)
1. Build the phase-one coupling mechanism (rigid joint, single robot to object) — still not started, carried over from Session 7
2. Use the newly-created 2-robot-required object (18kg) as the first real test case for multi-robot cooperative lift, once coupling exists
3. Port the validated pygame transport-negotiation design (recruitment, readiness gate, routing) into Isaac Sim — carried over from Session 5, still not begun
4. RL decision layer still not started — deliberately deferred until the mechanical/coordination layer is proven, per the same lesson learned in the farm project
5. Decide what to do about the un-version-controlled robot asset fixes (outside git via `.gitignore`) before they're lost or diverge further
6. `transport.md` / an Isaac-specific component log still proposed, not created — this session in particular had a lot of detailed bug/fix history that belongs in a component log rather than here; pending confirmation

---

## Session 8 (continued) — NeuroWare (2026-08-11)

### What was done
- Created two new component logs, `logs/isaac_robot.md` and `logs/isaac_environment.md`, splitting robot-mechanics history from shelf/scene history — addresses the "pending confirmation" item from earlier in this session
- Duplicated the robot and shelf to build a second full setup (`Robot2`, `Shelf2`) in a new scene file (`isaac/test/two_robot_two_shelf.usd`), exported separately so the original single-robot file was left untouched
- Built a full two-robot cooperative lift-and-carry sequence (`isaac/two_robot_pickup_demo.py`, P-triggered, same convention as the single-robot script): both robots straddle a payload object from opposite ends, lift it together, turn to face a delivery direction, and drive it to a second shelf
- Diagnosed and fixed a chain of issues that each masked the next one: the two robots colliding with each other while turning (fixed by increasing the payload object's length for more clearance, and by staggering the turns instead of doing them simultaneously), the carried object falling off during the drive (traced to an instant full-speed command from a standstill, not gradual drift as first suspected — fixed with a speed ramp), the plate tilting/not holding orientation under load (the ball-joint roll/pitch/yaw drives were damping-only, fixed with real position control), and a final-position bug that looked like a physics problem but was a plain control-logic bug (a symmetric stop-distance check that systematically undershot the target — see the `isaac_robot.md` component log for the exact mechanism)
- Along the way, also fixed a real stdout-buffering bug that was silently hiding the script's own diagnostic print output from the redirected log file

### What worked
- The full two-robot sequence now runs correctly end to end: pickup, synchronized lift, staggered exact-angle turn, straight-line drive, arriving within centimeters of the intended target — confirmed via the script's own logged world-coordinate output, not just visual inspection
- Turning to an exact, analytically-known angle and then driving with zero steering correction was far more reliable than continuously correcting steering toward a live bearing calculation — small residual heading error under continuous correction was enough to make two robots drift out of sync with each other
- Getting precise, printed ground-truth numbers (object mass, shelf gap coordinates, final robot/plate positions) repeatedly resolved disagreements that screenshots and visual inspection alone could not

### What did not work
- Several early hypotheses were wrong and cost real time before being corrected: assumed the object-falling problem was gradual path divergence (it was an instant-speed jolt), assumed a "wrong stopping location" was downstream of the missing rigid-coupling mechanism (it was an unrelated, plain control-logic bug), and briefly misjudged a coordinate discrepancy as a possible bug before realizing it was a local-vs-world reference frame mismatch
- A `replace_all` text edit meant to fix one joint's force value accidentally corrupted a different joint's already-correct value via an unintended substring match — caught and fixed, but worth remembering as a real risk of blind `replace_all` on numeric values

### Current state
Two robots can now reliably cooperate to lift an object too heavy for either one alone, turn together, and carry it to a second shelf, landing within centimeters of the intended spot with the object still on the plates the whole way — confirmed via direct logged coordinates, not just visual inspection. No rigid coupling mechanism was needed to achieve this; careful motion control (exact-angle turns, zero-correction straight driving, speed ramping, tight tolerances) was sufficient for this specific pickup→turn→carry→arrive case. Whether that holds for more complex routes (turning mid-path, obstacles) is untested.

### Next steps (high level)
1. Test whether the current motion-control-only approach holds up for more complex delivery paths, or whether the rigid coupling mechanism (still not built) becomes necessary once routes aren't a single clean turn-then-straight-line
2. Decide whether to unload/place the object at the destination (current sequence stops with it still lifted on the plates — no lower/release step exists yet)
3. Live-obstacle reaction (user can place something in the robots' path mid-run and have them re-route) — explicitly deferred until after two-robot coordination was working, which it now is
4. Carried over: phase-two friction-based holding; port the pygame transport-negotiation design; RL decision layer; decide what to do about the un-version-controlled robot asset fixes
