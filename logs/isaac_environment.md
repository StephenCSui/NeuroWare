# Isaac Sim Environment — Component Log

Detailed history of the shelf/scene/world-layout side of the Isaac Sim project: `isaac/test/shelves1_with_collision.usd` and `isaac/test/two_robot_two_shelf.usd`, the hand-built shelf structure, payload objects, and world coordinate layout. Robot mechanics/control history is in `isaac_robot.md`.

---

## Session 8 — NeuroWare (2026-08-11)

### Current state
`shelves1_with_collision.usd`: one shelf (3 tiers, corner legs + thin rails only, solid front/back panels removed), 4 payload objects, single robot. `two_robot_two_shelf.usd`: a second shelf (`Shelf2`, duplicated + shifted) and second robot (`Robot2`) added on top of that, used for the two-robot cooperative-carry work. Both files verified loading and settling cleanly.

### What was done
- **Invisible riser objects — added, then fully removed**: initially added small invisible static boxes under each payload object (5cm clearance) so the robot's platform could slide underneath. These turned out to be the direct cause of a later collision jam (the platform's rod colliding with the riser's own collision volume) and were fully removed at the user's request once a different fix (see `isaac_robot.md`'s collision-cylinder fix) made them unnecessary.
- **Shelf panel removal**: the user manually deleted the 6 full-width, 10cm-thick solid floor panels (front/back panel per tier — `Cube`, `Cube_02`, `Cube_08`, `Cube_11`, `Cube_12`, `Cube_15`) directly in the GUI, leaving only the thin lip/rail pieces and 4 corner legs per tier. Verified afterward: no orphaned collision, no dangling `FilteredPairsAPI` relationships pointing at the deleted prims.
- **Shelf gap discovery**: each tier's remaining rail layout has a genuine ~0.1m-wide **construction gap with no floor material at all** (`y[3.2,3.3]` on shelf 2, `y[-0.8,-0.7]` on shelf 1 pre-shift) between the "lip" rail and "mid-rail" piece. This is real hand-built-scene geometry, not a bug — and became load-bearing for the whole two-robot approach (the robots' pickup/delivery Y is deliberately centered on this gap so nothing needs to clear a solid panel vertically).
- **Payload object retuning** (`Cube_01`, later the "needs 2 robots" test object): resized multiple times to balance two goals — enough physical separation between two robots picking it up from either end (to avoid a chassis collision while turning), and a target mass in the "too heavy for 1 robot (200N cap), liftable by 2" range.
  - Height/depth trimmed first for an unrelated mass target (`0.06` height, `0.28` depth after a couple of iterations), landing around 18kg.
  - Length increased 0.6m → 1.0m → 0.8m specifically for robot-to-robot clearance during the two-robot pickup (0.6m nose-to-nose gave only ~0.09m of chassis clearance, which rammed the two robots together during a simultaneous turn). 1.0m clipped a shelf corner leg; 0.8m was the value that both cleared the leg and gave workable clearance.
  - Final dimensions: 0.8 (L) × 0.28 (D) × 0.06 (H) → **13.44kg** (mass = L×D×H×1000, density 1000 kg/m³, not independently authored — confirmed by resolved-mass computation matching the volume formula exactly at every size tested).
  - Mass note: since mass isn't independently set, changing any one dimension moves mass too — this was flagged live each time (e.g. lengthening for clearance also raised mass, which then needed depth/height trimmed back down).
- **Two-robot/two-shelf scene built** (`two_robot_two_shelf.usd`, exported from `shelves1_with_collision.usd` via `Sdf.CopySpec`, not overwriting the source file):
  - `Robot2`: deep-copied from `Robot`'s prim spec (carries the same external-asset reference, giving a second independent instance of the same robot).
  - `Shelf2`: deep-copied from `Shelf`'s local prim tree (no external reference, so a straight `Sdf.CopySpec`), then given its own `xformOp:translate=(0,4,0)` to shift it away from shelf 1.
  - Payload objects were **not** duplicated onto shelf 2 — it starts empty, since the point is delivering an object there.
- **World coordinate frame confusion, resolved**: `Robot`/`Robot2`'s own Xform retains a static spawn-position offset (`translate.y=-0.747` for both). `base_link` (and everything under it) is a child of that Xform, so Isaac Sim's Property panel shows `base_link`'s pose **local** to that parent, not true world position. `local_y = world_y + 0.747`. Caused real confusion mid-session (a world-Y of 3.25 reads as ~4.0 in the panel) before being pinned down by directly comparing `Shelf2`'s own Translate value (`Y=4.0`, the shift amount) against its actual measured geometric center (`Y=3.25`, since the copied child geometry itself carries a baked-in -0.75 offset from how shelf 1 was originally hand-placed). **Rule of thumb going forward: always compute/report world-frame coordinates via `get_world_poses()`, and if comparing to what's shown in the GUI Property panel, expect a `+0.747`-ish offset for anything under Robot/Robot2, and expect any Xform-group's own Translate value to differ from its child geometry's true center by whatever offset was originally baked into that geometry.**

### What worked
- `Sdf.CopySpec` cleanly duplicates both referenced prims (`Robot`, carrying its external-asset reference along) and locally-authored prim trees (`Shelf`, no references) within the same layer — confirmed via headless settle-and-verify each time.
- Exporting to a new file (`stage.GetRootLayer().Export(new_path)`) rather than saving in place reliably kept `shelves1_with_collision.usd` untouched while building `two_robot_two_shelf.usd` on top of it.
- Directly measuring collision-geometry bounding boxes (rather than reasoning from memory or screenshots) was the only reliable way to resolve the shelf-gap-location and world/local-coordinate confusions — both times a guess based on "should be" reasoning was wrong.

### What did not work
- Assuming a duplicated Xform-group's own Translate value directly tells you its geometric center — it doesn't, if the child geometry itself carries a baked-in local offset from how it was originally authored (which both shelf copies do, from the original hand-built scene).
- Diagnosing "why did it stop in the wrong place" from screenshots and visual impressions alone, more than once — wide-lens camera angles at this small scale made height/position differences visually misleading. Only got a real answer once actual printed world coordinates were captured from the running script.

### Dependencies
- `two_robot_two_shelf.usd` was exported from `shelves1_with_collision.usd` at a specific point in time — any *later* changes to `shelves1_with_collision.usd` (e.g. further shelf-1 edits) will **not** propagate to `two_robot_two_shelf.usd` automatically, since it's an independent exported copy, not a reference.
- Robot spawn positions/orientations for both files are hand-set world-space values in the script/file directly (not computed from shelf geometry at runtime) — if shelf geometry changes again, these will need re-deriving.

### [WORTH EXPLORING]
- Shelf 2 currently has no payload objects and no defined "destination slot" structure — it's just an empty duplicate shelf. If multi-object reorganization (moving several objects between shelves) becomes the active work, this will need real slot-tracking, not just an empty target.
- The gap-in-the-rail-layout that both shelves' pickup/delivery points rely on is a side effect of the original hand-built geometry, not a designed feature — worth deciding whether to formalize it (e.g. explicitly document/measure it as "the access gap" for every tier) or redesign the shelf with an intentional access point once shelf design work resumes.

---

## Session 9 — NeuroWare (2026-08-12)

### Current state
All 4 shelf-1 payload objects now have confirmed real masses/positions (queried directly from the running scene, not assumed) and are all actively used by the multi-object delivery routine. A stray manually-created test object (`/Cube_03`) is sitting in `two_robot_two_shelf.usd` itself, not part of the intended scene — pending user decision on removal.

### What was done
- **Real payload data queried directly** from `two_robot_two_shelf.usd` before designing the multi-object delivery task split (previous sessions only had confirmed numbers for `Cube` and `Cube_01`):

  | Object | Mass | World X (shelf-1 row) | Solo-liftable? |
  |---|---|---|---|
  | Cube_02 | 2.0 kg | 0.162 | yes |
  | Cylinder | 7.07 kg | 0.531 | yes |
  | Cube | 9.8 kg | 0.836 | yes (tuned to the solo-lift ceiling in Session 8) |
  | Cube_01 | 13.44 kg | -0.509 | no — needs both robots |

  Confirmed by directly playing physics for a few ticks and reading `RigidPrim.get_masses()` / `get_world_poses()`, not computed from scale values alone this time (though the resolved masses do match the established `L×W×H×1000` density formula from Session 8).
- **Delivery placement convention established**: each solo object is delivered to shelf 2 at the same X it had on shelf 1 (only Y changes, to `DELIVERY_Y=3.25`) — the original shelf-1 spacing between all 4 objects was already collision-free, so mirroring it onto shelf 2 avoids having to work out new drop slots from scratch.
- **Found a stray object baked into the scene file**: `/Cube_03`, a top-level `Mesh` prim (not one of the four known payload objects, and not under `/World/Shelf`'s structural rail hierarchy either), sitting at world position roughly (0.72, 1.33, 0.57) — directly in the open corridor between shelf 1 and shelf 2 that every carry leg drives through. Has no `PhysicsCollisionAPI` (`collision=False`). Confirmed present in the actual file on disk (not just a runtime artifact) by opening `two_robot_two_shelf.usd` fresh and checking `stage.GetPrimAtPath('/Cube_03')` directly — it exists, with real authored `xformOp:translate`. The file's mtime lines up with a live GUI session earlier in this session, and Isaac Sim's GUI auto-names newly created Cube primitives sequentially (`/Cube`, `/Cube_01`, `/Cube_02` already existed at the time, so a new one becomes `/Cube_03`) — near-certain this is a cube the user created by hand in the GUI while live-testing an obstacle, which then got auto-saved into the shared file rather than staying session-local.
- This explains a run of confusing results in `isaac_robot.md`'s Session 9 entry: a "clean" headless obstacle-detour test (no obstacle placed by anyone) still detected something at the same spot, every single time, because the scene it was opening was never actually clean — `/Cube_03` was already there.

### What worked
- Directly querying `stage.GetPrimAtPath(...)` against the actual file on disk (not the live in-memory session) was what finally settled whether `/Cube_03` was a real, persisted part of the scene or just something transient — same "check the actual saved file, don't trust the live session's apparent state" lesson from Session 7, recurring in a new form.

### What did not work
- Assuming a "clean" scene load meant a clean scene, without checking — cost a full round of misdiagnosis (chased a self-occlusion camera-geometry hypothesis) before the stray object was found. The scene file is not, in fact, guaranteed clean just because no obstacle was intentionally placed for a given test run.

### Key decisions
- Real object masses/positions are queried live from the running scene for any future task design in this file, not computed or assumed from stored scale values — this was already the established convention (Session 8) but is now the basis for an actual multi-object task split, not just a single-object sanity check.

### Dependencies
- `two_robot_pickup_demo.py`'s `run_full_delivery()` (see `isaac_robot.md`) directly depends on the 4 masses/positions recorded above being accurate for its solo-vs-duo task assignment. If shelf-1 payload objects are ever resized or repositioned again, that assignment needs re-deriving, not assumed to still hold.
- `two_robot_two_shelf.usd` currently contains `/Cube_03`, an unintended manually-created object with no corresponding entry in any script. Any future "fresh scene" assumption for this file is not actually safe until this is either removed or explicitly accounted for.

### [POTENTIAL FIX]
- Remove `/Cube_03` from `two_robot_two_shelf.usd` to restore a genuinely clean baseline scene — proposed to the user, not yet actioned as of this session's end.
- Going forward, obstacle testing should use the scripted `spawn_obstacle()` keybind (`O`/`I`, see `isaac_robot.md`) instead of manually creating geometry in the GUI during a live session on this shared file, specifically to avoid a repeat of this exact issue.

### [WORTH EXPLORING]
- No general safeguard exists against a live GUI session silently persisting ad-hoc changes into a shared scene file that scripts assume is a clean, known-good baseline. Worth considering whether test/demo scene files should be opened read-only, or from a scratch copy, during any live session where manual GUI edits might happen.

---

## Session 15 — NeuroWare (2026-08-14)

### Current state
Full ground-truth geometry for `two_robot_two_shelf.usd`'s shelf structure and shelf-1 payload objects independently queried and recorded below (headless `pxr` query against the live file, not derived from prior log narrative). A real, confirmed bug found the same way: the root-level `/Cube_03` obstacle prim has no physics collision in this scene.

### What was done
- Queried real size/mass/position for all 4 shelf-1 payload objects directly from the running scene (mass read via `RigidPrim.get_masses()` after a few physics ticks, not computed/assumed):

  | Object | Size L×W×H (m) | Mass | World pos (x,y,z) | Solo-liftable? |
  |---|---|---|---|---|
  | Cube | 0.20×0.35×0.14 | 9.80 kg | (0.835, -0.747, 0.325) | yes (tuned to the piston force ceiling) |
  | Cube_01 | 0.80×0.28×0.06 | 13.44 kg | (-0.509, -0.747, 0.285) | no — needs both robots |
  | Cube_02 | 0.10×0.40×0.05 | 2.00 kg | (0.162, -0.747, 0.280) | yes |
  | Cylinder | ⌀0.30 × 0.10 | 7.07 kg | (0.531, -0.751, 0.305) | yes |

  All masses match L×W×H×1000 density (cylinder: πr²h×1000) exactly, confirming the density-formula convention established in Session 8 still holds live.
- Queried the real shelf-1 rail structure under `/World/Shelf`: two rail bars, one at world Y=[-1.0,-0.8], one at Y=[-0.7,-0.5], leaving a bare **0.1m gap at Y=(-0.8,-0.7)** with no material at all — every payload object above is centered at Y≈-0.75, i.e. dead center in that gap. Four corner legs (`Cube_04`-`07`), 0.08×0.08m footprint, 1.0m tall, all with real collision. Shelf 2 (`/World/Shelf2`) confirmed identical, shifted +4m in Y (gap at Y=(3.2,3.3)).
- **Confirmed a real bug, not from log narrative**: the root-level `/Cube_03` obstacle prim (world pos (0.725, 1.328, 0.574), the same object driven around during this project's obstacle-avoidance work) has `UsdPhysics.CollisionAPI: False` in `two_robot_two_shelf.usd`. Directly compared against the proven reference world `isaac/legacy/test/robot1_cube03_test.usd`'s own `/Cube_03` — identical position/size, but `CollisionAPI: True` there. The collision was lost somewhere when this object was copied/recreated into the two-shelf scene; it was never re-applied. This means the robot can currently drive straight through Cube_03's real-world location with zero physical resistance, independent of whatever the `/scan`-based avoidance logic decides.
- This also falsifies a comment inside `nav2_bridge_robot1.py`'s AABB touch-trigger code, which asserts both worlds' Cube_03 are equally collision-less ("matching the earlier single-robot+obstacle test's own collision-less prop") — that claim was never actually checked against the source file and is wrong.

### What worked
- Querying real USD prim data directly (headless Isaac boot + `pxr`), rather than trusting the existing log narrative, is what caught the Cube_03 collision discrepancy — the rest of the geometry narrative (object sizes/masses, the shelf's 0.1m access gap) checked out exactly against live data.

### What did not work
- N/A — this was a read-only verification pass, no live control/navigation was exercised.

### Key decisions
- None this session on the environment side — the collision fix itself was deliberately deferred by the user to a future session, not applied yet.

### Dependencies
- The recorded masses/positions above are for `two_robot_two_shelf.usd` specifically, current as of this session — if payload objects are ever resized/repositioned again (as happened repeatedly in Sessions 8-9), this table needs re-deriving, not assumed to still hold.
- `isaac/legacy/test/robot1_cube03_test.usd` (moved this session from `isaac/test/`, see `isaac_robot.md`) is the reference copy this session's Cube_03 comparison was made against — if it's ever deleted, that comparison can no longer be independently re-verified.

### [POTENTIAL FIX]
- Apply `UsdPhysics.CollisionAPI` to `/Cube_03` in `two_robot_two_shelf.usd` (matching the reference world), then live-test whether `/scan` (the depth-camera-synthesized laser scan) actually reports it as an obstacle before/after — collision alone doesn't guarantee the camera-based detection path works, that's still unverified. Explicitly deferred by the user this session; revisit next.

### [WORTH EXPLORING]
- Whether the missing collision is the *whole* explanation for "the robot completely ignores the obstacle," or whether the `/scan`-based detection also has a real gap on top of it — not yet live-tested either way.

---

## Session 16 — NeuroWare (2026-08-15)

### What was done
- **Relayout for debugging clarity, not a design change**: moved the deliberate test obstacle (`/Cube_03`) and `Shelf2` out to a clean "+3 / +6 from shelf 1" convention (world Y, using shelf 1's own payload/gap center as the reference point — the same anchor already used for the delivery-mirroring convention). Shelf 2's Xform translate went from `(0,4,0)` to `(0,6,0)`; `Cube_03`'s world Y-center moved from ≈1.328 to 2.25. Shelf 1 itself and its corner legs were untouched. Reason: shelf 1's own structure (its corner leg `/World/Shelf/Cube_04` specifically) sits close enough to `Cube_03`'s old position that debugging kept conflating "shelf 1's own geometry" with "the deliberately-placed test obstacle" — this makes them unambiguous.
- Also updated `nav2_bridge_robot1.py`'s hardcoded `CUBE03_WORLD_BBOX` constant to the new position (re-queried via the scene-dump mechanism's `world_bbox=` field after the move, not hand-computed).
- **User-driven shelf gap widening, replicated onto shelf 2**: the user manually widened shelf 1's own rail gap live in the Isaac Sim GUI — each rail bar (`Cube_01`, `Cube_03` under `/World/Shelf`) narrowed from 0.20m to 0.15m wide, opening the gap between them from 0.10m to 0.20m — and saved. Replicated the identical structure onto shelf 2: deleted the existing `/World/Shelf2` subtree entirely and rebuilt it via `Sdf.CopySpec(/World/Shelf → /World/Shelf2)` (the same mechanism `isaac_environment.md`'s Session 2-era notes record was used to build shelf 2 originally), then re-applied the `+6` Y offset as a fresh `Translate` op (the source `/World/Shelf` Xform has no translate op of its own, so this couldn't just be copied — had to be added explicitly). Verified via a fresh reopen (not same-session trust): shelf 2's rails now read Y=[5.000,5.150] and Y=[5.350,5.500] (0.15m each, 0.2m gap between), matching shelf 1's new dimensions exactly, with the corner legs and Robot2's parked state untouched.
- **Root-caused and fixed a real, previously-unexplained bug causing every GUI-mode session to eventually crash**: a live GUI save (during the shelf-editing session above) baked the runtime-only `/World/Ros2NavGraph` OmniGraph node permanently into the USD file. Every subsequent headless-or-GUI boot of `nav2_bridge_robot1.py` tries to create a *new* graph at that same path on startup and crashes immediately (`OmniGraphError: Failed to wrap graph in node ... A graph already exists at this path`) — this was the real cause of essentially all of today's GUI-mode instability, not general flakiness as assumed earlier in the session. Fixed by removing the stale `OmniGraph`-type prim from the file (scanned for any others of the same type while at it, found only the one) and re-saving; verified with a clean GUI boot immediately after.

### What worked
- Verifying the replicated shelf 2 structure via a genuinely fresh stage reopen rather than trusting the same Python session that made the edit — same discipline as every other USD edit this project has made.
- Actually reading the GUI crash's real Python traceback instead of continuing to treat it as unexplained instability — the fix was a two-line prim removal once the actual cause was visible in the log.

### What did not work
- N/A this session on the environment side (no failed approaches, both changes worked on the first attempt once diagnosed).

### Current state
`two_robot_two_shelf.usd` now has: shelf 1 (user-widened gap, corner legs unchanged) mirrored cleanly onto shelf 2 at the established `+6` offset; the deliberate test obstacle (`Cube_03`) at a clean `+3` offset, no longer geometrically ambiguous with shelf 1's own structure; and no stale OmniGraph prim baked in (GUI boots cleanly now). Several timestamped `.bak_*` backups of the file accumulated in `isaac/test/` across this session's edits — not cleaned up, kept as a safety trail.

### Dependencies
- `nav2_bridge_robot1.py`'s `CUBE03_WORLD_BBOX` constant must be kept in sync with `Cube_03`'s actual position by hand — it's a hardcoded, re-queried-not-computed value; if the obstacle is ever repositioned again, this needs updating the same way (query the scene dump's `world_bbox=` field, don't guess an offset).
- The "+3/+6 from shelf 1" convention is a *world*-frame convention specifically; the ROS/controller-facing "odom" frame that `shelf_transfer_task.py` and `rotate_drive_controller.py` actually operate in is a rotated/offset transform of this (confirmed live this session: robot spawn yaw ≈179.7°, meaning world and odom are close to a 180° rotation of each other around a non-origin spawn point) — don't assume world-frame Y deltas translate directly into odom-frame Y deltas of the same sign/magnitude without checking, as already flagged in earlier sessions' notes on this exact confusion.

### [WORTH EXPLORING]
- Whether any *other* `.usd` file in this project (not just `two_robot_two_shelf.usd`) has ever had a live OmniGraph node saved into it the same way — worth a quick scan if GUI-mode crashes recur on a different scene file.
