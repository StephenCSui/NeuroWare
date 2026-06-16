# NeuroWare Swarm Simulation

A 2D swarm robotics simulator built with Pygame. Agents cooperatively push items from a loading zone to a storage zone, then dispatch them on demand to a goal zone. All operational decisions are made by agents coordinating peer-to-peer — captains only broadcast item existence and destination.

---

## Concept

NeuroWare is a simulation of decentralised warehouse robotics. It is grounded in Yadav et al. "Maintaining the Level of a Payload carried by Multi-Robot System on Irregular Surface" (arXiv:2512.16024v2) and extended toward a full swarm coordination model.

Key design principles:
- **No central controller.** Captains are memory/broadcast nodes only. They never assign agents, choose slots, or set speed.
- **Agents talk to agents.** Which slots to fill, when a side is ready to push, and how fast to move are all decided by agents observing the item's `claimed_slots` and their own neighbours.
- **Physical push model.** Items have mass and collision. Agents push items by positioning on the opposite face and applying force. A single agent cannot push at 45° — diagonal movement requires at least one agent per axis.

---

## Project structure

```
NeuroWare/
├── simulation/
│   ├── config.py       — all constants and item type definitions
│   ├── item.py         — Item class: physics, slots, collision
│   ├── agent.py        — Agent class: state machine, gossip, push logic
│   ├── captain.py      — Captain class: zone scan, memory, pings
│   ├── zone.py         — Zone class: loading / storing / goal zones
│   └── main.py         — Pygame loop, event handling, draw order
├── logs/
│   ├── DEVLOG.md       — session-by-session project log
│   ├── agent.md        — agent component log
│   ├── captain.md      — captain component log
│   └── environment.md  — environment/simulation component log
└── paper_robot_centered_actuator_v6/
    — Reference hardware model (URDF, SDF, meshes)
```

---

## Running the simulation

```bash
cd simulation
python3 main.py
```

Requires Python 3.10+ and Pygame 2.x:

```bash
pip install pygame
```

---

## Controls

| Input | Action |
|---|---|
| Left click in loading zone | Register placement position |
| 1 / 2 / 3 | Place item of type S / M / L at registered position |
| G then 1 / 2 / 3 | Request delivery of a stored S / M / L item to goal zone |
| Space | Pause / unpause |
| Q or Esc | Quit |

---

## Item types

| Key | Label | Agents per side | Size | Notes |
|---|---|---|---|---|
| 1 | S | 1 | 20 px | Single agent can push any axis solo |
| 2 | M | 2 | 40 px | Two agents required on each needed push side |
| 3 | L | 3 | 58 px | Three agents required on each needed push side |

A diagonal move (item not axis-aligned with destination) requires at least one agent per axis. The slot system enforces this structurally — there is no explicit diagonal rule in code.

---

## Architecture

### Zones

| Zone | Position | Role |
|---|---|---|
| Loading | Top-left | User drops items here |
| Storing | Right | Agents push items here for storage |
| Goal | Bottom-left | Final delivery destination |
| Captain A zone | Left half | Captain A's awareness region |
| Captain B zone | Right half | Captain B's awareness region |

### Captains

Two captains, one per half of the screen. Each captain:
- Scans its zone every `CAPTAIN_PING_INTERVAL` seconds for items without a destination
- Sets `item.destination` and `item.phase` on new items (`to_store`)
- Syncs newly seen items to the other captain after a `SYNC_DELAY` (simulates mesh propagation latency)
- Dispatches stored items to the goal zone on user request (`to_goal`)

Captains never touch slots, speed, or timing. Everything beyond destination + phase is agent territory.

### Agents

Each agent maintains:
- `known_tasks = {item_id: item}` — items the agent knows about, shared by reference via gossip
- `neighbours` — agents within `COMM_RANGE` (rebuilt each frame)
- `push_slot = {"item": Item, "side": str, "idx": int}` — the slot this agent currently owns

Each frame an agent:
1. Receives item references from captains and neighbours (`absorb_captain_info`, gossip from `sense_items`)
2. Prunes stale tasks (`cleanup_known_tasks`)
3. If idle, self-selects a task and slot (`evaluate_known_tasks`)
4. Executes its current state

#### Agent state machine

```
idle
  └─► seek_push_slot        (navigate to claimed slot)
        └─► waiting_for_co_pusher  (hold position, wait for side to be ready)
              └─► pushing          (apply force each frame)
                    └─► idle       (on delivery or break-off)
```

**Per-side independence:** Each push side runs this state machine independently. The LEFT-side agents can start pushing while the TOP-side agents are still filling their slots. This enables diagonal motion to begin before all sides are complete.

**Break-off:** When an agent's axis is no longer needed (item has reached destination on that axis), the agent calls `_release_and_reset()` and returns to idle.

**Displacement re-seek:** If a waiting agent is pushed >2 px from its slot (e.g., by the item moving), it re-enters `seek_push_slot` to re-anchor.

#### Speed logic

Speed in `seek_push_slot` is determined by what the agent can observe about the perpendicular side:

| Condition | Speed |
|---|---|
| No agent claimed perp side | `AGENT_SPEED × 2.0` (sprint — solo axis is only option) |
| Nearby neighbour seeking perp side | `AGENT_SPEED × 0.7` (yield — let them anchor) |
| Perp side claimed, no nearby seeker | `AGENT_SPEED × 1.0` (normal) |

### Push physics

Each pushing agent calls `item.apply_push(side, PUSH_FORCE)` per frame. At the end of the frame:
1. `item.physics_update()` integrates accumulated force into velocity
2. Velocity is capped at `PUSH_SPEED_MAX`
3. When no force is applied, velocity is damped by `ITEM_DAMPING`
4. Item is clamped to window bounds

Item-item AABB collision is resolved after all items have moved.

---

## Key constants (config.py)

| Constant | Value | Description |
|---|---|---|
| `AGENT_SPEED` | 2.5 | Base agent movement speed (px/frame) |
| `VISION_RANGE` | 200 | Radius for item sensing (360°) |
| `COMM_RANGE` | 250 | Radius for neighbour sensing and gossip |
| `PUSH_FORCE` | 1.5 | Velocity contribution per pushing agent per frame |
| `ITEM_DAMPING` | 0.78 | Velocity multiplier per frame when not being pushed |
| `PUSH_SLOT_OFFSET` | 24 | Distance from item edge to agent contact point |
| `SLOT_SPREAD` | 15 | Spacing between agents sharing a side (M/L types) |
| `PUSH_SPEED_MAX` | 3.5 | Maximum item speed (px/frame) |
| `DEST_THRESHOLD` | 30 | Distance at which item is considered arrived |

---

## Logs

Session-by-session history is in `logs/DEVLOG.md`. Detailed component-level history (what worked, what failed, key decisions, things worth exploring) is in the component logs:

- `logs/agent.md` — agent state machine, gossip protocol, push logic
- `logs/captain.md` — captain zone management, pings, sync
- `logs/environment.md` — item types, physics, main loop, zones
