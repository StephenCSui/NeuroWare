# Grid / farm layout — drives window size (template-based layout, not a repeating pattern)
CELL_SIZE = 45
GRID_COLS = 10
GRID_ROWS = 8

WINDOW_WIDTH  = GRID_COLS * CELL_SIZE
WINDOW_HEIGHT = GRID_ROWS * CELL_SIZE
FPS           = 30

AGENT_SIZE  = 22
AGENT_SPEED = 1.2      # deliberately slow — leaves room for a future AI layer to show measurable improvement
NUM_AGENTS  = 2

BUSY_CHANCE   = 0.003
BUSY_DURATION = 4.0

# Farm layout regions — (col, row, width, height), all in cells. Base fill is
# "walkway" (pathway); regions stamp over it in this order: water -> plant ->
# harvest_box. Compact 6-cell bed with a single-cell harvest drop-off and a
# water zone in the opposite corner — walkway fills everything else, which
# gives >=2-cell clearance on every side of the bed for free (no deadspace or
# one-way lanes needed at this scale — those existed only to manage 5-agent
# bridge contention, which doesn't apply with 2 agents and an open layout).
HARVEST_BOX  = (0, 0)          # single cell, col/row
WATER_REGION = (7, 0, 2, 2)
PLANT_REGION = (3, 2, 3, 2)    # 3x2 = 6 plant cells

# Cell state -> task thresholds (evaluated against KNOWN state, not true state)
MOISTURE_LOW_THRESHOLD    = 30.0
WEED_HIGH_THRESHOLD       = 60.0
STALENESS_THRESHOLD_TICKS = 300     # ~10s @30FPS since last physical visit

# True-state drift per second (stochastic, see Cell.tick). Tuned against the
# 30-day/300s season: a fully-neglected cell dies of dehydration in ~20 days
# (well inside the 28-day maturity window), but realistic periodic watering
# comfortably survives to harvest. The original 0.9 was tuned for an
# open-ended maintenance loop with no death consequence -- ~10x too fast
# once death was introduced (verified: caused 4-6/6 cells dead every run).
MOISTURE_DECAY_BASE = 0.5   # jittered 0.5x-1.5x per tick
WEED_SPROUT_CHANCE  = 0.005 # per-second probability of a sprout event per plant cell
WEED_SPROUT_MIN     = 15.0
WEED_SPROUT_MAX     = 40.0

# Task effects/durations (seconds) — kept slow, matches AGENT_SPEED
WATER_REFILL_AMOUNT = 70.0
TASK_DURATIONS = {
    "monitor":         1.0,
    "weed":            6.0,
    "water":           6.0,
    "obtain_water":    1.0,
    "harvest":         6.0,   # hand-picking a ready plant, matches weed/water labor time
    "deliver_harvest": 1.0,   # quick drop-off at the harvest box, matches monitor/obtain_water
}

# Claim safety
CLAIM_TIMEOUT_TICKS = 300   # ~10s @30FPS; auto-frees an abandoned/stuck claim

# Movement safety — if a cached path step stays blocked by another agent this
# long, force a reroute around current traffic instead of waiting forever
# (prevents two agents deadlocking on the same contested cell).
STUCK_REROUTE_FRAMES = 20

# Day cycle — reuses the same tick clock everything else runs on, no separate
# time system. A "day" is just this many ticks (~10s @30FPS, matching
# STALENESS_THRESHOLD_TICKS's granularity). 30-day season ~= 9000 ticks (~5 min).
DAY_LENGTH_TICKS  = 300
DAYS_TO_MATURE    = 28   # growing -> ready
SEASON_LENGTH_DAYS = 30  # ready-but-unharvested -> spoiled; agents halt

# Scoring — additive only, no penalties (losing the chance to ever harvest a
# dead/spoiled cell is already the cost of neglect). First-pass magnitudes,
# expect retuning after a playtest.
HARVEST_REWARD = 100.0
WATER_REWARD   = 5.0
WEED_REWARD    = 5.0

# RL task-prioritization policy — off by default so the random-choice baseline
# (evaluate_and_claim in agent.py) stays the default, comparable path. Flip on
# after training a policy with train.py.
USE_RL_POLICY      = True
POLICY_WEIGHTS_PATH = "models/task_policy.pkl"

COLORS = {
    "background":    (28,  28,  28),
    "grid":          (40,  40,  40),
    "text":          (220, 220, 220),
    "text_dim":      (120, 120, 120),
    "agent":         (200, 200, 200),
    "agent_push":    (255, 220,  80),   # performing_task
    "agent_waiting": (100, 160, 255),   # moving_to_task
    "agent_busy":    (100, 100, 100),
    "task_link":     (255, 180,  40),
    "claim_marker":  (255, 220,  80),
    "walkway":       (120, 84,  48),    # pathway
    "water":         (60,  90,  210),
    "unknown":       (55,  55,  65),
    "harvest_box":   (200, 150, 40),

    # Plant-cell condition indicator: starts healthy green, blends toward
    # blue as it needs more water, blends toward red as weeds get worse.
    "healthy":       (45,  150, 70),
    "needs_water":   (40,  90,  220),
    "weedy":         (215, 50,  50),

    # Resolved-cell states — dim/gray family, distinct from "unknown" (never
    # checked) so a done cell reads as "finished," not "needs a look."
    "harvested":     (90,  90,  90),
    "dead":          (60,  30,  30),
    "spoiled":       (90,  70,  30),
}
