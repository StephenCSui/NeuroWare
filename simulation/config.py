# Grid / farm layout — drives window size (template-based layout, not a repeating pattern)
CELL_SIZE = 45
GRID_COLS = 10
GRID_ROWS = 8

WINDOW_WIDTH  = GRID_COLS * CELL_SIZE
WINDOW_HEIGHT = GRID_ROWS * CELL_SIZE
FPS           = 30

AGENT_SIZE  = 22
NUM_AGENTS  = 2

# Global speed multiplier — scales movement/decay/durations/day-length
# together so relative game balance is preserved (a task still takes the
# same FRACTION of a day, a cell still decays the same amount PER day) while
# the whole season plays out faster in real time. Only affects live viewing
# and eval wall-clock — headless training's cost is dominated by the RL
# policy's batch refit over its replay buffer, not by tick count, so this
# doesn't meaningfully speed that up.
TIME_SCALE = 4.0

AGENT_SPEED = 1.2 * TIME_SCALE   # deliberately slow at TIME_SCALE=1 — leaves room for a future AI layer to show measurable improvement

BUSY_CHANCE   = 0.003 * TIME_SCALE   # per-tick probability -- scaled up to keep the same expected busy-events per day despite fewer ticks/day
BUSY_DURATION = 4.0 / TIME_SCALE

# Farm layout regions — (col, row, width, height), all in cells. Base fill is
# "walkway" (pathway); regions stamp over it in this order: water -> plant ->
# harvest_box. Single-cell harvest drop-off and a water zone in the opposite
# corner — walkway fills everything else, which gives >=2-cell clearance on
# every side of the bed for free (no deadspace or one-way lanes needed at
# this scale — those existed only to manage 5-agent bridge contention, which
# doesn't apply with 2 agents and an open layout).
HARVEST_BOX  = (0, 0)          # single cell, col/row
WATER_REGION = (7, 0, 2, 2)
PLANT_REGION = (3, 2, 4, 3)    # 4x3 = 12 plant cells — doubled from 6, deliberately more than 2 agents can comfortably keep up with

# Cell state -> task thresholds (evaluated against KNOWN state, not true state)
MOISTURE_LOW_THRESHOLD    = 30.0
WEED_HIGH_THRESHOLD       = 60.0
STALENESS_THRESHOLD_TICKS = int(300 / TIME_SCALE)   # ~10s @30FPS at TIME_SCALE=1, since last physical visit

# True-state drift per second (stochastic, see Cell.tick). Originally tuned
# against a 28-day maturity window (fully-neglected death in ~20 days, a
# comfortable buffer); DAYS_TO_MATURE is now 20, so full neglect can now
# kill a cell before it would even mature -- tighter margin than before,
# not retuned here since growing cells still comfortably survive realistic
# periodic watering. Worth revisiting if growing-phase deaths look too harsh
# once played live. The original 0.9 was tuned for an open-ended maintenance
# loop with no death consequence at all -- ~10x too fast once death was
# introduced (verified: caused 4-6/6 cells dead every run).
#
# DIFFICULTY_MULT (1.3x): deliberate, on top of TIME_SCALE — combined with
# doubling PLANT_REGION to 12 beds, this is meant to push the task-selection
# problem back into "2 agents genuinely can't keep up with everything"
# territory. The earlier 6-bed/10-day-harvest-window tuning made the random
# baseline hit ~94% harvest rate, leaving almost no room for smarter task
# selection to show a measurable edge -- this reintroduces real scarcity.
# (1.5x combined with 12 beds turned out too harsh -- 71% death rate under
# the random baseline, likely past the point where even a good policy could
# meaningfully help. Dropped to 1.3x to pull that back toward a range with
# real room for smart vs. dumb to actually differ.)
DIFFICULTY_MULT = 1.3
MOISTURE_DECAY_BASE = 0.5 * TIME_SCALE * DIFFICULTY_MULT   # jittered 0.5x-1.5x per tick, at TIME_SCALE=1
WEED_SPROUT_CHANCE  = 0.005 * TIME_SCALE * DIFFICULTY_MULT # per-second probability of a sprout event per plant cell, at TIME_SCALE=1
WEED_SPROUT_MIN     = 15.0
WEED_SPROUT_MAX     = 40.0

# Task effects/durations (seconds, at TIME_SCALE=1) — kept slow, matches AGENT_SPEED
WATER_REFILL_AMOUNT = 70.0
# One water-cell visit is worth this many watering actions before the agent
# needs to refill again — >1 specifically to cut down on how often watering
# requires a full round-trip to the water region relative to weeding (which
# has no such travel overhead), since that asymmetry could otherwise bias
# any policy — trained or random — toward neglecting watering just because
# it's more expensive in travel time, not because it matters less.
WATER_CAPACITY = 2
TASK_DURATIONS = {
    "monitor":         1.0 / TIME_SCALE,
    "weed":            6.0 / TIME_SCALE,
    "water":           6.0 / TIME_SCALE,
    "obtain_water":    1.0 / TIME_SCALE,
    "harvest":         6.0 / TIME_SCALE,   # hand-picking a ready plant, matches weed/water labor time
    "deliver_harvest": 1.0 / TIME_SCALE,   # quick drop-off at the harvest box, matches monitor/obtain_water
}

# Claim safety
CLAIM_TIMEOUT_TICKS = int(300 / TIME_SCALE)   # ~10s @30FPS at TIME_SCALE=1; auto-frees an abandoned/stuck claim

# Movement safety — if a cached path step stays blocked by another agent this
# long, force a reroute around current traffic instead of waiting forever.
# This is now just a generic safety net for ordinary temporary contention —
# the specific mutual-swap deadlock is detected and resolved immediately by
# Agent._should_yield()/_step_aside(), it doesn't wait on this counter at all.
STUCK_REROUTE_FRAMES = max(5, int(20 / TIME_SCALE))

# Day cycle — reuses the same tick clock everything else runs on, no separate
# time system. A "day" is just this many ticks (~10s @30FPS at TIME_SCALE=1,
# matching STALENESS_THRESHOLD_TICKS's granularity). 30-day season ~= 2250
# ticks (~75s) at TIME_SCALE=4, vs. 9000 ticks (~5 min) at TIME_SCALE=1.
DAY_LENGTH_TICKS  = int(300 / TIME_SCALE)
DAYS_TO_MATURE    = 20   # growing -> ready
SEASON_LENGTH_DAYS = 30  # ready-but-unharvested -> spoiled; agents halt (10-day harvest window)

# Scoring. First-pass magnitudes, expect retuning after a playtest.
HARVEST_REWARD = 100.0
WATER_REWARD   = 5.0
WEED_REWARD    = 5.0
# Applied once per dead cell, as a team-level hit (see grid.py tick()) rather
# than blamed on a specific agent's movement -- both agents share one
# TaskPolicy, so the penalty just needs to reach whichever decision resolves
# next, not be attributed to whoever happened to be moving nearby.
DEATH_PENALTY  = -50.0
# Dense per-tick bleed while a cell sits critically dry, on top of (not
# instead of) DEATH_PENALTY -- a single terminal penalty is too sparse/noisy
# to reliably teach "should have watered sooner" (it lands on whichever task
# happens to finish next, not necessarily a related one). A continuous bleed
# tied directly to the observable risk state gives a real, repeated
# watered-promptly vs. left-neglected differential instead of a coin-flip.
# moisture 100=fully hydrated, 0=dead -- threshold 40 means bleed starts once
# a cell is >60% dehydrated, but it's a warning line, not a life sentence:
# water it back above 40 and the bleed stops, no lasting mark against later
# harvest/water/weed rewards. Rate is per second (scaled by dt in grid.py's
# tick(), not per physics tick) so it stays meaningful independent of FPS/
# TIME_SCALE. First-pass magnitude, expect retuning.
DEHYDRATION_BLEED_THRESHOLD = 40.0
DEHYDRATION_BLEED_RATE      = -1.0   # per second, per cell at/below threshold

# RL task-prioritization policy — off by default so the random-choice baseline
# (evaluate_and_claim in agent.py) stays the default, comparable path. Flip on
# after training a policy with train.py.
USE_RL_POLICY      = True
POLICY_WEIGHTS_PATH = "models/task_policy.pkl"

# Per-event reward log (see reward_log.py) — every reward an agent actually
# receives, written out on quit. Debugging aid, not read by anything yet.
REWARD_LOG_PATH = "reward_logs/reward_log.csv"

# Per-death log (see death_log.py) — which bed died, when, and why
# (dehydration/weeds/both), written out on quit. Built specifically to check
# whether difficulty tuning is killing beds for the intended reason.
DEATH_LOG_PATH = "reward_logs/death_log.csv"

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
    "bleeding":      (255, 240, 0),    # flat override while DEHYDRATION_BLEED is actively accruing

    # Resolved-cell states — dim/gray family, distinct from "unknown" (never
    # checked) so a done cell reads as "finished," not "needs a look."
    "harvested":     (90,  90,  90),
    "dead":          (60,  30,  30),
    "spoiled":       (90,  70,  30),
}
