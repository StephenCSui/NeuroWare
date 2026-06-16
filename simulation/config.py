WINDOW_WIDTH  = 1200
WINDOW_HEIGHT = 750
FPS           = 30

AGENT_SIZE  = 22
AGENT_SPEED = 2.5
COMM_RANGE  = 250
VISION_RANGE = 200
NUM_AGENTS  = 6

BUSY_CHANCE    = 0.003
BUSY_DURATION  = 4.0

CAPTAIN_RADIUS        = 14
CAPTAIN_PING_INTERVAL = 3.0

ITEM_SIZE = 20   # base size for type 0; types 1 and 2 are larger

# agents_per_side: how many agents must occupy each required push side
ITEM_TYPES = {
    0: {"label": "S",  "color": (220,  80,  80), "agents_per_side": 1, "size": 20},
    1: {"label": "M",  "color": (240, 180,  40), "agents_per_side": 2, "size": 40},
    2: {"label": "L",  "color": (100, 200, 120), "agents_per_side": 3, "size": 58},
}

# Push physics
PUSH_FORCE       = 1.5    # velocity contribution per pushing agent per frame
ITEM_DAMPING     = 0.78   # velocity multiplier each frame when not being pushed
PUSH_SLOT_OFFSET = 24     # pixels from item edge to agent contact point
SLOT_SPREAD      = 15     # spacing between agents sharing the same side (type 2/3)
PUSH_SPEED_MAX   = 3.5    # max item speed (pixels/frame)
DEST_THRESHOLD   = 30     # pixels from destination to consider item "arrived"

# Agent behaviour
AGENT_SEP_DIST   = AGENT_SIZE + 2   # distance at which agents start separating
AGENT_SEP_FORCE  = 0.6              # separation push per frame
BREAKOFF_WAIT    = 5.0              # seconds in waiting_for_co_pusher before re-eval

LOADING_ZONE  = (30,   40, 180, 160)
GOAL_ZONE     = (30,  550, 180, 160)
STORING_ZONE  = (990, 250, 180, 250)

CAPTAIN_A_ZONE = (0,   0, 600, WINDOW_HEIGHT)
CAPTAIN_B_ZONE = (600, 0, 600, WINDOW_HEIGHT)

CAPTAIN_A_POS = (200, 375)
CAPTAIN_B_POS = (1000, 375)

COLORS = {
    "background":    (28,  28,  28),
    "grid":          (40,  40,  40),
    "text":          (220, 220, 220),
    "text_dim":      (120, 120, 120),
    "agent":         (200, 200, 200),
    "agent_push":    (255, 220,  80),
    "agent_busy":    (100, 100, 100),
    "agent_waiting": (100, 160, 255),
    "comm_link":     ( 70,  70,  70),
    "push_link":     (255, 180,  40),
    "captain_link":  (160, 160, 255),
    "zone_a":        ( 60,  90, 180),
    "zone_b":        ( 50, 140,  80),
    "loading":       ( 80, 160, 220),
    "storing":       (160, 100, 220),
    "goal":          (220, 140,  60),
    "ping":          (255, 255, 100),
}
