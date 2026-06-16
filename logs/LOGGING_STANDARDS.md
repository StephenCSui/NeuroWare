# NeuroWare Logging Standards

This document defines how sessions are documented on this project. Any AI assistant continuing work on NeuroWare should follow these conventions exactly so that logs stay consistent and useful across sessions.

---

## Why these logs exist

The primary reader of every log entry is **an AI assistant at the start of a future session, cold, with no memory of prior work.** Write accordingly. The goal is not a changelog — it is orientation. Cover what actually happened, what the current state is, and what traps to avoid.

---

## Log files

| File | Purpose |
|---|---|
| `logs/DEVLOG.md` | One entry per session. High-level only. What changed, what broke, current state, next steps. |
| `logs/agent.md` | Detailed history of `agent.py` across sessions. |
| `logs/captain.md` | Detailed history of `captain.py` across sessions. |
| `logs/environment.md` | Detailed history of `config.py`, `item.py`, `zone.py`, `main.py` — the simulation environment. |

Additional component logs may be created when a component grows complex enough to warrant one. Ask the user before creating a new log file.

---

## When to update

**Never auto-update.** Only update logs when the user explicitly asks. Do not update them at the end of a session without being asked.

When suggesting an update, tell the user it may be a good time to update — do not do it yourself unless asked.

---

## Session numbering

Sessions are numbered sequentially across all log files: `Session N — NeuroWare (YYYY-MM-DD)`.

- Session number increments once per working session (not per context window overflow).
- If a single session overflows into a new context window, the continued work is still the same session number. Call it `Session N (continued)` only if the new context window has a meaningful second phase of work; otherwise append to the existing session entry in the same log.
- Always use an absolute date (not "today" or "Monday").

---

## DEVLOG entry format

```markdown
## Session N — NeuroWare (YYYY-MM-DD)

### What was done
[Bullet list: what was built, fixed, or changed. One bullet per logical unit of work.]

### What worked
[Bullet list: confirmed working outcomes.]

### What did not work
[Bullet list: bugs hit, things that failed, anything that was attempted and reverted. Include brief root cause if known. "N/A" if nothing failed.]

### Current state
[One sentence. Where things actually stand right now. Not aspirational.]

### Next steps (high level)
[Numbered list. High level only — detail lives in component logs.]
```

Keep DEVLOG entries short. Do not include code snippets, method names, or debugging detail — those belong in component logs.

---

## Component log entry format

```markdown
## Session N — NeuroWare (YYYY-MM-DD)

### Current state
[One line: working / broken / partial — and what specifically is working or broken.]

### What was done
[Detailed bullet list. Include method names, parameter names, data structures. Be specific enough that a future assistant can understand what exists without reading all the code.]

### What worked
[Confirmed outcomes with enough detail to reproduce or rely on them.]

### What did not work
[Each failure gets: symptom, root cause (if found), fix applied (if fixed). Use "N/A" if nothing failed.]

### Key decisions
[Design decisions that are not obvious from the code. Include the reasoning. If a decision was a deliberate tradeoff, say so.]

### Dependencies
[What this component reads from or writes to other components/files. Keep this current — stale dependency info is worse than none.]

### [POTENTIAL FIX]
[Labelled separately: something that was not tried but is a credible fix for an open problem. Include enough context to act on it.]

### [WORTH EXPLORING]
[Labelled separately: non-urgent ideas, architectural concerns, known limitations, future work. Be honest about what is speculative vs. observed.]
```

---

## Rules for log content

**Be specific.** "Fixed gossip bug" is useless. "Fixed `cleanup_known_tasks` deleting goal tasks because `item.stored` was True — goal tasks only prune on `item.delivered` now" is useful.

**Record failures, not just successes.** A future assistant who knows what was tried and failed will not waste time repeating it. This is the most important part of the log.

**Never invent.** Do not log something as working if it has not been tested. Do not log something as fixed if the root cause is not confirmed. Use hedged language: "appears fixed", "believed to be caused by", "not yet tested."

**Separate observed facts from guesses.** If the root cause of a bug is not known, say so. Label guesses as guesses.

**No code in DEVLOG.** Code snippets, method signatures, and implementation detail belong in component logs only. DEVLOG is for orientation, not reference.

**Current state is mandatory and must be honest.** If something is broken, say it is broken. Do not write "mostly working" when there is a known bug. The current state line is what a future assistant reads first to understand whether they can trust the component.

---

## Starting a session

At the start of any session on this project:

1. Read `logs/DEVLOG.md` in full.
2. Read the component logs relevant to today's work.
3. Do not start implementation until you have oriented yourself on current state and known issues.

The logs are the source of truth for project history. Git history and code comments cover what changed; logs cover why it changed, what failed along the way, and what to watch out for.

---

## Creating new logs

A new component log is warranted when:
- A component has enough history that its section in `environment.md` is getting long
- A component has a complex enough design that key decisions need to be tracked separately
- A component has recurring bugs or known open issues that need a dedicated place

Propose the new log to the user before creating it. When created, add an entry to this standards document and note the new file in the DEVLOG.
