# Multi-Scene Datasets + Visualizer Switcher

**Date:** 2026-06-25
**Status:** Approved design

## Problem

The simulator currently runs one scene at a time (`configs/retail-example.json`),
producing a single `sim-output.json`/`.bin` dataset. The visualizer loads exactly
one dataset (`BASE`, overridable via `?data=`). To study how multipath behaves
across different environments, we want **many small, varied example scenes** —
variety in room shape, obstacle layout, antenna placement, and tag placement —
and the visualizer should let the user **switch between them** from the UI.

"Smaller numbers of tags" per scene keeps each run fast so the whole set builds
in minutes.

## Goals

- A curated set of **6 hand-authored scenes**, each exercising a different
  combination of room shape / obstacles / antennas / tag placement.
- **Explicit per-scene tag placement** (hand-placed tag coordinates), in addition
  to the existing uniform grid.
- A **single batch command** that runs all scenes and emits a manifest.
- A **dataset switcher** in the visualizer driven by that manifest.

## Non-Goals

- No procedural/random scene generation (scenes are hand-authored).
- No new obstacle shapes (rectangles only, as today).
- No FDTD engine changes — scene runs reuse the existing `runSweep` path.
- No new room-geometry primitive — non-rectangular rooms are expressed with
  interior walls inside the rectangular grid, as the engine already supports.

## Decisions (from brainstorming)

| Question | Decision |
|---|---|
| Scene source | Hand-authored set |
| Tag placement | Add an explicit tag list to the config |
| Build + discovery | New batch command writing a `scenes.json` manifest |
| Scope | 6 scenes |
| Fidelity | High — `grid_resolution: 0.015` (match retail-example) |

## Architecture

### 1. Scene configs — `configs/scenes/*.json`

One JSON file per scene, same schema as `retail-example.json` plus the new
optional `tags` field (below). Proposed initial set:

| File | Room shape | Obstacles | Antennas | Tag placement |
|---|---|---|---|---|
| `open-warehouse.json` | large open rectangle | few / none | 4 corners | sparse scattered (explicit) |
| `l-shaped-store.json` | L-shape via interior walls | metal shelving rects | 3 | clustered along aisles |
| `narrow-corridor.json` | long thin rectangle | a couple metal blocks | 2 at the ends | a line of tags |
| `cluttered-backroom.json` | small rectangle | many metal shelf rects | 4 wall-mounted | clustered |
| `glass-atrium.json` | rectangle with glass partitions | glass walls | 4 | loose explicit spread |
| `small-office.json` | drywall-partitioned rectangle | drywall partitions | 3 | a few tags per sub-room |

**Fidelity / runtime budget:** all scenes use `grid_resolution: 0.015`. To keep
the batch fast, each scene keeps tag count **≤ ~30** and sizes `timesteps` to its
room (enough for multipath to settle, roughly 2–3 diagonal crossings). Per-tag
cost at 0.015 is ~27 s for a 10×15 m room (measured) and scales with cell count,
so smaller rooms are proportionally cheaper. Target: **< ~3 min/scene, whole set
~10–15 min** on 8 threads.

### 2. Config change — explicit tag list (`config.zig`)

Add an optional tag list and make the grid spacing optional:

```zig
pub const TagPoint = struct { x: f64, y: f64 };

pub const Config = struct {
    // ... existing fields ...
    tag_grid_spacing: ?f64 = null,   // was required; now optional
    tags: []TagPoint = &.{},         // new: explicit tag positions
    snapshots: bool = false,         // new: opt in to wave-view snapshots (see §4)
    label: ?[]const u8 = null,       // new: display name for the manifest
    description: ?[]const u8 = null, // new: one-line blurb for the manifest
};
```

The `label`/`description` fields are parsed into `Config` (rather than re-read
from raw JSON) so the `scenes` command can populate the manifest directly. They
are ignored by every other code path.

**Selection rule** (`generator.tagPositions`):

- If `tags` is non-empty → use those exact coordinates. Each is mapped to its
  grid cell; a tag whose cell is **not free space** (wall/obstacle/PEC) or
  **coincides with an antenna cell** is **skipped with a warning** (same policy
  the grid already applies).
- Else if `tag_grid_spacing` is set → existing uniform-grid behavior.
- Else → validation error (`NoTagSource`).

This keeps `retail-example.json` (grid, no `tags`) working unchanged.

`config.validate` gains: explicit `tags` must lie within the room bounds
(out-of-room tag → `TagOutsideRoom`); a config with neither `tags` nor
`tag_grid_spacing` → `NoTagSource`.

### 3. New `scenes` command (`main.zig`)

```
rfid-sim scenes --dir configs/scenes --output <datadir> [--threads N]
```

Behavior:

1. Enumerate `*.json` files in `--dir` (sorted by name for stable order).
2. For each: parse + validate, build the grid, run the existing `runSweep`,
   writing `<datadir>/<name>.json` and `<datadir>/<name>.bin` (where `<name>` is
   the config filename without extension).
3. If a scene's config has `"snapshots": true`, also capture snapshots for its
   first tag (the existing `--save-snapshots` path) into
   `<datadir>/<name>_snapshots/`.
4. After all scenes, write `<datadir>/scenes.json` (the manifest).

Per-scene progress reuses the existing per-tag progress prints, prefixed with the
scene name so the batch is legible. A scene that fails to parse/validate is
reported and **skipped** (the batch continues); the command exits non-zero if any
scene failed.

**Manifest — `scenes.json`:**

```json
{
  "scenes": [
    { "name": "open-warehouse", "label": "Open Warehouse",
      "description": "Large open floor, 4 corner antennas, sparse tags" },
    { "name": "l-shaped-store", "label": "L-Shaped Store",
      "description": "L-shaped layout with shelving, tags along aisles" }
  ],
  "default": "open-warehouse"
}
```

`label` and `description` come from optional `"label"`/`"description"` fields in
each scene config (falling back to the filename if absent). `default` is the
first scene in sorted order.

### 4. Visualizer switcher (`viz/index.html`, `viz/viz.js`)

- Add a **dataset `<select>`** in the header (next to `#status`).
- On load: fetch `/data/scenes.json`. If present, populate the dropdown and set
  `BASE` to `?data=` (if given and valid) else the manifest `default`. If
  `scenes.json` is absent (single-dataset mode), hide the dropdown and keep
  today's `sim-output` / `?data=` behavior — **fully backward compatible.**
- On change: reset `state` (`meta`, `bin`, `snapshots`, tag selections), set
  `BASE` to the chosen scene, update the URL (`?data=<name>` via
  `history.replaceState`), reload meta, and re-render the **current** view.
- The wave tab already shows an "unavailable" message when a base has no
  snapshots — unchanged, so scenes without snapshots degrade gracefully.

**No server change.** `server.zig` already serves any file under `--dir`,
including `scenes.json` and each scene's `.json`/`.bin`/`_snapshots/`.

## Data Flow

```
configs/scenes/*.json
        │  rfid-sim scenes --dir configs/scenes --output data
        ▼
data/<name>.json + <name>.bin [+ <name>_snapshots/]   (one set per scene)
data/scenes.json   (manifest: names, labels, descriptions, default)
        │  rfid-sim serve --dir data
        ▼
viz: fetch scenes.json → populate dropdown → pick → fetch <name>.json/.bin → render
```

## Error Handling

- **Bad scene config:** reported with the filename and reason; skipped; batch
  continues; command exits non-zero if any scene failed.
- **Explicit tag in obstacle/wall/antenna cell:** skipped with a warning (not
  fatal) — mirrors grid behavior.
- **Tag outside room / no tag source:** config validation error (fatal for that
  scene).
- **Missing `scenes.json` in viz:** dropdown hidden; falls back to single-dataset
  mode. Missing chosen `.json`/`.bin`: existing fetch-error status message.

## Testing

- `config.zig`: parse explicit `tags`; `tag_grid_spacing` optional; validation
  rejects out-of-room tags and a config with neither tag source.
- `generator.zig`: explicit tags used verbatim; a tag inside an obstacle is
  skipped; grid fallback still produces the same positions as before.
- `scenes` command: a tiny temp dir with 1–2 minimal scene configs produces the
  expected `<name>.json`/`.bin` files and a well-formed `scenes.json`.
- Each authored scene config parses + validates (parameterized test over
  `configs/scenes/*.json`, like the existing `retail-example.json` test).
- `viz.js` has no test harness today; switcher verified manually via `serve`.

## Out of Scope / Future

- Procedural scene generation and template perturbation.
- Non-rectangular obstacle primitives (circles/polygons).
- Showing multiple scenes side-by-side simultaneously.
