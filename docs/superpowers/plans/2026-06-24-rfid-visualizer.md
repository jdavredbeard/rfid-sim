# RFID Visualizer + Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax. Per the team's TDD preference, the **Zig** tasks with pure logic are written **strict test-first** (write the test → RED → implement → GREEN → commit). The browser/Canvas tasks have **no JS test runner in this environment**, so they use explicit manual/`curl` verification instead — this is called out per task and is the one deliberate departure from TDD (there is no test harness to be strict with).

**Goal:** Add the `serve` command and a browser-based visualizer, plus the snapshot output that the wave-animation view needs. Completes the system: `simulate --save-snapshots` writes Ez field snapshots; `serve` hosts the viz + data over HTTP; the browser app renders four views (Room Layout, Wave Animation, Coverage Heatmap, Impulse Response Plot).

**Architecture:** Three layers.
1. **Snapshots (Zig, `snapshots.zig`)** — reuses the existing public FDTD API (`fdtd.init`/`step`/`Sim.ez`) to run one single-tag simulation and dump the full Ez field every N steps as float32 grids + a `snapshots.json` manifest. Wired into `simulate --save-snapshots`.
2. **Server (Zig, `server.zig`)** — a single-threaded `std.http.Server` loop. Static viz assets are `@embedFile`d (so they're located reliably regardless of CWD); simulation data files are served from a `--dir` under `/data/`. Route resolution and content-type are pure, test-first functions; the socket loop is verified with `curl`.
3. **Visualizer (`viz/index.html`, `viz/style.css`, `viz/viz.js`)** — vanilla JS + Canvas, no frameworks/Node. Fetches `sim-output.json` (room/antennas/tag grid), the `.bin` (sliced client-side per sample using the JSON byte offsets), and snapshot grids. Four tabbed views.

**Tech Stack:** Zig 0.14.0 (`std.http.Server`, `std.net`, `std.fs`, `@embedFile`), and browser HTML/CSS/JS + Canvas 2D. No new external dependencies, no Node.

**Scope note:** This is Plan 3 of 3 (final). Plans 1 (core simulator) and 2 (combiner) are merged to `main`. This plan depends on their output formats: `sim-output.json` (`grid{nx,ny,dx,dt}`, `config`, `antennas`, `impulse_length`, `samples[{tag_x,tag_y,offset}]`) and the antenna-major little-endian float32 `.bin`. The combiner's `training-data.*` are also serveable as plain `/data/` files but the viz focuses on `sim-output` (single-tag) data per the spec.

**Builds on (already on `main`):**
- `fdtd.zig`: `pub const Sim` (with public `ez: []f64`), `pub fn init(allocator, *const Grid, SourceParams, src_i, src_j)`, `pub fn step(*Sim, n)`, `pub fn courantDt(dx)`, `pub const SourceParams`.
- `grid.zig`: `pub const Grid` (+ `build`), `idx`.
- `generator.zig`: `pub const TagPos {x,y,i,j}`, `pub fn tagPositions(allocator, cfg, grid) ![]TagPos`.
- `main.zig`: `cmdSimulate` currently **warns and ignores** `--save-snapshots`/`--snapshot-interval` (Task 2 replaces that with real behavior). Dispatch is by `std.mem.eql(u8, args[1], "...")`.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `src/snapshots.zig` | Run one single-tag FDTD sim; write Ez float32 grids every interval + `snapshots.json` manifest. |
| `src/server.zig` | Pure route resolution + content-type (testable); HTTP serve loop with embedded assets and `/data/` file serving; `serve` orchestration. |
| `src/main.zig` (modify) | Real `--save-snapshots`/`--snapshot-interval` in `cmdSimulate`; add `serve` dispatch + `cmdServe`. |
| `viz/index.html` | Tabbed single-page shell + canvases + controls. |
| `viz/style.css` | Layout/styling. |
| `viz/viz.js` | Data loading + four Canvas views (room, wave, coverage, impulse). |

Snapshot output layout (for `--output <base> --save-snapshots`): a sibling directory `<base>_snapshots/` containing `snapshots.json` + `snap_0000.bin`, `snap_0001.bin`, … (each `nx*ny` float32, row-major `i*ny + j`, matching `grid.idx`).

Server data contract: viz fetches `/data/<base>.json`, `/data/<base>.bin`, and (if present) `/data/<base>_snapshots/snapshots.json` + the snap files. The viz `<base>` defaults to `sim-output` and is overridable via the URL query `?data=<base>`.

**Testing convention:** `zig test src/<file>.zig` for a single file; `zig build test` for the suite. Keep the per-allocation `errdefer` discipline from Plans 1–2.

---

## Task 1: Snapshot capture

**Files:**
- Create: `src/snapshots.zig`

Runs a single-tag FDTD simulation and writes the Ez field every `interval` steps as float32 grids, plus a manifest. Reuses the public `fdtd` API (no changes to `fdtd.zig`).

- [ ] **Step 1: Write ONLY the test first.** Create `src/snapshots.zig`:

```zig
const std = @import("std");
const grid_mod = @import("grid.zig");
const fdtd = @import("fdtd.zig");

// ===== TEST (written first) =====

test "capture writes the expected snapshot files and a parseable manifest" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir);
    const out_dir = try std.fs.path.join(std.testing.allocator, &.{ dir, "snaps" });
    defer std.testing.allocator.free(out_dir);

    var g = try grid_mod.Grid.initFreeSpace(std.testing.allocator, 20, 20, 0.03);
    defer g.deinit();

    // 100 steps, interval 25 => snapshots after steps 25,50,75,100 => 4 files.
    try capture(std.testing.allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, 10, 10, 0.3, 0.3, 100, 25, out_dir);

    // Manifest exists and reports 4 snapshots with correct grid dims.
    const manifest_path = try std.fs.path.join(std.testing.allocator, &.{ out_dir, "snapshots.json" });
    defer std.testing.allocator.free(manifest_path);
    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, manifest_path, 1 << 20);
    defer std.testing.allocator.free(bytes);
    const parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, bytes, .{});
    defer parsed.deinit();
    const obj = parsed.value.object;
    try std.testing.expectEqual(@as(i64, 20), obj.get("nx").?.integer);
    try std.testing.expectEqual(@as(i64, 20), obj.get("ny").?.integer);
    try std.testing.expectEqual(@as(i64, 4), obj.get("count").?.integer);
    try std.testing.expectEqual(@as(usize, 4), obj.get("files").?.array.items.len);

    // First snapshot file is nx*ny float32 = 20*20*4 = 1600 bytes.
    const snap0 = try std.fs.path.join(std.testing.allocator, &.{ out_dir, "snap_0000.bin" });
    defer std.testing.allocator.free(snap0);
    const stat = try std.fs.cwd().statFile(snap0);
    try std.testing.expectEqual(@as(u64, 20 * 20 * 4), stat.size);
}
```

- [ ] **Step 2: Run RED.** Run: `zig test src/snapshots.zig`
Expected: compile error — `capture` undefined. Paste as RED.

- [ ] **Step 3: Implement.** Insert ABOVE the test:

```zig
/// Run one single-tag FDTD simulation, writing the full Ez field as a float32 grid
/// every `interval` steps into `out_dir`, plus an `out_dir/snapshots.json` manifest.
/// Snapshots are taken after steps interval, 2*interval, ... (the field after that step).
pub fn capture(
    allocator: std.mem.Allocator,
    grid: *const grid_mod.Grid,
    source: fdtd.SourceParams,
    src_i: usize,
    src_j: usize,
    tag_x: f64,
    tag_y: f64,
    timesteps: u32,
    interval: u32,
    out_dir: []const u8,
) !void {
    try std.fs.cwd().makePath(out_dir);

    var sim = try fdtd.init(allocator, grid, source, src_i, src_j);
    defer sim.deinit();

    const ncells = grid.nx * grid.ny;
    const f32buf = try allocator.alloc(f32, ncells);
    defer allocator.free(f32buf);

    // Manifest accumulators.
    var files = std.ArrayList([]u8).init(allocator);
    defer {
        for (files.items) |f| allocator.free(f);
        files.deinit();
    }
    var steps = std.ArrayList(u32).init(allocator);
    defer steps.deinit();

    const safe_interval = @max(@as(u32, 1), interval);
    var snap_index: usize = 0;
    var n: u32 = 0;
    while (n < timesteps) : (n += 1) {
        fdtd.step(&sim, n);
        if ((n + 1) % safe_interval == 0) {
            for (sim.ez, 0..) |v, k| f32buf[k] = @floatCast(v);
            const fname = try std.fmt.allocPrint(allocator, "snap_{d:0>4}.bin", .{snap_index});
            errdefer allocator.free(fname);
            const fpath = try std.fs.path.join(allocator, &.{ out_dir, fname });
            defer allocator.free(fpath);
            try std.fs.cwd().writeFile(.{ .sub_path = fpath, .data = std.mem.sliceAsBytes(f32buf) });
            try files.append(fname); // ownership moves to `files`
            try steps.append(n + 1);
            snap_index += 1;
        }
    }

    try writeManifest(allocator, out_dir, grid, source, tag_x, tag_y, safe_interval, files.items, steps.items);
}

fn writeManifest(
    allocator: std.mem.Allocator,
    out_dir: []const u8,
    grid: *const grid_mod.Grid,
    source: fdtd.SourceParams,
    tag_x: f64,
    tag_y: f64,
    interval: u32,
    files: []const []const u8,
    steps: []const u32,
) !void {
    _ = source;
    var buf = std.ArrayList(u8).init(allocator);
    defer buf.deinit();
    const w = buf.writer();
    try w.writeAll("{\n");
    try w.print("  \"nx\": {d},\n", .{grid.nx});
    try w.print("  \"ny\": {d},\n", .{grid.ny});
    try w.print("  \"dx\": {d},\n", .{grid.dx});
    try w.print("  \"dt\": {e},\n", .{fdtd.courantDt(grid.dx)});
    try w.print("  \"interval\": {d},\n", .{interval});
    try w.print("  \"tag_x\": {d},\n", .{tag_x});
    try w.print("  \"tag_y\": {d},\n", .{tag_y});
    try w.print("  \"count\": {d},\n", .{files.len});
    try w.writeAll("  \"steps\": [");
    for (steps, 0..) |s, i| {
        if (i != 0) try w.writeAll(", ");
        try w.print("{d}", .{s});
    }
    try w.writeAll("],\n");
    try w.writeAll("  \"files\": [");
    for (files, 0..) |f, i| {
        if (i != 0) try w.writeAll(", ");
        try w.print("\"{s}\"", .{f});
    }
    try w.writeAll("]\n}\n");

    const manifest_path = try std.fs.path.join(allocator, &.{ out_dir, "snapshots.json" });
    defer allocator.free(manifest_path);
    try std.fs.cwd().writeFile(.{ .sub_path = manifest_path, .data = buf.items });
}
```

- [ ] **Step 4: Run GREEN.** Run: `zig test src/snapshots.zig`
Expected: the capture test PASSES (4 files, 1600-byte grid, parseable manifest).

- [ ] **Step 5: Commit.**
```bash
git add src/snapshots.zig
git commit -m "feat: FDTD Ez-field snapshot capture + manifest"
```

**Zig 0.14 notes:** `std.fs.cwd().makePath`, `std.fs.cwd().writeFile(.{ .sub_path, .data })`, `std.mem.sliceAsBytes([]f32)`, `@floatCast` are 0.14. The `errdefer allocator.free(fname)` only covers the window before `files.append(fname)` takes ownership; after a successful append, the `files` deinit frees it. If the compiler flags `var sim`/`var files` mutability, adjust minimally; don't weaken assertions.

---

## Task 2: Wire `--save-snapshots` into `simulate`

**Files:**
- Modify: `src/main.zig`

Replace the current warn-and-ignore for `--save-snapshots`/`--snapshot-interval` with real behavior: after the sweep, capture snapshots for the **first valid tag position** into `<output>_snapshots/`. (Snapshots are a single-propagation visualization; capturing one representative position keeps storage bounded — the spec's ~430 MB estimate is for one run.)

- [ ] **Step 1: Add the import** near the top of `src/main.zig`:
```zig
const snapshots = @import("snapshots.zig");
```

- [ ] **Step 2: Replace the snapshot-flag parsing in `cmdSimulate`.** The current branch looks like:
```zig
        } else if (std.mem.eql(u8, a, "--save-snapshots") or std.mem.eql(u8, a, "--snapshot-interval")) {
            std.debug.print("warning: snapshot flags are not supported in this build (see Visualizer plan)\n", .{});
            if (std.mem.eql(u8, a, "--snapshot-interval")) {
                // (bounds-checked increment from the Plan 1 fix)
            }
        }
```
Replace it with real flag capture. First add these locals next to the other flag vars at the top of `cmdSimulate` (near `var do_validate = false;`):
```zig
    var save_snapshots = false;
    var snapshot_interval: u32 = 50;
```
Then replace the warn-and-ignore branch with:
```zig
        } else if (std.mem.eql(u8, a, "--save-snapshots")) {
            save_snapshots = true;
        } else if (std.mem.eql(u8, a, "--snapshot-interval")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --snapshot-interval requires a value\n", .{});
                return;
            }
            snapshot_interval = try std.fmt.parseInt(u32, args[i], 10);
        }
```

- [ ] **Step 3: After the sweep, capture snapshots.** In `cmdSimulate`, after the existing `const res = try generator.runSweep(...)` and its `done:` print, add:
```zig
    if (save_snapshots) {
        const positions = try generator.tagPositions(allocator, parsed.value, grid);
        defer allocator.free(positions);
        if (positions.len == 0) {
            std.debug.print("warning: no tag positions to snapshot\n", .{});
        } else {
            const p = positions[0];
            const snap_dir = try std.fmt.allocPrint(allocator, "{s}_snapshots", .{output_base});
            defer allocator.free(snap_dir);
            std.debug.print("capturing snapshots for tag ({d},{d}) every {d} steps...\n", .{ p.x, p.y, snapshot_interval });
            try snapshots.capture(allocator, &grid, .{
                .center_freq = parsed.value.source.center_freq,
                .bandwidth = parsed.value.source.bandwidth,
            }, p.i, p.j, p.x, p.y, parsed.value.timesteps, snapshot_interval, snap_dir);
            std.debug.print("snapshots written to {s}/\n", .{snap_dir});
        }
    }
```
(Confirm `grid` and `parsed` are in scope at that point — they are, from earlier in `cmdSimulate`. `grid` is a `var` whose address is taken; pass `&grid`.)

- [ ] **Step 4: Build + full suite.**
Run: `zig build` → clean.
Run: `zig build test` → all pass (snapshot test included; the gated validation test stays skipped).

- [ ] **Step 5: Smoke run with snapshots.**
```bash
zig build run -Doptimize=ReleaseFast -- simulate --config configs/tiny.json --output /tmp/v-sim --save-snapshots --snapshot-interval 50
ls /tmp/v-sim_snapshots/ | head
python3 -c "
import json,os
m=json.load(open('/tmp/v-sim_snapshots/snapshots.json'))
print('nx',m['nx'],'ny',m['ny'],'count',m['count'],'interval',m['interval'],'tag',(m['tag_x'],m['tag_y']))
f=m['files'][0]; sz=os.path.getsize('/tmp/v-sim_snapshots/'+f)
print('snap0',f,'bytes',sz,'expected',m['nx']*m['ny']*4,'match',sz==m['nx']*m['ny']*4)
print('count matches files', m['count']==len(m['files'])==len(m['steps']))
"
```
Expected: `match True`, `count matches files True`. `tiny.json` has 1500 timesteps, interval 50 → 30 snapshots.

- [ ] **Step 6: Commit.**
```bash
git add src/main.zig
git commit -m "feat: --save-snapshots writes Ez snapshots for the first tag position"
```

---

## Task 3: Visualizer skeleton + Room Layout view

**Files:**
- Create: `viz/index.html`
- Create: `viz/style.css`
- Create: `viz/viz.js`

The single-page shell with four tabs, shared data loading, and the **Room Layout** view (the other three view functions are added in Tasks 5–6). **Verification is manual** (there is no JS test runner here); Task 4's server makes it loadable, and Task 7 does the full end-to-end check. For this task, verify the files exist and are internally consistent (the IDs/functions referenced in `index.html` are defined in `viz.js`).

- [ ] **Step 1: Create `viz/index.html`:**

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>RFID FDTD Visualizer</title>
  <link rel="stylesheet" href="style.css" />
</head>
<body>
  <header>
    <h1>RFID FDTD Visualizer</h1>
    <div id="status">loading…</div>
  </header>
  <nav id="tabs">
    <button data-view="room" class="active">Room Layout</button>
    <button data-view="wave">Wave Animation</button>
    <button data-view="coverage">Coverage Heatmap</button>
    <button data-view="impulse">Impulse Response</button>
  </nav>

  <section id="view-room" class="view active">
    <canvas id="room-canvas" width="800" height="800"></canvas>
    <div class="sidebar">
      <p>Click a tag dot to select it (used by the Impulse view).</p>
      <div id="room-selection">No tags selected.</div>
    </div>
  </section>

  <section id="view-wave" class="view">
    <canvas id="wave-canvas" width="800" height="800"></canvas>
    <div class="sidebar">
      <div id="wave-unavailable" class="hidden">No snapshots found. Re-run <code>simulate --save-snapshots</code>.</div>
      <div id="wave-controls">
        <button id="wave-play">▶ Play</button>
        <label>Speed <input id="wave-speed" type="range" min="1" max="30" value="10" /></label>
        <label>Frame <input id="wave-scrub" type="range" min="0" max="0" value="0" /></label>
        <div id="wave-frame-label">frame 0</div>
      </div>
    </div>
  </section>

  <section id="view-coverage" class="view">
    <canvas id="coverage-canvas" width="800" height="800"></canvas>
    <div class="sidebar">
      <label>Antenna <select id="coverage-antenna"></select></label>
      <div id="coverage-legend"></div>
    </div>
  </section>

  <section id="view-impulse" class="view">
    <canvas id="impulse-canvas" width="800" height="500"></canvas>
    <div class="sidebar">
      <div id="impulse-info">Select tag(s) in the Room Layout view, then return here.</div>
    </div>
  </section>

  <script src="viz.js"></script>
</body>
</html>
```

- [ ] **Step 2: Create `viz/style.css`:**

```css
* { box-sizing: border-box; }
body { font-family: system-ui, sans-serif; margin: 0; background: #0e1116; color: #e6edf3; }
header { display: flex; align-items: baseline; gap: 1rem; padding: 0.5rem 1rem; background: #161b22; }
header h1 { font-size: 1.1rem; margin: 0; }
#status { color: #8b949e; font-size: 0.85rem; }
nav#tabs { display: flex; gap: 0.25rem; padding: 0.5rem 1rem; background: #161b22; border-bottom: 1px solid #30363d; }
nav#tabs button { background: #21262d; color: #e6edf3; border: 1px solid #30363d; padding: 0.4rem 0.8rem; border-radius: 6px; cursor: pointer; }
nav#tabs button.active { background: #1f6feb; border-color: #1f6feb; }
.view { display: none; padding: 1rem; gap: 1rem; }
.view.active { display: flex; }
canvas { background: #0b0e13; border: 1px solid #30363d; border-radius: 6px; }
.sidebar { min-width: 220px; font-size: 0.9rem; }
.hidden { display: none; }
label { display: block; margin: 0.5rem 0; }
code { background: #21262d; padding: 0.1rem 0.3rem; border-radius: 4px; }
```

- [ ] **Step 3: Create `viz/viz.js`** with the shared framework + data loading + Room Layout (the `renderCoverage`/`renderImpulse`/wave functions are added in later tasks; calling an as-yet-undefined view simply does nothing until then because the dispatch table only wires what exists):

```javascript
"use strict";

// ---- shared state ----
const params = new URLSearchParams(location.search);
const BASE = params.get("data") || "sim-output";
const state = {
  meta: null,      // parsed sim-output.json
  bin: null,       // ArrayBuffer of sim-output.bin (lazy)
  snapshots: null, // parsed snapshots.json (lazy)
  selected: [],    // selected tag sample indices
};

function setStatus(msg) { document.getElementById("status").textContent = msg; }

// world->canvas transform for a given room size and canvas
function makeTransform(canvas, worldW, worldH) {
  const pad = 20;
  const sx = (canvas.width - 2 * pad) / worldW;
  const sy = (canvas.height - 2 * pad) / worldH;
  const s = Math.min(sx, sy);
  return {
    x: (wx) => pad + wx * s,
    y: (wy) => pad + wy * s, // y grows downward; room y=0 at top
    s,
  };
}

async function loadMeta() {
  const r = await fetch(`/data/${BASE}.json`);
  if (!r.ok) throw new Error(`fetch ${BASE}.json: ${r.status}`);
  state.meta = await r.json();
}

async function ensureBin() {
  if (state.bin) return state.bin;
  setStatus("loading impulse data…");
  const r = await fetch(`/data/${BASE}.bin`);
  if (!r.ok) throw new Error(`fetch ${BASE}.bin: ${r.status}`);
  state.bin = await r.arrayBuffer();
  setStatus(`ready (${state.meta.samples.length} tags, ${state.meta.antennas.length} antennas)`);
  return state.bin;
}

// Float32Array view of antenna `a`'s impulse response for sample index `s`.
function impulseFor(s, a) {
  const N = state.meta.impulse_length;
  const byteOffset = state.meta.samples[s].offset + a * N * 4;
  return new Float32Array(state.bin, byteOffset, N);
}

// ---- Room Layout view ----
function renderRoom() {
  const canvas = document.getElementById("room-canvas");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const cfg = state.meta.config;
  const T = makeTransform(canvas, cfg.room.width, cfg.room.height);

  // room border
  ctx.strokeStyle = "#30363d";
  ctx.strokeRect(T.x(0), T.y(0), cfg.room.width * T.s, cfg.room.height * T.s);

  // obstacles
  ctx.fillStyle = "rgba(248,81,73,0.5)";
  for (const o of cfg.obstacles || []) {
    ctx.fillRect(T.x(o.x), T.y(o.y), o.w * T.s, o.h * T.s);
  }

  // walls (centerline with thickness)
  ctx.strokeStyle = "#8b949e";
  for (const w of cfg.walls || []) {
    ctx.lineWidth = Math.max(1, (w.thickness || 0.1) * T.s);
    ctx.beginPath();
    ctx.moveTo(T.x(w.x1), T.y(w.y1));
    ctx.lineTo(T.x(w.x2), T.y(w.y2));
    ctx.stroke();
  }
  ctx.lineWidth = 1;

  // tag grid dots
  for (let i = 0; i < state.meta.samples.length; i++) {
    const s = state.meta.samples[i];
    ctx.fillStyle = state.selected.includes(i) ? "#f0c000" : "#3fb950";
    ctx.beginPath();
    ctx.arc(T.x(s.tag_x), T.y(s.tag_y), state.selected.includes(i) ? 5 : 2.5, 0, 2 * Math.PI);
    ctx.fill();
  }

  // antennas
  for (const a of cfg.antennas || []) {
    ctx.fillStyle = "#1f6feb";
    ctx.beginPath();
    ctx.arc(T.x(a.x), T.y(a.y), 6, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = "#e6edf3";
    ctx.font = "12px sans-serif";
    ctx.fillText(a.label, T.x(a.x) + 8, T.y(a.y) - 8);
  }

  canvas.onclick = (ev) => {
    const rect = canvas.getBoundingClientRect();
    const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
    const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
    let best = -1, bestD = 1e9;
    for (let i = 0; i < state.meta.samples.length; i++) {
      const s = state.meta.samples[i];
      const dx = T.x(s.tag_x) - px, dy = T.y(s.tag_y) - py;
      const d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; best = i; }
    }
    if (best >= 0 && bestD < 200) {
      const at = state.selected.indexOf(best);
      if (at >= 0) state.selected.splice(at, 1); else state.selected.push(best);
      updateSelectionLabel();
      renderRoom();
    }
  };
}

function updateSelectionLabel() {
  const el = document.getElementById("room-selection");
  if (state.selected.length === 0) { el.textContent = "No tags selected."; return; }
  el.textContent = "Selected: " + state.selected
    .map((i) => `(${state.meta.samples[i].tag_x.toFixed(2)}, ${state.meta.samples[i].tag_y.toFixed(2)})`)
    .join(", ");
}

// ---- view dispatch (extended in later tasks) ----
const VIEWS = {
  room: renderRoom,
  // wave: renderWave,        // added in Task 6
  // coverage: renderCoverage,// added in Task 5
  // impulse: renderImpulse,  // added in Task 5
};

function showView(name) {
  for (const sec of document.querySelectorAll(".view")) sec.classList.remove("active");
  for (const b of document.querySelectorAll("#tabs button")) b.classList.toggle("active", b.dataset.view === name);
  document.getElementById(`view-${name}`).classList.add("active");
  const fn = VIEWS[name];
  if (fn) fn();
}

function wireTabs() {
  for (const b of document.querySelectorAll("#tabs button")) {
    b.addEventListener("click", () => showView(b.dataset.view));
  }
}

async function main() {
  wireTabs();
  try {
    await loadMeta();
    setStatus(`loaded ${BASE}.json — ${state.meta.samples.length} tags, ${state.meta.antennas.length} antennas`);
    showView("room");
  } catch (e) {
    setStatus("error: " + e.message);
    console.error(e);
  }
}

main();
```

- [ ] **Step 4: Consistency check (no JS runner available).** Verify the IDs/functions line up:
```bash
# Every data-view in index.html should be handled or stubbed in viz.js
grep -o 'data-view="[a-z]*"' viz/index.html
grep -n "renderRoom\|VIEWS\|impulseFor\|loadMeta" viz/viz.js
# index.html references style.css and viz.js
grep -n "style.css\|viz.js" viz/index.html
```
Expected: `room` view is wired in `VIEWS`; `wave/coverage/impulse` are present as tabs (their render fns arrive in Tasks 5–6 — clicking them before then just shows an empty canvas, no error). `index.html` references both assets.

- [ ] **Step 5: Commit.**
```bash
git add viz/index.html viz/style.css viz/viz.js
git commit -m "feat: visualizer skeleton + room layout view"
```

---

## Task 4: HTTP server + `serve` command

**Files:**
- Create: `src/server.zig`
- Modify: `src/main.zig`

A single-threaded `std.http.Server` loop. Pure **route resolution** and **content-type** are test-first; the socket loop + embedded-asset serving + `/data/` file serving are verified with `curl` (Step 6). The viz files from Task 3 must exist now because they are `@embedFile`d.

- [ ] **Step 1: Write the pure-logic tests first.** Create `src/server.zig`:

```zig
const std = @import("std");

// Embedded static assets (viz/ must exist at compile time — created in Task 3).
const index_html = @embedFile("../viz/index.html");
const viz_js = @embedFile("../viz/viz.js");
const style_css = @embedFile("../viz/style.css");

// ===== TESTS (written first) =====

test "resolveRoute maps paths to handlers" {
    try std.testing.expectEqual(Route.index, resolveRoute("/"));
    try std.testing.expectEqual(Route.index, resolveRoute("/?data=foo")); // query stripped
    switch (resolveRoute("/viz.js")) {
        .asset => |a| try std.testing.expectEqualStrings("viz.js", a),
        else => return error.Wrong,
    }
    switch (resolveRoute("/data/sim-output.json")) {
        .data => |d| try std.testing.expectEqualStrings("sim-output.json", d),
        else => return error.Wrong,
    }
    switch (resolveRoute("/data/sim-output_snapshots/snap_0000.bin")) {
        .data => |d| try std.testing.expectEqualStrings("sim-output_snapshots/snap_0000.bin", d),
        else => return error.Wrong,
    }
    try std.testing.expectEqual(Route.not_found, resolveRoute("/nope"));
    // path traversal is rejected
    try std.testing.expectEqual(Route.not_found, resolveRoute("/data/../secret"));
}

test "contentType by extension" {
    try std.testing.expectEqualStrings("text/html", contentType("index.html"));
    try std.testing.expectEqualStrings("text/javascript", contentType("viz.js"));
    try std.testing.expectEqualStrings("text/css", contentType("style.css"));
    try std.testing.expectEqualStrings("application/json", contentType("sim-output.json"));
    try std.testing.expectEqualStrings("application/octet-stream", contentType("sim-output.bin"));
    try std.testing.expectEqualStrings("application/octet-stream", contentType("weird.xyz"));
}
```

- [ ] **Step 2: Run RED.** Run: `zig test src/server.zig`
Expected: compile error — `Route`, `resolveRoute`, `contentType` undefined (and possibly an `@embedFile` error if any `viz/` file is missing — Task 3 created them, so it should embed fine). Paste as RED.

- [ ] **Step 3: Implement the pure logic + serve loop.** Insert ABOVE the tests:

```zig
pub const RouteTag = enum { index, asset, data, not_found };
pub const Route = union(RouteTag) {
    index,
    asset: []const u8,
    data: []const u8,
    not_found,
};

/// Resolve an HTTP target path (may include a query string) to a route.
pub fn resolveRoute(target: []const u8) Route {
    // strip query
    const qpos = std.mem.indexOfScalar(u8, target, '?');
    const path = if (qpos) |q| target[0..q] else target;

    if (std.mem.eql(u8, path, "/")) return .index;
    if (std.mem.eql(u8, path, "/index.html")) return .index;
    if (std.mem.eql(u8, path, "/viz.js")) return .{ .asset = "viz.js" };
    if (std.mem.eql(u8, path, "/style.css")) return .{ .asset = "style.css" };
    if (std.mem.startsWith(u8, path, "/data/")) {
        const rel = path["/data/".len..];
        if (rel.len == 0) return .not_found;
        if (std.mem.indexOf(u8, rel, "..") != null) return .not_found; // traversal guard
        if (rel[0] == '/') return .not_found;
        return .{ .data = rel };
    }
    return .not_found;
}

pub fn contentType(name: []const u8) []const u8 {
    if (std.mem.endsWith(u8, name, ".html")) return "text/html";
    if (std.mem.endsWith(u8, name, ".js")) return "text/javascript";
    if (std.mem.endsWith(u8, name, ".css")) return "text/css";
    if (std.mem.endsWith(u8, name, ".json")) return "application/json";
    if (std.mem.endsWith(u8, name, ".bin")) return "application/octet-stream";
    return "application/octet-stream";
}

fn assetBytes(name: []const u8) []const u8 {
    if (std.mem.eql(u8, name, "viz.js")) return viz_js;
    if (std.mem.eql(u8, name, "style.css")) return style_css;
    return index_html;
}

/// Start the HTTP server. Serves embedded viz assets at / and files from `dir` under /data/.
/// Single-threaded: handles one request at a time (sufficient for a local viz tool).
pub fn serve(allocator: std.mem.Allocator, dir: []const u8, port: u16) !void {
    const address = try std.net.Address.parseIp("127.0.0.1", port);
    var net_server = try address.listen(.{ .reuse_address = true });
    defer net_server.deinit();
    std.debug.print("serving http://127.0.0.1:{d}/  (data dir: {s})\n", .{ port, dir });

    var read_buffer: [64 * 1024]u8 = undefined;
    while (true) {
        const conn = net_server.accept() catch |e| {
            std.debug.print("accept error: {s}\n", .{@errorName(e)});
            continue;
        };
        defer conn.stream.close();
        var http_server = std.http.Server.init(conn, &read_buffer);
        while (http_server.state == .ready) {
            var request = http_server.receiveHead() catch |e| {
                if (e != error.HttpConnectionClosing) std.debug.print("receiveHead: {s}\n", .{@errorName(e)});
                break;
            };
            handle(allocator, &request, dir) catch |e| {
                std.debug.print("handler error: {s}\n", .{@errorName(e)});
            };
        }
    }
}

fn handle(allocator: std.mem.Allocator, request: *std.http.Server.Request, dir: []const u8) !void {
    const route = resolveRoute(request.head.target);
    switch (route) {
        .index => try respondBytes(request, index_html, "text/html"),
        .asset => |name| try respondBytes(request, assetBytes(name), contentType(name)),
        .not_found => try request.respond("not found\n", .{ .status = .not_found }),
        .data => |rel| {
            const full = try std.fs.path.join(allocator, &.{ dir, rel });
            defer allocator.free(full);
            const bytes = std.fs.cwd().readFileAlloc(allocator, full, 1 << 30) catch {
                try request.respond("not found\n", .{ .status = .not_found });
                return;
            };
            defer allocator.free(bytes);
            try respondBytes(request, bytes, contentType(rel));
        },
    }
}

fn respondBytes(request: *std.http.Server.Request, bytes: []const u8, ctype: []const u8) !void {
    try request.respond(bytes, .{
        .extra_headers = &.{.{ .name = "content-type", .value = ctype }},
    });
}
```

- [ ] **Step 4: Run GREEN (pure logic).** Run: `zig test src/server.zig`
Expected: both pure-logic tests PASS. (The serve loop isn't unit-tested here; it's curl-verified in Step 6.)

- [ ] **Step 5: Add the `serve` CLI.** In `src/main.zig`: add `const server = @import("server.zig");` near the imports; add a dispatch branch next to `combine`:
```zig
    } else if (std.mem.eql(u8, args[1], "serve")) {
        try cmdServe(allocator, args[2..]);
```
and append `cmdServe`:
```zig
fn cmdServe(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var dir: []const u8 = ".";
    var port: u16 = 8080;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--dir")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --dir requires a value\n", .{});
                return;
            }
            dir = args[i];
        } else if (std.mem.eql(u8, a, "--port")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --port requires a value\n", .{});
                return;
            }
            port = try std.fmt.parseInt(u16, args[i], 10);
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }
    try server.serve(allocator, dir, port);
}
```

- [ ] **Step 6: Build + curl smoke (the real server check).**
```bash
zig build -Doptimize=ReleaseFast
# produce data to serve
zig build run -Doptimize=ReleaseFast -- simulate --config configs/tiny.json --output /tmp/srv/sim-output --save-snapshots --snapshot-interval 50
# start server in the background, serving /tmp/srv
( ./zig-out/bin/rfid-sim serve --dir /tmp/srv --port 8099 & echo $! > /tmp/srv.pid ) ; sleep 1
echo "--- / ---";            curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8099/
echo "--- /viz.js ---";      curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8099/viz.js
echo "--- /style.css ---";   curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8099/style.css
echo "--- data json ---";    curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8099/data/sim-output.json
echo "--- data bin ---";     curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8099/data/sim-output.bin
echo "--- snapshot manifest ---"; curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8099/data/sim-output_snapshots/snapshots.json
echo "--- traversal blocked ---"; curl -s -o /dev/null -w "%{http_code}\n" "http://127.0.0.1:8099/data/../src/main.zig"
echo "--- 404 ---";          curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8099/nope
kill "$(cat /tmp/srv.pid)" 2>/dev/null
```
Expected: `/` → `200 text/html`; `/viz.js` → `200 text/javascript`; `/style.css` → `200 text/css`; data json → `200 application/json`; data bin → `200 application/octet-stream`; snapshot manifest → `200 application/json`; traversal → `404`; `/nope` → `404`.

> Note on `std.http` API churn: the exact signatures of `std.http.Server.init`, `receiveHead`, `request.respond`, and the `state`/`error.HttpConnectionClosing` names are the most likely 0.14 friction points. The **pure** `resolveRoute`/`contentType` logic (which the tests pin down) is stable. If a `std.http` symbol differs in this exact 0.14 build, adapt the serve loop minimally to achieve the curl outcomes above — do NOT change the route/content-type semantics or weaken their tests.

- [ ] **Step 7: Commit.**
```bash
git add src/server.zig src/main.zig
git commit -m "feat: http server + serve command (embedded viz assets, /data file serving)"
```

---

## Task 5: Coverage Heatmap + Impulse Response views

**Files:**
- Modify: `viz/viz.js`

Add the two views that read the `.bin`. **Manual verification** (visual) by the user via the browser; this task's automated check is that the data plumbing works (Task 4's curl already proved the bin is fetchable; here we confirm the JS is wired without syntax errors by loading it — see Step 3).

- [ ] **Step 1: Add `renderCoverage` and `renderImpulse` to `viz/viz.js`** (place them above the `VIEWS` table):

```javascript
// ---- Coverage Heatmap view ----
function populateAntennaSelect() {
  const sel = document.getElementById("coverage-antenna");
  if (sel.options.length) return;
  state.meta.antennas.forEach((label, idx) => {
    const opt = document.createElement("option");
    opt.value = String(idx);
    opt.textContent = label;
    sel.appendChild(opt);
  });
  sel.addEventListener("change", renderCoverage);
}

function valueToColor(t) {
  // t in [0,1] -> blue(low) -> green -> yellow -> red(high)
  const r = Math.max(0, Math.min(255, Math.round(255 * (t * 1.5 - 0.2))));
  const g = Math.max(0, Math.min(255, Math.round(255 * (1 - Math.abs(t - 0.5) * 2))));
  const b = Math.max(0, Math.min(255, Math.round(255 * (1 - t * 1.5))));
  return `rgb(${r},${g},${b})`;
}

async function renderCoverage() {
  populateAntennaSelect();
  await ensureBin();
  const canvas = document.getElementById("coverage-canvas");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const cfg = state.meta.config;
  const T = makeTransform(canvas, cfg.room.width, cfg.room.height);
  const a = parseInt(document.getElementById("coverage-antenna").value || "0", 10);

  // peak |Ez| per tag for the chosen antenna
  const peaks = new Float64Array(state.meta.samples.length);
  let maxPeak = 0;
  for (let i = 0; i < state.meta.samples.length; i++) {
    const imp = impulseFor(i, a);
    let p = 0;
    for (let k = 0; k < imp.length; k++) { const v = Math.abs(imp[k]); if (v > p) p = v; }
    peaks[i] = p;
    if (p > maxPeak) maxPeak = p;
  }

  const cell = Math.max(6, (cfg.tag_grid_spacing || 0.25) * T.s);
  for (let i = 0; i < state.meta.samples.length; i++) {
    const s = state.meta.samples[i];
    const t = maxPeak > 0 ? peaks[i] / maxPeak : 0;
    ctx.fillStyle = valueToColor(t);
    ctx.fillRect(T.x(s.tag_x) - cell / 2, T.y(s.tag_y) - cell / 2, cell, cell);
  }
  // antennas on top
  for (const ant of cfg.antennas || []) {
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.arc(T.x(ant.x), T.y(ant.y), 5, 0, 2 * Math.PI);
    ctx.fill();
  }
  document.getElementById("coverage-legend").textContent =
    `peak |Ez| for ${state.meta.antennas[a]} — blue=low, red=high (max ${maxPeak.toExponential(2)})`;
}

// ---- Impulse Response view ----
const ANT_COLORS = ["#1f6feb", "#3fb950", "#f0c000", "#f85149", "#a371f7", "#39c5cf"];

async function renderImpulse() {
  await ensureBin();
  const canvas = document.getElementById("impulse-canvas");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const info = document.getElementById("impulse-info");
  if (state.selected.length === 0) {
    info.textContent = "Select tag(s) in the Room Layout view, then return here.";
    return;
  }
  const N = state.meta.impulse_length;
  const M = state.meta.antennas.length;

  // global amplitude for scaling
  let amp = 1e-12;
  for (const s of state.selected) for (let a = 0; a < M; a++) {
    const imp = impulseFor(s, a);
    for (let k = 0; k < N; k++) { const v = Math.abs(imp[k]); if (v > amp) amp = v; }
  }

  const pad = 30;
  const w = canvas.width - 2 * pad;
  const h = canvas.height - 2 * pad;
  const midY = pad + h / 2;
  ctx.strokeStyle = "#30363d";
  ctx.beginPath(); ctx.moveTo(pad, midY); ctx.lineTo(pad + w, midY); ctx.stroke();

  // one line per (selected tag, antenna). Solid for first tag; dashed for additional tags.
  state.selected.forEach((s, si) => {
    for (let a = 0; a < M; a++) {
      ctx.strokeStyle = ANT_COLORS[a % ANT_COLORS.length];
      ctx.setLineDash(si === 0 ? [] : [4, 3]);
      ctx.beginPath();
      const imp = impulseFor(s, a);
      for (let k = 0; k < N; k++) {
        const x = pad + (k / (N - 1)) * w;
        const y = midY - (imp[k] / amp) * (h / 2) * 0.95;
        if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  });
  ctx.setLineDash([]);
  info.innerHTML = "Antennas: " +
    state.meta.antennas.map((l, a) => `<span style="color:${ANT_COLORS[a % ANT_COLORS.length]}">${l}</span>`).join(", ") +
    `<br/>Tags: ${state.selected.length} (solid = first; dashed = others). Overlay shows superposition geometry.`;
}
```

- [ ] **Step 2: Wire the two views into the dispatch table.** In `viz/viz.js`, update the `VIEWS` object:
```javascript
const VIEWS = {
  room: renderRoom,
  // wave: renderWave,  // added in Task 6
  coverage: renderCoverage,
  impulse: renderImpulse,
};
```

- [ ] **Step 3: Verify wiring + serve smoke.** There is no JS unit runner; confirm the functions are defined and referenced, and that the page + data still serve:
```bash
grep -n "renderCoverage\|renderImpulse\|impulseFor\|ensureBin" viz/viz.js
# re-run the server smoke from Task 4 Step 6 and additionally fetch viz.js to confirm it serves the new code
( ./zig-out/bin/rfid-sim serve --dir /tmp/srv --port 8099 & echo $! > /tmp/srv.pid ); sleep 1
curl -s http://127.0.0.1:8099/viz.js | grep -c "renderCoverage"   # expect >= 1
curl -s -o /dev/null -w "data bin %{http_code}\n" http://127.0.0.1:8099/data/sim-output.bin
kill "$(cat /tmp/srv.pid)" 2>/dev/null
```
Expected: both render functions present in served `viz.js`; bin fetch `200`. (Visual correctness — heatmap colors, impulse waveforms — is confirmed by the user in a browser during Task 7.)

- [ ] **Step 4: Commit.**
```bash
git add viz/viz.js
git commit -m "feat: coverage heatmap + impulse response views"
```

---

## Task 6: Wave Animation view

**Files:**
- Modify: `viz/viz.js`

Fetch the snapshot manifest + grids and play them back as an Ez heatmap with play/pause, speed, and scrub controls. Gracefully shows an "unavailable" message if there are no snapshots.

- [ ] **Step 1: Add the wave view to `viz/viz.js`** (above the `VIEWS` table):

```javascript
// ---- Wave Animation view ----
const wave = { frames: [], nx: 0, ny: 0, playing: false, idx: 0, raf: 0, maxAbs: 1 };

async function loadSnapshots() {
  if (state.snapshots) return state.snapshots;
  const r = await fetch(`/data/${BASE}_snapshots/snapshots.json`);
  if (!r.ok) return null;
  const manifest = await r.json();
  const frames = [];
  let maxAbs = 1e-12;
  for (const f of manifest.files) {
    const fr = await fetch(`/data/${BASE}_snapshots/${f}`);
    if (!fr.ok) continue;
    const arr = new Float32Array(await fr.arrayBuffer());
    for (let k = 0; k < arr.length; k++) { const v = Math.abs(arr[k]); if (v > maxAbs) maxAbs = v; }
    frames.push(arr);
  }
  wave.frames = frames; wave.nx = manifest.nx; wave.ny = manifest.ny; wave.maxAbs = maxAbs;
  state.snapshots = manifest;
  return manifest;
}

function drawWaveFrame(idx) {
  const canvas = document.getElementById("wave-canvas");
  const ctx = canvas.getContext("2d");
  if (!wave.frames.length) return;
  const arr = wave.frames[idx];
  const { nx, ny, maxAbs } = wave;
  const img = ctx.createImageData(nx, ny);
  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < ny; j++) {
      const v = arr[i * ny + j] / maxAbs; // -1..1 (diverging)
      const t = (v + 1) / 2;
      // blue(neg) - black(0) - red(pos)
      const r = Math.max(0, Math.min(255, Math.round(255 * (t - 0.5) * 2)));
      const b = Math.max(0, Math.min(255, Math.round(255 * (0.5 - t) * 2)));
      const p = (j * nx + i) * 4; // image is ny rows of nx
      img.data[p] = r; img.data[p + 1] = 0; img.data[p + 2] = b; img.data[p + 3] = 255;
    }
  }
  // scale the nx*ny image to the canvas
  const off = document.createElement("canvas");
  off.width = nx; off.height = ny;
  off.getContext("2d").putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(off, 0, 0, canvas.width, canvas.height);
  document.getElementById("wave-frame-label").textContent =
    `frame ${idx + 1}/${wave.frames.length}` + (state.snapshots ? ` (step ${state.snapshots.steps[idx]})` : "");
  document.getElementById("wave-scrub").value = String(idx);
}

function waveTick() {
  if (!wave.playing) return;
  const speed = parseInt(document.getElementById("wave-speed").value, 10);
  wave.idx = (wave.idx + 1) % wave.frames.length;
  drawWaveFrame(wave.idx);
  wave.raf = setTimeout(() => requestAnimationFrame(waveTick), 1000 / speed);
}

function wireWaveControls() {
  const playBtn = document.getElementById("wave-play");
  if (playBtn.dataset.wired) return;
  playBtn.dataset.wired = "1";
  playBtn.addEventListener("click", () => {
    wave.playing = !wave.playing;
    playBtn.textContent = wave.playing ? "⏸ Pause" : "▶ Play";
    if (wave.playing) waveTick();
  });
  document.getElementById("wave-scrub").addEventListener("input", (e) => {
    wave.idx = parseInt(e.target.value, 10);
    drawWaveFrame(wave.idx);
  });
}

async function renderWave() {
  wireWaveControls();
  const unavailable = document.getElementById("wave-unavailable");
  const controls = document.getElementById("wave-controls");
  const m = await loadSnapshots();
  if (!m || wave.frames.length === 0) {
    unavailable.classList.remove("hidden");
    controls.classList.add("hidden");
    return;
  }
  unavailable.classList.add("hidden");
  controls.classList.remove("hidden");
  document.getElementById("wave-scrub").max = String(wave.frames.length - 1);
  drawWaveFrame(wave.idx);
}
```

- [ ] **Step 2: Wire the wave view into the dispatch table.** Final `VIEWS`:
```javascript
const VIEWS = {
  room: renderRoom,
  wave: renderWave,
  coverage: renderCoverage,
  impulse: renderImpulse,
};
```

- [ ] **Step 3: Verify wiring + serve smoke.**
```bash
grep -n "renderWave\|loadSnapshots\|drawWaveFrame" viz/viz.js
( ./zig-out/bin/rfid-sim serve --dir /tmp/srv --port 8099 & echo $! > /tmp/srv.pid ); sleep 1
curl -s http://127.0.0.1:8099/viz.js | grep -c "renderWave"  # expect >= 1
# the snapshot files are fetchable
curl -s -o /dev/null -w "snap0 %{http_code}\n" http://127.0.0.1:8099/data/sim-output_snapshots/snap_0000.bin
kill "$(cat /tmp/srv.pid)" 2>/dev/null
```
Expected: `renderWave` present in served JS; snapshot file `200`.

- [ ] **Step 4: Commit.**
```bash
git add viz/viz.js
git commit -m "feat: wave animation view with playback controls"
```

---

## Task 7: End-to-end integration + verification

**Files:** none (verification only; optional `README` note)

Exercise the whole pipeline and confirm every endpoint the viz depends on works. Visual confirmation in a browser is the user's final check; everything machine-verifiable is checked here.

- [ ] **Step 1: Full build + suite.**
Run: `zig build` and `zig build test`
Expected: clean build; all tests pass (snapshots + server pure-logic tests included; the heavy validation test stays env-gated/skipped).

- [ ] **Step 2: Produce data with snapshots and serve it.**
```bash
rm -rf /tmp/viz-demo && mkdir -p /tmp/viz-demo
zig build run -Doptimize=ReleaseFast -- simulate --config configs/tiny.json --output /tmp/viz-demo/sim-output --save-snapshots --snapshot-interval 50
( ./zig-out/bin/rfid-sim serve --dir /tmp/viz-demo --port 8100 & echo $! > /tmp/viz.pid ); sleep 1
```

- [ ] **Step 3: Verify every endpoint the four views use.**
```bash
code() { curl -s -o /dev/null -w "%{http_code} %{content_type}" "$1"; echo "  <- $1"; }
code http://127.0.0.1:8100/
code http://127.0.0.1:8100/viz.js
code http://127.0.0.1:8100/style.css
code http://127.0.0.1:8100/data/sim-output.json
code http://127.0.0.1:8100/data/sim-output.bin
code http://127.0.0.1:8100/data/sim-output_snapshots/snapshots.json
code http://127.0.0.1:8100/data/sim-output_snapshots/snap_0000.bin
# sanity: the JSON the room view parses has the expected shape
curl -s http://127.0.0.1:8100/data/sim-output.json | python3 -c "import sys,json;d=json.load(sys.stdin);print('room view ok:', all(k in d for k in ('config','antennas','samples','impulse_length')), 'tags', len(d['samples']))"
kill "$(cat /tmp/viz.pid)" 2>/dev/null
```
Expected: `200` for all seven endpoints with correct content types (html/js/css/json/octet-stream/json/octet-stream); `room view ok: True`.

- [ ] **Step 4: Manual visual check (user).** Print the instruction for the user to eyeball the four views:
```bash
echo "Open http://127.0.0.1:8100/ in a browser after running:"
echo "  ./zig-out/bin/rfid-sim serve --dir /tmp/viz-demo --port 8100"
echo "Check: Room Layout draws walls/obstacle/4? antennas/tag dots and click selects;"
echo "       Wave Animation plays the Ez heatmap; Coverage colors tags per antenna;"
echo "       Impulse Response plots waveforms for selected tags."
```
(No commit needed unless a README is added.)

- [ ] **Step 5 (optional): Add a short usage note.** If desired, create/append `README.md` documenting the three commands (`simulate`, `combine`, `serve`) and the `?data=<base>` query param, then:
```bash
git add README.md
git commit -m "docs: usage notes for simulate/combine/serve"
```

---

## Self-Review (completed during plan authoring)

**Spec coverage (Component 4 Visualizer + `serve` CLI + snapshot output):**

| Spec requirement | Task |
|------------------|------|
| `--save-snapshots` / `--snapshot-interval` save full Ez field at interval | Task 1 (capture) + Task 2 (wiring) |
| `snapshots/` as float32 binary grids, one per saved step | Task 1 (`snap_NNNN.bin`, nx*ny float32) |
| `serve --dir --port`: HTTP server for viz assets + data dir | Task 4 |
| Visualizer view 1 — Room Layout (walls, obstacles, antennas, tag grid, click-select) | Task 3 |
| Visualizer view 2 — Wave Animation (Ez snapshots heatmap, play/pause/speed/scrub) | Task 6 |
| Visualizer view 3 — Coverage Heatmap (peak amplitude per tag, per antenna) | Task 5 |
| Visualizer view 4 — Impulse Response Plot (per-antenna waveforms, overlay multiple tags) | Task 5 |
| Serves both `viz/` static assets and the data directory | Task 4 (embedded assets + `/data/`) |

**Type/contract consistency:** `snapshots.capture` (Task 1) signature is used verbatim by `cmdSimulate` (Task 2). The snapshot manifest fields (`nx,ny,dx,dt,interval,tag_x,tag_y,count,steps,files`) written in Task 1 are read by `renderWave` (Task 6). `resolveRoute`/`contentType` (Task 4) are pinned by tests and consumed by the serve loop. The viz `impulseFor(s,a)` math (`samples[s].offset + a*N*4`, `N=impulse_length`) matches the Plan 1 bin layout and the combiner's reader. `@embedFile("../viz/…")` requires Task 3 (create viz files) before Task 4 (server) — ordering is correct.

**Known limitations / decisions recorded:**
- Snapshots are captured for **one** tag position (the first valid one) per run — a single propagation animation, keeping storage bounded. Documented in Task 2.
- The server is **single-threaded** and reads each `/data/` file fully into memory (`readFileAlloc`, 1 GB cap). Fine for a localhost viz tool with the spec's file sizes; flagged for a future streaming/Range refinement if very large bins must be served.
- The viz fetches the whole `.bin` once and slices client-side. For very large sweeps this is heavy in the browser; the demo configs are small. Flagged.
- Path-traversal guard rejects `..` and absolute `/data/` subpaths (sufficient for a localhost dev tool).
- **Testing departure:** the browser/Canvas tasks (3, 5, 6) have no JS test runner in this environment, so they use `curl`/`grep` plumbing checks + a final user visual check rather than unit tests. All **Zig** logic that can be unit-tested (snapshot capture, route resolution, content-type) is strict test-first.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-24-rfid-visualizer.md`. This is the largest plan (backend + frontend). Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task, two-stage review on the Zig tasks; the frontend tasks reviewed for data-contract correctness + curl plumbing. Test-first for the Zig logic.
2. **Inline Execution** — batch with checkpoints.

It could also reasonably be split for execution into **3a (snapshots + server, fully Zig, test-first/curl-verified)** and **3b (the four browser views)** if you prefer a smaller first increment. Completing this plan finishes the full RFID FDTD system: `simulate` + `combine` + `serve`/visualize.
