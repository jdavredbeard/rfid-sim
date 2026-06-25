# Multi-Scene Datasets + Visualizer Switcher Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a curated set of hand-authored scenes, explicit per-scene tag placement, a `scenes` batch command that emits a manifest, and a visualizer dropdown to switch between scene datasets.

**Architecture:** Reuse the existing single-scene `runSweep` path. A new config field (`tags`) adds explicit tag coordinates; a new `scenes` CLI command runs every config in a directory and writes a `scenes.json` manifest alongside each `<name>.json`/`.bin`. The visualizer reads the manifest to populate a dropdown and reloads the chosen base on switch — falling back to today's single-dataset behavior when no manifest exists. No FDTD engine or HTTP server changes.

**Tech Stack:** Zig 0.14 (no external deps), vanilla JS/HTML/CSS visualizer.

**Spec:** `docs/superpowers/specs/2026-06-25-multi-scene-datasets.md`

---

## File Structure

- `src/config.zig` — add `TagPoint`, optional `tag_grid_spacing`, `tags`, `snapshots`, `label`, `description`; new validation rules. (modify)
- `src/generator.zig` — `tagPositions` uses explicit `tags` when present. (modify)
- `src/output.zig` — `writeScenesManifest` helper. (modify)
- `src/main.zig` — `scenes` command dispatch + `cmdScenes`. (modify)
- `configs/scenes/*.json` — 6 hand-authored scene configs. (create)
- `src/viz/index.html` — dataset `<select>` in header. (modify)
- `src/viz/viz.js` — manifest load + switcher. (modify)
- `src/viz/style.css` — minimal dropdown styling. (modify)
- `README.md` — document `scenes` command + switcher. (modify)

---

## Task 1: Config — explicit tags, optional spacing, scene metadata

**Files:**
- Modify: `src/config.zig`
- Test: `src/config.zig` (inline `test` blocks, same file)

- [ ] **Step 1: Write failing tests**

Add these tests at the end of `src/config.zig` (before the final `}` of the file is not needed — tests are top-level):

```zig
const explicit_tags_json =
    \\{
    \\  "room": { "width": 6.0, "height": 6.0 },
    \\  "grid_resolution": 0.05,
    \\  "materials": { "metal": { "epsilon_r": 1.0, "sigma": 1e7 } },
    \\  "walls": [],
    \\  "obstacles": [],
    \\  "antennas": [ { "x": 0.5, "y": 0.5, "label": "a" } ],
    \\  "source": { "type": "gaussian_pulse", "center_freq": 915e6, "bandwidth": 200e6 },
    \\  "tags": [ { "x": 1.0, "y": 1.0 }, { "x": 2.0, "y": 3.0 } ],
    \\  "timesteps": 100,
    \\  "label": "Tiny",
    \\  "description": "two explicit tags"
    \\}
;

test "parse explicit tags and optional spacing" {
    var parsed = try parse(std.testing.allocator, explicit_tags_json);
    defer parsed.deinit();
    const cfg = parsed.value;
    try std.testing.expectEqual(@as(usize, 2), cfg.tags.len);
    try std.testing.expectEqual(@as(f64, 1.0), cfg.tags[0].x);
    try std.testing.expectEqual(@as(?f64, null), cfg.tag_grid_spacing);
    try std.testing.expectEqualStrings("Tiny", cfg.label.?);
    try std.testing.expectEqualStrings("two explicit tags", cfg.description.?);
    try validate(cfg);
}

test "validate rejects tag outside room" {
    var parsed = try parse(std.testing.allocator, explicit_tags_json);
    defer parsed.deinit();
    parsed.value.tags[0].y = 99.0;
    try std.testing.expectError(ValidationError.TagOutsideRoom, validate(parsed.value));
}

test "validate rejects config with no tag source" {
    var parsed = try parse(std.testing.allocator, explicit_tags_json);
    defer parsed.deinit();
    parsed.value.tags = &.{};
    parsed.value.tag_grid_spacing = null;
    try std.testing.expectError(ValidationError.NoTagSource, validate(parsed.value));
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `zig build test`
Expected: FAIL — compile error (`TagPoint`/`tags`/`label` unknown; `TagOutsideRoom`/`NoTagSource` not in `ValidationError`).

- [ ] **Step 3: Add the `TagPoint` type and extend `Config`**

In `src/config.zig`, add after the `Antenna` struct (around line 35):

```zig
pub const TagPoint = struct {
    x: f64,
    y: f64,
};
```

Replace the `Config` struct (lines 43-53) with:

```zig
pub const Config = struct {
    room: Room,
    grid_resolution: f64,
    materials: std.json.ArrayHashMap(Material),
    walls: []Wall,
    obstacles: []Obstacle,
    antennas: []Antenna,
    source: Source,
    tag_grid_spacing: ?f64 = null, // uniform grid spacing; used only when `tags` is empty
    tags: []TagPoint = &.{}, // explicit tag positions; take precedence over the grid
    timesteps: u32,
    snapshots: bool = false, // opt in to wave-view snapshots in the `scenes` batch
    label: ?[]const u8 = null, // display name for the scenes manifest
    description: ?[]const u8 = null, // one-line blurb for the scenes manifest
};
```

- [ ] **Step 4: Add validation rules**

In `src/config.zig`, add the two error tags to `ValidationError` (the enum around line 102):

```zig
pub const ValidationError = error{
    EmptyRoom,
    BadResolution,
    UnknownMaterial,
    AntennaOutsideRoom,
    AntennaInObstacle,
    NoAntennas,
    TagOutsideRoom,
    NoTagSource,
};
```

Then in `validate`, just before the final closing brace of the function (after the antenna loop, around line 134), add:

```zig
    if (cfg.tags.len == 0 and cfg.tag_grid_spacing == null) return ValidationError.NoTagSource;
    for (cfg.tags) |t| {
        if (t.x < 0 or t.x > cfg.room.width or t.y < 0 or t.y > cfg.room.height) {
            return ValidationError.TagOutsideRoom;
        }
    }
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `zig build test`
Expected: PASS (all config tests, including the new three).

- [ ] **Step 6: Commit**

```bash
git add src/config.zig
git commit -m "feat: explicit tag list + optional grid spacing in config"
```

---

## Task 2: generator — use explicit tags when present

**Files:**
- Modify: `src/generator.zig` (`tagPositions`, around lines 33-47)
- Test: `src/generator.zig` (inline `test` blocks)

- [ ] **Step 1: Write failing test**

Add at the end of `src/generator.zig`:

```zig
test "explicit tags are used verbatim and obstacle tags skipped" {
    const cfg = config.Config{
        .room = .{ .width = 6.0, .height = 6.0 },
        .grid_resolution = 0.05,
        .materials = .{},
        .walls = &.{},
        .obstacles = &.{
            .{ .type = "rect", .x = 4.0, .y = 4.0, .w = 1.0, .h = 1.0, .material = "metal" },
        },
        .antennas = &.{.{ .x = 0.5, .y = 0.5, .label = "a" }},
        .source = .{ .type = "gaussian_pulse", .center_freq = 915e6, .bandwidth = 200e6 },
        .tags = &.{
            .{ .x = 1.0, .y = 1.0 }, // free -> kept
            .{ .x = 4.5, .y = 4.5 }, // inside the metal obstacle -> skipped
        },
        .timesteps = 100,
    };
    // materials map must contain "metal" for grid.build.
    var cfg2 = cfg;
    try cfg2.materials.map.put(std.testing.allocator, "metal", .{ .epsilon_r = 1.0, .sigma = 1e7 });
    defer cfg2.materials.map.deinit(std.testing.allocator);

    var g = try grid_mod.build(std.testing.allocator, cfg2);
    defer g.deinit();

    const positions = try tagPositions(std.testing.allocator, cfg2, g);
    defer std.testing.allocator.free(positions);

    try std.testing.expectEqual(@as(usize, 1), positions.len);
    try std.testing.expectEqual(@as(f64, 1.0), positions[0].x);
    try std.testing.expectEqual(@as(f64, 1.0), positions[0].y);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `zig build test`
Expected: FAIL — both explicit tags are ignored today (grid path uses `cfg.tag_grid_spacing` which is now `null`), so the count is wrong / it dereferences a null optional.

- [ ] **Step 3: Add the explicit-tag branch**

In `src/generator.zig`, inside `tagPositions`, the code currently reads (around line 33):

```zig
    const spacing = cfg.tag_grid_spacing;
    var y = spacing;
```

Replace those two lines and the loop start with an explicit-tag branch first, then the grid fallback. Insert immediately after the antenna-cells `for` loop (line 31, after the `}` that closes it) and before `const spacing`:

```zig
    // Explicit tag list takes precedence over the uniform grid.
    if (cfg.tags.len > 0) {
        for (cfg.tags) |t| {
            const c = grid.cellOf(t.x, t.y);
            const k = grid.idx(c.i, c.j);
            const free = grid.eps_r[k] == 1.0 and grid.sigma[k] == 0.0 and !grid.pec[k];
            if (!free) {
                std.debug.print("  warning: tag ({d},{d}) is inside a wall/obstacle; skipping\n", .{ t.x, t.y });
                continue;
            }
            if (antenna_cells.contains(k)) {
                std.debug.print("  warning: tag ({d},{d}) coincides with an antenna; skipping\n", .{ t.x, t.y });
                continue;
            }
            try list.append(.{ .x = t.x, .y = t.y, .i = c.i, .j = c.j });
        }
        return list.toOwnedSlice();
    }

    const spacing = cfg.tag_grid_spacing.?;
```

(Delete the old `const spacing = cfg.tag_grid_spacing;` line — it is replaced by the `.?` version above. The `var y = spacing;` and the rest of the grid loop stay unchanged.)

- [ ] **Step 4: Run tests to verify they pass**

Run: `zig build test`
Expected: PASS (new test plus all existing generator tests — the grid-path tests still set `tag_grid_spacing`, so the fallback is exercised).

- [ ] **Step 5: Commit**

```bash
git add src/generator.zig
git commit -m "feat: generator honors explicit tag positions"
```

---

## Task 3: output — scenes manifest writer

**Files:**
- Modify: `src/output.zig`
- Test: `src/output.zig` (inline `test`)

- [ ] **Step 1: Write failing test**

Add at the end of `src/output.zig`:

```zig
test "writeScenesManifest emits expected json" {
    const entries = [_]SceneEntry{
        .{ .name = "open-warehouse", .label = "Open Warehouse", .description = "big open floor" },
        .{ .name = "corridor", .label = "Corridor", .description = "" },
    };
    const path = "test-scenes.json";
    try writeScenesManifest(std.testing.allocator, path, &entries, "open-warehouse");
    defer std.fs.cwd().deleteFile(path) catch {};

    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, path, 1 << 16);
    defer std.testing.allocator.free(bytes);

    try std.testing.expect(std.mem.indexOf(u8, bytes, "\"name\": \"open-warehouse\"") != null);
    try std.testing.expect(std.mem.indexOf(u8, bytes, "\"label\": \"Corridor\"") != null);
    try std.testing.expect(std.mem.indexOf(u8, bytes, "\"default\": \"open-warehouse\"") != null);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `zig build test`
Expected: FAIL — `SceneEntry` and `writeScenesManifest` are undefined.

- [ ] **Step 3: Implement `SceneEntry` + `writeScenesManifest`**

Add to `src/output.zig` (after `writeJson`, around line 69):

```zig
pub const SceneEntry = struct {
    name: []const u8,
    label: []const u8,
    description: []const u8,
};

/// Write the scenes.json manifest listing every dataset the visualizer can load.
pub fn writeScenesManifest(
    allocator: std.mem.Allocator,
    path: []const u8,
    entries: []const SceneEntry,
    default_name: []const u8,
) !void {
    var buf = std.ArrayList(u8).init(allocator);
    defer buf.deinit();
    const w = buf.writer();

    try w.writeAll("{\n  \"scenes\": [\n");
    for (entries, 0..) |e, i| {
        try w.print(
            "    {{ \"name\": \"{s}\", \"label\": \"{s}\", \"description\": \"{s}\" }}",
            .{ e.name, e.label, e.description },
        );
        if (i + 1 != entries.len) try w.writeAll(",");
        try w.writeAll("\n");
    }
    try w.print("  ],\n  \"default\": \"{s}\"\n}}\n", .{default_name});

    try std.fs.cwd().writeFile(.{ .sub_path = path, .data = buf.items });
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `zig build test`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/output.zig
git commit -m "feat: scenes.json manifest writer"
```

---

## Task 4: `scenes` command

**Files:**
- Modify: `src/main.zig` (dispatch in `main`, new `cmdScenes`)

This task has no unit test (it is CLI glue over already-tested units); it is verified by a build + a real run in Step 4.

- [ ] **Step 1: Add dispatch + usage**

In `src/main.zig` `main()`, update the usage string (line 21) and add a branch. Change the `else if` chain (lines 25-33) to include `scenes`:

```zig
    if (std.mem.eql(u8, args[1], "simulate")) {
        try cmdSimulate(allocator, args[2..]);
    } else if (std.mem.eql(u8, args[1], "combine")) {
        try cmdCombine(allocator, args[2..]);
    } else if (std.mem.eql(u8, args[1], "scenes")) {
        try cmdScenes(allocator, args[2..]);
    } else if (std.mem.eql(u8, args[1], "serve")) {
        try cmdServe(allocator, args[2..]);
    } else {
        std.debug.print("unknown command: {s}\n", .{args[1]});
    }
```

- [ ] **Step 2: Implement `cmdScenes`**

Add this function in `src/main.zig` (e.g. after `cmdSimulate`, before `cmdCombine`):

```zig
fn lessThanStr(_: void, a: []const u8, b: []const u8) bool {
    return std.mem.lessThan(u8, a, b);
}

fn cmdScenes(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var dir_path: ?[]const u8 = null;
    var out_dir: []const u8 = ".";
    var threads: usize = std.Thread.getCpuCount() catch 1;

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--dir")) {
            i += 1;
            if (i >= args.len) { std.debug.print("error: --dir requires a value\n", .{}); return; }
            dir_path = args[i];
        } else if (std.mem.eql(u8, a, "--output")) {
            i += 1;
            if (i >= args.len) { std.debug.print("error: --output requires a value\n", .{}); return; }
            out_dir = args[i];
        } else if (std.mem.eql(u8, a, "--threads")) {
            i += 1;
            if (i >= args.len) { std.debug.print("error: --threads requires a value\n", .{}); return; }
            threads = try std.fmt.parseInt(usize, args[i], 10);
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }

    const scenes_dir = dir_path orelse {
        std.debug.print("error: --dir <scene config dir> is required\n", .{});
        return;
    };
    try std.fs.cwd().makePath(out_dir);

    // Collect *.json scene filenames, sorted for stable manifest order.
    var names = std.ArrayList([]const u8).init(allocator);
    defer {
        for (names.items) |n| allocator.free(n);
        names.deinit();
    }
    {
        var d = try std.fs.cwd().openDir(scenes_dir, .{ .iterate = true });
        defer d.close();
        var it = d.iterate();
        while (try it.next()) |entry| {
            if (entry.kind != .file) continue;
            if (!std.mem.endsWith(u8, entry.name, ".json")) continue;
            try names.append(try allocator.dupe(u8, entry.name));
        }
    }
    std.mem.sort([]const u8, names.items, {}, lessThanStr);

    if (names.items.len == 0) {
        std.debug.print("error: no *.json scene configs found in {s}\n", .{scenes_dir});
        return;
    }

    // Arena holds manifest strings (stem/label/description) until the manifest is written.
    var arena = std.heap.ArenaAllocator.init(allocator);
    defer arena.deinit();
    const aa = arena.allocator();

    var entries = std.ArrayList(output.SceneEntry).init(aa);
    var had_error = false;

    for (names.items) |fname| {
        const stem = fname[0 .. fname.len - ".json".len];
        const cfg_path = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ scenes_dir, fname });
        defer allocator.free(cfg_path);

        const cfg_json = std.fs.cwd().readFileAlloc(allocator, cfg_path, 16 << 20) catch |e| {
            std.debug.print("scene {s}: read failed: {s}\n", .{ fname, @errorName(e) });
            had_error = true;
            continue;
        };
        defer allocator.free(cfg_json);

        var parsed = config.parse(allocator, cfg_json) catch |e| {
            std.debug.print("scene {s}: parse failed: {s}\n", .{ fname, @errorName(e) });
            had_error = true;
            continue;
        };
        defer parsed.deinit();

        config.validate(parsed.value) catch |e| {
            std.debug.print("scene {s}: invalid config: {s}\n", .{ fname, @errorName(e) });
            had_error = true;
            continue;
        };

        var grid = grid_mod.build(allocator, parsed.value) catch |e| {
            std.debug.print("scene {s}: grid build failed: {s}\n", .{ fname, @errorName(e) });
            had_error = true;
            continue;
        };
        defer grid.deinit();

        grid_mod.checkAntennaPlacement(grid, parsed.value) catch |e| {
            std.debug.print("scene {s}: invalid config: {s}\n", .{ fname, @errorName(e) });
            had_error = true;
            continue;
        };

        const base = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ out_dir, stem });
        defer allocator.free(base);

        std.debug.print("scene {s}: {d}x{d} grid, {d} threads...\n", .{ stem, grid.nx, grid.ny, threads });
        const res = try generator.runSweep(allocator, parsed.value, &grid, cfg_json, base, threads);
        std.debug.print("scene {s}: {d} tags -> {s}.bin / {s}.json\n", .{ stem, res.num_samples, base, base });

        if (parsed.value.snapshots) {
            const positions = try generator.tagPositions(allocator, parsed.value, grid);
            defer allocator.free(positions);
            if (positions.len > 0) {
                const p = positions[0];
                const snap_dir = try std.fmt.allocPrint(allocator, "{s}_snapshots", .{base});
                defer allocator.free(snap_dir);
                try snapshots.capture(allocator, &grid, .{
                    .center_freq = parsed.value.source.center_freq,
                    .bandwidth = parsed.value.source.bandwidth,
                }, p.i, p.j, p.x, p.y, parsed.value.timesteps, 50, snap_dir);
                std.debug.print("scene {s}: snapshots -> {s}/\n", .{ stem, snap_dir });
            }
        }

        try entries.append(.{
            .name = try aa.dupe(u8, stem),
            .label = try aa.dupe(u8, parsed.value.label orelse stem),
            .description = try aa.dupe(u8, parsed.value.description orelse ""),
        });
    }

    if (entries.items.len == 0) {
        std.debug.print("error: no scenes succeeded; manifest not written\n", .{});
        return;
    }

    const manifest_path = try std.fmt.allocPrint(allocator, "{s}/scenes.json", .{out_dir});
    defer allocator.free(manifest_path);
    try output.writeScenesManifest(allocator, manifest_path, entries.items, entries.items[0].name);
    std.debug.print("wrote {s} ({d} scenes)\n", .{ manifest_path, entries.items.len });

    if (had_error) std.debug.print("note: some scenes failed; see messages above\n", .{});
}
```

Add the `output` import at the top of `src/main.zig` if not present. Check line 1-10; if `const output = @import("output.zig");` is missing, add it.

- [ ] **Step 3: Build**

Run: `zig build -Doptimize=ReleaseFast && zig build test`
Expected: both exit 0.

- [ ] **Step 4: Smoke-test against a temp scene dir**

Run:
```bash
mkdir -p /tmp/scenes-smoke
cp configs/tiny.json /tmp/scenes-smoke/tiny-a.json
cp configs/tiny.json /tmp/scenes-smoke/tiny-b.json
./zig-out/bin/rfid-sim scenes --dir /tmp/scenes-smoke --output /tmp/scenes-out
ls /tmp/scenes-out
cat /tmp/scenes-out/scenes.json
```
Expected: `tiny-a.json tiny-a.bin tiny-b.json tiny-b.bin scenes.json` present; `scenes.json` lists both scenes with `"default": "tiny-a"`.

- [ ] **Step 5: Commit**

```bash
git add src/main.zig
git commit -m "feat: scenes batch command writes datasets + manifest"
```

---

## Task 5: Author the 6 scene configs

**Files:**
- Create: `configs/scenes/open-warehouse.json`, `l-shaped-store.json`, `narrow-corridor.json`, `cluttered-backroom.json`, `glass-atrium.json`, `small-office.json`
- Test: `src/config.zig` (parametrized validation test)

All scenes use `grid_resolution: 0.015`, keep explicit tag counts ≤ ~30, and set `timesteps` sized to the room. Each file includes `label` and `description`. Use this template and adjust room/walls/obstacles/antennas/tags per the spec table. Example (`open-warehouse.json`):

```json
{
  "label": "Open Warehouse",
  "description": "Large open floor, 4 corner antennas, sparse scattered tags",
  "room": { "width": 12.0, "height": 12.0 },
  "grid_resolution": 0.015,
  "materials": {
    "concrete": { "epsilon_r": 4.5, "sigma": 0.02 },
    "metal": { "epsilon_r": 1.0, "sigma": 1e7 }
  },
  "walls": [
    { "x1": 0, "y1": 0, "x2": 12, "y2": 0, "material": "concrete", "thickness": 0.2 },
    { "x1": 12, "y1": 0, "x2": 12, "y2": 12, "material": "concrete", "thickness": 0.2 },
    { "x1": 12, "y1": 12, "x2": 0, "y2": 12, "material": "concrete", "thickness": 0.2 },
    { "x1": 0, "y1": 12, "x2": 0, "y2": 0, "material": "concrete", "thickness": 0.2 }
  ],
  "obstacles": [
    { "type": "rect", "x": 5.0, "y": 5.0, "w": 2.0, "h": 2.0, "material": "metal" }
  ],
  "antennas": [
    { "x": 0.5, "y": 0.5, "label": "ant1" },
    { "x": 11.5, "y": 0.5, "label": "ant2" },
    { "x": 0.5, "y": 11.5, "label": "ant3" },
    { "x": 11.5, "y": 11.5, "label": "ant4" }
  ],
  "source": { "type": "gaussian_pulse", "center_freq": 915e6, "bandwidth": 200e6 },
  "tags": [
    { "x": 2.0, "y": 2.0 }, { "x": 9.0, "y": 2.5 }, { "x": 3.5, "y": 8.0 },
    { "x": 8.5, "y": 9.0 }, { "x": 6.0, "y": 3.0 }, { "x": 2.5, "y": 10.0 },
    { "x": 10.0, "y": 6.0 }, { "x": 6.5, "y": 10.5 }
  ],
  "timesteps": 6000
}
```

For the other five, follow the spec §1 table:
- `l-shaped-store.json`: add interior `walls` that block a rectangular corner to form an L; shelving `obstacles` (metal rects); 3 antennas; ~12 tags clustered along the two aisles.
- `narrow-corridor.json`: `room` ~ `{ "width": 20.0, "height": 3.0 }`; 2 antennas at the ends; a line of ~10 tags down the middle; 1-2 metal blocks.
- `cluttered-backroom.json`: small room ~ `{ "width": 8.0, "height": 8.0 }`; many metal shelf `obstacles`; 4 wall-mounted antennas; ~15 clustered tags between shelves. Set `"snapshots": true` here so the wave view has a showcase scene.
- `glass-atrium.json`: add a `glass` material `{ "epsilon_r": 6.0, "sigma": 0.004 }`; interior glass partition `walls`; 4 antennas; ~12 tags spread loosely.
- `small-office.json`: add a `drywall` material `{ "epsilon_r": 2.1, "sigma": 0.001 }`; drywall partition `walls` dividing the room into ~3 sub-rooms; 3 antennas; ~9 tags (a few per sub-room).

Keep every tag strictly inside the room and clear of walls/obstacles/antenna cells (the runner warns and skips otherwise — watch the `scenes` output for skip warnings and nudge coordinates if any appear).

- [ ] **Step 1: Write failing parametrized validation test**

Add to `src/config.zig`:

```zig
test "all scene configs parse and validate" {
    var dir = try std.fs.cwd().openDir("configs/scenes", .{ .iterate = true });
    defer dir.close();
    var it = dir.iterate();
    var count: usize = 0;
    while (try it.next()) |entry| {
        if (entry.kind != .file) continue;
        if (!std.mem.endsWith(u8, entry.name, ".json")) continue;
        const bytes = try dir.readFileAlloc(std.testing.allocator, entry.name, 1 << 20);
        defer std.testing.allocator.free(bytes);
        var parsed = try parse(std.testing.allocator, bytes);
        defer parsed.deinit();
        try validate(parsed.value);
        count += 1;
    }
    try std.testing.expect(count >= 6);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `zig build test`
Expected: FAIL — `configs/scenes` does not exist yet (or `count < 6`).

- [ ] **Step 3: Create the 6 config files**

Create `configs/scenes/` and write all six JSON files per the template and the bullet adjustments above.

- [ ] **Step 4: Run test to verify it passes**

Run: `zig build test`
Expected: PASS (`count >= 6`).

- [ ] **Step 5: Build the scene datasets and check for skipped tags**

Run:
```bash
zig build -Doptimize=ReleaseFast
./zig-out/bin/rfid-sim scenes --dir configs/scenes --output viz-data
```
Expected: each scene reports its tag count with **no "skipping" warnings**; `viz-data/scenes.json` lists all 6. If any tag is skipped, adjust that coordinate in the config and re-run.

- [ ] **Step 6: Commit**

```bash
git add configs/scenes src/config.zig
git commit -m "feat: 6 hand-authored varied scene configs"
```

---

## Task 6: Visualizer dataset switcher

**Files:**
- Modify: `src/viz/index.html`, `src/viz/viz.js`, `src/viz/style.css`

No automated test (no JS harness); verified manually via `serve` in Step 5.

- [ ] **Step 1: Add the dropdown to the header**

In `src/viz/index.html`, replace the `<header>` block (lines 9-12) with:

```html
  <header>
    <h1>RFID FDTD Visualizer</h1>
    <select id="dataset-select" class="hidden"></select>
    <div id="status">loading…</div>
  </header>
```

- [ ] **Step 2: Make `BASE` reassignable and track the current view**

In `src/viz/viz.js`, change line 5 from `const BASE = ...` to:

```js
let BASE = params.get("data") || "sim-output";
let currentView = "room";
```

In `showView(name)` (around line 340), add as the first line of the function body:

```js
  currentView = name;
```

- [ ] **Step 3: Add manifest load + switcher functions**

In `src/viz/viz.js`, add these functions just above `async function main()` (around line 353):

```js
async function loadScenes() {
  try {
    const r = await fetch("/data/scenes.json");
    if (!r.ok) return null;
    return await r.json();
  } catch {
    return null;
  }
}

function populateDatasetSelect(manifest) {
  const sel = document.getElementById("dataset-select");
  sel.innerHTML = "";
  for (const s of manifest.scenes) {
    const opt = document.createElement("option");
    opt.value = s.name;
    opt.textContent = s.label || s.name;
    if (s.description) opt.title = s.description;
    sel.appendChild(opt);
  }
  sel.value = BASE;
  sel.classList.remove("hidden");
  sel.addEventListener("change", () => switchDataset(sel.value));
}

async function switchDataset(name) {
  BASE = name;
  const u = new URL(location);
  u.searchParams.set("data", name);
  history.replaceState(null, "", u);
  state.meta = null;
  state.bin = null;
  state.snapshots = null;
  state.selected = [];
  try {
    await loadMeta();
    setStatus(`loaded ${BASE}.json — ${state.meta.samples.length} tags, ${state.meta.antennas.length} antennas`);
    showView(currentView);
  } catch (e) {
    setStatus("error: " + e.message);
    console.error(e);
  }
}
```

- [ ] **Step 4: Wire manifest load into `main()`**

In `src/viz/viz.js`, replace the body of `main()` (lines 353-363) with:

```js
async function main() {
  wireTabs();
  const manifest = await loadScenes();
  if (manifest && manifest.scenes && manifest.scenes.length) {
    if (!params.get("data")) BASE = manifest.default || manifest.scenes[0].name;
    populateDatasetSelect(manifest);
  }
  try {
    await loadMeta();
    setStatus(`loaded ${BASE}.json — ${state.meta.samples.length} tags, ${state.meta.antennas.length} antennas`);
    showView("room");
  } catch (e) {
    setStatus("error: " + e.message);
    console.error(e);
  }
}
```

- [ ] **Step 5: Style the dropdown**

In `src/viz/style.css`, append:

```css
#dataset-select {
  margin-left: 1rem;
  font-size: 1rem;
  padding: 0.2rem 0.4rem;
}
#dataset-select.hidden { display: none; }
```

(If a generic `.hidden { display: none; }` already exists in this file, the second rule is redundant but harmless — keep it for clarity.)

- [ ] **Step 6: Manual verification**

Run (uses the `viz-data` dir built in Task 5):
```bash
zig build -Doptimize=ReleaseFast
./zig-out/bin/rfid-sim serve --dir viz-data --port 8080
```
Then open `http://127.0.0.1:8080/` and confirm:
- The dropdown appears with all 6 scene labels.
- Selecting a different scene reloads the Room Layout (different walls/obstacles/antennas/tags) and updates the URL to `?data=<name>`.
- Switch to Coverage / Impulse tabs, change scene, and confirm they re-render for the new scene.
- Open `http://127.0.0.1:8080/?data=cluttered-backroom` directly and confirm the dropdown preselects it and the Wave tab plays (snapshots scene).
- Stop the server (Ctrl-C).

- [ ] **Step 7: Commit**

```bash
git add src/viz/index.html src/viz/viz.js src/viz/style.css
git commit -m "feat: visualizer dataset switcher driven by scenes.json"
```

---

## Task 7: Docs

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Document the `scenes` command and switcher**

In `README.md`, add a `### scenes` subsection under "## Commands" (after the `simulate` section), describing:

```
### `scenes` — build a set of varied scene datasets

Runs every `*.json` config in a directory and writes one dataset per scene plus a
`scenes.json` manifest into the output dir.

    rfid-sim scenes --dir configs/scenes --output viz-data [--threads N]

Each scene config is the normal simulate config, optionally with `"tags"` (an
explicit list of `{x, y}` positions, used instead of `tag_grid_spacing`),
`"label"`/`"description"` (shown in the visualizer dropdown), and
`"snapshots": true` (capture wave-animation snapshots for this scene).

Serve the output dir and the visualizer shows a dataset dropdown to switch
between scenes:

    rfid-sim serve --dir viz-data --port 8080
```

Also update the `## Config` section to mention the optional `tags`, `label`,
`description`, and `snapshots` fields, and note that `tag_grid_spacing` is
optional when `tags` is provided.

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: document scenes command and dataset switcher"
```

---

## Final verification

- [ ] Run full suite: `zig build test` → all pass.
- [ ] Release build: `zig build -Doptimize=ReleaseFast` → exit 0.
- [ ] `./zig-out/bin/rfid-sim scenes --dir configs/scenes --output viz-data` → 6 datasets + manifest, no skip warnings.
- [ ] `rfid-sim serve --dir viz-data` → dropdown switches between all 6 scenes across all four views.
```
