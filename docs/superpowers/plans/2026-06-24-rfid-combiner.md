# RFID Superposition Combiner Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Per the team's TDD preference, the substantive tasks here are written **strict test-first**: write the test, run it to see a genuine RED failure, then implement to GREEN, then commit.

**Goal:** Add the `combine` command that turns the single-tag impulse-response data produced by `simulate` (Plan 1) into multi-tag training data via superposition + additive Gaussian noise, written as `training-data.json` + `training-data.bin`.

**Architecture:** A post-processing tool. It loads `sim-output.json` (metadata + per-sample byte offsets) and the entire `sim-output.bin` (packed float32) into memory. For each of `num_samples` training samples it: picks a random count of distinct tags (seeded PRNG), sums their per-antenna impulse responses (linearity of Maxwell's equations), adds Gaussian noise at a random target SNR, appends the combined waveform to `training-data.bin`, and records a label `{offset, tags:[{x,y}...], snr_db}`. The combiner is I/O-bound; a full 50K-sample run takes seconds. Reproducible given the config `seed`.

**Tech Stack:** Zig 0.14.0 (`std.json`, `std.Random`, `std.fs`). Reuses `output.appendSample` from Plan 1 for the binary layout (identical per-sample format: `[ant1[0..N], ant2[0..N], ..., antM[0..N]]`, little-endian float32). No new external dependencies.

**Scope note:** This is Plan 2 of 3. Plan 1 (core simulator) is merged to `main`. Plan 3 (Visualizer + `serve`) is a separate follow-on. The `serve` command and any visualization are OUT of scope here.

**Builds on (already on `main`):**
- `src/output.zig` exposes `pub fn appendSample(file: std.fs.File, responses: []const []const f32) !u64` (returns the byte offset written) and `pub const GridMeta`/`SampleMeta`. The combiner reuses `appendSample` for `training-data.bin` and adds new training-data JSON writers here.
- `sim-output.json` shape (confirmed from a real run): keys `version, config, grid{nx,ny,dx,dt}, antennas:[string], impulse_length:int, samples:[{tag_x,tag_y,offset}]`. `offset` is the **byte** offset into `sim-output.bin`. Per sample the bin holds `num_antennas × impulse_length` float32 laid out antenna-major.
- `src/main.zig` dispatches subcommands by `std.mem.eql(u8, args[1], "...")` and has `cmdSimulate`. This plan adds a `combine` branch + `cmdCombine`.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `src/combine_config.zig` | Parse + validate `combine-config.json` → typed `CombineConfig`. |
| `src/simdata.zig` | Load `sim-output.json` + `sim-output.bin` into memory; accessor for a tag's per-antenna impulse response. Owns plain copied arrays (not the json.Parsed) so it is directly constructible in tests. |
| `src/output.zig` (modify) | Add `TagLabel`, `TrainingSampleMeta`, and `writeTrainingJson` next to the existing sim writers. |
| `src/combiner.zig` | Pure `superposeInto` / `addNoiseInto`, and the `generate` orchestration (PRNG, sample loop, file writes). |
| `src/main.zig` (modify) | Add `combine` dispatch + `cmdCombine` (flags: `--input`, `--config`, `--output`). |
| `configs/combine-example.json` | Example combiner config. |

Import direction: `simdata.zig` and `combine_config.zig` are leaf modules (only `std` + `config`/`output` as needed). `combiner.zig` imports `simdata`, `combine_config`, `output`. `main.zig` imports all. The heavy `RUN_VALIDATION_TEST`-gated test from Plan 1 is unaffected.

**Testing convention (unchanged from Plan 1):** run a single file's tests with `zig test src/<file>.zig`; the full suite runs via `zig build test` (aggregated through `main.zig`'s `refAllDeclsRecursive`). Use the same per-allocation `errdefer` discipline established in Plan 1 (no `errdefer x.deinit()` placed *after* a struct literal that contains fallible allocations).

---

## Task 1: Combine config parsing and validation

**Files:**
- Create: `src/combine_config.zig`

The combiner config (per the spec):
```json
{ "source": "sim-output", "num_samples": 50000, "tags_per_sample": { "min": 1, "max": 5 }, "noise_snr_db": { "min": 10, "max": 40 }, "seed": 42 }
```
`source` is informational (the actual input path comes from the `--input` CLI flag); it is parsed but not required to be non-empty.

- [ ] **Step 1: Write ONLY the tests first.** Create `src/combine_config.zig`:

```zig
const std = @import("std");

// ===== TESTS (written first) =====

const good_json =
    \\{
    \\  "source": "sim-output",
    \\  "num_samples": 50000,
    \\  "tags_per_sample": { "min": 1, "max": 5 },
    \\  "noise_snr_db": { "min": 10, "max": 40 },
    \\  "seed": 42
    \\}
;

test "parse combine config fields" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    const c = parsed.value;
    try std.testing.expectEqual(@as(u32, 50000), c.num_samples);
    try std.testing.expectEqual(@as(u32, 1), c.tags_per_sample.min);
    try std.testing.expectEqual(@as(u32, 5), c.tags_per_sample.max);
    try std.testing.expectEqual(@as(f64, 10), c.noise_snr_db.min);
    try std.testing.expectEqual(@as(f64, 40), c.noise_snr_db.max);
    try std.testing.expectEqual(@as(u64, 42), c.seed);
}

test "validate accepts good config" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    try validate(parsed.value);
}

test "validate rejects zero samples" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.num_samples = 0;
    try std.testing.expectError(ValidationError.NoSamples, validate(parsed.value));
}

test "validate rejects inverted tag range" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.tags_per_sample = .{ .min = 5, .max = 1 };
    try std.testing.expectError(ValidationError.BadTagRange, validate(parsed.value));
}

test "validate rejects zero min tags" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.tags_per_sample = .{ .min = 0, .max = 3 };
    try std.testing.expectError(ValidationError.BadTagRange, validate(parsed.value));
}

test "validate rejects inverted snr range" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.noise_snr_db = .{ .min = 40, .max = 10 };
    try std.testing.expectError(ValidationError.BadSnrRange, validate(parsed.value));
}
```

- [ ] **Step 2: Run RED.** Run: `zig test src/combine_config.zig`
Expected: compile error — `parse`, `validate`, `ValidationError`, the config types are undefined. Paste as RED.

- [ ] **Step 3: Implement.** Insert ABOVE the tests (after the `std` import):

```zig
pub const IntRange = struct { min: u32, max: u32 };
pub const FloatRange = struct { min: f64, max: f64 };

pub const CombineConfig = struct {
    source: []const u8 = "",
    num_samples: u32,
    tags_per_sample: IntRange,
    noise_snr_db: FloatRange,
    seed: u64,
};

pub const ParsedConfig = std.json.Parsed(CombineConfig);

/// Parse a combine config from JSON. Caller owns the returned Parsed (call `.deinit()`).
pub fn parse(allocator: std.mem.Allocator, json: []const u8) !ParsedConfig {
    return std.json.parseFromSlice(CombineConfig, allocator, json, .{
        .ignore_unknown_fields = true,
    });
}

pub const ValidationError = error{
    NoSamples,
    BadTagRange,
    BadSnrRange,
};

pub fn validate(cfg: CombineConfig) ValidationError!void {
    if (cfg.num_samples == 0) return ValidationError.NoSamples;
    if (cfg.tags_per_sample.min == 0 or cfg.tags_per_sample.min > cfg.tags_per_sample.max) {
        return ValidationError.BadTagRange;
    }
    if (cfg.noise_snr_db.min > cfg.noise_snr_db.max) return ValidationError.BadSnrRange;
}
```

- [ ] **Step 4: Run GREEN.** Run: `zig test src/combine_config.zig`
Expected: all 6 tests PASS.

- [ ] **Step 5: Commit.**
```bash
git add src/combine_config.zig
git commit -m "feat: combine config parsing and validation"
```

**Zig 0.14 note:** if a test's `var parsed`/`parsed.value` mutation triggers a never-mutated complaint, keep `var parsed` (it is mutated via `parsed.value.x = ...`). Do not weaken assertions.

---

## Task 2: SimData loader

**Files:**
- Create: `src/simdata.zig`

Loads `<base>.json` (the sim-output metadata/index) and `<base>.bin` (the whole packed float32 buffer) into memory. `SimData` holds **plain owned arrays** copied out of the parsed JSON (so it can be constructed directly in unit tests, and its lifetime is independent of `std.json.Parsed`).

Accessor math: for tag (sample) index `s` and antenna `a`, the impulse response is `data[ offset_floats + a*N .. + N ]` where `offset_floats = offsets[s] / 4` (offset is a **byte** offset; 4 bytes per float32) and `N = impulse_length`.

- [ ] **Step 1: Write ONLY the tests first.** Create `src/simdata.zig`:

```zig
const std = @import("std");
const output = @import("output.zig");

// ===== TESTS (written first) =====

test "impulse accessor slices the right antenna-major window" {
    // 2 samples, 2 antennas, impulse_length 3.
    // sample0: ant0=[1,2,3] ant1=[4,5,6]; sample1: ant0=[7,8,9] ant1=[10,11,12]
    var data = [_]f32{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    var tx = [_]f64{ 0.1, 0.2 };
    var ty = [_]f64{ 0.3, 0.4 };
    var offs = [_]u64{ 0, 24 }; // sample1 starts at 2 ant * 3 floats * 4 bytes = 24
    const sim = SimData{
        .allocator = std.testing.allocator,
        .num_antennas = 2,
        .impulse_length = 3,
        .tag_x = &tx,
        .tag_y = &ty,
        .offsets = &offs,
        .data = &data,
    };
    try std.testing.expectEqual(@as(usize, 2), sim.numSamples());
    try std.testing.expectEqualSlices(f32, &[_]f32{ 1, 2, 3 }, sim.impulse(0, 0));
    try std.testing.expectEqualSlices(f32, &[_]f32{ 4, 5, 6 }, sim.impulse(0, 1));
    try std.testing.expectEqualSlices(f32, &[_]f32{ 7, 8, 9 }, sim.impulse(1, 0));
    try std.testing.expectEqualSlices(f32, &[_]f32{ 10, 11, 12 }, sim.impulse(1, 1));
}

test "load round-trips a written sim-output" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir);
    const base = try std.fs.path.join(std.testing.allocator, &.{ dir, "sim" });
    defer std.testing.allocator.free(base);

    // Write a tiny sim-output using the Plan 1 writers.
    const bin_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.bin", .{base});
    defer std.testing.allocator.free(bin_path);
    const json_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.json", .{base});
    defer std.testing.allocator.free(json_path);

    const file = try std.fs.cwd().createFile(bin_path, .{});
    const a0 = [_]f32{ 1, 2 };
    const a1 = [_]f32{ 3, 4 };
    const s0 = [_][]const f32{ &a0, &a1 };
    const off0 = try output.appendSample(file, &s0);
    const b0 = [_]f32{ 5, 6 };
    const b1 = [_]f32{ 7, 8 };
    const s1 = [_][]const f32{ &b0, &b1 };
    const off1 = try output.appendSample(file, &s1);
    file.close();

    const labels = [_][]const u8{ "ant1", "ant2" };
    const metas = [_]output.SampleMeta{
        .{ .tag_x = 1.0, .tag_y = 2.0, .offset = off0 },
        .{ .tag_x = 3.0, .tag_y = 4.0, .offset = off1 },
    };
    try output.writeJson(std.testing.allocator, json_path, "{}", .{ .nx = 10, .ny = 10, .dx = 0.03, .dt = 1.0e-11 }, &labels, 2, &metas);

    var sim = try load(std.testing.allocator, base);
    defer sim.deinit();
    try std.testing.expectEqual(@as(usize, 2), sim.num_antennas);
    try std.testing.expectEqual(@as(usize, 2), sim.impulse_length);
    try std.testing.expectEqual(@as(usize, 2), sim.numSamples());
    try std.testing.expectEqual(@as(f64, 3.0), sim.tag_x[1]);
    try std.testing.expectEqualSlices(f32, &[_]f32{ 5, 6 }, sim.impulse(1, 0));
    try std.testing.expectEqualSlices(f32, &[_]f32{ 7, 8 }, sim.impulse(1, 1));
}
```

- [ ] **Step 2: Run RED.** Run: `zig test src/simdata.zig`
Expected: compile error — `SimData`, `load` undefined. Paste as RED.

- [ ] **Step 3: Implement.** Insert ABOVE the tests:

```zig
pub const SimData = struct {
    allocator: std.mem.Allocator,
    num_antennas: usize,
    impulse_length: usize,
    tag_x: []f64,
    tag_y: []f64,
    offsets: []u64, // byte offsets into `data`-as-bytes, per sample
    data: []f32, // the entire bin as float32

    pub fn numSamples(self: SimData) usize {
        return self.tag_x.len;
    }

    /// Slice of `data` holding antenna `a`'s impulse response for tag/sample `s`.
    pub fn impulse(self: SimData, s: usize, a: usize) []const f32 {
        const n = self.impulse_length;
        const base = self.offsets[s] / 4 + a * n;
        return self.data[base .. base + n];
    }

    pub fn deinit(self: *SimData) void {
        self.allocator.free(self.tag_x);
        self.allocator.free(self.tag_y);
        self.allocator.free(self.offsets);
        self.allocator.free(self.data);
    }
};

// Subset of sim-output.json we need. ignore_unknown_fields skips config/grid/version extras.
const SimSampleMeta = struct { tag_x: f64, tag_y: f64, offset: u64 };
const SimMeta = struct {
    antennas: [][]const u8,
    impulse_length: u32,
    samples: []SimSampleMeta,
};

/// Load `<base>.json` + `<base>.bin` into memory. Caller must call `sim.deinit()`.
pub fn load(allocator: std.mem.Allocator, base: []const u8) !SimData {
    const json_path = try std.fmt.allocPrint(allocator, "{s}.json", .{base});
    defer allocator.free(json_path);
    const bin_path = try std.fmt.allocPrint(allocator, "{s}.bin", .{base});
    defer allocator.free(bin_path);

    const json_bytes = try std.fs.cwd().readFileAlloc(allocator, json_path, 64 << 20);
    defer allocator.free(json_bytes);

    var parsed = try std.json.parseFromSlice(SimMeta, allocator, json_bytes, .{
        .ignore_unknown_fields = true,
    });
    defer parsed.deinit();
    const meta = parsed.value;

    const ns = meta.samples.len;
    const tag_x = try allocator.alloc(f64, ns);
    errdefer allocator.free(tag_x);
    const tag_y = try allocator.alloc(f64, ns);
    errdefer allocator.free(tag_y);
    const offsets = try allocator.alloc(u64, ns);
    errdefer allocator.free(offsets);
    for (meta.samples, 0..) |sm, i| {
        tag_x[i] = sm.tag_x;
        tag_y[i] = sm.tag_y;
        offsets[i] = sm.offset;
    }

    const bin_file = try std.fs.cwd().openFile(bin_path, .{});
    defer bin_file.close();
    const sz = (try bin_file.stat()).size;
    const data = try allocator.alloc(f32, sz / 4);
    errdefer allocator.free(data);
    const read = try bin_file.readAll(std.mem.sliceAsBytes(data));
    if (read != sz) return error.ShortBinRead;

    return SimData{
        .allocator = allocator,
        .num_antennas = meta.antennas.len,
        .impulse_length = meta.impulse_length,
        .tag_x = tag_x,
        .tag_y = tag_y,
        .offsets = offsets,
        .data = data,
    };
}
```

- [ ] **Step 4: Run GREEN.** Run: `zig test src/simdata.zig`
Expected: both tests PASS.

- [ ] **Step 5: Commit.**
```bash
git add src/simdata.zig
git commit -m "feat: SimData loader for sim-output json + bin"
```

**Zig 0.14 notes:** `std.mem.sliceAsBytes` on `[]f32` yields the f32-aligned byte view to read into. `readFileAlloc`, `openFile`, `file.stat()`, `file.readAll` are 0.14 APIs. If `var sim`/`var parsed` mutability complaints appear, adjust minimally. Do not weaken assertions.

---

## Task 3: Training-data JSON writer

**Files:**
- Modify: `src/output.zig`

Add the training-data label/metadata writer next to the existing sim writers. The training-data **bin** uses the existing `appendSample` (identical layout), so no new bin writer is needed.

Target `training-data.json` shape (per spec):
```json
{ "version": 1, "source_config": "sim-output.json", "num_samples": 50000, "impulse_length": 8000, "num_antennas": 4,
  "samples": [ { "offset": 0, "snr_db": 25.3, "tags": [ { "x": 1.25, "y": 2.50 } ] } ] }
```

- [ ] **Step 1: Write the failing test.** Append to `src/output.zig`:

```zig
test "writeTrainingJson is parseable with correct counts and labels" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir);
    const full = try std.fs.path.join(std.testing.allocator, &.{ dir, "train.json" });
    defer std.testing.allocator.free(full);

    const tags0 = [_]TagLabel{ .{ .x = 1.25, .y = 2.5 }, .{ .x = 5.0, .y = 8.0 } };
    const tags1 = [_]TagLabel{.{ .x = 3.0, .y = 3.0 }};
    const samples = [_]TrainingSampleMeta{
        .{ .offset = 0, .tags = &tags0, .snr_db = 25.3 },
        .{ .offset = 64, .tags = &tags1, .snr_db = 12.0 },
    };
    try writeTrainingJson(std.testing.allocator, full, "sim-output.json", 8, 4, &samples);

    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, full, 1 << 20);
    defer std.testing.allocator.free(bytes);
    const parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, bytes, .{});
    defer parsed.deinit();
    const obj = parsed.value.object;
    try std.testing.expectEqual(@as(i64, 4), obj.get("num_antennas").?.integer);
    try std.testing.expectEqual(@as(i64, 8), obj.get("impulse_length").?.integer);
    const arr = obj.get("samples").?.array;
    try std.testing.expectEqual(@as(usize, 2), arr.items.len);
    // First sample has two tag labels.
    try std.testing.expectEqual(@as(usize, 2), arr.items[0].object.get("tags").?.array.items.len);
    try std.testing.expectEqualStrings("sim-output.json", obj.get("source_config").?.string);
}
```

- [ ] **Step 2: Run RED.** Run: `zig test src/output.zig`
Expected: compile error — `TagLabel`, `TrainingSampleMeta`, `writeTrainingJson` undefined. Paste as RED.

- [ ] **Step 3: Implement.** Add these to `src/output.zig` (top-level, e.g. after `writeJson`):

```zig
pub const TagLabel = struct { x: f64, y: f64 };

pub const TrainingSampleMeta = struct {
    offset: u64,
    tags: []const TagLabel,
    snr_db: f64,
};

/// Write the training-data.json labels/metadata file.
pub fn writeTrainingJson(
    allocator: std.mem.Allocator,
    path: []const u8,
    source_config: []const u8,
    impulse_length: u32,
    num_antennas: usize,
    samples: []const TrainingSampleMeta,
) !void {
    var buf = std.ArrayList(u8).init(allocator);
    defer buf.deinit();
    const w = buf.writer();

    try w.writeAll("{\n");
    try w.print("  \"version\": 1,\n", .{});
    try w.print("  \"source_config\": \"{s}\",\n", .{source_config});
    try w.print("  \"num_samples\": {d},\n", .{samples.len});
    try w.print("  \"impulse_length\": {d},\n", .{impulse_length});
    try w.print("  \"num_antennas\": {d},\n", .{num_antennas});
    try w.writeAll("  \"samples\": [\n");
    for (samples, 0..) |s, i| {
        try w.print("    {{ \"offset\": {d}, \"snr_db\": {d}, \"tags\": [", .{ s.offset, s.snr_db });
        for (s.tags, 0..) |t, ti| {
            if (ti != 0) try w.writeAll(", ");
            try w.print("{{ \"x\": {d}, \"y\": {d} }}", .{ t.x, t.y });
        }
        try w.writeAll("] }");
        if (i + 1 != samples.len) try w.writeAll(",");
        try w.writeAll("\n");
    }
    try w.writeAll("  ]\n}\n");

    try std.fs.cwd().writeFile(.{ .sub_path = path, .data = buf.items });
}
```

- [ ] **Step 4: Run GREEN.** Run: `zig test src/output.zig`
Expected: all output tests pass (the 2 existing + this new one).

- [ ] **Step 5: Commit.**
```bash
git add src/output.zig
git commit -m "feat: training-data.json writer"
```

---

## Task 4: Superposition (pure)

**Files:**
- Create: `src/combiner.zig`

`superposeInto` sums the per-antenna impulse responses of the selected tags into a caller-provided reusable buffer (zeroed first). Linearity of Maxwell's equations is what makes this valid (see the spec's reciprocity/superposition note).

- [ ] **Step 1: Write ONLY the test first.** Create `src/combiner.zig`:

```zig
const std = @import("std");
const simdata = @import("simdata.zig");
const combine_config = @import("combine_config.zig");
const output = @import("output.zig");

// ===== TESTS (written first) =====

test "superposeInto sums selected tags per antenna" {
    // 3 samples, 1 antenna, length 3.
    var data = [_]f32{ 1, 2, 3, 10, 20, 30, 100, 200, 300 };
    var tx = [_]f64{ 0, 0, 0 };
    var ty = [_]f64{ 0, 0, 0 };
    var offs = [_]u64{ 0, 12, 24 }; // 1 ant * 3 floats * 4 bytes = 12 per sample
    const sim = simdata.SimData{
        .allocator = std.testing.allocator,
        .num_antennas = 1,
        .impulse_length = 3,
        .tag_x = &tx,
        .tag_y = &ty,
        .offsets = &offs,
        .data = &data,
    };

    var ant0 = [_]f32{ -1, -1, -1 }; // pre-filled to prove it gets zeroed first
    var out = [_][]f32{&ant0};
    superposeInto(&out, sim, &[_]usize{ 0, 2 }); // tags 0 and 2
    try std.testing.expectEqualSlices(f32, &[_]f32{ 101, 202, 303 }, out[0]);

    superposeInto(&out, sim, &[_]usize{1}); // single tag
    try std.testing.expectEqualSlices(f32, &[_]f32{ 10, 20, 30 }, out[0]);
}
```

- [ ] **Step 2: Run RED.** Run: `zig test src/combiner.zig`
Expected: compile error — `superposeInto` undefined. Paste as RED.

- [ ] **Step 3: Implement.** Add ABOVE the test:

```zig
/// Zero `out` then add each selected tag's per-antenna impulse response into it.
/// `out` is `num_antennas` slices, each of length `sim.impulse_length`.
pub fn superposeInto(out: [][]f32, sim: simdata.SimData, tag_indices: []const usize) void {
    for (out) |o| @memset(o, 0);
    for (tag_indices) |t| {
        for (out, 0..) |o, a| {
            const imp = sim.impulse(t, a);
            for (o, 0..) |*v, n| v.* += imp[n];
        }
    }
}
```

- [ ] **Step 4: Run GREEN.** Run: `zig test src/combiner.zig`
Expected: the superpose test PASSES.

- [ ] **Step 5: Commit.**
```bash
git add src/combiner.zig
git commit -m "feat: superposition of selected tag impulse responses"
```

---

## Task 5: Gaussian noise injection at target SNR (pure)

**Files:**
- Modify: `src/combiner.zig`

`addNoiseInto` measures the combined waveform's signal power `P = mean(x²)` across all antennas/timesteps, derives noise power `P_n = P / 10^(snr_db/10)` and standard deviation `σ = √P_n`, then adds `σ·N(0,1)` to every sample. The test is statistical: with a known constant signal and a large buffer, the realized noise power must land near the target.

- [ ] **Step 1: Write the failing test.** Append to `src/combiner.zig`:

```zig
test "addNoiseInto realizes the target SNR within tolerance" {
    const M = 20000;
    const sig = try std.testing.allocator.alloc(f32, M);
    defer std.testing.allocator.free(sig);
    @memset(sig, 1.0); // constant signal => signal power = 1.0
    const orig = try std.testing.allocator.dupe(f32, sig);
    defer std.testing.allocator.free(orig);

    var prng = std.Random.DefaultPrng.init(12345);
    var buf = [_][]f32{sig};
    const snr_db: f64 = 20.0; // => noise power = 1/100 = 0.01
    addNoiseInto(&buf, snr_db, prng.random());

    // Realized noise power = mean((noisy - orig)^2) ~= 0.01.
    var sumsq: f64 = 0;
    for (sig, 0..) |v, i| {
        const d = @as(f64, v) - @as(f64, orig[i]);
        sumsq += d * d;
    }
    const realized = sumsq / @as(f64, @floatFromInt(M));
    const target = 0.01;
    try std.testing.expect(@abs(realized - target) / target < 0.10); // within 10%
}

test "addNoiseInto is a no-op on a zero signal" {
    var z = [_]f32{ 0, 0, 0, 0 };
    var buf = [_][]f32{&z};
    var prng = std.Random.DefaultPrng.init(7);
    addNoiseInto(&buf, 20.0, prng.random());
    try std.testing.expectEqualSlices(f32, &[_]f32{ 0, 0, 0, 0 }, &z);
}
```

- [ ] **Step 2: Run RED.** Run: `zig test src/combiner.zig`
Expected: compile error — `addNoiseInto` undefined. Paste as RED.

- [ ] **Step 3: Implement.** Add ABOVE the tests (next to `superposeInto`):

```zig
/// Add Gaussian noise to `buf` so that signal-power / noise-power == 10^(snr_db/10).
/// Signal power is measured as mean(x^2) over all antennas/timesteps of the current buffer.
/// No-op if the signal power is zero (avoids division by zero on an all-zero waveform).
pub fn addNoiseInto(buf: [][]f32, snr_db: f64, rand: std.Random) void {
    var sumsq: f64 = 0;
    var count: usize = 0;
    for (buf) |b| {
        for (b) |v| {
            sumsq += @as(f64, v) * @as(f64, v);
            count += 1;
        }
    }
    if (count == 0) return;
    const sig_power = sumsq / @as(f64, @floatFromInt(count));
    if (sig_power <= 0) return;

    const noise_power = sig_power / std.math.pow(f64, 10.0, snr_db / 10.0);
    const sigma = @sqrt(noise_power);
    for (buf) |b| {
        for (b) |*v| {
            v.* += @floatCast(sigma * rand.floatNorm(f64));
        }
    }
}
```

- [ ] **Step 4: Run GREEN.** Run: `zig test src/combiner.zig`
Expected: both noise tests PASS (and the superpose test). If the statistical test is flaky, the tolerance is 10% over 20000 samples which is comfortable; do NOT loosen it — a failure means the power math is wrong.

- [ ] **Step 5: Commit.**
```bash
git add src/combiner.zig
git commit -m "feat: gaussian noise injection at target SNR"
```

**Zig 0.14 notes:** `std.Random.DefaultPrng` is the 0.14 path (formerly `std.rand.DefaultPrng`); `prng.random()` yields a `std.Random`; `rand.floatNorm(f64)` gives a standard-normal sample. `std.math.pow(f64, ...)` is correct. If `std.Random` is spelled differently in this exact build, use the correct 0.14 namespace and note it.

---

## Task 6: `generate` orchestration

**Files:**
- Modify: `src/combiner.zig`

Ties it together: seeded PRNG, per-sample distinct-tag selection (partial Fisher–Yates over a persistent index permutation — O(k) per sample, stays a valid permutation across iterations so draws remain uniform), superpose, noise, append to bin, accumulate labels, write JSON. Reuses one set of `num_antennas × impulse_length` buffers across all samples.

- [ ] **Step 1: Write the failing test.** Append to `src/combiner.zig`:

```zig
fn buildTinySim(allocator: std.mem.Allocator) !simdata.SimData {
    // 4 tags, 2 antennas, length 3. Distinct per-tag values so sums are checkable.
    const data = try allocator.dupe(f32, &[_]f32{
        1,  1,  1,   2,  2,  2, // tag0: ant0, ant1
        3,  3,  3,   4,  4,  4, // tag1
        5,  5,  5,   6,  6,  6, // tag2
        7,  7,  7,   8,  8,  8, // tag3
    });
    const tx = try allocator.dupe(f64, &[_]f64{ 0.1, 0.2, 0.3, 0.4 });
    const ty = try allocator.dupe(f64, &[_]f64{ 1.1, 1.2, 1.3, 1.4 });
    const offs = try allocator.dupe(u64, &[_]u64{ 0, 24, 48, 72 }); // 2 ant*3*4 = 24 per tag
    return simdata.SimData{
        .allocator = allocator,
        .num_antennas = 2,
        .impulse_length = 3,
        .tag_x = tx,
        .tag_y = ty,
        .offsets = offs,
        .data = data,
    };
}

test "generate writes training bin+json with correct counts, sizes, label ranges" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir);
    const base = try std.fs.path.join(std.testing.allocator, &.{ dir, "train" });
    defer std.testing.allocator.free(base);

    var sim = try buildTinySim(std.testing.allocator);
    defer sim.deinit();

    const cfg = combine_config.CombineConfig{
        .source = "sim",
        .num_samples = 20,
        .tags_per_sample = .{ .min = 1, .max = 3 },
        .noise_snr_db = .{ .min = 15, .max = 30 },
        .seed = 99,
    };

    const res = try generate(std.testing.allocator, &sim, cfg, base);
    try std.testing.expectEqual(@as(usize, 20), res.num_samples);

    // bin size = num_samples * num_antennas * impulse_length * 4
    const bin_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.bin", .{base});
    defer std.testing.allocator.free(bin_path);
    const stat = try std.fs.cwd().statFile(bin_path);
    try std.testing.expectEqual(@as(u64, 20 * 2 * 3 * 4), stat.size);

    // JSON: 20 samples, each with 1..3 tags, snr in [15,30], offsets stride 24 bytes.
    const json_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.json", .{base});
    defer std.testing.allocator.free(json_path);
    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, json_path, 1 << 20);
    defer std.testing.allocator.free(bytes);
    const parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, bytes, .{});
    defer parsed.deinit();
    const arr = parsed.value.object.get("samples").?.array;
    try std.testing.expectEqual(@as(usize, 20), arr.items.len);
    for (arr.items, 0..) |item, i| {
        const o = item.object;
        const ntags = o.get("tags").?.array.items.len;
        try std.testing.expect(ntags >= 1 and ntags <= 3);
        const snr = o.get("snr_db").?.float;
        try std.testing.expect(snr >= 15.0 and snr <= 30.0);
        try std.testing.expectEqual(@as(i64, @intCast(i * 2 * 3 * 4)), o.get("offset").?.integer);
    }
}

test "generate is reproducible for a fixed seed" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir);

    var sim = try buildTinySim(std.testing.allocator);
    defer sim.deinit();
    const cfg = combine_config.CombineConfig{
        .source = "sim",
        .num_samples = 10,
        .tags_per_sample = .{ .min = 1, .max = 3 },
        .noise_snr_db = .{ .min = 10, .max = 40 },
        .seed = 2024,
    };

    const baseA = try std.fs.path.join(std.testing.allocator, &.{ dir, "a" });
    defer std.testing.allocator.free(baseA);
    const baseB = try std.fs.path.join(std.testing.allocator, &.{ dir, "b" });
    defer std.testing.allocator.free(baseB);
    _ = try generate(std.testing.allocator, &sim, cfg, baseA);
    _ = try generate(std.testing.allocator, &sim, cfg, baseB);

    const aBin = try std.fs.cwd().readFileAlloc(std.testing.allocator, try pathBin(std.testing.allocator, baseA), 1 << 20);
    defer std.testing.allocator.free(aBin);
    const bBin = try std.fs.cwd().readFileAlloc(std.testing.allocator, try pathBin(std.testing.allocator, baseB), 1 << 20);
    defer std.testing.allocator.free(bBin);
    try std.testing.expect(std.mem.eql(u8, aBin, bBin));
}

fn pathBin(allocator: std.mem.Allocator, base: []const u8) ![]u8 {
    return std.fmt.allocPrint(allocator, "{s}.bin", .{base});
}
```

> Note: `pathBin` leaks its returned string inside the reproducibility test's `readFileAlloc` arg (it's never freed). To keep the test clean, the implementer should bind it to a `defer`-freed local instead of calling inline. Implement it as: allocate `aBinPath`/`bBinPath` with their own `defer allocator.free(...)`, then `readFileAlloc` from those. Adjust the test accordingly when writing it — do not leave a leak (the testing allocator will fail on a leak).

- [ ] **Step 2: Run RED.** Run: `zig test src/combiner.zig`
Expected: compile error — `generate`, `GenResult` undefined. Paste as RED.

- [ ] **Step 3: Implement.** Add ABOVE the tests:

```zig
pub const GenResult = struct { num_samples: usize };

pub const GenerateError = error{NotEnoughTags};

/// Generate `cfg.num_samples` multi-tag training samples and write
/// `<output_base>.bin` + `<output_base>.json`. Reproducible for a fixed `cfg.seed`.
pub fn generate(
    allocator: std.mem.Allocator,
    sim: *const simdata.SimData,
    cfg: combine_config.CombineConfig,
    output_base: []const u8,
) !GenResult {
    const m = sim.num_antennas;
    const n = sim.impulse_length;
    const num_tags = sim.numSamples();
    if (cfg.tags_per_sample.max > num_tags) return GenerateError.NotEnoughTags;

    // Reusable combined buffers (m antennas x n samples) + a const view for appendSample.
    const out = try allocator.alloc([]f32, m);
    defer {
        for (out) |o| allocator.free(o);
        allocator.free(out);
    }
    {
        var built: usize = 0;
        errdefer {
            var z: usize = 0;
            while (z < built) : (z += 1) allocator.free(out[z]);
        }
        while (built < m) : (built += 1) out[built] = try allocator.alloc(f32, n);
    }
    const view = try allocator.alloc([]const f32, m);
    defer allocator.free(view);
    for (out, 0..) |o, a| view[a] = o;

    // Persistent permutation for distinct selection.
    const idx = try allocator.alloc(usize, num_tags);
    defer allocator.free(idx);
    for (idx, 0..) |*v, i| v.* = i;

    var prng = std.Random.DefaultPrng.init(cfg.seed);
    const rand = prng.random();

    const bin_path = try std.fmt.allocPrint(allocator, "{s}.bin", .{output_base});
    defer allocator.free(bin_path);
    const json_path = try std.fmt.allocPrint(allocator, "{s}.json", .{output_base});
    defer allocator.free(json_path);

    const bin = try std.fs.cwd().createFile(bin_path, .{});
    defer bin.close();

    var samples = std.ArrayList(output.TrainingSampleMeta).init(allocator);
    defer {
        for (samples.items) |s| allocator.free(@constCast(s.tags));
        samples.deinit();
    }

    var s: u32 = 0;
    while (s < cfg.num_samples) : (s += 1) {
        const k = rand.intRangeAtMost(u32, cfg.tags_per_sample.min, cfg.tags_per_sample.max);
        // Partial Fisher-Yates: shuffle the first k positions; idx stays a permutation.
        var j: usize = 0;
        while (j < k) : (j += 1) {
            const swap_with = rand.intRangeLessThan(usize, j, num_tags);
            const tmp = idx[j];
            idx[j] = idx[swap_with];
            idx[swap_with] = tmp;
        }
        const selected = idx[0..k];

        superposeInto(out, sim.*, selected);
        const snr = cfg.noise_snr_db.min + rand.float(f64) * (cfg.noise_snr_db.max - cfg.noise_snr_db.min);
        addNoiseInto(out, snr, rand);

        const offset = try output.appendSample(bin, view);

        const tags = try allocator.alloc(output.TagLabel, k);
        for (selected, 0..) |ti, z| tags[z] = .{ .x = sim.tag_x[ti], .y = sim.tag_y[ti] };
        try samples.append(.{ .offset = offset, .tags = tags, .snr_db = snr });
    }

    try output.writeTrainingJson(allocator, json_path, "sim-output.json", @intCast(n), m, samples.items);
    return .{ .num_samples = cfg.num_samples };
}
```

- [ ] **Step 4: Run GREEN.** Run: `zig test src/combiner.zig`
Expected: all combiner tests pass (superpose, 2 noise, generate counts, reproducibility). The reproducibility test proves identical bytes for a fixed seed.

- [ ] **Step 5: Commit.**
```bash
git add src/combiner.zig
git commit -m "feat: combine generate orchestration (select, superpose, noise, write)"
```

**Zig 0.14 notes:** `rand.intRangeAtMost(u32, lo, hi)` is inclusive; `rand.intRangeLessThan(usize, lo, hi)` is `[lo, hi)`; `rand.float(f64)` is `[0,1)`. `@constCast(s.tags)` frees the `[]const TagLabel` we allocated as mutable then stored as const — if the compiler prefers, store `tags` as `[]output.TagLabel` in a parallel list and free that instead. Keep the partial-allocation `errdefer` on the `out` buffer loop. Confirm no leaks (the testing allocator enforces this).

---

## Task 7: Wire up the `combine` CLI

**Files:**
- Modify: `src/main.zig`
- Create: `configs/combine-example.json`

Add the `combine` subcommand. Flags: `--input <base>` (the sim-output base, e.g. `sim-output`), `--config <path>` (combine config JSON), `--output <base>` (default `training-data`). Apply the same trailing-flag bounds-checking pattern used by `cmdSimulate` (a value-taking flag with no following argument prints a clean error, never panics).

- [ ] **Step 1: Create `configs/combine-example.json`:**

```json
{
  "source": "sim-output",
  "num_samples": 50000,
  "tags_per_sample": { "min": 1, "max": 5 },
  "noise_snr_db": { "min": 10, "max": 40 },
  "seed": 42
}
```

- [ ] **Step 2: Add the imports and dispatch.** In `src/main.zig`, add near the existing imports:

```zig
const combine_config = @import("combine_config.zig");
const simdata = @import("simdata.zig");
const combiner = @import("combiner.zig");
```

And in `main`, add a branch next to the `simulate` branch (keep the existing `simulate` and the final `else`):

```zig
    } else if (std.mem.eql(u8, args[1], "combine")) {
        try cmdCombine(allocator, args[2..]);
```

- [ ] **Step 3: Add `cmdCombine`.** Append to `src/main.zig` (before the trailing `test {}` block):

```zig
fn cmdCombine(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var input_base: ?[]const u8 = null;
    var config_path: ?[]const u8 = null;
    var output_base: []const u8 = "training-data";

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--input")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --input requires a value\n", .{});
                return;
            }
            input_base = args[i];
        } else if (std.mem.eql(u8, a, "--config")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --config requires a value\n", .{});
                return;
            }
            config_path = args[i];
        } else if (std.mem.eql(u8, a, "--output")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --output requires a value\n", .{});
                return;
            }
            output_base = args[i];
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }

    const inb = input_base orelse {
        std.debug.print("error: --input <sim-output base> is required\n", .{});
        return;
    };
    const cfgp = config_path orelse {
        std.debug.print("error: --config <combine config> is required\n", .{});
        return;
    };

    const cfg_json = try std.fs.cwd().readFileAlloc(allocator, cfgp, 1 << 20);
    defer allocator.free(cfg_json);

    var parsed = combine_config.parse(allocator, cfg_json) catch |e| {
        std.debug.print("error: failed to parse combine config: {s}\n", .{@errorName(e)});
        return;
    };
    defer parsed.deinit();

    combine_config.validate(parsed.value) catch |e| {
        std.debug.print("error: invalid combine config: {s}\n", .{@errorName(e)});
        return;
    };

    var sim = simdata.load(allocator, inb) catch |e| {
        std.debug.print("error: failed to load sim-output '{s}': {s}\n", .{ inb, @errorName(e) });
        return;
    };
    defer sim.deinit();

    if (parsed.value.tags_per_sample.max > sim.numSamples()) {
        std.debug.print("error: tags_per_sample.max ({d}) exceeds available tag positions ({d})\n", .{ parsed.value.tags_per_sample.max, sim.numSamples() });
        return;
    }

    std.debug.print("combining: {d} samples from {d} tags, {d} antennas...\n", .{ parsed.value.num_samples, sim.numSamples(), sim.num_antennas });
    var timer = try std.time.Timer.start();
    const res = try combiner.generate(allocator, &sim, parsed.value, output_base);
    const secs = @as(f64, @floatFromInt(timer.read())) / 1e9;
    std.debug.print("done: {d} training samples in {d:.2}s -> {s}.bin / {s}.json\n", .{ res.num_samples, secs, output_base, output_base });
}
```

- [ ] **Step 4: Build and run the full suite.**

Run: `zig build`
Expected: clean build.

Run: `zig build test`
Expected: all tests pass (Plan 1 + the new combiner tests; the heavy validation test stays env-gated/skipped).

- [ ] **Step 5: End-to-end smoke run (depends on a real sim-output).**

First produce a small sim-output with the existing `simulate`, then combine it:

```bash
zig build run -Doptimize=ReleaseFast -- simulate --config configs/tiny.json --output /tmp/c-sim
zig build run -Doptimize=ReleaseFast -- combine --input /tmp/c-sim --config configs/combine-example.json --output /tmp/c-train
```

The combine config requests 50000 samples but `tiny.json` yields only 16 tags with `max: 5` — that's fine (5 ≤ 16). Expected: prints `done: 50000 training samples in <…>s ...`. Then verify:

```bash
python3 -c "
import json,os
d=json.load(open('/tmp/c-train.json'))
print('num_samples', d['num_samples'], 'num_antennas', d['num_antennas'], 'impulse_length', d['impulse_length'])
binb=os.path.getsize('/tmp/c-train.bin'); exp=d['num_samples']*d['num_antennas']*d['impulse_length']*4
print('bin bytes', binb, 'expected', exp, 'match', binb==exp)
s=d['samples'][0]; print('sample0 keys', sorted(s.keys()), 'ntags', len(s['tags']), 'snr', s['snr_db'])
# offsets must be strictly increasing by num_antennas*impulse_length*4
stride=d['num_antennas']*d['impulse_length']*4
print('offset stride ok', all(d['samples'][i]['offset']==i*stride for i in range(len(d['samples']))))
print('all tag counts in 1..5', all(1<=len(x['tags'])<=5 for x in d['samples']))
"
```
Expected: `bin bytes == expected`, `match True`, offset stride ok `True`, tag counts in range `True`, each sample has keys `[offset, snr_db, tags]`.

- [ ] **Step 6: Reproducibility check via the CLI.**

```bash
zig build run -Doptimize=ReleaseFast -- combine --input /tmp/c-sim --config configs/combine-example.json --output /tmp/c-train2
cmp /tmp/c-train.bin /tmp/c-train2.bin && echo "REPRODUCIBLE: identical bytes"
```
Expected: prints `REPRODUCIBLE: identical bytes` (same seed → identical output).

- [ ] **Step 7: Commit.**
```bash
git add src/main.zig configs/combine-example.json
git commit -m "feat: wire up combine CLI"
```

---

## Self-Review (completed during plan authoring)

**Spec coverage (Component 3 Superposition Combiner + `combine` CLI + training-data output):**

| Spec requirement | Task |
|------------------|------|
| Read combine config (`source`, `num_samples`, `tags_per_sample{min,max}`, `noise_snr_db{min,max}`, `seed`) | Task 1 |
| Reproducible PRNG via `seed` | Task 6 (seeded `DefaultPrng`), verified in Task 6 test + Task 7 CLI check |
| Pick random count of active tags in `[min,max]` | Task 6 |
| Select random **distinct** tag positions from precomputed set | Task 6 (partial Fisher–Yates) |
| Sum impulse responses at each antenna (superposition/linearity) | Task 4 |
| Inject additive Gaussian noise at random SNR in range | Task 5 |
| Label with active tags' (x,y) positions + snr | Task 6 + Task 3 (writer) |
| `training-data.json` shape (`version, source_config, num_samples, impulse_length, num_antennas, samples[{offset,tags,snr_db}]`) | Task 3 |
| `training-data.bin` same per-sample layout as sim output, little-endian float32 | Task 6 reuses Plan 1 `output.appendSample` (LE float32, comptime-guarded) |
| CLI `combine --input --config --output` | Task 7 |
| Reads `sim-output.json` + `sim-output.bin` | Task 2 |

**Out of scope (correctly absent):** `serve`/visualizer (Plan 3), any re-running of the FDTD (the whole point is superposition without re-simulation).

**Type consistency check:** `output.TagLabel`/`TrainingSampleMeta`/`writeTrainingJson` defined in Task 3 are used unchanged in Task 6. `simdata.SimData` fields (`num_antennas`, `impulse_length`, `tag_x`, `tag_y`, `offsets`, `data`, `numSamples()`, `impulse()`) defined in Task 2 are used unchanged in Tasks 4/6. `combine_config.CombineConfig` (Task 1) is used unchanged in Tasks 6/7. `output.appendSample` signature matches what Task 6 calls.

**Known assumptions / limitations recorded:**
- The combiner loads the entire `sim-output.bin` into memory (up to ~300 MB for a full retail sweep). Acceptable per the spec (post-processing, I/O-bound, seconds); flagged for a future streaming/mmap refinement if needed.
- `signal power` for SNR is measured as `mean(x²)` over all antennas/timesteps of the combined waveform (a single noise σ per training sample, applied uniformly across antennas) — matches the spec's "additive Gaussian noise at random SNR" without over-specifying per-antenna scaling.
- Native-endian float32 output is little-endian on supported targets (guarded at comptime in `output.zig` from Plan 1).

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-24-rfid-combiner.md`. Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task, two-stage review (spec then quality), strict test-first for the substantive tasks (1–6), continuous execution.
2. **Inline Execution** — batch tasks in this session with checkpoints.

After this, only Plan 3 (Visualizer + `serve`) remains to complete the full system.
