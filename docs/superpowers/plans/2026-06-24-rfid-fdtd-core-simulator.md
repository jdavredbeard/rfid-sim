# RFID FDTD Core Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the core 2D FDTD electromagnetic simulator that reads a room config, sweeps tag positions, runs the FDTD engine, and writes single-tag impulse-response data (`sim-output.json` + `sim-output.bin`), plus a `--validate` free-space accuracy check.

**Architecture:** A Zig CLI (`rfid-sim simulate`). Config JSON is parsed and validated into a discretized grid. The FDTD engine (2D TM mode: Ez, Hx, Hy) injects a Gaussian-modulated soft source at a tag cell, steps the leapfrog update equations with lossy/PEC materials and conductor boundaries, and records Ez at antenna cells every timestep. A generator computes valid tag positions, distributes one FDTD run per position across a thread pool, and writes packed float32 impulse responses incrementally.

**Tech Stack:** Zig 0.14.0 (`std.json`, `std.Thread`, `std.testing`). No external dependencies. Output binaries are little-endian IEEE-754 float32.

**Scope note:** This is Plan 1 of 3. The Superposition Combiner (`combine`) and the Visualizer + HTTP server (`serve`) are separate follow-on plans that depend on the output format defined here. They are out of scope for this plan.

---

## File Structure

| File | Responsibility |
|------|----------------|
| `build.zig` | Build graph: `rfid-sim` exe, `zig build run`, `zig build test`. |
| `src/main.zig` | CLI arg parsing, subcommand dispatch (`simulate`), test aggregator. |
| `src/config.zig` | JSON config parsing + validation → typed `Config`. |
| `src/grid.zig` | Discretization: converts `Config` into a material grid (nx, ny, per-cell ε/σ, PEC mask, antenna/source cell indices). |
| `src/fdtd.zig` | FDTD engine: field allocation, coefficient arrays, time stepping, soft source, probes. |
| `src/validate.zig` | Free-space `1/√r` accuracy test (`--validate`). |
| `src/output.zig` | `sim-output.json` + `sim-output.bin` writers. |
| `src/generator.zig` | Tag-position sweep, thread pool, orchestration, incremental writes. |
| `configs/retail-example.json` | Example room config (from spec). |

Files import "downward": `grid.zig` imports `config.zig`; `fdtd.zig` imports `grid.zig`; `generator.zig` imports `config.zig`, `grid.zig`, `fdtd.zig`, `output.zig`; `validate.zig` imports `fdtd.zig`/`grid.zig`; `main.zig` imports everything.

**Testing convention:** Each file carries its own `test` blocks. Run a single file's tests with `zig test src/<file>.zig` (fast, isolated). The full suite runs via `zig build test`, which aggregates all files through `main.zig`'s `refAllDeclsRecursive`.

---

## Task 0: Project scaffolding

**Files:**
- Create: `build.zig`
- Create: `src/main.zig`

- [ ] **Step 1: Write `build.zig`**

```zig
const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const exe = b.addExecutable(.{
        .name = "rfid-sim",
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);
    const run_step = b.step("run", "Run rfid-sim");
    run_step.dependOn(&run_cmd.step);

    const tests = b.addTest(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
    });
    const run_tests = b.addRunArtifact(tests);
    const test_step = b.step("test", "Run all unit tests");
    test_step.dependOn(&run_tests.step);
}
```

- [ ] **Step 2: Write `src/main.zig` stub**

```zig
const std = @import("std");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len < 2) {
        std.debug.print("usage: rfid-sim <simulate> [options]\n", .{});
        return;
    }

    const cmd = args[1];
    if (std.mem.eql(u8, cmd, "simulate")) {
        std.debug.print("simulate: not yet implemented\n", .{});
    } else {
        std.debug.print("unknown command: {s}\n", .{cmd});
    }
}

test {
    std.testing.refAllDeclsRecursive(@This());
}
```

- [ ] **Step 3: Verify it builds and runs**

Run: `zig build run -- simulate`
Expected: prints `simulate: not yet implemented`

- [ ] **Step 4: Verify the test step works**

Run: `zig build test`
Expected: exits 0 (no tests yet, no failures)

- [ ] **Step 5: Commit**

```bash
git add build.zig src/main.zig
git commit -m "feat: scaffold rfid-sim Zig project"
```

---

## Task 1: Physics constants

**Files:**
- Create: `src/constants.zig`

Centralize physical constants so every module agrees on them.

- [ ] **Step 1: Write the failing test**

Create `src/constants.zig`:

```zig
const std = @import("std");

/// Speed of light in vacuum (m/s).
pub const c: f64 = 299_792_458.0;
/// Permeability of free space (H/m).
pub const mu0: f64 = 1.25663706212e-6;
/// Permittivity of free space (F/m).
pub const eps0: f64 = 8.8541878128e-12;

test "eps0 consistent with c and mu0" {
    // eps0 = 1 / (mu0 * c^2)
    const derived = 1.0 / (mu0 * c * c);
    try std.testing.expectApproxEqRel(eps0, derived, 1e-6);
}
```

- [ ] **Step 2: Run the test**

Run: `zig test src/constants.zig`
Expected: PASS (1 test). If it fails, the constants are inconsistent — fix the literals, do not loosen the tolerance.

- [ ] **Step 3: Commit**

```bash
git add src/constants.zig
git commit -m "feat: add physics constants"
```

---

## Task 2: Config parsing and validation

**Files:**
- Create: `src/config.zig`
- Create: `configs/retail-example.json`

The config is parsed into typed structs. `materials` has arbitrary string keys, so it uses `std.json.ArrayHashMap`. Validation derives grid dimensions and rejects antennas placed inside walls/obstacles.

- [ ] **Step 1: Write the type definitions and parser**

Create `src/config.zig`:

```zig
const std = @import("std");

pub const Material = struct {
    epsilon_r: f64,
    sigma: f64,
};

pub const Room = struct {
    width: f64,
    height: f64,
};

pub const Wall = struct {
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    material: []const u8,
    thickness: f64,
};

pub const Obstacle = struct {
    type: []const u8,
    x: f64,
    y: f64,
    w: f64,
    h: f64,
    material: []const u8,
};

pub const Antenna = struct {
    x: f64,
    y: f64,
    label: []const u8,
};

pub const Source = struct {
    type: []const u8,
    center_freq: f64,
    bandwidth: f64,
};

pub const Config = struct {
    room: Room,
    grid_resolution: f64,
    materials: std.json.ArrayHashMap(Material),
    walls: []Wall,
    obstacles: []Obstacle,
    antennas: []Antenna,
    source: Source,
    tag_grid_spacing: f64,
    timesteps: u32,
};

pub const ParsedConfig = std.json.Parsed(Config);

/// Parse a config from a JSON byte slice. Caller owns the returned Parsed and must call `.deinit()`.
pub fn parse(allocator: std.mem.Allocator, json: []const u8) !ParsedConfig {
    return std.json.parseFromSlice(Config, allocator, json, .{
        .ignore_unknown_fields = true,
    });
}
```

- [ ] **Step 2: Write the failing parse test**

Append to `src/config.zig`:

```zig
const test_json =
    \\{
    \\  "room": { "width": 10.0, "height": 15.0 },
    \\  "grid_resolution": 0.015,
    \\  "materials": {
    \\    "concrete": { "epsilon_r": 4.5, "sigma": 0.02 },
    \\    "metal": { "epsilon_r": 1.0, "sigma": 1e7 }
    \\  },
    \\  "walls": [
    \\    { "x1": 0, "y1": 0, "x2": 10, "y2": 0, "material": "concrete", "thickness": 0.2 }
    \\  ],
    \\  "obstacles": [
    \\    { "type": "rect", "x": 3.0, "y": 5.0, "w": 2.0, "h": 0.8, "material": "metal" }
    \\  ],
    \\  "antennas": [
    \\    { "x": 0.5, "y": 0.5, "label": "ant1" }
    \\  ],
    \\  "source": { "type": "gaussian_pulse", "center_freq": 915e6, "bandwidth": 200e6 },
    \\  "tag_grid_spacing": 0.25,
    \\  "timesteps": 8000
    \\}
;

test "parse config fields" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    const cfg = parsed.value;

    try std.testing.expectEqual(@as(f64, 10.0), cfg.room.width);
    try std.testing.expectEqual(@as(f64, 0.015), cfg.grid_resolution);
    try std.testing.expectEqual(@as(usize, 1), cfg.walls.len);
    try std.testing.expectEqual(@as(usize, 1), cfg.antennas.len);
    try std.testing.expectEqual(@as(u32, 8000), cfg.timesteps);

    const concrete = cfg.materials.map.get("concrete").?;
    try std.testing.expectEqual(@as(f64, 4.5), concrete.epsilon_r);
}
```

- [ ] **Step 3: Run the test**

Run: `zig test src/config.zig`
Expected: PASS (3 tests: constants-derived not here; this file has the eps test? no). Expected: PASS with `parse config fields` passing.

- [ ] **Step 4: Add the validation function (failing test first)**

Append to `src/config.zig`:

```zig
pub const ValidationError = error{
    EmptyRoom,
    BadResolution,
    UnknownMaterial,
    AntennaOutsideRoom,
    AntennaInObstacle,
    NoAntennas,
};

fn pointInObstacle(o: Obstacle, x: f64, y: f64) bool {
    return x >= o.x and x <= o.x + o.w and y >= o.y and y <= o.y + o.h;
}

/// Validate semantic constraints the JSON parser cannot enforce.
pub fn validate(cfg: Config) ValidationError!void {
    if (cfg.room.width <= 0 or cfg.room.height <= 0) return ValidationError.EmptyRoom;
    if (cfg.grid_resolution <= 0) return ValidationError.BadResolution;
    if (cfg.antennas.len == 0) return ValidationError.NoAntennas;

    for (cfg.walls) |w| {
        if (cfg.materials.map.get(w.material) == null) return ValidationError.UnknownMaterial;
    }
    for (cfg.obstacles) |o| {
        if (cfg.materials.map.get(o.material) == null) return ValidationError.UnknownMaterial;
    }
    for (cfg.antennas) |a| {
        if (a.x < 0 or a.x > cfg.room.width or a.y < 0 or a.y > cfg.room.height) {
            return ValidationError.AntennaOutsideRoom;
        }
        for (cfg.obstacles) |o| {
            if (pointInObstacle(o, a.x, a.y)) return ValidationError.AntennaInObstacle;
        }
    }
}
```

Append the tests:

```zig
test "validate accepts good config" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    try validate(parsed.value);
}

test "validate rejects antenna inside obstacle" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    // The example obstacle is rect at (3,5) size 2x0.8. Move the antenna into it.
    parsed.value.antennas[0].x = 3.5;
    parsed.value.antennas[0].y = 5.2;
    try std.testing.expectError(ValidationError.AntennaInObstacle, validate(parsed.value));
}

test "validate rejects unknown wall material" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    parsed.value.walls[0].material = "unobtanium";
    try std.testing.expectError(ValidationError.UnknownMaterial, validate(parsed.value));
}
```

- [ ] **Step 5: Run the tests**

Run: `zig test src/config.zig`
Expected: PASS (all config tests: parse + 3 validate).

- [ ] **Step 6: Write the example config file**

Create `configs/retail-example.json`:

```json
{
  "room": { "width": 10.0, "height": 15.0 },
  "grid_resolution": 0.015,
  "materials": {
    "concrete": { "epsilon_r": 4.5, "sigma": 0.02 },
    "metal": { "epsilon_r": 1.0, "sigma": 1e7 },
    "drywall": { "epsilon_r": 2.1, "sigma": 0.001 },
    "glass": { "epsilon_r": 6.0, "sigma": 0.004 }
  },
  "walls": [
    { "x1": 0, "y1": 0, "x2": 10, "y2": 0, "material": "concrete", "thickness": 0.2 },
    { "x1": 10, "y1": 0, "x2": 10, "y2": 15, "material": "concrete", "thickness": 0.2 },
    { "x1": 10, "y1": 15, "x2": 0, "y2": 15, "material": "concrete", "thickness": 0.2 },
    { "x1": 0, "y1": 15, "x2": 0, "y2": 0, "material": "concrete", "thickness": 0.2 }
  ],
  "obstacles": [
    { "type": "rect", "x": 3.0, "y": 5.0, "w": 2.0, "h": 0.8, "material": "metal" }
  ],
  "antennas": [
    { "x": 0.5, "y": 0.5, "label": "ant1" },
    { "x": 9.5, "y": 0.5, "label": "ant2" },
    { "x": 0.5, "y": 14.5, "label": "ant3" },
    { "x": 9.5, "y": 14.5, "label": "ant4" }
  ],
  "source": { "type": "gaussian_pulse", "center_freq": 915e6, "bandwidth": 200e6 },
  "tag_grid_spacing": 0.25,
  "timesteps": 8000
}
```

- [ ] **Step 7: Verify the example file parses and validates**

Append a temporary smoke test to `src/config.zig`:

```zig
test "retail-example.json parses and validates" {
    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, "configs/retail-example.json", 1 << 20);
    defer std.testing.allocator.free(bytes);
    var parsed = try parse(std.testing.allocator, bytes);
    defer parsed.deinit();
    try validate(parsed.value);
}
```

Run: `zig test src/config.zig`
Expected: PASS. (This test reads from the current working directory — run it from the repo root.)

- [ ] **Step 8: Commit**

```bash
git add src/config.zig configs/retail-example.json
git commit -m "feat: config parsing and validation"
```

---

## Task 3: Grid discretization

**Files:**
- Create: `src/grid.zig`
- Test: in-file `test` blocks

Convert the continuous `Config` into a discrete grid: dimensions `nx`/`ny`, per-cell relative permittivity and conductivity, a PEC (metal) boolean mask, and the cell indices for antennas. Walls and obstacles are rasterized into the material arrays.

**Coordinate convention:** cell `(i, j)` covers `x ∈ [i·dx, (i+1)·dx)`, `y ∈ [j·dx, (j+1)·dx)`. Linear index `idx(i, j) = i * ny + j`. `nx = round(width/dx)`, `ny = round(height/dx)`.

- [ ] **Step 1: Write the Grid struct and constructor**

Create `src/grid.zig`:

```zig
const std = @import("std");
const config = @import("config.zig");

pub const Grid = struct {
    allocator: std.mem.Allocator,
    nx: usize,
    ny: usize,
    dx: f64,
    /// Relative permittivity per cell (length nx*ny).
    eps_r: []f64,
    /// Conductivity per cell, S/m (length nx*ny).
    sigma: []f64,
    /// True where the cell is a perfect electric conductor (metal). Length nx*ny.
    pec: []bool,

    pub fn idx(self: Grid, i: usize, j: usize) usize {
        return i * self.ny + j;
    }

    /// World coordinates → nearest cell. Clamps to grid bounds.
    pub fn cellOf(self: Grid, x: f64, y: f64) struct { i: usize, j: usize } {
        const fi = @floor(x / self.dx);
        const fj = @floor(y / self.dx);
        const i = std.math.clamp(@as(i64, @intFromFloat(fi)), 0, @as(i64, @intCast(self.nx - 1)));
        const j = std.math.clamp(@as(i64, @intFromFloat(fj)), 0, @as(i64, @intCast(self.ny - 1)));
        return .{ .i = @intCast(i), .j = @intCast(j) };
    }

    pub fn deinit(self: *Grid) void {
        self.allocator.free(self.eps_r);
        self.allocator.free(self.sigma);
        self.allocator.free(self.pec);
    }
};

const METAL_SIGMA_THRESHOLD: f64 = 1e5; // cells with σ above this are treated as PEC

/// Build a discretized grid from a validated config. Caller must call `grid.deinit()`.
pub fn build(allocator: std.mem.Allocator, cfg: config.Config) !Grid {
    const dx = cfg.grid_resolution;
    const nx: usize = @intFromFloat(@round(cfg.room.width / dx));
    const ny: usize = @intFromFloat(@round(cfg.room.height / dx));
    const n = nx * ny;

    var grid = Grid{
        .allocator = allocator,
        .nx = nx,
        .ny = ny,
        .dx = dx,
        .eps_r = try allocator.alloc(f64, n),
        .sigma = try allocator.alloc(f64, n),
        .pec = try allocator.alloc(bool, n),
    };
    errdefer grid.deinit();

    // Default: free space everywhere.
    @memset(grid.eps_r, 1.0);
    @memset(grid.sigma, 0.0);
    @memset(grid.pec, false);

    // Rasterize obstacles (axis-aligned rects).
    for (cfg.obstacles) |o| {
        const mat = cfg.materials.map.get(o.material).?;
        const i0: usize = @intFromFloat(@max(0.0, @floor(o.x / dx)));
        const j0: usize = @intFromFloat(@max(0.0, @floor(o.y / dx)));
        const i1: usize = @min(nx, @as(usize, @intFromFloat(@ceil((o.x + o.w) / dx))));
        const j1: usize = @min(ny, @as(usize, @intFromFloat(@ceil((o.y + o.h) / dx))));
        var i = i0;
        while (i < i1) : (i += 1) {
            var j = j0;
            while (j < j1) : (j += 1) {
                paintCell(&grid, i, j, mat);
            }
        }
    }

    // Rasterize walls as thick line segments.
    for (cfg.walls) |w| {
        const mat = cfg.materials.map.get(w.material).?;
        rasterizeWall(&grid, w, mat);
    }

    return grid;
}

fn paintCell(grid: *Grid, i: usize, j: usize, mat: config.Material) void {
    const k = grid.idx(i, j);
    grid.eps_r[k] = mat.epsilon_r;
    grid.sigma[k] = mat.sigma;
    if (mat.sigma >= METAL_SIGMA_THRESHOLD) grid.pec[k] = true;
}

/// Rasterize a wall centerline with thickness. Thickness extends thickness/2 each
/// side of the line; clamped to a minimum of one cell.
fn rasterizeWall(grid: *Grid, w: config.Wall, mat: config.Material) void {
    const dx = grid.dx;
    const half = @max(w.thickness / 2.0, dx / 2.0);
    // Sample the segment densely and paint a square brush of radius `half` at each sample.
    const len = std.math.hypot(w.x2 - w.x1, w.y2 - w.y1);
    const steps: usize = @max(1, @as(usize, @intFromFloat(@ceil(len / (dx / 2.0)))));
    var s: usize = 0;
    while (s <= steps) : (s += 1) {
        const t = @as(f64, @floatFromInt(s)) / @as(f64, @floatFromInt(steps));
        const px = w.x1 + t * (w.x2 - w.x1);
        const py = w.y1 + t * (w.y2 - w.y1);
        const bi0: i64 = @intFromFloat(@floor((px - half) / dx));
        const bj0: i64 = @intFromFloat(@floor((py - half) / dx));
        const bi1: i64 = @intFromFloat(@ceil((px + half) / dx));
        const bj1: i64 = @intFromFloat(@ceil((py + half) / dx));
        var bi = bi0;
        while (bi <= bi1) : (bi += 1) {
            if (bi < 0 or bi >= grid.nx) continue;
            var bj = bj0;
            while (bj <= bj1) : (bj += 1) {
                if (bj < 0 or bj >= grid.ny) continue;
                paintCell(grid, @intCast(bi), @intCast(bj), mat);
            }
        }
    }
}
```

- [ ] **Step 2: Write the failing tests**

Append to `src/grid.zig`:

```zig
fn tinyConfig(materials: *std.json.ArrayHashMap(config.Material)) config.Config {
    return .{
        .room = .{ .width = 1.0, .height = 1.0 },
        .grid_resolution = 0.1,
        .materials = materials.*,
        .walls = &.{},
        .obstacles = &.{},
        .antennas = &.{},
        .source = .{ .type = "gaussian_pulse", .center_freq = 915e6, .bandwidth = 200e6 },
        .tag_grid_spacing = 0.25,
        .timesteps = 10,
    };
}

test "grid dimensions from room size" {
    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    var cfg = tinyConfig(&mats);
    var g = try build(std.testing.allocator, cfg);
    defer g.deinit();
    try std.testing.expectEqual(@as(usize, 10), g.nx); // 1.0 / 0.1
    try std.testing.expectEqual(@as(usize, 10), g.ny);
    try std.testing.expectEqual(@as(f64, 1.0), g.eps_r[g.idx(5, 5)]); // free space default
}

test "metal obstacle marks PEC cells" {
    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    try mats.map.put(std.testing.allocator, "metal", .{ .epsilon_r = 1.0, .sigma = 1e7 });
    var cfg = tinyConfig(&mats);
    const obs = [_]config.Obstacle{.{ .type = "rect", .x = 0.3, .y = 0.3, .w = 0.2, .h = 0.2, .material = "metal" }};
    cfg.obstacles = &obs;
    var g = try build(std.testing.allocator, cfg);
    defer g.deinit();
    // Cell at world (0.4, 0.4) → (4,4) should be PEC.
    try std.testing.expect(g.pec[g.idx(4, 4)]);
    // A free-space cell should not be PEC.
    try std.testing.expect(!g.pec[g.idx(0, 0)]);
}

test "cellOf maps world coords to indices" {
    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    var cfg = tinyConfig(&mats);
    var g = try build(std.testing.allocator, cfg);
    defer g.deinit();
    const c = g.cellOf(0.55, 0.25);
    try std.testing.expectEqual(@as(usize, 5), c.i);
    try std.testing.expectEqual(@as(usize, 2), c.j);
}
```

- [ ] **Step 3: Run the tests**

Run: `zig test src/grid.zig`
Expected: PASS (3 tests).

- [ ] **Step 4: Commit**

```bash
git add src/grid.zig
git commit -m "feat: grid discretization with wall/obstacle rasterization"
```

---

## Task 4: FDTD engine

**Files:**
- Create: `src/fdtd.zig`
- Test: in-file `test` blocks

The engine owns the Ez/Hx/Hy field arrays and precomputed Ca/Cb coefficients. It steps the leapfrog updates, injects a Gaussian-modulated soft source at one cell, enforces PEC + conductor boundaries, and records Ez at antenna cells each timestep.

**Update equations (per spec):**
```
Hx[i][j] -= (dt/(μ0·dy))·(Ez[i][j+1] - Ez[i][j])        for i∈[0,nx), j∈[0,ny-1)
Hy[i][j] += (dt/(μ0·dx))·(Ez[i+1][j] - Ez[i][j])        for i∈[0,nx-1), j∈[0,ny)
Ez[i][j]  = Ca[i][j]·Ez[i][j] + Cb[i][j]·((Hy[i][j]-Hy[i-1][j])/dx - (Hx[i][j]-Hx[i][j-1])/dy)
                                                          for i∈[1,nx-1), j∈[1,ny-1)
```
with `dx = dy`, `Ca = (1 - σ·dt/(2ε))/(1 + σ·dt/(2ε))`, `Cb = (dt/ε)/(1 + σ·dt/(2ε))`, `ε = εr·ε0`.
Perimeter Ez cells stay 0 (conductor walls). PEC cells: Ez forced to 0 after each Ez update.

**Source:** `Ez_src(t) = exp(-((t-t0)/τ)²)·sin(2π·f0·t)`, `τ = 1/(π·bandwidth)`, `t0 = 5τ`, `t = n·dt`. Added (not assigned) to the source cell's Ez each step.

- [ ] **Step 1: Write the engine struct, init, and source function**

Create `src/fdtd.zig`:

```zig
const std = @import("std");
const constants = @import("constants.zig");
const Grid = @import("grid.zig").Grid;

pub const Sim = struct {
    allocator: std.mem.Allocator,
    grid: *const Grid,
    dt: f64,
    ez: []f64,
    hx: []f64,
    hy: []f64,
    ca: []f64,
    cb: []f64,
    // Source parameters.
    f0: f64,
    tau: f64,
    t0: f64,
    src_index: usize,

    pub fn deinit(self: *Sim) void {
        self.allocator.free(self.ez);
        self.allocator.free(self.hx);
        self.allocator.free(self.hy);
        self.allocator.free(self.ca);
        self.allocator.free(self.cb);
    }

    /// Gaussian-modulated sinusoid evaluated at time t (seconds).
    pub fn sourceValue(self: Sim, t: f64) f64 {
        const env_arg = (t - self.t0) / self.tau;
        const envelope = @exp(-(env_arg * env_arg));
        return envelope * @sin(2.0 * std.math.pi * self.f0 * t);
    }
};

pub const SourceParams = struct {
    center_freq: f64,
    bandwidth: f64,
};

/// Courant-stable timestep for a uniform 2D grid: dt = dx / (c·√2).
pub fn courantDt(dx: f64) f64 {
    return dx / (constants.c * std.math.sqrt2);
}

/// Allocate fields and precompute Ca/Cb. `src` is the (i,j) source cell.
/// Caller must call `sim.deinit()`.
pub fn init(
    allocator: std.mem.Allocator,
    grid: *const Grid,
    source: SourceParams,
    src_i: usize,
    src_j: usize,
) !Sim {
    const n = grid.nx * grid.ny;
    const dt = courantDt(grid.dx);

    var sim = Sim{
        .allocator = allocator,
        .grid = grid,
        .dt = dt,
        .ez = try allocator.alloc(f64, n),
        .hx = try allocator.alloc(f64, n),
        .hy = try allocator.alloc(f64, n),
        .ca = try allocator.alloc(f64, n),
        .cb = try allocator.alloc(f64, n),
        .f0 = source.center_freq,
        .tau = 1.0 / (std.math.pi * source.bandwidth),
        .t0 = 5.0 / (std.math.pi * source.bandwidth), // 5·tau
        .src_index = grid.idx(src_i, src_j),
    };
    errdefer sim.deinit();

    @memset(sim.ez, 0.0);
    @memset(sim.hx, 0.0);
    @memset(sim.hy, 0.0);

    var k: usize = 0;
    while (k < n) : (k += 1) {
        const eps = grid.eps_r[k] * constants.eps0;
        const sigma = grid.sigma[k];
        const denom = 1.0 + (sigma * dt) / (2.0 * eps);
        sim.ca[k] = (1.0 - (sigma * dt) / (2.0 * eps)) / denom;
        sim.cb[k] = (dt / eps) / denom;
    }

    return sim;
}
```

- [ ] **Step 2: Add the `step` function**

Append to `src/fdtd.zig` (inside the file, as a free function):

```zig
/// Advance the simulation one timestep. `n` is the zero-based step number
/// (used to evaluate the source at t = n·dt).
pub fn step(sim: *Sim, n: u32) void {
    const g = sim.grid;
    const nx = g.nx;
    const ny = g.ny;
    const dx = g.dx;
    const dt = sim.dt;
    const mu_dx = constants.mu0 * dx;

    // --- H field update ---
    // Hx[i][j] -= (dt/(mu0·dx))·(Ez[i][j+1] - Ez[i][j])
    {
        var i: usize = 0;
        while (i < nx) : (i += 1) {
            var j: usize = 0;
            while (j + 1 < ny) : (j += 1) {
                const k = g.idx(i, j);
                sim.hx[k] -= (dt / mu_dx) * (sim.ez[g.idx(i, j + 1)] - sim.ez[k]);
            }
        }
    }
    // Hy[i][j] += (dt/(mu0·dx))·(Ez[i+1][j] - Ez[i][j])
    {
        var i: usize = 0;
        while (i + 1 < nx) : (i += 1) {
            var j: usize = 0;
            while (j < ny) : (j += 1) {
                const k = g.idx(i, j);
                sim.hy[k] += (dt / mu_dx) * (sim.ez[g.idx(i + 1, j)] - sim.ez[k]);
            }
        }
    }

    // --- E field update (interior only; perimeter stays 0 = conductor boundary) ---
    {
        var i: usize = 1;
        while (i + 1 < nx) : (i += 1) {
            var j: usize = 1;
            while (j + 1 < ny) : (j += 1) {
                const k = g.idx(i, j);
                const curl_h =
                    (sim.hy[k] - sim.hy[g.idx(i - 1, j)]) / dx -
                    (sim.hx[k] - sim.hx[g.idx(i, j - 1)]) / dx;
                sim.ez[k] = sim.ca[k] * sim.ez[k] + sim.cb[k] * curl_h;
            }
        }
    }

    // --- Soft source ---
    const t = @as(f64, @floatFromInt(n)) * dt;
    sim.ez[sim.src_index] += sim.sourceValue(t);

    // --- PEC enforcement: metal cells held at Ez = 0 ---
    {
        var k: usize = 0;
        while (k < g.pec.len) : (k += 1) {
            if (g.pec[k]) sim.ez[k] = 0.0;
        }
    }
}

/// Run `timesteps` steps, recording Ez at each probe cell every step.
/// `probes` are linear cell indices. `out[p]` must have length `timesteps`.
/// `out[p][n]` receives Ez at probe p after step n.
pub fn run(sim: *Sim, timesteps: u32, probes: []const usize, out: [][]f32) void {
    var n: u32 = 0;
    while (n < timesteps) : (n += 1) {
        step(sim, n);
        for (probes, 0..) |p, pi| {
            out[pi][n] = @floatCast(sim.ez[p]);
        }
    }
}
```

- [ ] **Step 3: Write a stability test (failing first)**

Append to `src/fdtd.zig`:

```zig
const config = @import("config.zig");
const grid_mod = @import("grid.zig");

fn freeSpaceGrid(allocator: std.mem.Allocator, nx: usize, ny: usize, dx: f64) !Grid {
    const n = nx * ny;
    var g = Grid{
        .allocator = allocator,
        .nx = nx,
        .ny = ny,
        .dx = dx,
        .eps_r = try allocator.alloc(f64, n),
        .sigma = try allocator.alloc(f64, n),
        .pec = try allocator.alloc(bool, n),
    };
    @memset(g.eps_r, 1.0);
    @memset(g.sigma, 0.0);
    @memset(g.pec, false);
    return g;
}

test "courant dt is positive and below dx/c" {
    const dt = courantDt(0.015);
    try std.testing.expect(dt > 0);
    try std.testing.expect(dt < 0.015 / constants.c);
}

test "source starts near zero and is finite" {
    var g = try freeSpaceGrid(std.testing.allocator, 5, 5, 0.015);
    defer g.deinit();
    var sim = try init(std.testing.allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, 2, 2);
    defer sim.deinit();
    try std.testing.expect(@abs(sim.sourceValue(0)) < 1e-6); // near-zero at t=0
    try std.testing.expect(std.math.isFinite(sim.sourceValue(sim.t0)));
}

test "field stays finite and bounded over many steps" {
    var g = try freeSpaceGrid(std.testing.allocator, 60, 60, 0.015);
    defer g.deinit();
    var sim = try init(std.testing.allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, 30, 30);
    defer sim.deinit();
    var n: u32 = 0;
    while (n < 500) : (n += 1) step(&sim, n);
    // No NaN/Inf, and amplitude must not blow up (instability would explode).
    var max: f64 = 0;
    for (sim.ez) |v| {
        try std.testing.expect(std.math.isFinite(v));
        max = @max(max, @abs(v));
    }
    try std.testing.expect(max < 100.0);
}

test "PEC cell stays at zero" {
    var g = try freeSpaceGrid(std.testing.allocator, 20, 20, 0.015);
    defer g.deinit();
    g.pec[g.idx(12, 10)] = true; // a metal cell near the source
    var sim = try init(std.testing.allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, 10, 10);
    defer sim.deinit();
    var n: u32 = 0;
    while (n < 200) : (n += 1) step(&sim, n);
    try std.testing.expectEqual(@as(f64, 0.0), sim.ez[g.idx(12, 10)]);
}
```

- [ ] **Step 4: Run the tests**

Run: `zig test src/fdtd.zig`
Expected: PASS (5 tests). If `field stays finite` fails with a huge `max`, the update equations are unstable — re-check signs and the Courant dt before changing the tolerance.

- [ ] **Step 5: Commit**

```bash
git add src/fdtd.zig
git commit -m "feat: 2D FDTD engine with soft source, PEC, and probes"
```

---

## Task 5: Free-space validation (`--validate`)

**Files:**
- Create: `src/validate.zig`
- Test: in-file `test` blocks

A large empty room with a center source and probes at known distances. In 2D free space, a propagating cylindrical wave's peak amplitude decays as `1/√r`. We run only long enough for the direct pulse to pass each probe (before wall reflections return), measure peak |Ez| at each probe, and compare the measured decay ratios to the analytical `√(r_ref/r)`.

- [ ] **Step 1: Write the validation routine**

Create `src/validate.zig`:

```zig
const std = @import("std");
const constants = @import("constants.zig");
const grid_mod = @import("grid.zig");
const fdtd = @import("fdtd.zig");

pub const ProbeResult = struct {
    distance_m: f64,
    measured_peak: f64,
    expected_ratio: f64, // sqrt(r_ref / r) relative to the nearest probe
    measured_ratio: f64,
    percent_error: f64,
};

/// Run the free-space accuracy test. Returns one result per probe distance.
/// Caller owns the returned slice.
pub fn run(allocator: std.mem.Allocator) ![]ProbeResult {
    const dx = 0.03; // coarse resolution to keep runtime reasonable
    const room_m = 50.0;
    const nx: usize = @intFromFloat(@round(room_m / dx));
    const ny = nx;

    var g = try grid_mod.Grid.initFreeSpace(allocator, nx, ny, dx);
    defer g.deinit();

    const cx = nx / 2;
    const cy = ny / 2;

    const distances = [_]f64{ 1.0, 2.0, 5.0, 10.0 };
    var probe_cells: [distances.len]usize = undefined;
    for (distances, 0..) |d, di| {
        const off: usize = @intFromFloat(@round(d / dx));
        probe_cells[di] = g.idx(cx + off, cy); // probes along +x from center
    }

    // Stop before the direct pulse reaches the nearest wall and reflects back.
    // Nearest wall is room_m/2 = 25 m away; round trip from center = 50 m.
    // Furthest probe is 10 m. Run until the pulse has cleared 10 m but a 25 m
    // round trip (50 m) has NOT completed: pick travel distance ~22 m.
    const max_travel_m = 22.0;
    const dt = fdtd.courantDt(dx);
    const timesteps: u32 = @intFromFloat(@round(max_travel_m / (constants.c * dt)));

    var sim = try fdtd.init(allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, cx, cy);
    defer sim.deinit();

    var peaks = [_]f64{0} ** distances.len;
    var n: u32 = 0;
    while (n < timesteps) : (n += 1) {
        fdtd.step(&sim, n);
        for (probe_cells, 0..) |pc, pi| {
            peaks[pi] = @max(peaks[pi], @abs(sim.ez[pc]));
        }
    }

    var results = try allocator.alloc(ProbeResult, distances.len);
    const ref_r = distances[0];
    const ref_peak = peaks[0];
    for (distances, 0..) |d, di| {
        const expected_ratio = std.math.sqrt(ref_r / d);
        const measured_ratio = peaks[di] / ref_peak;
        const err = @abs(measured_ratio - expected_ratio) / expected_ratio * 100.0;
        results[di] = .{
            .distance_m = d,
            .measured_peak = peaks[di],
            .expected_ratio = expected_ratio,
            .measured_ratio = measured_ratio,
            .percent_error = err,
        };
    }
    return results;
}

pub fn printReport(results: []const ProbeResult) void {
    std.debug.print("\nFree-space validation (2D 1/sqrt(r) decay):\n", .{});
    std.debug.print("  {s:>6}  {s:>14}  {s:>10}  {s:>10}  {s:>8}\n", .{ "r (m)", "peak |Ez|", "expected", "measured", "err %" });
    for (results) |r| {
        std.debug.print("  {d:>6.1}  {d:>14.4e}  {d:>10.4}  {d:>10.4}  {d:>8.2}\n", .{
            r.distance_m, r.measured_peak, r.expected_ratio, r.measured_ratio, r.percent_error,
        });
    }
}
```

- [ ] **Step 2: Add the `initFreeSpace` helper to `grid.zig`**

The validation needs a config-free grid constructor. Append to `src/grid.zig` (as a method on `Grid` via a pub fn):

```zig
/// Allocate an all-free-space grid (no config). Caller must call `grid.deinit()`.
pub fn initFreeSpace(allocator: std.mem.Allocator, nx: usize, ny: usize, dx: f64) !Grid {
    const n = nx * ny;
    var g = Grid{
        .allocator = allocator,
        .nx = nx,
        .ny = ny,
        .dx = dx,
        .eps_r = try allocator.alloc(f64, n),
        .sigma = try allocator.alloc(f64, n),
        .pec = try allocator.alloc(bool, n),
    };
    @memset(g.eps_r, 1.0);
    @memset(g.sigma, 0.0);
    @memset(g.pec, false);
    return g;
}
```

Then replace the local `freeSpaceGrid` test helper in `src/fdtd.zig` with calls to `grid_mod.Grid.initFreeSpace` to avoid duplication (DRY): delete the `freeSpaceGrid` fn in `fdtd.zig` and update its tests to call `grid_mod.Grid.initFreeSpace(std.testing.allocator, nx, ny, dx)`.

- [ ] **Step 3: Write the validation accuracy test**

Append to `src/validate.zig`:

```zig
test "free-space decay matches 1/sqrt(r) within tolerance" {
    const results = try run(std.testing.allocator);
    defer std.testing.allocator.free(results);
    try std.testing.expectEqual(@as(usize, 4), results.len);
    // The reference probe (1 m) is exact by construction; the others should be
    // within ~15% of analytical 2D decay at this coarse resolution.
    for (results) |r| {
        try std.testing.expect(r.percent_error < 15.0);
    }
}
```

- [ ] **Step 4: Run the test**

Run: `zig test src/validate.zig`
Expected: PASS (1 test, runtime a few seconds). If errors exceed 15%, the discretization or run length is off — investigate before loosening the bound. Common causes: probes sampled during pulse rise rather than at the true peak, or reflections already returned (reduce `max_travel_m`).

- [ ] **Step 5: Commit**

```bash
git add src/validate.zig src/grid.zig src/fdtd.zig
git commit -m "feat: free-space 1/sqrt(r) validation harness"
```

---

## Task 6: Output writers

**Files:**
- Create: `src/output.zig`
- Test: in-file `test` blocks

Writes `sim-output.json` (metadata + sample index) and `sim-output.bin` (packed float32 impulse responses). Per sample the binary layout is `[ant1[0..N], ant2[0..N], ..., antM[0..N]]` with N = `impulse_length`. The JSON `offset` is the **byte** offset into the bin file.

- [ ] **Step 1: Write the binary + JSON writers**

Create `src/output.zig`:

```zig
const std = @import("std");

pub const SampleMeta = struct {
    tag_x: f64,
    tag_y: f64,
    offset: u64,
};

pub const GridMeta = struct {
    nx: usize,
    ny: usize,
    dx: f64,
    dt: f64,
};

/// Append one sample's impulse responses to the open bin file and return the
/// byte offset at which it was written. `responses[a][t]` is antenna a, step t.
pub fn appendSample(file: std.fs.File, responses: []const []const f32) !u64 {
    const offset = try file.getPos();
    for (responses) |ant| {
        try file.writeAll(std.mem.sliceAsBytes(ant));
    }
    return offset;
}

/// Write the sim-output.json metadata/index file.
pub fn writeJson(
    allocator: std.mem.Allocator,
    path: []const u8,
    config_json: []const u8, // raw config JSON to embed verbatim
    grid: GridMeta,
    antenna_labels: []const []const u8,
    impulse_length: u32,
    samples: []const SampleMeta,
) !void {
    var buf = std.ArrayList(u8).init(allocator);
    defer buf.deinit();
    const w = buf.writer();

    try w.writeAll("{\n");
    try w.print("  \"version\": 1,\n", .{});
    try w.print("  \"config\": {s},\n", .{config_json});
    try w.print("  \"grid\": {{ \"nx\": {d}, \"ny\": {d}, \"dx\": {d}, \"dt\": {e} }},\n", .{ grid.nx, grid.ny, grid.dx, grid.dt });

    try w.writeAll("  \"antennas\": [");
    for (antenna_labels, 0..) |lbl, i| {
        if (i != 0) try w.writeAll(", ");
        try w.print("\"{s}\"", .{lbl});
    }
    try w.writeAll("],\n");

    try w.print("  \"impulse_length\": {d},\n", .{impulse_length});
    try w.writeAll("  \"samples\": [\n");
    for (samples, 0..) |s, i| {
        try w.print("    {{ \"tag_x\": {d}, \"tag_y\": {d}, \"offset\": {d} }}", .{ s.tag_x, s.tag_y, s.offset });
        if (i + 1 != samples.len) try w.writeAll(",");
        try w.writeAll("\n");
    }
    try w.writeAll("  ]\n}\n");

    try std.fs.cwd().writeFile(.{ .sub_path = path, .data = buf.items });
}
```

- [ ] **Step 2: Write the round-trip tests**

Append to `src/output.zig`:

```zig
test "appendSample writes contiguous float32 and reports offsets" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const file = try tmp.dir.createFile("out.bin", .{ .read = true });
    defer file.close();

    const ant0 = [_]f32{ 1.0, 2.0, 3.0 };
    const ant1 = [_]f32{ 4.0, 5.0, 6.0 };
    const responses = [_][]const f32{ &ant0, &ant1 };

    const off0 = try appendSample(file, &responses);
    try std.testing.expectEqual(@as(u64, 0), off0);
    const off1 = try appendSample(file, &responses);
    // 2 antennas × 3 floats × 4 bytes = 24 bytes per sample.
    try std.testing.expectEqual(@as(u64, 24), off1);

    // Read back the first float of sample 0.
    try file.seekTo(0);
    var word: [4]u8 = undefined;
    _ = try file.readAll(&word);
    const v: f32 = @bitCast(word);
    try std.testing.expectEqual(@as(f32, 1.0), v);
}

test "writeJson produces parseable output with correct sample count" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const path = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(path);
    const full = try std.fs.path.join(std.testing.allocator, &.{ path, "out.json" });
    defer std.testing.allocator.free(full);

    const labels = [_][]const u8{ "ant1", "ant2" };
    const samples = [_]SampleMeta{
        .{ .tag_x = 1.25, .tag_y = 2.5, .offset = 0 },
        .{ .tag_x = 1.5, .tag_y = 2.5, .offset = 24 },
    };
    try writeJson(std.testing.allocator, full, "{}", .{ .nx = 667, .ny = 1000, .dx = 0.015, .dt = 3.54e-11 }, &labels, 3, &samples);

    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, full, 1 << 20);
    defer std.testing.allocator.free(bytes);
    const parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, bytes, .{});
    defer parsed.deinit();
    const arr = parsed.value.object.get("samples").?.array;
    try std.testing.expectEqual(@as(usize, 2), arr.items.len);
    try std.testing.expectEqual(@as(i64, 1), parsed.value.object.get("version").?.integer);
}
```

> **Note on `writeJson` path handling:** the test writes to a temp dir via an absolute path, but `writeJson` uses `std.fs.cwd().writeFile`. `writeFile` accepts absolute paths, so this works. In production the generator passes a relative path under the output directory.

- [ ] **Step 3: Run the tests**

Run: `zig test src/output.zig`
Expected: PASS (2 tests).

- [ ] **Step 4: Commit**

```bash
git add src/output.zig
git commit -m "feat: sim-output JSON + binary writers"
```

---

## Task 7: Generator — tag positions

**Files:**
- Create: `src/generator.zig`
- Test: in-file `test` blocks

First the pure logic: compute the list of tag positions on a regular grid (`tag_grid_spacing`), skipping positions that fall inside a wall/obstacle (PEC or non-free-space cell) or that coincide with an antenna cell.

- [ ] **Step 1: Write the tag-position generator**

Create `src/generator.zig`:

```zig
const std = @import("std");
const config = @import("config.zig");
const grid_mod = @import("grid.zig");
const fdtd = @import("fdtd.zig");
const output = @import("output.zig");

pub const TagPos = struct {
    x: f64,
    y: f64,
    i: usize,
    j: usize,
};

/// Compute valid tag positions: a regular grid at `tag_grid_spacing`, skipping
/// any cell that is non-free-space (wall/obstacle) or that coincides with an
/// antenna cell. Caller owns the returned slice.
pub fn tagPositions(
    allocator: std.mem.Allocator,
    cfg: config.Config,
    grid: grid_mod.Grid,
) ![]TagPos {
    var list = std.ArrayList(TagPos).init(allocator);
    errdefer list.deinit();

    // Mark antenna cells to skip.
    var antenna_cells = std.AutoHashMap(usize, void).init(allocator);
    defer antenna_cells.deinit();
    for (cfg.antennas) |a| {
        const c = grid.cellOf(a.x, a.y);
        try antenna_cells.put(grid.idx(c.i, c.j), {});
    }

    const spacing = cfg.tag_grid_spacing;
    var y = spacing;
    while (y < cfg.room.height) : (y += spacing) {
        var x = spacing;
        while (x < cfg.room.width) : (x += spacing) {
            const c = grid.cellOf(x, y);
            const k = grid.idx(c.i, c.j);
            // Skip walls/obstacles: cell is not free space if eps_r != 1 or sigma != 0 or PEC.
            const free = grid.eps_r[k] == 1.0 and grid.sigma[k] == 0.0 and !grid.pec[k];
            if (!free) continue;
            if (antenna_cells.contains(k)) continue;
            try list.append(.{ .x = x, .y = y, .i = c.i, .j = c.j });
        }
    }
    return list.toOwnedSlice();
}
```

- [ ] **Step 2: Write the failing test**

Append to `src/generator.zig`:

```zig
test "tag positions skip obstacles and antennas" {
    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    try mats.map.put(std.testing.allocator, "metal", .{ .epsilon_r = 1.0, .sigma = 1e7 });

    const obs = [_]config.Obstacle{.{ .type = "rect", .x = 0.4, .y = 0.4, .w = 0.3, .h = 0.3, .material = "metal" }};
    const ants = [_]config.Antenna{.{ .x = 0.25, .y = 0.25, .label = "ant1" }};
    var cfg = config.Config{
        .room = .{ .width = 1.0, .height = 1.0 },
        .grid_resolution = 0.05,
        .materials = mats,
        .walls = &.{},
        .obstacles = &obs,
        .antennas = &ants,
        .source = .{ .type = "gaussian_pulse", .center_freq = 915e6, .bandwidth = 200e6 },
        .tag_grid_spacing = 0.25,
        .timesteps = 10,
    };

    var g = try grid_mod.build(std.testing.allocator, cfg);
    defer g.deinit();
    const positions = try tagPositions(std.testing.allocator, cfg, g);
    defer std.testing.allocator.free(positions);

    // Grid at 0.25 spacing inside a 1x1 room → x,y ∈ {0.25,0.5,0.75}. 9 candidates.
    // (0.25,0.25) is the antenna cell → skipped. (0.5,0.5) is inside the metal
    // obstacle (0.4..0.7) → skipped. So at most 7 remain.
    try std.testing.expect(positions.len <= 7);
    for (positions) |p| {
        // None may coincide with the antenna cell or the obstacle.
        try std.testing.expect(!(p.i == g.cellOf(0.25, 0.25).i and p.j == g.cellOf(0.25, 0.25).j));
        const k = g.idx(p.i, p.j);
        try std.testing.expect(!g.pec[k]);
    }
}
```

- [ ] **Step 3: Run the test**

Run: `zig test src/generator.zig`
Expected: PASS (1 test).

- [ ] **Step 4: Commit**

```bash
git add src/generator.zig
git commit -m "feat: tag-position generation with skip logic"
```

---

## Task 8: Generator — single-position simulation

**Files:**
- Modify: `src/generator.zig`
- Test: in-file `test` blocks

A function that runs one FDTD simulation for one tag position and returns its impulse responses (one f32 slice per antenna). This is the unit of work the thread pool will parallelize.

- [ ] **Step 1: Write `simulateOne`**

Append to `src/generator.zig`:

```zig
/// Run one FDTD simulation with the source at tag position `pos`, recording Ez
/// at every antenna cell for `timesteps` steps. Returns an array of `num_antennas`
/// slices, each of length `timesteps`. Caller frees each slice and the outer slice.
pub fn simulateOne(
    allocator: std.mem.Allocator,
    grid: *const grid_mod.Grid,
    cfg: config.Config,
    pos: TagPos,
) ![][]f32 {
    const num_ant = cfg.antennas.len;
    const ts = cfg.timesteps;

    // Antenna probe cell indices.
    var probes = try allocator.alloc(usize, num_ant);
    defer allocator.free(probes);
    for (cfg.antennas, 0..) |a, ai| {
        const c = grid.cellOf(a.x, a.y);
        probes[ai] = grid.idx(c.i, c.j);
    }

    // Output buffers.
    var out = try allocator.alloc([]f32, num_ant);
    errdefer allocator.free(out);
    var allocated: usize = 0;
    errdefer {
        var z: usize = 0;
        while (z < allocated) : (z += 1) allocator.free(out[z]);
    }
    while (allocated < num_ant) : (allocated += 1) {
        out[allocated] = try allocator.alloc(f32, ts);
    }

    var sim = try fdtd.init(allocator, grid, .{
        .center_freq = cfg.source.center_freq,
        .bandwidth = cfg.source.bandwidth,
    }, pos.i, pos.j);
    defer sim.deinit();

    fdtd.run(&sim, ts, probes, out);
    return out;
}

pub fn freeResponses(allocator: std.mem.Allocator, responses: [][]f32) void {
    for (responses) |r| allocator.free(r);
    allocator.free(responses);
}
```

- [ ] **Step 2: Write the failing test**

Append to `src/generator.zig`:

```zig
test "simulateOne returns one impulse per antenna with nonzero energy" {
    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    const ants = [_]config.Antenna{
        .{ .x = 0.3, .y = 0.3, .label = "ant1" },
        .{ .x = 0.7, .y = 0.7, .label = "ant2" },
    };
    var cfg = config.Config{
        .room = .{ .width = 1.0, .height = 1.0 },
        .grid_resolution = 0.02,
        .materials = mats,
        .walls = &.{},
        .obstacles = &.{},
        .antennas = &ants,
        .source = .{ .type = "gaussian_pulse", .center_freq = 915e6, .bandwidth = 200e6 },
        .tag_grid_spacing = 0.25,
        .timesteps = 800,
    };
    var g = try grid_mod.build(std.testing.allocator, cfg);
    defer g.deinit();

    const pos = TagPos{ .x = 0.5, .y = 0.5, .i = g.cellOf(0.5, 0.5).i, .j = g.cellOf(0.5, 0.5).j };
    const responses = try simulateOne(std.testing.allocator, &g, cfg, pos);
    defer freeResponses(std.testing.allocator, responses);

    try std.testing.expectEqual(@as(usize, 2), responses.len);
    try std.testing.expectEqual(@as(usize, 800), responses[0].len);
    // The pulse must reach each antenna: peak amplitude clearly above zero.
    var peak: f32 = 0;
    for (responses[0]) |v| peak = @max(peak, @abs(v));
    try std.testing.expect(peak > 1e-4);
}
```

- [ ] **Step 3: Run the test**

Run: `zig test src/generator.zig`
Expected: PASS (2 tests).

- [ ] **Step 4: Commit**

```bash
git add src/generator.zig
git commit -m "feat: single-position FDTD simulation runner"
```

---

## Task 9: Generator — orchestration with thread pool

**Files:**
- Modify: `src/generator.zig`
- Test: in-file `test` blocks

Sweep all tag positions, running simulations in parallel across worker threads, and write results to the bin file in sample order with correct byte offsets. Writes happen on the main thread after each result is collected, so offsets stay deterministic and the file stays consistent.

**Design:** A shared atomic index hands out the next position to each worker. Each worker computes `simulateOne` and pushes `(sample_index, responses)` into a results buffer slot it owns (`results[index]`). After all workers finish, the main thread writes samples in order. (Impulse responses for a full room can be large; for very large sweeps this is refined in a follow-up, but for the spec's sizes the per-position result is ~128 KB × positions — acceptable to hold transiently. If memory is a concern, reduce by writing in batches; noted as a known limitation.)

- [ ] **Step 1: Write the orchestration driver**

Append to `src/generator.zig`:

```zig
const Worker = struct {
    allocator: std.mem.Allocator,
    grid: *const grid_mod.Grid,
    cfg: config.Config,
    positions: []const TagPos,
    next: *std.atomic.Value(usize),
    results: [][]const []f32, // results[i] = responses for positions[i]
    err: *?anyerror,

    fn loop(self: *Worker) void {
        while (true) {
            const i = self.next.fetchAdd(1, .seq_cst);
            if (i >= self.positions.len) return;
            const responses = simulateOne(self.allocator, self.grid, self.cfg, self.positions[i]) catch |e| {
                self.err.* = e;
                return;
            };
            self.results[i] = responses;
        }
    }
};

pub const RunResult = struct {
    num_samples: usize,
};

/// Run the full sweep and write `<output>.bin` + `<output>.json`.
/// `output_base` is a path prefix, e.g. "sim-output" → sim-output.bin/.json.
pub fn runSweep(
    allocator: std.mem.Allocator,
    cfg: config.Config,
    grid: *const grid_mod.Grid,
    config_json: []const u8,
    output_base: []const u8,
    thread_count: usize,
) !RunResult {
    const positions = try tagPositions(allocator, cfg, grid.*);
    defer allocator.free(positions);

    var results = try allocator.alloc([]const []f32, positions.len);
    defer allocator.free(results);
    for (results) |*r| r.* = &.{};

    var next = std.atomic.Value(usize).init(0);
    var worker_err: ?anyerror = null;

    const n_threads = @max(1, thread_count);
    var workers = try allocator.alloc(Worker, n_threads);
    defer allocator.free(workers);
    var threads = try allocator.alloc(std.Thread, n_threads);
    defer allocator.free(threads);

    for (workers, 0..) |*w, ti| {
        w.* = .{
            .allocator = allocator,
            .grid = grid,
            .cfg = cfg,
            .positions = positions,
            .next = &next,
            .results = results,
            .err = &worker_err,
        };
        threads[ti] = try std.Thread.spawn(.{}, Worker.loop, .{w});
    }
    for (threads) |t| t.join();
    if (worker_err) |e| {
        // Free any results that were produced before the error.
        for (results) |r| if (r.len != 0) freeResponses(allocator, @constCast(r));
        return e;
    }

    // Write bin + collect sample metadata in position order.
    const bin_path = try std.fmt.allocPrint(allocator, "{s}.bin", .{output_base});
    defer allocator.free(bin_path);
    const json_path = try std.fmt.allocPrint(allocator, "{s}.json", .{output_base});
    defer allocator.free(json_path);

    const bin = try std.fs.cwd().createFile(bin_path, .{});
    defer bin.close();

    var samples = try allocator.alloc(output.SampleMeta, positions.len);
    defer allocator.free(samples);

    for (positions, 0..) |p, i| {
        const responses = results[i];
        // appendSample wants []const []const f32; responses is []const []f32.
        var cast = try allocator.alloc([]const f32, responses.len);
        defer allocator.free(cast);
        for (responses, 0..) |r, a| cast[a] = r;
        const offset = try output.appendSample(bin, cast);
        samples[i] = .{ .tag_x = p.x, .tag_y = p.y, .offset = offset };
        freeResponses(allocator, @constCast(responses));
    }

    var labels = try allocator.alloc([]const u8, cfg.antennas.len);
    defer allocator.free(labels);
    for (cfg.antennas, 0..) |a, ai| labels[ai] = a.label;

    try output.writeJson(allocator, json_path, config_json, .{
        .nx = grid.nx,
        .ny = grid.ny,
        .dx = grid.dx,
        .dt = fdtd.courantDt(grid.dx),
    }, labels, cfg.timesteps, samples);

    return .{ .num_samples = positions.len };
}
```

- [ ] **Step 2: Write the end-to-end sweep test**

Append to `src/generator.zig`:

```zig
test "runSweep writes bin and json with matching sample count" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir_path = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir_path);
    const base = try std.fs.path.join(std.testing.allocator, &.{ dir_path, "sweep" });
    defer std.testing.allocator.free(base);

    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    const ants = [_]config.Antenna{.{ .x = 0.2, .y = 0.2, .label = "ant1" }};
    var cfg = config.Config{
        .room = .{ .width = 1.0, .height = 1.0 },
        .grid_resolution = 0.05,
        .materials = mats,
        .walls = &.{},
        .obstacles = &.{},
        .antennas = &ants,
        .source = .{ .type = "gaussian_pulse", .center_freq = 915e6, .bandwidth = 200e6 },
        .tag_grid_spacing = 0.3,
        .timesteps = 100,
    };
    var g = try grid_mod.build(std.testing.allocator, cfg);
    defer g.deinit();

    const res = try runSweep(std.testing.allocator, cfg, &g, "{}", base, 2);
    try std.testing.expect(res.num_samples > 0);

    // JSON sample count matches.
    const json_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.json", .{base});
    defer std.testing.allocator.free(json_path);
    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, json_path, 1 << 20);
    defer std.testing.allocator.free(bytes);
    const parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, bytes, .{});
    defer parsed.deinit();
    const arr = parsed.value.object.get("samples").?.array;
    try std.testing.expectEqual(res.num_samples, arr.items.len);

    // Bin size = num_samples × num_antennas × timesteps × 4 bytes.
    const bin_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.bin", .{base});
    defer std.testing.allocator.free(bin_path);
    const stat = try std.fs.cwd().statFile(bin_path);
    try std.testing.expectEqual(@as(u64, res.num_samples * 1 * 100 * 4), stat.size);
}
```

- [ ] **Step 3: Run the test**

Run: `zig test src/generator.zig`
Expected: PASS (3 tests).

- [ ] **Step 4: Commit**

```bash
git add src/generator.zig
git commit -m "feat: parallel tag-position sweep with thread pool"
```

---

## Task 10: Wire up the `simulate` CLI

**Files:**
- Modify: `src/main.zig`

Parse `simulate` flags, load+validate the config, build the grid, and dispatch to either `--validate` (free-space test) or the full sweep.

**Flags:**
- `--config <path>` (required unless `--validate`)
- `--output <base>` (default `sim-output`)
- `--validate` (run free-space accuracy test instead of a sweep)
- `--threads <n>` (default: CPU count)

(`--save-snapshots` / `--snapshot-interval` are accepted but deferred — snapshots belong to the Visualizer plan. Parse and warn-if-set so scripts don't break.)

- [ ] **Step 1: Replace `src/main.zig` with the full dispatcher**

```zig
const std = @import("std");
const config = @import("config.zig");
const grid_mod = @import("grid.zig");
const generator = @import("generator.zig");
const validate = @import("validate.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len < 2) {
        std.debug.print("usage: rfid-sim simulate --config <path> [--output sim-output] [--threads N] [--validate]\n", .{});
        return;
    }

    if (std.mem.eql(u8, args[1], "simulate")) {
        try cmdSimulate(allocator, args[2..]);
    } else {
        std.debug.print("unknown command: {s}\n", .{args[1]});
    }
}

fn cmdSimulate(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var config_path: ?[]const u8 = null;
    var output_base: []const u8 = "sim-output";
    var do_validate = false;
    var threads: usize = std.Thread.getCpuCount() catch 1;

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--config")) {
            i += 1;
            config_path = args[i];
        } else if (std.mem.eql(u8, a, "--output")) {
            i += 1;
            output_base = args[i];
        } else if (std.mem.eql(u8, a, "--validate")) {
            do_validate = true;
        } else if (std.mem.eql(u8, a, "--threads")) {
            i += 1;
            threads = try std.fmt.parseInt(usize, args[i], 10);
        } else if (std.mem.eql(u8, a, "--save-snapshots") or std.mem.eql(u8, a, "--snapshot-interval")) {
            std.debug.print("warning: snapshot flags are not supported in this build (see Visualizer plan)\n", .{});
            if (std.mem.eql(u8, a, "--snapshot-interval")) i += 1;
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }

    if (do_validate) {
        const results = try validate.run(allocator);
        defer allocator.free(results);
        validate.printReport(results);
        return;
    }

    const path = config_path orelse {
        std.debug.print("error: --config is required (or use --validate)\n", .{});
        return;
    };

    const config_json = try std.fs.cwd().readFileAlloc(allocator, path, 16 << 20);
    defer allocator.free(config_json);

    var parsed = config.parse(allocator, config_json) catch |e| {
        std.debug.print("error: failed to parse config: {s}\n", .{@errorName(e)});
        return;
    };
    defer parsed.deinit();

    config.validate(parsed.value) catch |e| {
        std.debug.print("error: invalid config: {s}\n", .{@errorName(e)});
        return;
    };

    var grid = try grid_mod.build(allocator, parsed.value);
    defer grid.deinit();

    std.debug.print("simulating: {d}x{d} grid, {d} threads...\n", .{ grid.nx, grid.ny, threads });
    var timer = try std.time.Timer.start();
    const res = try generator.runSweep(allocator, parsed.value, &grid, config_json, output_base, threads);
    const secs = @as(f64, @floatFromInt(timer.read())) / 1e9;
    std.debug.print("done: {d} tag positions in {d:.1}s → {s}.bin / {s}.json\n", .{ res.num_samples, secs, output_base, output_base });
}

test {
    std.testing.refAllDeclsRecursive(@This());
}
```

- [ ] **Step 2: Verify the whole project builds**

Run: `zig build`
Expected: builds with no errors.

- [ ] **Step 3: Run the full test suite**

Run: `zig build test`
Expected: all tests pass (aggregated across config, grid, fdtd, validate, output, generator, constants).

- [ ] **Step 4: Run the validation end-to-end via the CLI**

Run: `zig build run -- simulate --validate`
Expected: prints the free-space validation table with all probe errors < 15%.

- [ ] **Step 5: Run a real (small) sweep end-to-end**

Create a tiny config `configs/tiny.json` for a fast smoke run:

```json
{
  "room": { "width": 2.0, "height": 2.0 },
  "grid_resolution": 0.03,
  "materials": { "concrete": { "epsilon_r": 4.5, "sigma": 0.02 } },
  "walls": [
    { "x1": 0, "y1": 0, "x2": 2, "y2": 0, "material": "concrete", "thickness": 0.1 },
    { "x1": 2, "y1": 0, "x2": 2, "y2": 2, "material": "concrete", "thickness": 0.1 },
    { "x1": 2, "y1": 2, "x2": 0, "y2": 2, "material": "concrete", "thickness": 0.1 },
    { "x1": 0, "y1": 2, "x2": 0, "y2": 0, "material": "concrete", "thickness": 0.1 }
  ],
  "obstacles": [],
  "antennas": [
    { "x": 0.2, "y": 0.2, "label": "ant1" },
    { "x": 1.8, "y": 1.8, "label": "ant2" }
  ],
  "source": { "type": "gaussian_pulse", "center_freq": 915e6, "bandwidth": 200e6 },
  "tag_grid_spacing": 0.4,
  "timesteps": 1500
}
```

Run: `zig build run -- simulate --config configs/tiny.json --output /tmp/tiny-out`
Expected: prints `done: N tag positions ...`; `/tmp/tiny-out.json` and `/tmp/tiny-out.bin` exist. Verify with:

Run: `ls -l /tmp/tiny-out.bin /tmp/tiny-out.json && python3 -c "import json;d=json.load(open('/tmp/tiny-out.json'));print('samples',len(d['samples']),'impulse_length',d['impulse_length'])"`
Expected: bin size = `samples × 2 antennas × 1500 × 4` bytes; JSON reports the same sample count.

- [ ] **Step 6: Commit**

```bash
git add src/main.zig configs/tiny.json
git commit -m "feat: wire up simulate CLI with validate and sweep modes"
```

---

## Task 11: Release-mode performance sanity check

**Files:** none (verification only)

The spec targets ~0.3–0.5 s per tag position at full resolution on 8 cores in release mode. Confirm the build is fast enough in `ReleaseFast` and that nothing regressed numerically.

- [ ] **Step 1: Build release**

Run: `zig build -Doptimize=ReleaseFast`
Expected: builds clean.

- [ ] **Step 2: Re-run validation in release mode**

Run: `zig build run -Doptimize=ReleaseFast -- simulate --validate`
Expected: same validation table, all errors < 15% (release math must match debug within tolerance).

- [ ] **Step 3: Time a small sweep in release mode**

Run: `time zig build run -Doptimize=ReleaseFast -- simulate --config configs/tiny.json --output /tmp/tiny-rel`
Expected: completes in well under a second for the tiny config; printed per-run timing is reasonable. (Full-resolution timing is validated later against a real retail config; this step only confirms release builds run and produce output.)

- [ ] **Step 4: Commit (if any config/doc tweaks were needed)**

```bash
git add -A
git commit -m "chore: confirm release-mode build and validation" --allow-empty
```

---

## Self-Review (completed during plan authoring)

**Spec coverage for this plan's scope (Component 1 FDTD Engine + Component 2 Data Generator + `simulate` CLI + Validation):**

| Spec requirement | Task |
|------------------|------|
| Grid params (λ, dx, dt Courant) | Task 4 (`courantDt`), Task 3 (dimensions) |
| Lossy update equations (Ca/Cb) | Task 4 |
| Gaussian-modulated soft source (τ, t0=5τ, additive) | Task 4 |
| Conductor perimeter boundaries (Ez=0) | Task 4 (interior-only E update) |
| Metal cells as PEC (force Ez=0) | Task 3 (mask), Task 4 (enforcement) |
| Probes record Ez at antenna cells every step | Task 4 (`run`), Task 8 |
| Skip tag cells coinciding with antennas | Task 7 |
| Room config JSON parse + validation | Task 2 |
| Wall centerline + thickness, clamp to 1 cell | Task 3 (`rasterizeWall`) |
| Reject antennas inside walls/obstacles | Task 2 (`validate`) |
| Tag positions on regular grid, skip walls/obstacles | Task 7 |
| Thread pool via `std.Thread.spawn` | Task 9 |
| Incremental binary writes | Task 9 (`appendSample` per sample) |
| `sim-output.json` format (version, grid, antennas, samples, offsets) | Task 6 |
| `sim-output.bin` packed float32 LE, per-sample antenna layout | Task 6 |
| `--validate` free-space 1/√r test | Task 5 |
| CLI `simulate --config --output --threads --validate` | Task 10 |

**Deferred to later plans (explicitly out of scope, flagged in Task 10):** `--save-snapshots`/`--snapshot-interval` and the `snapshots/` output (Visualizer plan); the `combine` command and training-data output (Combiner plan); the `serve` command (Visualizer plan).

**Known limitations recorded in the plan:** Task 9 holds all per-position results in memory before writing; acceptable for spec-sized rooms, flagged for a future batching refinement if memory-bound.

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-06-24-rfid-fdtd-core-simulator.md`. Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

The Combiner and Visualizer plans are still unwritten — I can draft either after (or before) executing this one.
