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

/// Run one FDTD simulation with the source at tag position `pos`, recording Ez
/// at every antenna cell for `timesteps` steps. Returns an array of `num_antennas`
/// slices, each of length `timesteps`. Caller frees each slice and the outer slice
/// (use `freeResponses`).
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

    // Output buffers (one slice per antenna).
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

const Worker = struct {
    allocator: std.mem.Allocator,
    grid: *const grid_mod.Grid,
    cfg: config.Config,
    positions: []const TagPos,
    next: *std.atomic.Value(usize),
    done: *std.atomic.Value(usize),
    results: [][][]f32, // results[i] = responses for positions[i] (one []f32 per antenna)
    total: usize,
    step: usize,
    start: std.time.Instant,
    err: ?anyerror = null,

    fn loop(self: *Worker) void {
        while (true) {
            const i = self.next.fetchAdd(1, .seq_cst);
            if (i >= self.positions.len) return;
            const responses = simulateOne(self.allocator, self.grid, self.cfg, self.positions[i]) catch |e| {
                self.err = e;
                return;
            };
            self.results[i] = responses;

            // Report progress on completion (every `step` tags, plus the last one).
            // std.debug.print locks stderr internally, so lines won't interleave.
            const d = self.done.fetchAdd(1, .seq_cst) + 1;
            if (d == self.total or d % self.step == 0) {
                const now = std.time.Instant.now() catch self.start;
                const elapsed_s = @as(f64, @floatFromInt(now.since(self.start))) / 1e9;
                const fd: f64 = @floatFromInt(d);
                const ft: f64 = @floatFromInt(self.total);
                const eta_s = if (d > 0) elapsed_s / fd * (ft - fd) else 0;
                std.debug.print(
                    "  progress: {d}/{d} ({d:.0}%)  elapsed {d:.0}s  eta {d:.0}s\n",
                    .{ d, self.total, fd / ft * 100.0, elapsed_s, eta_s },
                );
            }
        }
    }
};

pub const RunResult = struct {
    num_samples: usize,
};

/// Run the full sweep and write `<output_base>.bin` + `<output_base>.json`.
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

    const results = try allocator.alloc([][]f32, positions.len);
    defer allocator.free(results);
    const empty: [][]f32 = &.{};
    for (results) |*r| r.* = empty;

    var next = std.atomic.Value(usize).init(0);
    var done = std.atomic.Value(usize).init(0);
    const start = std.time.Instant.now() catch unreachable;
    const step = @max(@as(usize, 1), positions.len / 100); // ~1% increments
    std.debug.print("  {d} tag positions to simulate\n", .{positions.len});

    const n_threads = @max(1, thread_count);
    const workers = try allocator.alloc(Worker, n_threads);
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
            .done = &done,
            .results = results,
            .total = positions.len,
            .step = step,
            .start = start,
        };
        threads[ti] = try std.Thread.spawn(.{}, Worker.loop, .{w});
    }
    for (threads) |t| t.join();

    var sweep_err: ?anyerror = null;
    for (workers) |w| {
        if (w.err) |e| { sweep_err = e; break; }
    }
    if (sweep_err) |e| {
        for (results) |r| if (r.len != 0) freeResponses(allocator, r);
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
        const responses = results[i]; // [][]f32
        // appendSample wants []const []const f32; build a const view.
        var view = try allocator.alloc([]const f32, responses.len);
        defer allocator.free(view);
        for (responses, 0..) |r, a| view[a] = r;
        const offset = try output.appendSample(bin, view);
        samples[i] = .{ .tag_x = p.x, .tag_y = p.y, .offset = offset };
        freeResponses(allocator, responses);
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

// ===== TESTS =====

test "tag positions skip obstacles and antennas" {
    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    try mats.map.put(std.testing.allocator, "metal", .{ .epsilon_r = 1.0, .sigma = 1e7 });

    var obs = [_]config.Obstacle{.{ .type = "rect", .x = 0.4, .y = 0.4, .w = 0.3, .h = 0.3, .material = "metal" }};
    var ants = [_]config.Antenna{.{ .x = 0.25, .y = 0.25, .label = "ant1" }};
    const cfg = config.Config{
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

    // Grid at 0.25 spacing inside a 1x1 room -> x,y in {0.25,0.5,0.75}. 9 candidates.
    // (0.25,0.25) is the antenna cell -> skipped. (0.5,0.5) is inside the metal
    // obstacle (0.4..0.7) -> skipped. So at most 7 remain.
    try std.testing.expect(positions.len <= 7);
    for (positions) |p| {
        // None may coincide with the antenna cell or the obstacle.
        try std.testing.expect(!(p.i == g.cellOf(0.25, 0.25).i and p.j == g.cellOf(0.25, 0.25).j));
        const k = g.idx(p.i, p.j);
        try std.testing.expect(!g.pec[k]);
    }
}

test "runSweep writes bin and json with matching sample count" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir_path = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir_path);
    const base = try std.fs.path.join(std.testing.allocator, &.{ dir_path, "sweep" });
    defer std.testing.allocator.free(base);

    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    var ants = [_]config.Antenna{.{ .x = 0.2, .y = 0.2, .label = "ant1" }};
    const cfg = config.Config{
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

test "simulateOne returns one impulse per antenna with nonzero energy" {
    var mats = std.json.ArrayHashMap(config.Material){};
    defer mats.deinit(std.testing.allocator);
    var ants = [_]config.Antenna{
        .{ .x = 0.3, .y = 0.3, .label = "ant1" },
        .{ .x = 0.7, .y = 0.7, .label = "ant2" },
    };
    const cfg = config.Config{
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
