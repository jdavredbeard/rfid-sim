const std = @import("std");
const constants = @import("constants.zig");
const Grid = @import("grid.zig").Grid;

// ===== IMPLEMENTATION =====

pub const Sim = struct {
    allocator: std.mem.Allocator,
    grid: *const Grid,
    dt: f64,
    ez: []f64,
    hx: []f64,
    hy: []f64,
    ca: []f64,
    cb: []f64,
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

/// Allocate fields and precompute Ca/Cb. Caller must call `sim.deinit()`.
pub fn init(
    allocator: std.mem.Allocator,
    grid: *const Grid,
    source: SourceParams,
    src_i: usize,
    src_j: usize,
) !Sim {
    const n = grid.nx * grid.ny;
    const dt = courantDt(grid.dx);
    const tau = 1.0 / (std.math.pi * source.bandwidth);

    const ez = try allocator.alloc(f64, n);
    errdefer allocator.free(ez);
    const hx = try allocator.alloc(f64, n);
    errdefer allocator.free(hx);
    const hy = try allocator.alloc(f64, n);
    errdefer allocator.free(hy);
    const ca = try allocator.alloc(f64, n);
    errdefer allocator.free(ca);
    const cb = try allocator.alloc(f64, n);
    errdefer allocator.free(cb);

    var sim = Sim{
        .allocator = allocator,
        .grid = grid,
        .dt = dt,
        .ez = ez,
        .hx = hx,
        .hy = hy,
        .ca = ca,
        .cb = cb,
        .f0 = source.center_freq,
        .tau = tau,
        .t0 = 5.0 * tau,
        .src_index = grid.idx(src_i, src_j),
    };

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

/// Advance the simulation one timestep. `n` is the zero-based step number.
pub fn step(sim: *Sim, n: u32) void {
    const g = sim.grid;
    const nx = g.nx;
    const ny = g.ny;
    const dx = g.dx;
    const dt = sim.dt;
    const mu_dx = constants.mu0 * dx;

    // --- H field update ---
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

    // --- PEC enforcement ---
    {
        var k: usize = 0;
        while (k < g.pec.len) : (k += 1) {
            if (g.pec[k]) sim.ez[k] = 0.0;
        }
    }
}

/// Run `timesteps` steps, recording Ez at each probe cell every step.
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

// ===== TESTS (written first) =====

test "courant dt is positive and below dx/c" {
    const dt = courantDt(0.015);
    try std.testing.expect(dt > 0);
    try std.testing.expect(dt < 0.015 / constants.c);
}

test "source starts near zero and is finite" {
    var g = try Grid.initFreeSpace(std.testing.allocator, 5, 5, 0.015);
    defer g.deinit();
    var sim = try init(std.testing.allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, 2, 2);
    defer sim.deinit();
    try std.testing.expect(@abs(sim.sourceValue(0)) < 1e-6); // near-zero at t=0
    try std.testing.expect(std.math.isFinite(sim.sourceValue(sim.t0)));
}

test "field stays finite and bounded over many steps" {
    var g = try Grid.initFreeSpace(std.testing.allocator, 60, 60, 0.015);
    defer g.deinit();
    var sim = try init(std.testing.allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, 30, 30);
    defer sim.deinit();
    var n: u32 = 0;
    while (n < 500) : (n += 1) step(&sim, n);
    var max: f64 = 0;
    for (sim.ez) |v| {
        try std.testing.expect(std.math.isFinite(v));
        max = @max(max, @abs(v));
    }
    try std.testing.expect(max < 100.0);
}

test "PEC cell stays at zero" {
    var g = try Grid.initFreeSpace(std.testing.allocator, 20, 20, 0.015);
    defer g.deinit();
    g.pec[g.idx(12, 10)] = true; // a metal cell near the source
    var sim = try init(std.testing.allocator, &g, .{ .center_freq = 915e6, .bandwidth = 200e6 }, 10, 10);
    defer sim.deinit();
    var n: u32 = 0;
    while (n < 200) : (n += 1) step(&sim, n);
    try std.testing.expectEqual(@as(f64, 0.0), sim.ez[g.idx(12, 10)]);
}
