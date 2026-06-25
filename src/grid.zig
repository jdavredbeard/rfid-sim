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

    /// Allocate an all-free-space grid (no config). Caller must call `grid.deinit()`.
    pub fn initFreeSpace(allocator: std.mem.Allocator, nx: usize, ny: usize, dx: f64) !Grid {
        const n = nx * ny;
        const eps_r = try allocator.alloc(f64, n);
        errdefer allocator.free(eps_r);
        const sigma = try allocator.alloc(f64, n);
        errdefer allocator.free(sigma);
        const pec = try allocator.alloc(bool, n);
        errdefer allocator.free(pec);
        @memset(eps_r, 1.0);
        @memset(sigma, 0.0);
        @memset(pec, false);
        return Grid{
            .allocator = allocator,
            .nx = nx,
            .ny = ny,
            .dx = dx,
            .eps_r = eps_r,
            .sigma = sigma,
            .pec = pec,
        };
    }
};

const METAL_SIGMA_THRESHOLD: f64 = 1e5; // cells with σ above this are treated as PEC

/// Build a discretized grid from a validated config. Caller must call `grid.deinit()`.
pub fn build(allocator: std.mem.Allocator, cfg: config.Config) !Grid {
    const dx = cfg.grid_resolution;
    const nx: usize = @intFromFloat(@round(cfg.room.width / dx));
    const ny: usize = @intFromFloat(@round(cfg.room.height / dx));
    const n = nx * ny;

    const eps_r = try allocator.alloc(f64, n);
    errdefer allocator.free(eps_r);
    const sigma = try allocator.alloc(f64, n);
    errdefer allocator.free(sigma);
    const pec = try allocator.alloc(bool, n);
    errdefer allocator.free(pec);

    var grid = Grid{
        .allocator = allocator,
        .nx = nx,
        .ny = ny,
        .dx = dx,
        .eps_r = eps_r,
        .sigma = sigma,
        .pec = pec,
    };

    // Default: free space everywhere.
    @memset(grid.eps_r, 1.0);
    @memset(grid.sigma, 0.0);
    @memset(grid.pec, false);

    // Rasterize obstacles (axis-aligned rects).
    for (cfg.obstacles) |o| {
        const mat = cfg.materials.map.get(o.material).?;
        const oi0: usize = @intFromFloat(@max(0.0, @floor(o.x / dx)));
        const oj0: usize = @intFromFloat(@max(0.0, @floor(o.y / dx)));
        const oi1: usize = @min(nx, @as(usize, @intFromFloat(@ceil((o.x + o.w) / dx))));
        const oj1: usize = @min(ny, @as(usize, @intFromFloat(@ceil((o.y + o.h) / dx))));
        var i = oi0;
        while (i < oi1) : (i += 1) {
            var j = oj0;
            while (j < oj1) : (j += 1) {
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
            if (bi < 0 or bi >= @as(i64, @intCast(grid.nx))) continue;
            var bj = bj0;
            while (bj <= bj1) : (bj += 1) {
                if (bj < 0 or bj >= @as(i64, @intCast(grid.ny))) continue;
                paintCell(grid, @intCast(bi), @intCast(bj), mat);
            }
        }
    }
}

// ===== TESTS (written first) =====

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
    const cfg = tinyConfig(&mats);
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
    var obs = [_]config.Obstacle{.{ .type = "rect", .x = 0.3, .y = 0.3, .w = 0.2, .h = 0.2, .material = "metal" }};
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
    const cfg = tinyConfig(&mats);
    var g = try build(std.testing.allocator, cfg);
    defer g.deinit();
    const c = g.cellOf(0.55, 0.25);
    try std.testing.expectEqual(@as(usize, 5), c.i);
    try std.testing.expectEqual(@as(usize, 2), c.j);
}
