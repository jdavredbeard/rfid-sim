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

// ===== TEST (written first) =====

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
