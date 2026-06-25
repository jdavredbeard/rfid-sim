const std = @import("std");
const simdata = @import("simdata.zig");
const combine_config = @import("combine_config.zig");
const output = @import("output.zig");

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
