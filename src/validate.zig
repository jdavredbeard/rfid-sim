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

    // Stop before the direct pulse reaches the nearest wall (25 m away) and
    // reflects back (50 m round trip). Furthest probe is 10 m. Use ~22 m travel.
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
        std.debug.print("  {d:>6.1}  {e:>14.4}  {d:>10.4}  {d:>10.4}  {d:>8.2}\n", .{
            r.distance_m, r.measured_peak, r.expected_ratio, r.measured_ratio, r.percent_error,
        });
    }
}

test "free-space decay matches 1/sqrt(r) within tolerance" {
    // Heavy test (large grid). Gated so the default `zig build test` stays fast.
    // Run explicitly with: RUN_VALIDATION_TEST=1 zig test -OReleaseFast src/validate.zig
    if (!std.process.hasEnvVarConstant("RUN_VALIDATION_TEST")) return error.SkipZigTest;
    const results = try run(std.testing.allocator);
    defer std.testing.allocator.free(results);
    try std.testing.expectEqual(@as(usize, 4), results.len);
    // Reference probe (1 m) is exact by construction; others within ~15% of
    // analytical 2D decay at this resolution.
    for (results) |r| {
        try std.testing.expect(r.percent_error < 15.0);
    }
}
