const std = @import("std");
const grid_mod = @import("grid.zig");
const fdtd = @import("fdtd.zig");

// ===== IMPLEMENTATION =====

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

    try writeManifest(allocator, out_dir, grid, tag_x, tag_y, safe_interval, files.items, steps.items);
}

fn writeManifest(
    allocator: std.mem.Allocator,
    out_dir: []const u8,
    grid: *const grid_mod.Grid,
    tag_x: f64,
    tag_y: f64,
    interval: u32,
    files: []const []const u8,
    steps: []const u32,
) !void {
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

    const snap0 = try std.fs.path.join(std.testing.allocator, &.{ out_dir, "snap_0000.bin" });
    defer std.testing.allocator.free(snap0);
    const stat = try std.fs.cwd().statFile(snap0);
    try std.testing.expectEqual(@as(u64, 20 * 20 * 4), stat.size);
}
