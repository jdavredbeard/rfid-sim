const std = @import("std");

comptime {
    if (@import("builtin").cpu.arch.endian() != .little) {
        @compileError("output.zig writes native-endian float32 but the spec requires little-endian; add byte-swapping for big-endian targets");
    }
}

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
        // native-endian == little-endian on supported targets (guarded at comptime above)
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

pub const SceneEntry = struct {
    name: []const u8,
    label: []const u8,
    description: []const u8,
};

/// Write the scenes.json manifest listing every dataset the visualizer can load.
/// Field values are emitted verbatim and are NOT JSON-escaped (matching writeJson);
/// callers must pass scene names/labels/descriptions free of `"` and `\`.
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

// ===== TESTS (written first) =====

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
    try std.testing.expectEqual(@as(usize, 2), arr.items[0].object.get("tags").?.array.items.len);
    try std.testing.expectEqualStrings("sim-output.json", obj.get("source_config").?.string);
}

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

    var parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, bytes, .{});
    defer parsed.deinit();

    try std.testing.expect(std.mem.indexOf(u8, bytes, "\"name\": \"open-warehouse\"") != null);
    try std.testing.expect(std.mem.indexOf(u8, bytes, "\"label\": \"Corridor\"") != null);
    try std.testing.expect(std.mem.indexOf(u8, bytes, "\"description\": \"\"") != null);
    try std.testing.expect(std.mem.indexOf(u8, bytes, "\"default\": \"open-warehouse\"") != null);
}
