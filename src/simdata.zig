const std = @import("std");
const output = @import("output.zig");

pub const SimData = struct {
    allocator: std.mem.Allocator,
    num_antennas: usize,
    impulse_length: usize,
    tag_x: []f64,
    tag_y: []f64,
    offsets: []u64, // byte offsets into the bin, per sample
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
        .impulse_length = @intCast(meta.impulse_length),
        .tag_x = tag_x,
        .tag_y = tag_y,
        .offsets = offsets,
        .data = data,
    };
}

// ===== TESTS (written first) =====

test "impulse accessor slices the right antenna-major window" {
    // 2 samples, 2 antennas, impulse_length 3.
    var data = [_]f32{ 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    var tx = [_]f64{ 0.1, 0.2 };
    var ty = [_]f64{ 0.3, 0.4 };
    var offs = [_]u64{ 0, 24 }; // sample1: 2 ant * 3 floats * 4 bytes = 24
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
