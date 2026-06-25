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

/// Add Gaussian noise to `buf` so that signal-power / noise-power == 10^(snr_db/10).
/// Signal power is measured as mean(x^2) over all antennas/timesteps of the current buffer.
/// No-op if the signal power is zero (avoids division by zero on an all-zero waveform).
pub fn addNoiseInto(buf: [][]f32, snr_db: f64, rand: std.Random) void {
    var sumsq: f64 = 0;
    var count: usize = 0;
    for (buf) |b| {
        for (b) |v| {
            sumsq += @as(f64, v) * @as(f64, v);
            count += 1;
        }
    }
    if (count == 0) return;
    const sig_power = sumsq / @as(f64, @floatFromInt(count));
    if (sig_power <= 0) return;

    const noise_power = sig_power / std.math.pow(f64, 10.0, snr_db / 10.0);
    const sigma = @sqrt(noise_power);
    for (buf) |b| {
        for (b) |*v| {
            v.* += @floatCast(sigma * rand.floatNorm(f64));
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

test "addNoiseInto realizes the target SNR within tolerance" {
    const M = 20000;
    const sig = try std.testing.allocator.alloc(f32, M);
    defer std.testing.allocator.free(sig);
    @memset(sig, 1.0); // constant signal => signal power = 1.0
    const orig = try std.testing.allocator.dupe(f32, sig);
    defer std.testing.allocator.free(orig);

    var prng = std.Random.DefaultPrng.init(12345);
    var buf = [_][]f32{sig};
    const snr_db: f64 = 20.0; // => noise power = 1/100 = 0.01
    addNoiseInto(&buf, snr_db, prng.random());

    var sumsq: f64 = 0;
    for (sig, 0..) |v, i| {
        const d = @as(f64, v) - @as(f64, orig[i]);
        sumsq += d * d;
    }
    const realized = sumsq / @as(f64, @floatFromInt(M));
    const target = 0.01;
    try std.testing.expect(@abs(realized - target) / target < 0.10); // within 10%
}

test "addNoiseInto is a no-op on a zero signal" {
    var z = [_]f32{ 0, 0, 0, 0 };
    var buf = [_][]f32{&z};
    var prng = std.Random.DefaultPrng.init(7);
    addNoiseInto(&buf, 20.0, prng.random());
    try std.testing.expectEqualSlices(f32, &[_]f32{ 0, 0, 0, 0 }, &z);
}

pub const GenResult = struct { num_samples: usize };

pub const GenerateError = error{NotEnoughTags};

/// Generate `cfg.num_samples` multi-tag training samples and write
/// `<output_base>.bin` + `<output_base>.json`. Reproducible for a fixed `cfg.seed`.
pub fn generate(
    allocator: std.mem.Allocator,
    sim: *const simdata.SimData,
    cfg: combine_config.CombineConfig,
    output_base: []const u8,
) !GenResult {
    const m = sim.num_antennas;
    const n = sim.impulse_length;
    const num_tags = sim.numSamples();
    if (cfg.tags_per_sample.max > num_tags) return GenerateError.NotEnoughTags;

    // Reusable combined buffers (m antennas x n) + a const view for appendSample.
    const out = try allocator.alloc([]f32, m);
    defer {
        for (out) |o| allocator.free(o);
        allocator.free(out);
    }
    {
        var built: usize = 0;
        errdefer {
            var z: usize = 0;
            while (z < built) : (z += 1) allocator.free(out[z]);
        }
        while (built < m) : (built += 1) out[built] = try allocator.alloc(f32, n);
    }
    const view = try allocator.alloc([]const f32, m);
    defer allocator.free(view);
    for (out, 0..) |o, a| view[a] = o;

    // Persistent permutation for distinct selection.
    const idx = try allocator.alloc(usize, num_tags);
    defer allocator.free(idx);
    for (idx, 0..) |*v, i| v.* = i;

    var prng = std.Random.DefaultPrng.init(cfg.seed);
    const rand = prng.random();

    const bin_path = try std.fmt.allocPrint(allocator, "{s}.bin", .{output_base});
    defer allocator.free(bin_path);
    const json_path = try std.fmt.allocPrint(allocator, "{s}.json", .{output_base});
    defer allocator.free(json_path);

    const bin = try std.fs.cwd().createFile(bin_path, .{});
    defer bin.close();

    var samples = std.ArrayList(output.TrainingSampleMeta).init(allocator);
    defer {
        for (samples.items) |s| allocator.free(@constCast(s.tags));
        samples.deinit();
    }

    var s: u32 = 0;
    while (s < cfg.num_samples) : (s += 1) {
        const k = rand.intRangeAtMost(u32, cfg.tags_per_sample.min, cfg.tags_per_sample.max);
        var j: usize = 0;
        while (j < k) : (j += 1) {
            const swap_with = rand.intRangeLessThan(usize, j, num_tags);
            const tmp = idx[j];
            idx[j] = idx[swap_with];
            idx[swap_with] = tmp;
        }
        const selected = idx[0..k];

        superposeInto(out, sim.*, selected);
        const snr = cfg.noise_snr_db.min + rand.float(f64) * (cfg.noise_snr_db.max - cfg.noise_snr_db.min);
        addNoiseInto(out, snr, rand);

        const offset = try output.appendSample(bin, view);

        const tags = try allocator.alloc(output.TagLabel, k);
        for (selected, 0..) |ti, z| tags[z] = .{ .x = sim.tag_x[ti], .y = sim.tag_y[ti] };
        try samples.append(.{ .offset = offset, .tags = tags, .snr_db = snr });
    }

    try output.writeTrainingJson(allocator, json_path, "sim-output.json", @intCast(n), m, samples.items);
    return .{ .num_samples = cfg.num_samples };
}

fn buildTinySim(allocator: std.mem.Allocator) !simdata.SimData {
    // 4 tags, 2 antennas, length 3. Distinct per-tag values.
    const data = try allocator.dupe(f32, &[_]f32{
        1,  1,  1,   2,  2,  2, // tag0: ant0, ant1
        3,  3,  3,   4,  4,  4, // tag1
        5,  5,  5,   6,  6,  6, // tag2
        7,  7,  7,   8,  8,  8, // tag3
    });
    const tx = try allocator.dupe(f64, &[_]f64{ 0.1, 0.2, 0.3, 0.4 });
    const ty = try allocator.dupe(f64, &[_]f64{ 1.1, 1.2, 1.3, 1.4 });
    const offs = try allocator.dupe(u64, &[_]u64{ 0, 24, 48, 72 }); // 2 ant*3*4 = 24 per tag
    return simdata.SimData{
        .allocator = allocator,
        .num_antennas = 2,
        .impulse_length = 3,
        .tag_x = tx,
        .tag_y = ty,
        .offsets = offs,
        .data = data,
    };
}

test "generate writes training bin+json with correct counts, sizes, label ranges" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir);
    const base = try std.fs.path.join(std.testing.allocator, &.{ dir, "train" });
    defer std.testing.allocator.free(base);

    var sim = try buildTinySim(std.testing.allocator);
    defer sim.deinit();

    const cfg = combine_config.CombineConfig{
        .source = "sim",
        .num_samples = 20,
        .tags_per_sample = .{ .min = 1, .max = 3 },
        .noise_snr_db = .{ .min = 15, .max = 30 },
        .seed = 99,
    };

    const res = try generate(std.testing.allocator, &sim, cfg, base);
    try std.testing.expectEqual(@as(usize, 20), res.num_samples);

    const bin_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.bin", .{base});
    defer std.testing.allocator.free(bin_path);
    const stat = try std.fs.cwd().statFile(bin_path);
    try std.testing.expectEqual(@as(u64, 20 * 2 * 3 * 4), stat.size);

    const json_path = try std.fmt.allocPrint(std.testing.allocator, "{s}.json", .{base});
    defer std.testing.allocator.free(json_path);
    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, json_path, 1 << 20);
    defer std.testing.allocator.free(bytes);
    const parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, bytes, .{});
    defer parsed.deinit();
    const arr = parsed.value.object.get("samples").?.array;
    try std.testing.expectEqual(@as(usize, 20), arr.items.len);
    for (arr.items, 0..) |item, i| {
        const o = item.object;
        const ntags = o.get("tags").?.array.items.len;
        try std.testing.expect(ntags >= 1 and ntags <= 3);
        const snr = o.get("snr_db").?.float;
        try std.testing.expect(snr >= 15.0 and snr <= 30.0);
        try std.testing.expectEqual(@as(i64, @intCast(i * 2 * 3 * 4)), o.get("offset").?.integer);
    }
}

test "generate is reproducible for a fixed seed" {
    var tmp = std.testing.tmpDir(.{});
    defer tmp.cleanup();
    const dir = try tmp.dir.realpathAlloc(std.testing.allocator, ".");
    defer std.testing.allocator.free(dir);

    var sim = try buildTinySim(std.testing.allocator);
    defer sim.deinit();
    const cfg = combine_config.CombineConfig{
        .source = "sim",
        .num_samples = 10,
        .tags_per_sample = .{ .min = 1, .max = 3 },
        .noise_snr_db = .{ .min = 10, .max = 40 },
        .seed = 2024,
    };

    const baseA = try std.fs.path.join(std.testing.allocator, &.{ dir, "a" });
    defer std.testing.allocator.free(baseA);
    const baseB = try std.fs.path.join(std.testing.allocator, &.{ dir, "b" });
    defer std.testing.allocator.free(baseB);
    _ = try generate(std.testing.allocator, &sim, cfg, baseA);
    _ = try generate(std.testing.allocator, &sim, cfg, baseB);

    const aBinPath = try std.fmt.allocPrint(std.testing.allocator, "{s}.bin", .{baseA});
    defer std.testing.allocator.free(aBinPath);
    const bBinPath = try std.fmt.allocPrint(std.testing.allocator, "{s}.bin", .{baseB});
    defer std.testing.allocator.free(bBinPath);
    const aBin = try std.fs.cwd().readFileAlloc(std.testing.allocator, aBinPath, 1 << 20);
    defer std.testing.allocator.free(aBin);
    const bBin = try std.fs.cwd().readFileAlloc(std.testing.allocator, bBinPath, 1 << 20);
    defer std.testing.allocator.free(bBin);
    try std.testing.expect(std.mem.eql(u8, aBin, bBin));
}
