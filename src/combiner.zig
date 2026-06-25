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
