const std = @import("std");

pub const IntRange = struct { min: u32, max: u32 };
pub const FloatRange = struct { min: f64, max: f64 };

pub const CombineConfig = struct {
    source: []const u8 = "",
    num_samples: u32,
    tags_per_sample: IntRange,
    noise_snr_db: FloatRange,
    seed: u64,
};

pub const ParsedConfig = std.json.Parsed(CombineConfig);

/// Parse a combine config from JSON. Caller owns the returned Parsed (call `.deinit()`).
pub fn parse(allocator: std.mem.Allocator, json: []const u8) !ParsedConfig {
    return std.json.parseFromSlice(CombineConfig, allocator, json, .{
        .ignore_unknown_fields = true,
    });
}

pub const ValidationError = error{
    NoSamples,
    BadTagRange,
    BadSnrRange,
};

pub fn validate(cfg: CombineConfig) ValidationError!void {
    if (cfg.num_samples == 0) return ValidationError.NoSamples;
    if (cfg.tags_per_sample.min == 0 or cfg.tags_per_sample.min > cfg.tags_per_sample.max) {
        return ValidationError.BadTagRange;
    }
    if (cfg.noise_snr_db.min > cfg.noise_snr_db.max) return ValidationError.BadSnrRange;
}

// ===== TESTS (written first) =====

const good_json =
    \\{
    \\  "source": "sim-output",
    \\  "num_samples": 50000,
    \\  "tags_per_sample": { "min": 1, "max": 5 },
    \\  "noise_snr_db": { "min": 10, "max": 40 },
    \\  "seed": 42
    \\}
;

test "parse combine config fields" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    const c = parsed.value;
    try std.testing.expectEqual(@as(u32, 50000), c.num_samples);
    try std.testing.expectEqual(@as(u32, 1), c.tags_per_sample.min);
    try std.testing.expectEqual(@as(u32, 5), c.tags_per_sample.max);
    try std.testing.expectEqual(@as(f64, 10), c.noise_snr_db.min);
    try std.testing.expectEqual(@as(f64, 40), c.noise_snr_db.max);
    try std.testing.expectEqual(@as(u64, 42), c.seed);
}

test "validate accepts good config" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    try validate(parsed.value);
}

test "validate rejects zero samples" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.num_samples = 0;
    try std.testing.expectError(ValidationError.NoSamples, validate(parsed.value));
}

test "validate rejects inverted tag range" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.tags_per_sample = .{ .min = 5, .max = 1 };
    try std.testing.expectError(ValidationError.BadTagRange, validate(parsed.value));
}

test "validate rejects zero min tags" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.tags_per_sample = .{ .min = 0, .max = 3 };
    try std.testing.expectError(ValidationError.BadTagRange, validate(parsed.value));
}

test "validate rejects inverted snr range" {
    var parsed = try parse(std.testing.allocator, good_json);
    defer parsed.deinit();
    parsed.value.noise_snr_db = .{ .min = 40, .max = 10 };
    try std.testing.expectError(ValidationError.BadSnrRange, validate(parsed.value));
}
