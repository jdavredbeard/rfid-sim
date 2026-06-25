const std = @import("std");

pub const Material = struct {
    epsilon_r: f64,
    sigma: f64,
};

pub const Room = struct {
    width: f64,
    height: f64,
};

pub const Wall = struct {
    x1: f64,
    y1: f64,
    x2: f64,
    y2: f64,
    material: []const u8,
    thickness: f64,
};

pub const Obstacle = struct {
    type: []const u8,
    x: f64,
    y: f64,
    w: f64,
    h: f64,
    material: []const u8,
};

pub const Antenna = struct {
    x: f64,
    y: f64,
    label: []const u8,
};

pub const TagPoint = struct {
    x: f64,
    y: f64,
};

pub const Source = struct {
    type: []const u8,
    center_freq: f64,
    bandwidth: f64,
};

pub const Config = struct {
    room: Room,
    grid_resolution: f64,
    materials: std.json.ArrayHashMap(Material),
    walls: []Wall,
    obstacles: []Obstacle,
    antennas: []Antenna,
    source: Source,
    tag_grid_spacing: ?f64 = null, // uniform grid spacing; used only when `tags` is empty
    tags: []TagPoint = &.{}, // explicit tag positions; take precedence over the grid
    timesteps: u32,
    snapshots: bool = false, // opt in to wave-view snapshots in the `scenes` batch
    label: ?[]const u8 = null, // display name for the scenes manifest
    description: ?[]const u8 = null, // one-line blurb for the scenes manifest
};

pub const ParsedConfig = std.json.Parsed(Config);

/// Parse a config from a JSON byte slice. Caller owns the returned Parsed and must call `.deinit()`.
pub fn parse(allocator: std.mem.Allocator, json: []const u8) !ParsedConfig {
    return std.json.parseFromSlice(Config, allocator, json, .{
        .ignore_unknown_fields = true,
    });
}

const test_json =
    \\{
    \\  "room": { "width": 10.0, "height": 15.0 },
    \\  "grid_resolution": 0.015,
    \\  "materials": {
    \\    "concrete": { "epsilon_r": 4.5, "sigma": 0.02 },
    \\    "metal": { "epsilon_r": 1.0, "sigma": 1e7 }
    \\  },
    \\  "walls": [
    \\    { "x1": 0, "y1": 0, "x2": 10, "y2": 0, "material": "concrete", "thickness": 0.2 }
    \\  ],
    \\  "obstacles": [
    \\    { "type": "rect", "x": 3.0, "y": 5.0, "w": 2.0, "h": 0.8, "material": "metal" }
    \\  ],
    \\  "antennas": [
    \\    { "x": 0.5, "y": 0.5, "label": "ant1" }
    \\  ],
    \\  "source": { "type": "gaussian_pulse", "center_freq": 915e6, "bandwidth": 200e6 },
    \\  "tag_grid_spacing": 0.25,
    \\  "timesteps": 8000
    \\}
;

test "parse config fields" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    const cfg = parsed.value;

    try std.testing.expectEqual(@as(f64, 10.0), cfg.room.width);
    try std.testing.expectEqual(@as(f64, 0.015), cfg.grid_resolution);
    try std.testing.expectEqual(@as(usize, 1), cfg.walls.len);
    try std.testing.expectEqual(@as(usize, 1), cfg.antennas.len);
    try std.testing.expectEqual(@as(u32, 8000), cfg.timesteps);

    const concrete = cfg.materials.map.get("concrete").?;
    try std.testing.expectEqual(@as(f64, 4.5), concrete.epsilon_r);
}

pub const ValidationError = error{
    EmptyRoom,
    BadResolution,
    UnknownMaterial,
    AntennaOutsideRoom,
    AntennaInObstacle,
    NoAntennas,
    TagOutsideRoom,
    NoTagSource,
};

fn pointInObstacle(o: Obstacle, x: f64, y: f64) bool {
    return x >= o.x and x <= o.x + o.w and y >= o.y and y <= o.y + o.h;
}

/// Validate semantic constraints the JSON parser cannot enforce.
pub fn validate(cfg: Config) ValidationError!void {
    if (cfg.room.width <= 0 or cfg.room.height <= 0) return ValidationError.EmptyRoom;
    if (cfg.grid_resolution <= 0) return ValidationError.BadResolution;
    if (cfg.antennas.len == 0) return ValidationError.NoAntennas;

    for (cfg.walls) |w| {
        if (cfg.materials.map.get(w.material) == null) return ValidationError.UnknownMaterial;
    }
    for (cfg.obstacles) |o| {
        if (cfg.materials.map.get(o.material) == null) return ValidationError.UnknownMaterial;
    }
    for (cfg.antennas) |a| {
        if (a.x < 0 or a.x > cfg.room.width or a.y < 0 or a.y > cfg.room.height) {
            return ValidationError.AntennaOutsideRoom;
        }
        for (cfg.obstacles) |o| {
            if (pointInObstacle(o, a.x, a.y)) return ValidationError.AntennaInObstacle;
        }
    }
    if (cfg.tags.len == 0 and cfg.tag_grid_spacing == null) return ValidationError.NoTagSource;
    for (cfg.tags) |t| {
        if (t.x < 0 or t.x > cfg.room.width or t.y < 0 or t.y > cfg.room.height) {
            return ValidationError.TagOutsideRoom;
        }
    }
}

test "validate accepts good config" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    try validate(parsed.value);
}

test "validate rejects antenna inside obstacle" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    // The example obstacle is rect at (3,5) size 2x0.8. Move the antenna into it.
    parsed.value.antennas[0].x = 3.5;
    parsed.value.antennas[0].y = 5.2;
    try std.testing.expectError(ValidationError.AntennaInObstacle, validate(parsed.value));
}

test "validate rejects unknown wall material" {
    var parsed = try parse(std.testing.allocator, test_json);
    defer parsed.deinit();
    parsed.value.walls[0].material = "unobtanium";
    try std.testing.expectError(ValidationError.UnknownMaterial, validate(parsed.value));
}

test "retail-example.json parses and validates" {
    const bytes = try std.fs.cwd().readFileAlloc(std.testing.allocator, "configs/retail-example.json", 1 << 20);
    defer std.testing.allocator.free(bytes);
    var parsed = try parse(std.testing.allocator, bytes);
    defer parsed.deinit();
    try validate(parsed.value);
}

const explicit_tags_json =
    \\{
    \\  "room": { "width": 6.0, "height": 6.0 },
    \\  "grid_resolution": 0.05,
    \\  "materials": { "metal": { "epsilon_r": 1.0, "sigma": 1e7 } },
    \\  "walls": [],
    \\  "obstacles": [],
    \\  "antennas": [ { "x": 0.5, "y": 0.5, "label": "a" } ],
    \\  "source": { "type": "gaussian_pulse", "center_freq": 915e6, "bandwidth": 200e6 },
    \\  "tags": [ { "x": 1.0, "y": 1.0 }, { "x": 2.0, "y": 3.0 } ],
    \\  "timesteps": 100,
    \\  "label": "Tiny",
    \\  "description": "two explicit tags"
    \\}
;

test "parse explicit tags and optional spacing" {
    var parsed = try parse(std.testing.allocator, explicit_tags_json);
    defer parsed.deinit();
    const cfg = parsed.value;
    try std.testing.expectEqual(@as(usize, 2), cfg.tags.len);
    try std.testing.expectEqual(@as(f64, 1.0), cfg.tags[0].x);
    try std.testing.expectEqual(@as(?f64, null), cfg.tag_grid_spacing);
    try std.testing.expectEqualStrings("Tiny", cfg.label.?);
    try std.testing.expectEqualStrings("two explicit tags", cfg.description.?);
    try validate(cfg);
}

test "validate rejects tag outside room" {
    var parsed = try parse(std.testing.allocator, explicit_tags_json);
    defer parsed.deinit();
    parsed.value.tags[0].y = 99.0;
    try std.testing.expectError(ValidationError.TagOutsideRoom, validate(parsed.value));
}

test "validate rejects config with no tag source" {
    var parsed = try parse(std.testing.allocator, explicit_tags_json);
    defer parsed.deinit();
    parsed.value.tags = &.{};
    parsed.value.tag_grid_spacing = null;
    try std.testing.expectError(ValidationError.NoTagSource, validate(parsed.value));
}

test "all scene configs parse and validate" {
    var dir = try std.fs.cwd().openDir("configs/scenes", .{ .iterate = true });
    defer dir.close();
    var it = dir.iterate();
    var count: usize = 0;
    while (try it.next()) |entry| {
        if (entry.kind != .file) continue;
        if (!std.mem.endsWith(u8, entry.name, ".json")) continue;
        const bytes = try dir.readFileAlloc(std.testing.allocator, entry.name, 1 << 20);
        defer std.testing.allocator.free(bytes);
        var parsed = try parse(std.testing.allocator, bytes);
        defer parsed.deinit();
        try validate(parsed.value);
        count += 1;
    }
    try std.testing.expect(count >= 6);
}
