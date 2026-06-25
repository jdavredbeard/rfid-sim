const std = @import("std");
const config = @import("config.zig");
const grid_mod = @import("grid.zig");
const generator = @import("generator.zig");
const validate = @import("validate.zig");
const combine_config = @import("combine_config.zig");
const simdata = @import("simdata.zig");
const combiner = @import("combiner.zig");
const snapshots = @import("snapshots.zig");
const server = @import("server.zig");
const output = @import("output.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len < 2) {
        std.debug.print("usage: rfid-sim <simulate|combine> ...\n  simulate --config <path> [--output sim-output] [--threads N] [--validate]\n  combine --input <sim-output base> --config <combine config> [--output training-data]\n", .{});
        return;
    }

    if (std.mem.eql(u8, args[1], "simulate")) {
        try cmdSimulate(allocator, args[2..]);
    } else if (std.mem.eql(u8, args[1], "combine")) {
        try cmdCombine(allocator, args[2..]);
    } else if (std.mem.eql(u8, args[1], "scenes")) {
        try cmdScenes(allocator, args[2..]);
    } else if (std.mem.eql(u8, args[1], "serve")) {
        try cmdServe(allocator, args[2..]);
    } else {
        std.debug.print("unknown command: {s}\n", .{args[1]});
    }
}

fn cmdSimulate(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var config_path: ?[]const u8 = null;
    var output_base: []const u8 = "sim-output";
    var do_validate = false;
    var threads: usize = std.Thread.getCpuCount() catch 1;
    var save_snapshots = false;
    var snapshot_interval: u32 = 50;

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--config")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --config requires a value\n", .{});
                return;
            }
            config_path = args[i];
        } else if (std.mem.eql(u8, a, "--output")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --output requires a value\n", .{});
                return;
            }
            output_base = args[i];
        } else if (std.mem.eql(u8, a, "--validate")) {
            do_validate = true;
        } else if (std.mem.eql(u8, a, "--threads")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --threads requires a value\n", .{});
                return;
            }
            threads = try std.fmt.parseInt(usize, args[i], 10);
        } else if (std.mem.eql(u8, a, "--save-snapshots")) {
            save_snapshots = true;
        } else if (std.mem.eql(u8, a, "--snapshot-interval")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --snapshot-interval requires a value\n", .{});
                return;
            }
            snapshot_interval = try std.fmt.parseInt(u32, args[i], 10);
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }

    if (do_validate) {
        const results = try validate.run(allocator);
        defer allocator.free(results);
        validate.printReport(results);
        return;
    }

    const path = config_path orelse {
        std.debug.print("error: --config is required (or use --validate)\n", .{});
        return;
    };

    const config_json = try std.fs.cwd().readFileAlloc(allocator, path, 16 << 20);
    defer allocator.free(config_json);

    var parsed = config.parse(allocator, config_json) catch |e| {
        std.debug.print("error: failed to parse config: {s}\n", .{@errorName(e)});
        return;
    };
    defer parsed.deinit();

    config.validate(parsed.value) catch |e| {
        std.debug.print("error: invalid config: {s}\n", .{@errorName(e)});
        return;
    };

    var grid = try grid_mod.build(allocator, parsed.value);
    defer grid.deinit();

    grid_mod.checkAntennaPlacement(grid, parsed.value) catch |e| {
        std.debug.print("error: invalid config: {s}\n", .{@errorName(e)});
        return;
    };

    std.debug.print("simulating: {d}x{d} grid, {d} threads...\n", .{ grid.nx, grid.ny, threads });
    var timer = try std.time.Timer.start();
    const res = try generator.runSweep(allocator, parsed.value, &grid, config_json, output_base, threads);
    const secs = @as(f64, @floatFromInt(timer.read())) / 1e9;
    std.debug.print("done: {d} tag positions in {d:.1}s -> {s}.bin / {s}.json\n", .{ res.num_samples, secs, output_base, output_base });

    if (save_snapshots) {
        const positions = try generator.tagPositions(allocator, parsed.value, grid);
        defer allocator.free(positions);
        if (positions.len == 0) {
            std.debug.print("warning: no tag positions to snapshot\n", .{});
        } else {
            const p = positions[0];
            const snap_dir = try std.fmt.allocPrint(allocator, "{s}_snapshots", .{output_base});
            defer allocator.free(snap_dir);
            std.debug.print("capturing snapshots for tag ({d},{d}) every {d} steps...\n", .{ p.x, p.y, snapshot_interval });
            try snapshots.capture(allocator, &grid, .{
                .center_freq = parsed.value.source.center_freq,
                .bandwidth = parsed.value.source.bandwidth,
            }, p.i, p.j, p.x, p.y, parsed.value.timesteps, snapshot_interval, snap_dir);
            std.debug.print("snapshots written to {s}/\n", .{snap_dir});
        }
    }
}

fn lessThanStr(_: void, a: []const u8, b: []const u8) bool {
    return std.mem.lessThan(u8, a, b);
}

fn runOneScene(
    allocator: std.mem.Allocator,
    aa: std.mem.Allocator, // arena allocator for manifest strings
    scenes_dir: []const u8,
    out_dir: []const u8,
    fname: []const u8,
    threads: usize,
    entries: *std.ArrayList(output.SceneEntry),
) !void {
    const stem = fname[0 .. fname.len - ".json".len];

    const cfg_path = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ scenes_dir, fname });
    defer allocator.free(cfg_path);

    const cfg_json = try std.fs.cwd().readFileAlloc(allocator, cfg_path, 16 << 20);
    defer allocator.free(cfg_json);

    var parsed = try config.parse(allocator, cfg_json);
    defer parsed.deinit();

    try config.validate(parsed.value);

    var grid = try grid_mod.build(allocator, parsed.value);
    defer grid.deinit();

    try grid_mod.checkAntennaPlacement(grid, parsed.value);

    const base = try std.fmt.allocPrint(allocator, "{s}/{s}", .{ out_dir, stem });
    defer allocator.free(base);

    std.debug.print("scene {s}: {d}x{d} grid, {d} threads...\n", .{ stem, grid.nx, grid.ny, threads });
    const res = try generator.runSweep(allocator, parsed.value, &grid, cfg_json, base, threads);
    std.debug.print("scene {s}: {d} tags -> {s}.bin / {s}.json\n", .{ stem, res.num_samples, base, base });

    if (parsed.value.snapshots) {
        const positions = try generator.tagPositions(allocator, parsed.value, grid);
        defer allocator.free(positions);
        if (positions.len > 0) {
            const p = positions[0];
            const snap_dir = try std.fmt.allocPrint(allocator, "{s}_snapshots", .{base});
            defer allocator.free(snap_dir);
            try snapshots.capture(allocator, &grid, .{
                .center_freq = parsed.value.source.center_freq,
                .bandwidth = parsed.value.source.bandwidth,
            }, p.i, p.j, p.x, p.y, parsed.value.timesteps, 50, snap_dir);
            std.debug.print("scene {s}: snapshots -> {s}/\n", .{ stem, snap_dir });
        }
    }

    try entries.append(.{
        .name = try aa.dupe(u8, stem),
        .label = try aa.dupe(u8, parsed.value.label orelse stem),
        .description = try aa.dupe(u8, parsed.value.description orelse ""),
    });
}

fn cmdScenes(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var dir_path: ?[]const u8 = null;
    var out_dir: []const u8 = ".";
    var threads: usize = std.Thread.getCpuCount() catch 1;

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--dir")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --dir requires a value\n", .{});
                return;
            }
            dir_path = args[i];
        } else if (std.mem.eql(u8, a, "--output")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --output requires a value\n", .{});
                return;
            }
            out_dir = args[i];
        } else if (std.mem.eql(u8, a, "--threads")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --threads requires a value\n", .{});
                return;
            }
            threads = try std.fmt.parseInt(usize, args[i], 10);
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }

    const scenes_dir = dir_path orelse {
        std.debug.print("error: --dir <scene config dir> is required\n", .{});
        return;
    };
    try std.fs.cwd().makePath(out_dir);

    // Collect *.json scene filenames, sorted for stable manifest order.
    var names = std.ArrayList([]const u8).init(allocator);
    defer {
        for (names.items) |n| allocator.free(n);
        names.deinit();
    }
    {
        var d = try std.fs.cwd().openDir(scenes_dir, .{ .iterate = true });
        defer d.close();
        var it = d.iterate();
        while (try it.next()) |entry| {
            if (entry.kind != .file) continue;
            if (!std.mem.endsWith(u8, entry.name, ".json")) continue;
            try names.append(try allocator.dupe(u8, entry.name));
        }
    }
    std.mem.sort([]const u8, names.items, {}, lessThanStr);

    if (names.items.len == 0) {
        std.debug.print("error: no *.json scene configs found in {s}\n", .{scenes_dir});
        return;
    }

    // Arena holds manifest strings (stem/label/description) until the manifest is written.
    var arena = std.heap.ArenaAllocator.init(allocator);
    defer arena.deinit();
    const aa = arena.allocator();

    var entries = std.ArrayList(output.SceneEntry).init(aa);
    var had_error = false;

    for (names.items) |fname| {
        runOneScene(allocator, aa, scenes_dir, out_dir, fname, threads, &entries) catch |e| {
            std.debug.print("scene {s}: failed: {s}\n", .{ fname, @errorName(e) });
            had_error = true;
        };
    }

    if (entries.items.len == 0) {
        std.debug.print("error: no scenes succeeded; manifest not written\n", .{});
        return;
    }

    const manifest_path = try std.fmt.allocPrint(allocator, "{s}/scenes.json", .{out_dir});
    defer allocator.free(manifest_path);
    try output.writeScenesManifest(allocator, manifest_path, entries.items, entries.items[0].name);
    std.debug.print("wrote {s} ({d} scenes)\n", .{ manifest_path, entries.items.len });

    if (had_error) std.debug.print("note: some scenes failed; see messages above\n", .{});
}

fn cmdCombine(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var input_base: ?[]const u8 = null;
    var config_path: ?[]const u8 = null;
    var output_base: []const u8 = "training-data";

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--input")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --input requires a value\n", .{});
                return;
            }
            input_base = args[i];
        } else if (std.mem.eql(u8, a, "--config")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --config requires a value\n", .{});
                return;
            }
            config_path = args[i];
        } else if (std.mem.eql(u8, a, "--output")) {
            i += 1;
            if (i >= args.len) {
                std.debug.print("error: --output requires a value\n", .{});
                return;
            }
            output_base = args[i];
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }

    const inb = input_base orelse {
        std.debug.print("error: --input <sim-output base> is required\n", .{});
        return;
    };
    const cfgp = config_path orelse {
        std.debug.print("error: --config <combine config> is required\n", .{});
        return;
    };

    const cfg_json = try std.fs.cwd().readFileAlloc(allocator, cfgp, 1 << 20);
    defer allocator.free(cfg_json);

    var parsed = combine_config.parse(allocator, cfg_json) catch |e| {
        std.debug.print("error: failed to parse combine config: {s}\n", .{@errorName(e)});
        return;
    };
    defer parsed.deinit();

    combine_config.validate(parsed.value) catch |e| {
        std.debug.print("error: invalid combine config: {s}\n", .{@errorName(e)});
        return;
    };

    var sim = simdata.load(allocator, inb) catch |e| {
        std.debug.print("error: failed to load sim-output '{s}': {s}\n", .{ inb, @errorName(e) });
        return;
    };
    defer sim.deinit();

    if (parsed.value.tags_per_sample.max > sim.numSamples()) {
        std.debug.print("error: tags_per_sample.max ({d}) exceeds available tag positions ({d})\n", .{ parsed.value.tags_per_sample.max, sim.numSamples() });
        return;
    }

    const source_config = try std.fmt.allocPrint(allocator, "{s}.json", .{inb});
    defer allocator.free(source_config);

    std.debug.print("combining: {d} samples from {d} tags, {d} antennas...\n", .{ parsed.value.num_samples, sim.numSamples(), sim.num_antennas });
    var timer = try std.time.Timer.start();
    const res = try combiner.generate(allocator, &sim, parsed.value, output_base, source_config);
    const secs = @as(f64, @floatFromInt(timer.read())) / 1e9;
    std.debug.print("done: {d} training samples in {d:.2}s -> {s}.bin / {s}.json\n", .{ res.num_samples, secs, output_base, output_base });
}

fn cmdServe(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var dir: []const u8 = ".";
    var port: u16 = 8080;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--dir")) {
            i += 1;
            if (i >= args.len) { std.debug.print("error: --dir requires a value\n", .{}); return; }
            dir = args[i];
        } else if (std.mem.eql(u8, a, "--port")) {
            i += 1;
            if (i >= args.len) { std.debug.print("error: --port requires a value\n", .{}); return; }
            port = try std.fmt.parseInt(u16, args[i], 10);
        } else {
            std.debug.print("unknown flag: {s}\n", .{a});
            return;
        }
    }
    try server.serve(allocator, dir, port);
}

test {
    std.testing.refAllDeclsRecursive(@This());
}
