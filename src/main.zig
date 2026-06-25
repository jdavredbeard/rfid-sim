const std = @import("std");
const config = @import("config.zig");
const grid_mod = @import("grid.zig");
const generator = @import("generator.zig");
const validate = @import("validate.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len < 2) {
        std.debug.print("usage: rfid-sim simulate --config <path> [--output sim-output] [--threads N] [--validate]\n", .{});
        return;
    }

    if (std.mem.eql(u8, args[1], "simulate")) {
        try cmdSimulate(allocator, args[2..]);
    } else {
        std.debug.print("unknown command: {s}\n", .{args[1]});
    }
}

fn cmdSimulate(allocator: std.mem.Allocator, args: []const []const u8) !void {
    var config_path: ?[]const u8 = null;
    var output_base: []const u8 = "sim-output";
    var do_validate = false;
    var threads: usize = std.Thread.getCpuCount() catch 1;

    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const a = args[i];
        if (std.mem.eql(u8, a, "--config")) {
            i += 1;
            config_path = args[i];
        } else if (std.mem.eql(u8, a, "--output")) {
            i += 1;
            output_base = args[i];
        } else if (std.mem.eql(u8, a, "--validate")) {
            do_validate = true;
        } else if (std.mem.eql(u8, a, "--threads")) {
            i += 1;
            threads = try std.fmt.parseInt(usize, args[i], 10);
        } else if (std.mem.eql(u8, a, "--save-snapshots") or std.mem.eql(u8, a, "--snapshot-interval")) {
            std.debug.print("warning: snapshot flags are not supported in this build (see Visualizer plan)\n", .{});
            if (std.mem.eql(u8, a, "--snapshot-interval")) i += 1;
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

    std.debug.print("simulating: {d}x{d} grid, {d} threads...\n", .{ grid.nx, grid.ny, threads });
    var timer = try std.time.Timer.start();
    const res = try generator.runSweep(allocator, parsed.value, &grid, config_json, output_base, threads);
    const secs = @as(f64, @floatFromInt(timer.read())) / 1e9;
    std.debug.print("done: {d} tag positions in {d:.1}s -> {s}.bin / {s}.json\n", .{ res.num_samples, secs, output_base, output_base });
}

test {
    std.testing.refAllDeclsRecursive(@This());
}
