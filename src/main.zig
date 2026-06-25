const std = @import("std");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len < 2) {
        std.debug.print("usage: rfid-sim <simulate> [options]\n", .{});
        return;
    }

    const cmd = args[1];
    if (std.mem.eql(u8, cmd, "simulate")) {
        std.debug.print("simulate: not yet implemented\n", .{});
    } else {
        std.debug.print("unknown command: {s}\n", .{cmd});
    }
}

test {
    std.testing.refAllDeclsRecursive(@This());
}
