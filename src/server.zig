const std = @import("std");

// Embedded static assets (viz/ lives in src/viz/ within the package root).
const index_html = @embedFile("viz/index.html");
const viz_js = @embedFile("viz/viz.js");
const style_css = @embedFile("viz/style.css");

pub const RouteTag = enum { index, asset, data, not_found };
pub const Route = union(RouteTag) {
    index,
    asset: []const u8,
    data: []const u8,
    not_found,
};

pub fn resolveRoute(target: []const u8) Route {
    const qpos = std.mem.indexOfScalar(u8, target, '?');
    const path = if (qpos) |q| target[0..q] else target;

    if (std.mem.eql(u8, path, "/")) return .index;
    if (std.mem.eql(u8, path, "/index.html")) return .index;
    if (std.mem.eql(u8, path, "/viz.js")) return .{ .asset = "viz.js" };
    if (std.mem.eql(u8, path, "/style.css")) return .{ .asset = "style.css" };
    if (std.mem.startsWith(u8, path, "/data/")) {
        const rel = path["/data/".len..];
        if (rel.len == 0) return .not_found;
        if (std.mem.indexOf(u8, rel, "..") != null) return .not_found;
        if (rel[0] == '/') return .not_found;
        return .{ .data = rel };
    }
    return .not_found;
}

pub fn contentType(name: []const u8) []const u8 {
    if (std.mem.endsWith(u8, name, ".html")) return "text/html";
    if (std.mem.endsWith(u8, name, ".js")) return "text/javascript";
    if (std.mem.endsWith(u8, name, ".css")) return "text/css";
    if (std.mem.endsWith(u8, name, ".json")) return "application/json";
    if (std.mem.endsWith(u8, name, ".bin")) return "application/octet-stream";
    return "application/octet-stream";
}

fn assetBytes(name: []const u8) []const u8 {
    if (std.mem.eql(u8, name, "viz.js")) return viz_js;
    if (std.mem.eql(u8, name, "style.css")) return style_css;
    return index_html;
}

/// Start the HTTP server. Serves embedded viz assets at / and files from `dir` under /data/.
/// Single-threaded: one request at a time (sufficient for a local viz tool).
pub fn serve(allocator: std.mem.Allocator, dir: []const u8, port: u16) !void {
    const address = try std.net.Address.parseIp("127.0.0.1", port);
    var net_server = try address.listen(.{ .reuse_address = true });
    defer net_server.deinit();
    std.debug.print("serving http://127.0.0.1:{d}/  (data dir: {s})\n", .{ port, dir });

    var read_buffer: [64 * 1024]u8 = undefined;
    while (true) {
        const conn = net_server.accept() catch |e| {
            std.debug.print("accept error: {s}\n", .{@errorName(e)});
            continue;
        };
        defer conn.stream.close();
        var http_server = std.http.Server.init(conn, &read_buffer);
        while (http_server.state == .ready) {
            var request = http_server.receiveHead() catch |e| {
                if (e != error.HttpConnectionClosing) std.debug.print("receiveHead: {s}\n", .{@errorName(e)});
                break;
            };
            handle(allocator, &request, dir) catch |e| {
                std.debug.print("handler error: {s}\n", .{@errorName(e)});
            };
        }
    }
}

fn handle(allocator: std.mem.Allocator, request: *std.http.Server.Request, dir: []const u8) !void {
    const route = resolveRoute(request.head.target);
    switch (route) {
        .index => try respondBytes(request, index_html, "text/html"),
        .asset => |name| try respondBytes(request, assetBytes(name), contentType(name)),
        .not_found => try request.respond("not found\n", .{ .status = .not_found }),
        .data => |rel| {
            const full = try std.fs.path.join(allocator, &.{ dir, rel });
            defer allocator.free(full);
            const bytes = std.fs.cwd().readFileAlloc(allocator, full, 1 << 30) catch {
                try request.respond("not found\n", .{ .status = .not_found });
                return;
            };
            defer allocator.free(bytes);
            try respondBytes(request, bytes, contentType(rel));
        },
    }
}

fn respondBytes(request: *std.http.Server.Request, bytes: []const u8, ctype: []const u8) !void {
    try request.respond(bytes, .{
        .extra_headers = &.{.{ .name = "content-type", .value = ctype }},
    });
}

// ===== TESTS (written first) =====

test "resolveRoute maps paths to handlers" {
    try std.testing.expectEqual(Route.index, resolveRoute("/"));
    try std.testing.expectEqual(Route.index, resolveRoute("/?data=foo"));
    switch (resolveRoute("/viz.js")) {
        .asset => |a| try std.testing.expectEqualStrings("viz.js", a),
        else => return error.Wrong,
    }
    switch (resolveRoute("/data/sim-output.json")) {
        .data => |d| try std.testing.expectEqualStrings("sim-output.json", d),
        else => return error.Wrong,
    }
    switch (resolveRoute("/data/sim-output_snapshots/snap_0000.bin")) {
        .data => |d| try std.testing.expectEqualStrings("sim-output_snapshots/snap_0000.bin", d),
        else => return error.Wrong,
    }
    try std.testing.expectEqual(Route.not_found, resolveRoute("/nope"));
    try std.testing.expectEqual(Route.not_found, resolveRoute("/data/../secret"));
}

test "contentType by extension" {
    try std.testing.expectEqualStrings("text/html", contentType("index.html"));
    try std.testing.expectEqualStrings("text/javascript", contentType("viz.js"));
    try std.testing.expectEqualStrings("text/css", contentType("style.css"));
    try std.testing.expectEqualStrings("application/json", contentType("sim-output.json"));
    try std.testing.expectEqualStrings("application/octet-stream", contentType("sim-output.bin"));
    try std.testing.expectEqualStrings("application/octet-stream", contentType("weird.xyz"));
}
