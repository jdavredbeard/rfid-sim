const std = @import("std");

/// Speed of light in vacuum (m/s).
pub const c: f64 = 299_792_458.0;
/// Permeability of free space (H/m).
pub const mu0: f64 = 1.25663706212e-6;
/// Permittivity of free space (F/m).
pub const eps0: f64 = 8.8541878128e-12;

test "eps0 consistent with c and mu0" {
    // eps0 = 1 / (mu0 * c^2)
    const derived = 1.0 / (mu0 * c * c);
    try std.testing.expectApproxEqRel(eps0, derived, 1e-6);
}
