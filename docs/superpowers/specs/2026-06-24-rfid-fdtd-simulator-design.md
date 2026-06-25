# RFID FDTD Simulator — Design Spec

## Problem

RFID inventory systems use multiple antennas to triangulate tag positions, but multipath interference (reflections off walls, shelves, etc.) degrades accuracy. We need simulated training data that captures realistic radio wave propagation — including reflections, absorption, and interference — so an ML model can learn to extract true tag positions from noisy, multipath-corrupted antenna signals.

## Solution

A 2D FDTD (Finite-Difference Time-Domain) electromagnetic simulator that:

1. Models UHF RFID (915 MHz) wave propagation in configurable room layouts
2. Records impulse responses at fixed antenna positions for each tag placement
3. Uses superposition to efficiently generate multi-tag interference scenarios
4. Outputs labeled training data for ML consumption
5. Provides browser-based visualization of wave propagation and coverage

## Architecture

Four components:

### 1. FDTD Engine (Zig)

Core electromagnetic simulation using 2D TM mode (Ez, Hx, Hy fields).

**Grid parameters (915 MHz):**
- Wavelength: λ = c/f ≈ 0.328 m
- Cell size: dx = dy = 0.015 m (~22 cells per wavelength)
- Timestep: dt = dx / (c × √2) ≈ 35.4 ps (Courant stability condition)
- Typical run: 8000 timesteps for impulse to propagate and decay

**Update equations (lossy materials):**
```
Hx[i][j] -= (dt / (μ₀ × dy)) × (Ez[i][j+1] - Ez[i][j])
Hy[i][j] += (dt / (μ₀ × dx)) × (Ez[i+1][j] - Ez[i][j])
Ez[i][j]  = Ca[i][j] × Ez[i][j] + Cb[i][j] × ((Hy[i][j] - Hy[i-1][j])/dx - (Hx[i][j] - Hx[i][j-1])/dy)
```

Where:
- `Ca = (1 - σdt/2ε) / (1 + σdt/2ε)` — damping from conductivity
- `Cb = (dt/ε) / (1 + σdt/2ε)` — curl coupling coefficient

**Source:** Gaussian-modulated sinusoidal pulse, injected as a **soft source** (additive to Ez, not overwrite) to avoid artificial reflections from the source point:

```
Ez_src(t) = exp(-((t - t0) / τ)²) × sin(2π × f0 × t)
```

Where:
- `f0` = center frequency (915 MHz)
- `τ` = 1 / (π × bandwidth)` — pulse width derived from configured bandwidth
- `t0` = 5τ — delay so the pulse starts from near-zero (avoids startup discontinuity)

**Note on physical model:** In real RFID, antennas transmit and tags backscatter. By reciprocity, modeling tags as sources and antennas as receivers yields identical impulse responses. This simulation assumes that tags don't significantly scatter each other's signals — i.e., tag A's response is the same whether tag B is present or not. This is valid for passive UHF RFID tags, which are tiny weak scatterers compared to walls and shelves. This assumption is what allows the combiner to generate multi-tag data by simply summing single-tag impulse responses (superposition) rather than re-running the FDTD for every tag combination.

**Boundaries:** Conductor walls on room perimeter (Ez = 0). This models a fully enclosed room where waves reflect off all boundaries. Rooms with openings (doors, windows) are not currently supported — this is a stated constraint.

**Metal cells:** Handled as PEC (perfect electric conductor) — directly set Ez = 0 at metal cells each timestep, rather than using extreme-sigma lossy update equations that could cause floating-point issues.

**Probes:** Record Ez at each antenna grid cell every timestep. Tag positions that coincide with antenna grid cells are skipped during the sweep to avoid meaningless self-driven readings.

**Snapshots (optional):** Save full Ez field at a configurable interval (`--snapshot-interval N`, default every 50 timesteps) for visualization. At 667×1000 grid × 4 bytes × 160 snapshots ≈ 430 MB.

### 2. Data Generator (Zig)

Orchestrator that sweeps tag positions and manages parallel simulation runs.

- Reads room config JSON
- Computes tag positions on a regular grid, skipping positions inside walls/obstacles
- Distributes simulations across CPU threads (custom thread pool using `std.Thread.spawn`)
- Writes output files incrementally

### 3. Superposition Combiner (Zig)

Post-processing tool that generates multi-tag training data from single-tag results.

**Process per training sample:**
1. Pick random count of active tags (configurable range, e.g. 1-5)
2. Select random tag positions from pre-computed set
3. Sum their impulse responses at each antenna (linearity of Maxwell's equations)
4. Inject additive Gaussian noise at random SNR (configurable range)
5. Label with list of active tag (x, y) positions

This turns K single-tag simulations into combinatorial training data without re-running the FDTD.

### 4. Visualizer (HTML/JS/Canvas)

Browser-based tool served from a local HTTP server. Four views:

1. **Room Layout** — Walls, obstacles, antenna positions, tag grid. Click to select tag positions.
2. **Wave Animation** — Plays back Ez field snapshots as a color heatmap with playback controls (play/pause/speed/scrub). Requires `--save-snapshots` flag during simulation.
3. **Coverage Heatmap** — Peak impulse response amplitude at every tag position, per antenna. Shows dead zones and multipath hotspots.
4. **Impulse Response Plot** — Time-domain waveform at each antenna for selected tag position(s). Can overlay multiple tags to visualize superposition.

## Room Config Format

```json
{
  "room": { "width": 10.0, "height": 15.0 },
  "grid_resolution": 0.015,
  "materials": {
    "concrete": { "epsilon_r": 4.5, "sigma": 0.02 },
    "metal": { "epsilon_r": 1.0, "sigma": 1e7 },
    "drywall": { "epsilon_r": 2.1, "sigma": 0.001 },
    "glass": { "epsilon_r": 6.0, "sigma": 0.004 }
  },
  "walls": [
    { "x1": 0, "y1": 0, "x2": 10, "y2": 0, "material": "concrete", "thickness": 0.2 }
  ],
  "obstacles": [
    { "type": "rect", "x": 3.0, "y": 5.0, "w": 2.0, "h": 0.8, "material": "metal" }
  ],
  "antennas": [
    { "x": 0.5, "y": 0.5, "label": "ant1" },
    { "x": 9.5, "y": 0.5, "label": "ant2" },
    { "x": 0.5, "y": 14.5, "label": "ant3" },
    { "x": 9.5, "y": 14.5, "label": "ant4" }
  ],
  "source": {
    "type": "gaussian_pulse",
    "center_freq": 915e6,
    "bandwidth": 200e6
  },
  "tag_grid_spacing": 0.25,
  "timesteps": 8000
}
```

### Material Properties Reference

| Material | ε_r | σ (S/m) | Behavior |
|----------|-----|---------|----------|
| Free space | 1.0 | 0.0 | Full transmission |
| Drywall | 2.1 | 0.001 | Mostly transparent |
| Concrete | 4.5 | 0.02 | Partial absorption/reflection |
| Glass | 6.0 | 0.004 | Partial reflection |
| Metal | 1.0 | 1e7 | Near-total reflection |

## Output Format

### Simulation Output

All binary files use **little-endian** byte order (IEEE 754 float32).

**`sim-output.json`** — metadata and sample index:
```json
{
  "version": 1,
  "config": { "...room config..." },
  "grid": { "nx": 667, "ny": 1000, "dx": 0.015, "dt": 3.54e-11 },
  "antennas": ["ant1", "ant2", "ant3", "ant4"],
  "impulse_length": 8000,
  "samples": [
    { "tag_x": 1.25, "tag_y": 2.50, "offset": 0 },
    { "tag_x": 1.50, "tag_y": 2.50, "offset": 128000 }
  ]
}
```

**`sim-output.bin`** — packed float32 impulse responses. Per sample: `[ant1[0..N], ant2[0..N], ..., antM[0..N]]` where N = impulse_length. Offset in JSON is byte offset into this file.

**`snapshots/` (optional)** — Ez field snapshots as float32 binary grids, one file per saved timestep, for wave animation visualization.

### Training Data Output (from combiner)

**`training-data.json`** — labels and metadata:
```json
{
  "source_config": "sim-output.json",
  "num_samples": 50000,
  "impulse_length": 8000,
  "num_antennas": 4,
  "samples": [
    {
      "offset": 0,
      "tags": [{ "x": 1.25, "y": 2.50 }, { "x": 5.75, "y": 8.00 }],
      "snr_db": 25.3
    }
  ]
}
```

**`training-data.bin`** — packed float32 combined waveforms, same per-sample layout as sim output.

### Combiner Config

```json
{
  "source": "sim-output",
  "num_samples": 50000,
  "tags_per_sample": { "min": 1, "max": 5 },
  "noise_snr_db": { "min": 10, "max": 40 },
  "seed": 42
}
```

- `seed` — PRNG seed for reproducible training data generation

## CLI Interface

```
rfid-sim simulate --config configs/retail.json --output sim-output [--save-snapshots] [--snapshot-interval 50] [--threads 8]
rfid-sim combine --input sim-output --config combine-config.json --output training-data
rfid-sim serve --dir sim-output --port 8080
```

- `simulate` — Run FDTD for all tag positions, write sim-output files
- `combine` — Generate multi-tag training data via superposition
- `serve` — Start local HTTP server that serves both the `viz/` static assets (HTML/JS/CSS) and the simulation data directory, so the browser app can fetch output files via `fetch()`

## File Structure

```
rfid-sim/
  build.zig
  src/
    main.zig           -- CLI parsing, subcommand dispatch
    fdtd.zig           -- FDTD engine: grid allocation, material coefficients, time stepping
    config.zig         -- JSON config parsing and validation
    output.zig         -- JSON + binary file I/O
    combiner.zig       -- Superposition, noise injection, training data generation
    generator.zig      -- Tag position sweep, thread pool, orchestration
    server.zig         -- HTTP server for viz + data serving
  configs/
    retail-example.json
  viz/
    index.html
    viz.js             -- Room layout, wave animation, heatmap, impulse response plots
    style.css
```

## Performance Estimates

For a 10m × 15m room at 1.5cm resolution (667 × 1000 grid), 8000 timesteps:

- 667K cells × 3 fields = ~2M float updates per timestep
- ~16G float ops per tag position (2M × 8000 timesteps)
- Zig (single core): ~2-4s per tag position (memory-bound stencil computation)
- Zig (8 cores): ~0.3-0.5s per tag position
- Tag positions at 25cm spacing: ~2,400 positions (skipping obstacles)
- Full sweep (8 cores): ~12-20 minutes

Superposition combiner is I/O-bound — generating 50K multi-tag samples takes seconds.

## Validation

The `simulate` command supports a `--validate` flag that runs a free-space accuracy test:

1. Creates a large empty room (e.g. 50m × 50m) with source at center and probes at known distances (1m, 2m, 5m, 10m)
2. Runs the FDTD for only enough timesteps for the direct pulse to reach all probes, but stops before wall reflections return to any probe (travel time to nearest wall and back > simulation duration)
3. Compares measured peak amplitude at each probe to the analytical 2D free-space decay (`1/√r`)
4. Reports percentage error at each probe distance

No external validation data needed — the analytical solution is the reference. This confirms the FDTD engine is correctly implemented before running real room simulations.

## Non-Goals

- 3D simulation (2D floor-plan propagation is sufficient for horizontal triangulation)
- Exact antenna radiation patterns (point probes are sufficient for training data)
- Real-time simulation (batch generation is fine)
- ML model training (separate concern; this tool produces the data)
- Rooms with openings (doors, windows) — all rooms are fully enclosed
