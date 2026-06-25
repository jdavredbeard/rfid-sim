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

**Source:** Gaussian-modulated pulse centered at 915 MHz. Broadband, captures full impulse response in a single run.

**Boundaries:** Conductor walls on room perimeter (physically correct for enclosed rooms — waves reflect).

**Probes:** Record Ez at each antenna grid cell every timestep.

**Snapshots (optional):** Save full Ez field at configurable intervals for visualization.

### 2. Data Generator (Zig)

Orchestrator that sweeps tag positions and manages parallel simulation runs.

- Reads room config JSON
- Computes tag positions on a regular grid, skipping positions inside walls/obstacles
- Distributes simulations across CPU threads (Zig's `std.Thread` pool)
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

**`sim-output.json`** — metadata and sample index:
```json
{
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
  "output": "training-data"
}
```

## CLI Interface

```
rfid-sim simulate --config configs/retail.json --output sim-output [--save-snapshots] [--threads 8]
rfid-sim combine --input sim-output --config combine-config.json --output training-data
rfid-sim serve --dir sim-output --port 8080
```

- `simulate` — Run FDTD for all tag positions, write sim-output files
- `combine` — Generate multi-tag training data via superposition
- `serve` — Start local HTTP server for the browser visualizer

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
  configs/
    retail-example.json
  viz/
    index.html
    viz.js             -- Room layout, wave animation, heatmap, impulse response plots
    style.css
```

## Performance Estimates

For a 10m × 15m room at 1.5cm resolution (667 × 1000 grid), 8000 timesteps:

- ~5.3M cells × 3 fields = ~16M float updates per timestep
- ~128G float ops per tag position
- Zig (single core): ~0.2-0.5s per tag position
- Zig (8 cores): ~0.03-0.06s per tag position
- Tag positions at 25cm spacing: ~2,400 positions (skipping obstacles)
- Full sweep (8 cores): ~1-2.5 minutes

Superposition combiner is I/O-bound — generating 50K multi-tag samples takes seconds.

## Non-Goals

- 3D simulation (2D floor-plan propagation is sufficient for horizontal triangulation)
- Exact antenna radiation patterns (point probes are sufficient for training data)
- Real-time simulation (batch generation is fine)
- ML model training (separate concern; this tool produces the data)
