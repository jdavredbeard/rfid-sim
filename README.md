# rfid-sim

A 2D FDTD (Finite-Difference Time-Domain) electromagnetic simulator for UHF RFID
(915 MHz). It models radio-wave propagation in configurable room layouts —
including reflections, absorption, and multipath interference — and generates
labeled training data for ML models that extract tag positions from noisy,
multipath-corrupted antenna signals.

Built with Zig 0.14. No external dependencies.

## Build

```bash
zig build                      # debug build -> zig-out/bin/rfid-sim
zig build -Doptimize=ReleaseFast   # optimized build (use for real runs)
zig build test                 # run the unit test suite
```

## Commands

### `simulate` — run the FDTD sweep

Runs the FDTD engine for every tag position on a grid and writes single-tag
impulse responses.

```bash
rfid-sim simulate --config configs/retail-example.json --output sim-output \
  [--threads N] [--save-snapshots] [--snapshot-interval 50]

rfid-sim simulate --validate     # free-space 1/sqrt(r) accuracy self-test
```

Outputs `sim-output.json` (metadata + per-sample byte offsets) and
`sim-output.bin` (packed little-endian float32, antenna-major per sample).
With `--save-snapshots`, also writes `sim-output_snapshots/` (full Ez-field
grids every `--snapshot-interval` steps + a `snapshots.json` manifest) for the
first tag position, used by the wave-animation view.

### `scenes` — build a set of varied scene datasets

Runs every `*.json` config in a directory and writes one dataset per scene plus a
`scenes.json` manifest into the output dir.

```bash
rfid-sim scenes --dir configs/scenes --output viz-data [--threads N]
```

Each scene config is the normal simulate config, optionally with `tags` (an explicit
list of `{ "x": …, "y": … }` positions, used instead of `tag_grid_spacing`),
`label`/`description` (shown in the visualizer dropdown), and `snapshots: true`
(capture wave-animation snapshots for that scene). Scenes that fail to parse/validate
are reported and skipped; the rest still build.

Serve the output dir and the visualizer shows a dataset dropdown to switch between
scenes:

```bash
rfid-sim serve --dir viz-data --port 8080
```

The bundled `configs/scenes/` has six example layouts (open warehouse, L-shaped store,
narrow corridor, cluttered backroom, glass atrium, small office) varying room shape,
obstacles, antennas, and tag placement.

### `combine` — generate multi-tag training data

Generates multi-tag training samples from the single-tag results via
superposition + additive Gaussian noise. Reproducible for a fixed `seed`.

```bash
rfid-sim combine --input sim-output --config configs/combine-example.json \
  --output training-data
```

Outputs `training-data.json` (labels: active tags' `(x,y)` + `snr_db`) and
`training-data.bin` (same layout as the sim output).

### `serve` — browser visualizer

Starts a local HTTP server that hosts the visualizer and serves the data
directory.

```bash
rfid-sim serve --dir <data-dir> --port 8080
# then open http://127.0.0.1:8080/
```

The visualizer has four views: **Room Layout** (walls, obstacles, antennas, tag
grid; click a tag to select it), **Wave Animation** (Ez snapshot playback;
requires `--save-snapshots`), **Coverage Heatmap** (peak amplitude per tag, per
antenna), and **Impulse Response** (per-antenna waveforms for selected tags;
overlay multiple to see superposition).

The viz loads `sim-output.*` by default; serve a directory containing a
different base name and point the page at it with the `?data=<base>` query
param, e.g. `http://127.0.0.1:8080/?data=training-data`. When a `scenes.json`
manifest is present (as written by `scenes`), the page shows a dataset dropdown
to switch between scenes; `?data=<name>` still deep-links a specific scene.

## Config

See `configs/retail-example.json` (room geometry, materials, walls, obstacles,
antennas, source) and `configs/combine-example.json` (sample count, tags per
sample, SNR range, seed). Design details are in
`docs/superpowers/specs/2026-06-24-rfid-fdtd-simulator-design.md`.

In addition to the fields above, a simulate config may include these optional
fields (the `configs/scenes/*.json` files are worked examples):

- `tags`: an explicit list of `{ "x", "y" }` tag positions; when present it is
  used instead of `tag_grid_spacing` (which then becomes optional). Tags that
  land in a wall/obstacle or on an antenna cell are skipped with a warning.
- `label` / `description`: shown in the visualizer's dataset dropdown (used by
  the `scenes` command).
- `snapshots`: `true` to capture wave-animation snapshots for the scene in the
  `scenes` batch.
