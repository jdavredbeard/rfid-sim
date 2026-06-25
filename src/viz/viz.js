"use strict";

// ---- shared state ----
const params = new URLSearchParams(location.search);
let BASE = params.get("data") || "sim-output";
let currentView = "room";
const state = {
  meta: null,      // parsed sim-output.json
  bin: null,       // ArrayBuffer of sim-output.bin (lazy)
  snapshots: null, // parsed snapshots.json (lazy)
  selected: [],    // selected tag sample indices
};

function setStatus(msg) { document.getElementById("status").textContent = msg; }

// world->canvas transform for a given room size and canvas
function makeTransform(canvas, worldW, worldH) {
  const pad = 20;
  const sx = (canvas.width - 2 * pad) / worldW;
  const sy = (canvas.height - 2 * pad) / worldH;
  const s = Math.min(sx, sy);
  return {
    x: (wx) => pad + wx * s,
    y: (wy) => pad + wy * s, // y grows downward; room y=0 at top
    s,
  };
}

async function loadMeta() {
  const r = await fetch(`/data/${BASE}.json`);
  if (!r.ok) throw new Error(`fetch ${BASE}.json: ${r.status}`);
  state.meta = await r.json();
}

async function ensureBin() {
  if (state.bin) return state.bin;
  setStatus("loading impulse data…");
  const r = await fetch(`/data/${BASE}.bin`);
  if (!r.ok) throw new Error(`fetch ${BASE}.bin: ${r.status}`);
  state.bin = await r.arrayBuffer();
  setStatus(`ready (${state.meta.samples.length} tags, ${state.meta.antennas.length} antennas)`);
  return state.bin;
}

// Float32Array view of antenna `a`'s impulse response for sample index `s`.
function impulseFor(s, a) {
  const N = state.meta.impulse_length;
  const byteOffset = state.meta.samples[s].offset + a * N * 4;
  return new Float32Array(state.bin, byteOffset, N);
}

// ---- Room Layout view ----
function renderRoom() {
  const canvas = document.getElementById("room-canvas");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const cfg = state.meta.config;
  const T = makeTransform(canvas, cfg.room.width, cfg.room.height);

  ctx.strokeStyle = "#30363d";
  ctx.strokeRect(T.x(0), T.y(0), cfg.room.width * T.s, cfg.room.height * T.s);

  ctx.fillStyle = "rgba(248,81,73,0.5)";
  for (const o of cfg.obstacles || []) {
    ctx.fillRect(T.x(o.x), T.y(o.y), o.w * T.s, o.h * T.s);
  }

  ctx.strokeStyle = "#8b949e";
  for (const w of cfg.walls || []) {
    ctx.lineWidth = Math.max(1, (w.thickness || 0.1) * T.s);
    ctx.beginPath();
    ctx.moveTo(T.x(w.x1), T.y(w.y1));
    ctx.lineTo(T.x(w.x2), T.y(w.y2));
    ctx.stroke();
  }
  ctx.lineWidth = 1;

  for (let i = 0; i < state.meta.samples.length; i++) {
    const s = state.meta.samples[i];
    ctx.fillStyle = state.selected.includes(i) ? "#f0c000" : "#3fb950";
    ctx.beginPath();
    ctx.arc(T.x(s.tag_x), T.y(s.tag_y), state.selected.includes(i) ? 5 : 2.5, 0, 2 * Math.PI);
    ctx.fill();
  }

  for (const a of cfg.antennas || []) {
    ctx.fillStyle = "#1f6feb";
    ctx.beginPath();
    ctx.arc(T.x(a.x), T.y(a.y), 6, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = "#e6edf3";
    ctx.font = "12px sans-serif";
    ctx.fillText(a.label, T.x(a.x) + 8, T.y(a.y) - 8);
  }

  canvas.onclick = (ev) => {
    const rect = canvas.getBoundingClientRect();
    const px = (ev.clientX - rect.left) * (canvas.width / rect.width);
    const py = (ev.clientY - rect.top) * (canvas.height / rect.height);
    let best = -1, bestD = 1e9;
    for (let i = 0; i < state.meta.samples.length; i++) {
      const s = state.meta.samples[i];
      const dx = T.x(s.tag_x) - px, dy = T.y(s.tag_y) - py;
      const d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; best = i; }
    }
    if (best >= 0 && bestD < 200) {
      const at = state.selected.indexOf(best);
      if (at >= 0) state.selected.splice(at, 1); else state.selected.push(best);
      updateSelectionLabel();
      renderRoom();
    }
  };
}

function updateSelectionLabel() {
  const el = document.getElementById("room-selection");
  if (state.selected.length === 0) { el.textContent = "No tags selected."; return; }
  el.textContent = "Selected: " + state.selected
    .map((i) => `(${state.meta.samples[i].tag_x.toFixed(2)}, ${state.meta.samples[i].tag_y.toFixed(2)})`)
    .join(", ");
}

// ---- Coverage Heatmap view ----
function populateAntennaSelect() {
  const sel = document.getElementById("coverage-antenna");
  if (sel.options.length) return;
  state.meta.antennas.forEach((label, idx) => {
    const opt = document.createElement("option");
    opt.value = String(idx);
    opt.textContent = label;
    sel.appendChild(opt);
  });
  sel.addEventListener("change", renderCoverage);
}

function valueToColor(t) {
  // t in [0,1] -> blue(low) -> green -> yellow -> red(high)
  const r = Math.max(0, Math.min(255, Math.round(255 * (t * 1.5 - 0.2))));
  const g = Math.max(0, Math.min(255, Math.round(255 * (1 - Math.abs(t - 0.5) * 2))));
  const b = Math.max(0, Math.min(255, Math.round(255 * (1 - t * 1.5))));
  return `rgb(${r},${g},${b})`;
}

async function renderCoverage() {
  populateAntennaSelect();
  await ensureBin();
  const canvas = document.getElementById("coverage-canvas");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const cfg = state.meta.config;
  const T = makeTransform(canvas, cfg.room.width, cfg.room.height);
  const a = parseInt(document.getElementById("coverage-antenna").value || "0", 10);

  const peaks = new Float64Array(state.meta.samples.length);
  let maxPeak = 0;
  for (let i = 0; i < state.meta.samples.length; i++) {
    const imp = impulseFor(i, a);
    let p = 0;
    for (let k = 0; k < imp.length; k++) { const v = Math.abs(imp[k]); if (v > p) p = v; }
    peaks[i] = p;
    if (p > maxPeak) maxPeak = p;
  }

  const cell = Math.max(6, (cfg.tag_grid_spacing || 0.25) * T.s);
  for (let i = 0; i < state.meta.samples.length; i++) {
    const s = state.meta.samples[i];
    const t = maxPeak > 0 ? peaks[i] / maxPeak : 0;
    ctx.fillStyle = valueToColor(t);
    ctx.fillRect(T.x(s.tag_x) - cell / 2, T.y(s.tag_y) - cell / 2, cell, cell);
  }
  for (const ant of cfg.antennas || []) {
    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.arc(T.x(ant.x), T.y(ant.y), 5, 0, 2 * Math.PI);
    ctx.fill();
  }
  document.getElementById("coverage-legend").textContent =
    `peak |Ez| for ${state.meta.antennas[a]} — blue=low, red=high (max ${maxPeak.toExponential(2)})`;
}

// ---- Impulse Response view ----
const ANT_COLORS = ["#1f6feb", "#3fb950", "#f0c000", "#f85149", "#a371f7", "#39c5cf"];

async function renderImpulse() {
  await ensureBin();
  const canvas = document.getElementById("impulse-canvas");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const info = document.getElementById("impulse-info");
  if (state.selected.length === 0) {
    info.textContent = "Select tag(s) in the Room Layout view, then return here.";
    return;
  }
  const N = state.meta.impulse_length;
  const M = state.meta.antennas.length;

  let amp = 1e-12;
  for (const s of state.selected) for (let a = 0; a < M; a++) {
    const imp = impulseFor(s, a);
    for (let k = 0; k < N; k++) { const v = Math.abs(imp[k]); if (v > amp) amp = v; }
  }

  const pad = 30;
  const w = canvas.width - 2 * pad;
  const h = canvas.height - 2 * pad;
  const midY = pad + h / 2;
  ctx.strokeStyle = "#30363d";
  ctx.beginPath(); ctx.moveTo(pad, midY); ctx.lineTo(pad + w, midY); ctx.stroke();

  state.selected.forEach((s, si) => {
    for (let a = 0; a < M; a++) {
      ctx.strokeStyle = ANT_COLORS[a % ANT_COLORS.length];
      ctx.setLineDash(si === 0 ? [] : [4, 3]);
      ctx.beginPath();
      const imp = impulseFor(s, a);
      for (let k = 0; k < N; k++) {
        const x = pad + (N > 1 ? k / (N - 1) : 0) * w;
        const y = midY - (imp[k] / amp) * (h / 2) * 0.95;
        if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
  });
  ctx.setLineDash([]);
  info.innerHTML = "Antennas: " +
    state.meta.antennas.map((l, a) => `<span style="color:${ANT_COLORS[a % ANT_COLORS.length]}">${l}</span>`).join(", ") +
    `<br/>Tags: ${state.selected.length} (solid = first; dashed = others). Overlay shows superposition geometry.`;
}

// ---- Wave Animation view ----
const wave = { frames: [], nx: 0, ny: 0, playing: false, idx: 0, raf: 0, maxAbs: 1 };

async function loadSnapshots() {
  if (state.snapshots) return state.snapshots;
  const r = await fetch(`/data/${BASE}_snapshots/snapshots.json`);
  if (!r.ok) return null;
  const manifest = await r.json();
  const frames = [];
  let maxAbs = 1e-12;
  for (const f of manifest.files) {
    const fr = await fetch(`/data/${BASE}_snapshots/${f}`);
    if (!fr.ok) continue;
    const arr = new Float32Array(await fr.arrayBuffer());
    for (let k = 0; k < arr.length; k++) { const v = Math.abs(arr[k]); if (v > maxAbs) maxAbs = v; }
    frames.push(arr);
  }
  wave.frames = frames; wave.nx = manifest.nx; wave.ny = manifest.ny; wave.maxAbs = maxAbs;
  state.snapshots = manifest;
  return manifest;
}

function drawWaveFrame(idx) {
  const canvas = document.getElementById("wave-canvas");
  const ctx = canvas.getContext("2d");
  if (!wave.frames.length) return;
  const arr = wave.frames[idx];
  const { nx, ny, maxAbs } = wave;
  const img = ctx.createImageData(nx, ny);
  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < ny; j++) {
      const v = arr[i * ny + j] / maxAbs; // -1..1 (diverging)
      const t = (v + 1) / 2;
      const r = Math.max(0, Math.min(255, Math.round(255 * (t - 0.5) * 2)));
      const b = Math.max(0, Math.min(255, Math.round(255 * (0.5 - t) * 2)));
      const p = (j * nx + i) * 4; // image is ny rows of nx
      img.data[p] = r; img.data[p + 1] = 0; img.data[p + 2] = b; img.data[p + 3] = 255;
    }
  }
  const off = document.createElement("canvas");
  off.width = nx; off.height = ny;
  off.getContext("2d").putImageData(img, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.drawImage(off, 0, 0, canvas.width, canvas.height);
  document.getElementById("wave-frame-label").textContent =
    `frame ${idx + 1}/${wave.frames.length}` + (state.snapshots ? ` (step ${state.snapshots.steps[idx]})` : "");
  document.getElementById("wave-scrub").value = String(idx);
}

function waveTick() {
  if (!wave.playing) return;
  const speed = parseInt(document.getElementById("wave-speed").value, 10);
  wave.idx = (wave.idx + 1) % wave.frames.length;
  drawWaveFrame(wave.idx);
  wave.raf = setTimeout(() => requestAnimationFrame(waveTick), 1000 / speed);
}

function wireWaveControls() {
  const playBtn = document.getElementById("wave-play");
  if (playBtn.dataset.wired) return;
  playBtn.dataset.wired = "1";
  playBtn.addEventListener("click", () => {
    wave.playing = !wave.playing;
    playBtn.textContent = wave.playing ? "⏸ Pause" : "▶ Play";
    if (wave.playing) waveTick();
  });
  document.getElementById("wave-scrub").addEventListener("input", (e) => {
    if (wave.playing) {
      wave.playing = false;
      clearTimeout(wave.raf);
      document.getElementById("wave-play").textContent = "▶ Play";
    }
    wave.idx = parseInt(e.target.value, 10);
    drawWaveFrame(wave.idx);
  });
}

async function renderWave() {
  wireWaveControls();
  const unavailable = document.getElementById("wave-unavailable");
  const controls = document.getElementById("wave-controls");
  const m = await loadSnapshots();
  if (!m || wave.frames.length === 0) {
    unavailable.classList.remove("hidden");
    controls.classList.add("hidden");
    return;
  }
  unavailable.classList.add("hidden");
  controls.classList.remove("hidden");
  document.getElementById("wave-scrub").max = String(wave.frames.length - 1);
  drawWaveFrame(wave.idx);
}

// ---- view dispatch (extended in later tasks) ----
const VIEWS = {
  room: renderRoom,
  wave: renderWave,
  coverage: renderCoverage,
  impulse: renderImpulse,
};

function showView(name) {
  currentView = name;
  // stop wave playback when switching views
  if (typeof wave !== "undefined" && wave.playing) {
    wave.playing = false;
    clearTimeout(wave.raf);
    const pb = document.getElementById("wave-play");
    if (pb) pb.textContent = "▶ Play";
  }
  for (const sec of document.querySelectorAll(".view")) sec.classList.remove("active");
  for (const b of document.querySelectorAll("#tabs button")) b.classList.toggle("active", b.dataset.view === name);
  document.getElementById(`view-${name}`).classList.add("active");
  const fn = VIEWS[name];
  if (fn) fn();
}

function wireTabs() {
  for (const b of document.querySelectorAll("#tabs button")) {
    b.addEventListener("click", () => showView(b.dataset.view));
  }
}

async function loadScenes() {
  try {
    const r = await fetch("/data/scenes.json");
    if (!r.ok) return null;
    return await r.json();
  } catch {
    return null;
  }
}

function populateDatasetSelect(manifest) {
  const sel = document.getElementById("dataset-select");
  sel.innerHTML = "";
  for (const s of manifest.scenes) {
    const opt = document.createElement("option");
    opt.value = s.name;
    opt.textContent = s.label || s.name;
    if (s.description) opt.title = s.description;
    sel.appendChild(opt);
  }
  sel.value = BASE;
  if (sel.value !== BASE) {
    const opt = document.createElement("option");
    opt.value = BASE;
    opt.textContent = BASE + " (custom)";
    sel.insertBefore(opt, sel.firstChild);
    sel.value = BASE;
  }
  sel.classList.remove("hidden");
  if (!sel.dataset.wired) {
    sel.addEventListener("change", () => switchDataset(sel.value));
    sel.dataset.wired = "1";
  }
}

async function switchDataset(name) {
  BASE = name;
  const u = new URL(location);
  u.searchParams.set("data", name);
  history.replaceState(null, "", u);
  state.meta = null;
  state.bin = null;
  state.snapshots = null;
  state.selected = [];
  // Stop any wave playback and drop the previous scene's frames so they can't
  // render under the new scene or index out of bounds.
  wave.playing = false;
  clearTimeout(wave.raf);
  wave.frames = [];
  wave.idx = 0;
  const playBtn = document.getElementById("wave-play");
  if (playBtn) playBtn.textContent = "▶ Play";
  const covSel = document.getElementById("coverage-antenna");
  if (covSel) covSel.innerHTML = "";
  try {
    await loadMeta();
    setStatus(`loaded ${BASE}.json — ${state.meta.samples.length} tags, ${state.meta.antennas.length} antennas`);
    showView(currentView);
  } catch (e) {
    setStatus("error: " + e.message);
    console.error(e);
  }
}

async function main() {
  wireTabs();
  const manifest = await loadScenes();
  if (manifest && manifest.scenes && manifest.scenes.length) {
    if (!params.get("data")) BASE = manifest.default || manifest.scenes[0].name;
    populateDatasetSelect(manifest);
  }
  try {
    await loadMeta();
    setStatus(`loaded ${BASE}.json — ${state.meta.samples.length} tags, ${state.meta.antennas.length} antennas`);
    showView("room");
  } catch (e) {
    setStatus("error: " + e.message);
    console.error(e);
  }
}

main();
