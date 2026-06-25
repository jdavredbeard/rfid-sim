"use strict";

// ---- shared state ----
const params = new URLSearchParams(location.search);
const BASE = params.get("data") || "sim-output";
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
        const x = pad + (k / (N - 1)) * w;
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

// ---- view dispatch (extended in later tasks) ----
const VIEWS = {
  room: renderRoom,
  // wave: renderWave,  // added in Task 6
  coverage: renderCoverage,
  impulse: renderImpulse,
};

function showView(name) {
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

async function main() {
  wireTabs();
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
