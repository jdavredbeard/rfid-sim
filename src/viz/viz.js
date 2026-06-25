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

// ---- view dispatch (extended in later tasks) ----
const VIEWS = {
  room: renderRoom,
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
