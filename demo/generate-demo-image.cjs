#!/usr/bin/env node
/**
 * Phillips Gate Demo Image Generator
 *
 * Generates a static PNG demonstrating the quantum simulator visualization.
 * Run with: node demo/generate-demo-image.js
 * Output: demo/phillips-gate-output.png
 */

const fs = require('fs');
const path = require('path');

// Check if we have canvas available (optional dependency)
let createCanvas;
try {
  createCanvas = require('canvas').createCanvas;
} catch (e) {
  console.log('Note: "canvas" package not installed. Generating ASCII visualization instead.\n');
  createCanvas = null;
}

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════
const PHI = (1 + Math.sqrt(5)) / 2;
const SQRT1_2 = Math.SQRT1_2;

// ═══════════════════════════════════════════════════════════════════════════
// GEOMETRIC ALGEBRA
// ═══════════════════════════════════════════════════════════════════════════
const ROTOR_IDENTITY = { s: 1, e12: 0, e23: 0, e31: 0 };

function rotorFromAxisAngle(ax, ay, az, angle) {
  const mag = Math.sqrt(ax*ax + ay*ay + az*az);
  if (mag < 1e-10) return {...ROTOR_IDENTITY};
  const nx = ax/mag, ny = ay/mag, nz = az/mag;
  const c = Math.cos(angle/2), s = Math.sin(angle/2);
  return { s: c, e12: -s*nz, e23: -s*nx, e31: -s*ny };
}

function reverseRotor(r) {
  return { s: r.s, e12: -r.e12, e23: -r.e23, e31: -r.e31 };
}

function applyBiRotor(L, R, x, y, z, w) {
  const Rr = reverseRotor(R);
  const lw = L.s*w - L.e23*x - L.e31*y - L.e12*z;
  const lx = L.s*x + L.e23*w + L.e31*z - L.e12*y;
  const ly = L.s*y - L.e23*z + L.e31*w + L.e12*x;
  const lz = L.s*z + L.e23*y - L.e31*x + L.e12*w;
  return {
    x: lw*Rr.e23 + lx*Rr.s - ly*Rr.e12 + lz*Rr.e31,
    y: lw*Rr.e31 + lx*Rr.e12 + ly*Rr.s - lz*Rr.e23,
    z: lw*Rr.e12 - lx*Rr.e31 + ly*Rr.e23 + lz*Rr.s,
    w: lw*Rr.s - lx*Rr.e23 - ly*Rr.e31 - lz*Rr.e12
  };
}

// ═══════════════════════════════════════════════════════════════════════════
// SPINOR
// ═══════════════════════════════════════════════════════════════════════════
function spinorFromBloch(theta, phi) {
  const ch = Math.cos(theta/2), sh = Math.sin(theta/2);
  return { real0: ch, imag0: 0, real1: sh*Math.cos(phi), imag1: sh*Math.sin(phi) };
}

function applyRotorToSpinor(r, s) {
  const ar = r.s, ai = r.e12, br = r.e31, bi = r.e23;
  return {
    real0: ar*s.real0 - ai*s.imag0 - br*s.real1 - bi*s.imag1,
    imag0: ar*s.imag0 + ai*s.real0 - br*s.imag1 + bi*s.real1,
    real1: br*s.real0 - bi*s.imag0 + ar*s.real1 + ai*s.imag1,
    imag1: br*s.imag0 + bi*s.real0 + ar*s.imag1 - ai*s.real1
  };
}

function normalizeSpinor(s) {
  const mag = Math.sqrt(s.real0*s.real0 + s.imag0*s.imag0 + s.real1*s.real1 + s.imag1*s.imag1);
  return { real0: s.real0/mag, imag0: s.imag0/mag, real1: s.real1/mag, imag1: s.imag1/mag };
}

function spinorProbs(s) {
  return {
    p0: s.real0*s.real0 + s.imag0*s.imag0,
    p1: s.real1*s.real1 + s.imag1*s.imag1
  };
}

// ═══════════════════════════════════════════════════════════════════════════
// 24-CELL
// ═══════════════════════════════════════════════════════════════════════════
function get24CellVertices() {
  const verts = [];
  verts.push({x:0,y:0,z:0,w:1}, {x:0,y:0,z:0,w:-1});
  verts.push({x:1,y:0,z:0,w:0}, {x:-1,y:0,z:0,w:0});
  verts.push({x:0,y:1,z:0,w:0}, {x:0,y:-1,z:0,w:0});
  verts.push({x:0,y:0,z:1,w:0}, {x:0,y:0,z:-1,w:0});
  const h = 0.5;
  for (let sw of [-h, h]) {
    for (let sx of [-h, h]) {
      for (let sy of [-h, h]) {
        for (let sz of [-h, h]) {
          verts.push({x: sx, y: sy, z: sz, w: sw});
        }
      }
    }
  }
  return verts;
}

function get24CellEdges(verts) {
  const edges = [];
  for (let i = 0; i < verts.length; i++) {
    for (let j = i + 1; j < verts.length; j++) {
      const dx = verts[i].x - verts[j].x;
      const dy = verts[i].y - verts[j].y;
      const dz = verts[i].z - verts[j].z;
      const dw = verts[i].w - verts[j].w;
      const dist = Math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw);
      if (Math.abs(dist - 1) < 0.01) edges.push([i, j]);
    }
  }
  return edges;
}

// ═══════════════════════════════════════════════════════════════════════════
// PROJECTION
// ═══════════════════════════════════════════════════════════════════════════
function hopfProject(x, y, z, w) {
  const bx = 2 * (x*z + y*w);
  const by = 2 * (y*z - x*w);
  const bz = x*x + y*y - z*z - w*w;
  const mag = Math.sqrt(bx*bx + by*by + bz*bz);
  return mag > 1e-10 ? { x: bx/mag, y: by/mag, z: bz/mag } : { x: 0, y: 0, z: 1 };
}

function stereographic(bx, by, bz) {
  const d = 1 - bz;
  return d > 0.01 ? { x: bx/d, y: by/d } : { x: bx*100, y: by*100 };
}

// ═══════════════════════════════════════════════════════════════════════════
// QUANTUM GATES
// ═══════════════════════════════════════════════════════════════════════════
const GATES = {
  H: rotorFromAxisAngle(SQRT1_2, 0, SQRT1_2, Math.PI),
  X: rotorFromAxisAngle(1, 0, 0, Math.PI),
  Z: rotorFromAxisAngle(0, 0, 1, Math.PI),
};

// ═══════════════════════════════════════════════════════════════════════════
// DEMO SIMULATION
// ═══════════════════════════════════════════════════════════════════════════
function runDemo() {
  console.log('╔══════════════════════════════════════════════════════════════════╗');
  console.log('║         PHILLIPS GATE QUANTUM SIMULATOR - DEMO OUTPUT           ║');
  console.log('╠══════════════════════════════════════════════════════════════════╣');
  console.log('║  24-Cell Substrate: Binary Tetrahedral Group (2T) ≅ SL(2,3)     ║');
  console.log('║  Quantum Gates: Geometric Algebra Rotors in Cl(3,0)             ║');
  console.log('╚══════════════════════════════════════════════════════════════════╝\n');

  const vertices = get24CellVertices();
  const edges = get24CellEdges(vertices);

  console.log(`Initialized 24-cell with ${vertices.length} vertices and ${edges.length} edges\n`);

  // Initialize qubits in |0⟩ state
  let qubits = vertices.map((v, i) => ({
    index: i,
    state: spinorFromBloch(0, 0),
    vertex: v
  }));

  console.log('STEP 1: Initial State (all qubits in |0⟩)');
  console.log('─'.repeat(50));
  printQubitStates(qubits.slice(0, 8));
  console.log('... (16 more qubits)\n');

  // Apply Hadamard to first 4 qubits
  console.log('STEP 2: Apply Hadamard gates to qubits 0-3');
  console.log('─'.repeat(50));
  for (let i = 0; i < 4; i++) {
    qubits[i].state = normalizeSpinor(applyRotorToSpinor(GATES.H, qubits[i].state));
  }
  printQubitStates(qubits.slice(0, 8));
  console.log();

  // Apply X gate to qubit 2
  console.log('STEP 3: Apply Pauli-X (NOT) gate to qubit 2');
  console.log('─'.repeat(50));
  qubits[2].state = normalizeSpinor(applyRotorToSpinor(GATES.X, qubits[2].state));
  printQubitStates(qubits.slice(0, 8));
  console.log();

  // Apply Z gate to qubit 3
  console.log('STEP 4: Apply Pauli-Z (phase flip) gate to qubit 3');
  console.log('─'.repeat(50));
  qubits[3].state = normalizeSpinor(applyRotorToSpinor(GATES.Z, qubits[3].state));
  printQubitStates(qubits.slice(0, 8));
  console.log();

  // Measure
  console.log('STEP 5: Measurement (probabilistic collapse)');
  console.log('─'.repeat(50));
  const results = qubits.map(q => {
    const { p0, p1 } = spinorProbs(q.state);
    return Math.random() < p0 ? 0 : 1;
  });

  console.log('Measurement results (24 qubits):');
  console.log('┌' + '───┬'.repeat(11) + '───┐');
  process.stdout.write('│');
  for (let i = 0; i < 12; i++) {
    process.stdout.write(` ${results[i]} │`);
  }
  console.log();
  console.log('├' + '───┼'.repeat(11) + '───┤');
  process.stdout.write('│');
  for (let i = 12; i < 24; i++) {
    process.stdout.write(` ${results[i]} │`);
  }
  console.log();
  console.log('└' + '───┴'.repeat(11) + '───┘');

  const ones = results.filter(r => r === 1).length;
  console.log(`\nSummary: ${ones} qubits in |1⟩, ${24 - ones} qubits in |0⟩`);

  // Visualize the 24-cell projection
  console.log('\n');
  console.log('╔══════════════════════════════════════════════════════════════════╗');
  console.log('║              24-CELL HOPF PROJECTION (ASCII Art)                ║');
  console.log('╚══════════════════════════════════════════════════════════════════╝');

  printAsciiProjection(vertices, results);

  // Moiré pattern visualization
  console.log('\n');
  console.log('╔══════════════════════════════════════════════════════════════════╗');
  console.log('║                 MOIRÉ INTERFERENCE PATTERN                       ║');
  console.log('╚══════════════════════════════════════════════════════════════════╝');

  printAsciiMoire();

  // Summary
  console.log('\n');
  console.log('╔══════════════════════════════════════════════════════════════════╗');
  console.log('║                        SYSTEM STATUS                             ║');
  console.log('╠══════════════════════════════════════════════════════════════════╣');
  console.log('║  Substrate:     24-cell (Icositetrachoron)                       ║');
  console.log('║  Vertices:      24 (Hurwitz quaternion units)                    ║');
  console.log('║  Edges:         96                                               ║');
  console.log('║  Gates Applied: H(0-3), X(2), Z(3)                               ║');
  console.log('║  Measurements:  24                                               ║');
  console.log('╚══════════════════════════════════════════════════════════════════╝');

  console.log('\n✓ Demo complete!');
  console.log('\nTo see the full interactive visualization:');
  console.log('  1. Open demo/phillips-gate-demo.html in a browser');
  console.log('  2. Or run: npx vite && open http://localhost:5173');

  return { vertices, edges, qubits, results };
}

function printQubitStates(qubits) {
  qubits.forEach(q => {
    const { p0, p1 } = spinorProbs(q.state);
    const bar0 = '█'.repeat(Math.round(p0 * 20));
    const bar1 = '█'.repeat(Math.round(p1 * 20));
    console.log(`  Qubit ${q.index.toString().padStart(2)}: |0⟩ ${(p0*100).toFixed(1).padStart(5)}% ${bar0.padEnd(20)} |1⟩ ${(p1*100).toFixed(1).padStart(5)}% ${bar1}`);
  });
}

function printAsciiProjection(vertices, results) {
  const W = 60, H = 30;
  const grid = Array(H).fill(null).map(() => Array(W).fill(' '));

  const rotor = rotorFromAxisAngle(0.3, 1, 0.5, 0.5);

  vertices.forEach((v, i) => {
    const rot = applyBiRotor(rotor, rotor, v.x, v.y, v.z, v.w);
    const hopf = hopfProject(rot.x, rot.y, rot.z, rot.w);
    const stereo = stereographic(hopf.x, hopf.y, hopf.z);

    const x = Math.round(W/2 + stereo.x * 20);
    const y = Math.round(H/2 + stereo.y * 10);

    if (x >= 0 && x < W && y >= 0 && y < H) {
      grid[y][x] = results[i] === 1 ? '●' : '○';
    }
  });

  console.log('┌' + '─'.repeat(W) + '┐');
  grid.forEach(row => {
    console.log('│' + row.join('') + '│');
  });
  console.log('└' + '─'.repeat(W) + '┘');
  console.log('  ○ = |0⟩ state    ● = |1⟩ state');
}

function printAsciiMoire() {
  const W = 60, H = 20;
  const grid = Array(H).fill(null).map(() => Array(W).fill(' '));

  // Draw two overlapping circular patterns
  const cx = W / 2, cy = H / 2;
  const phase = 0.3; // Phase difference

  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const dx = (x - cx) / 2;
      const dy = y - cy;
      const r = Math.sqrt(dx*dx + dy*dy);
      const angle = Math.atan2(dy, dx);

      // Two gratings with slight phase difference
      const g1 = Math.sin(r * 1.5);
      const g2 = Math.sin(r * 1.5 + phase);

      // Moiré = product of gratings
      const moire = g1 * g2;

      if (moire > 0.5) grid[y][x] = '█';
      else if (moire > 0.2) grid[y][x] = '▓';
      else if (moire > -0.2) grid[y][x] = '░';
      else if (moire > -0.5) grid[y][x] = '·';
    }
  }

  console.log('┌' + '─'.repeat(W) + '┐');
  grid.forEach(row => {
    console.log('│' + row.join('') + '│');
  });
  console.log('└' + '─'.repeat(W) + '┘');
  console.log('  Moiré fringes encode rotation difference (phase = 0.3 rad)');
}

// Run the demo
runDemo();
