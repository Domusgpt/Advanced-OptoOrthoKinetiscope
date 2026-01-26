/**
 * PHILLIPS GATE SIMULATOR - Quantum Computing on Geometric Substrate
 *
 * This module implements a visual quantum computing simulator using:
 * - The 24-cell (Icositetrachoron) as the qubit register substrate
 * - Geometric Algebra rotors as quantum gate operations
 * - The 600-cell surface for cellular automaton evolution
 * - Moiré visualization for quantum state readout
 *
 * Core Concept: "Phillips Gate"
 * The 24 vertices of the 24-cell correspond to the Binary Tetrahedral Group (2T),
 * which contains the Pauli group. A 4D double isoclinic rotation acts as a
 * complex quantum gate operation on the data stored at the vertices.
 *
 * Mathematical Foundations:
 * - Unit quaternions ≅ SU(2) ≅ Spin(3) - the double cover of SO(3)
 * - Binary Tetrahedral Group 2T ≅ SL(2,3) - 24 elements
 * - Hurwitz quaternion units = vertices of 24-cell
 * - Pauli matrices σx, σy, σz generate SU(2)
 *
 * Simulation Modes:
 * 1. Single Qubit: One vertex as Bloch sphere
 * 2. Multi-Qubit: Multiple vertices with entanglement links
 * 3. Cellular Automaton: Glider on 600-cell surface
 * 4. Annealing: Energy minimization for optimization
 */

import { Vector4 } from "./cpeMath";
import {
  Rotor3D, Spinor, BiRotor4D,
  rotorFromAxisAngle, rotorFromBivectorAngle,
  multiplyRotors, reverseRotor, normalizeRotor,
  applyRotorToSpinor, spinorFromBlochSphere, spinorToBlochSphere,
  spinorProbabilities, normalizeSpinor, slerpRotor,
  createBiRotor, applyBiRotor, cliffordRotation,
  ROTOR_IDENTITY, PAULI_X, PAULI_Y, PAULI_Z
} from "./GeometricAlgebra";
import {
  get24CellAsQuaternions, get24CellGroupStructure,
  get600CellVertices, get600CellEdges,
  hopfProjection, PHI
} from "./E8Projection";

// ═══════════════════════════════════════════════════════════════════════════
// TYPE DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * A qubit stored at a 24-cell vertex
 */
export interface VertexQubit {
  vertexIndex: number;      // Index in 24-cell (0-23)
  state: Spinor;            // Quantum state |ψ⟩ = α|0⟩ + β|1⟩
  blochCoords: {            // Bloch sphere representation
    theta: number;
    phi: number;
  };
  groupElement: string;     // Relation to Pauli group
  entangledWith: number[];  // Indices of entangled qubits
}

/**
 * A quantum gate as a geometric algebra rotor
 */
export interface QuantumGate {
  name: string;
  rotor: Rotor3D;
  axis: { x: number, y: number, z: number };
  angle: number;
  matrix: [[number, number], [number, number]];  // 2x2 complex (stored as [re, im] pairs would be complex but simplified here)
}

/**
 * A quantum circuit as a sequence of gates
 */
export interface QuantumCircuit {
  name: string;
  gates: {
    gate: QuantumGate;
    targetQubit: number;
    controlQubit?: number;  // For controlled gates
    time: number;           // Relative time in circuit
  }[];
  measurementBasis: 'Z' | 'X' | 'Y';
}

/**
 * Cellular automaton cell on 600-cell surface
 */
export interface CACell {
  vertexIndex: number;
  state: number;            // 0 or 1 for simple CA
  neighbors: number[];      // Adjacent vertex indices
  energy: number;           // For annealing
}

/**
 * Complete Phillips Gate simulation state
 */
export interface PhillipsGateState {
  qubits: VertexQubit[];
  substrate: '24cell' | '600cell';
  globalRotor: BiRotor4D;   // Current 4D orientation
  time: number;             // Simulation time
  energy: number;           // Total system energy
  temperature: number;      // For annealing (Boltzmann factor)
  history: {
    time: number;
    measurement: { index: number, result: 0 | 1 }[];
  }[];
}

// ═══════════════════════════════════════════════════════════════════════════
// STANDARD QUANTUM GATES (As GA Rotors)
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Identity gate (no operation)
 */
export const GATE_I: QuantumGate = {
  name: 'I',
  rotor: ROTOR_IDENTITY,
  axis: { x: 0, y: 0, z: 1 },
  angle: 0,
  matrix: [[1, 0], [0, 1]]
};

/**
 * Pauli-X gate (bit flip, rotation by π about X-axis)
 * |0⟩ → |1⟩, |1⟩ → |0⟩
 */
export const GATE_X: QuantumGate = {
  name: 'X',
  rotor: rotorFromAxisAngle(1, 0, 0, Math.PI),
  axis: { x: 1, y: 0, z: 0 },
  angle: Math.PI,
  matrix: [[0, 1], [1, 0]]
};

/**
 * Pauli-Y gate (rotation by π about Y-axis)
 * |0⟩ → i|1⟩, |1⟩ → -i|0⟩
 */
export const GATE_Y: QuantumGate = {
  name: 'Y',
  rotor: rotorFromAxisAngle(0, 1, 0, Math.PI),
  axis: { x: 0, y: 1, z: 0 },
  angle: Math.PI,
  matrix: [[0, -1], [1, 0]]  // Simplified, should be [[0, -i], [i, 0]]
};

/**
 * Pauli-Z gate (phase flip, rotation by π about Z-axis)
 * |0⟩ → |0⟩, |1⟩ → -|1⟩
 */
export const GATE_Z: QuantumGate = {
  name: 'Z',
  rotor: rotorFromAxisAngle(0, 0, 1, Math.PI),
  axis: { x: 0, y: 0, z: 1 },
  angle: Math.PI,
  matrix: [[1, 0], [0, -1]]
};

/**
 * Hadamard gate (creates superposition)
 * |0⟩ → (|0⟩ + |1⟩)/√2, |1⟩ → (|0⟩ - |1⟩)/√2
 * Rotation by π about the (X+Z)/√2 axis
 */
export const GATE_H: QuantumGate = {
  name: 'H',
  rotor: rotorFromAxisAngle(Math.SQRT1_2, 0, Math.SQRT1_2, Math.PI),
  axis: { x: Math.SQRT1_2, y: 0, z: Math.SQRT1_2 },
  angle: Math.PI,
  matrix: [[Math.SQRT1_2, Math.SQRT1_2], [Math.SQRT1_2, -Math.SQRT1_2]]
};

/**
 * S gate (phase gate, π/2 rotation about Z)
 * |0⟩ → |0⟩, |1⟩ → i|1⟩
 */
export const GATE_S: QuantumGate = {
  name: 'S',
  rotor: rotorFromAxisAngle(0, 0, 1, Math.PI / 2),
  axis: { x: 0, y: 0, z: 1 },
  angle: Math.PI / 2,
  matrix: [[1, 0], [0, 1]]  // Simplified, should have i in [1,1]
};

/**
 * T gate (π/8 gate, π/4 rotation about Z)
 * The "magic" gate for universal quantum computation
 */
export const GATE_T: QuantumGate = {
  name: 'T',
  rotor: rotorFromAxisAngle(0, 0, 1, Math.PI / 4),
  axis: { x: 0, y: 0, z: 1 },
  angle: Math.PI / 4,
  matrix: [[1, 0], [0, 1]]  // Simplified
};

/**
 * Rotation gates (parameterized)
 */
export function createRxGate(theta: number): QuantumGate {
  return {
    name: `Rx(${(theta * 180 / Math.PI).toFixed(1)}°)`,
    rotor: rotorFromAxisAngle(1, 0, 0, theta),
    axis: { x: 1, y: 0, z: 0 },
    angle: theta,
    matrix: [[Math.cos(theta/2), 0], [0, Math.cos(theta/2)]]
  };
}

export function createRyGate(theta: number): QuantumGate {
  return {
    name: `Ry(${(theta * 180 / Math.PI).toFixed(1)}°)`,
    rotor: rotorFromAxisAngle(0, 1, 0, theta),
    axis: { x: 0, y: 1, z: 0 },
    angle: theta,
    matrix: [[Math.cos(theta/2), -Math.sin(theta/2)], [Math.sin(theta/2), Math.cos(theta/2)]]
  };
}

export function createRzGate(theta: number): QuantumGate {
  return {
    name: `Rz(${(theta * 180 / Math.PI).toFixed(1)}°)`,
    rotor: rotorFromAxisAngle(0, 0, 1, theta),
    axis: { x: 0, y: 0, z: 1 },
    angle: theta,
    matrix: [[1, 0], [0, 1]]
  };
}

// ═══════════════════════════════════════════════════════════════════════════
// PHILLIPS GATE SIMULATOR CORE
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Initialize the Phillips Gate simulator with qubits on 24-cell vertices
 */
export function initializePhillipsGate(
  numQubits: number = 24,
  substrate: '24cell' | '600cell' = '24cell'
): PhillipsGateState {
  const vertices = substrate === '24cell'
    ? get24CellAsQuaternions()
    : get600CellVertices().slice(0, numQubits);

  const groupStructure = substrate === '24cell'
    ? get24CellGroupStructure()
    : null;

  const qubits: VertexQubit[] = vertices.slice(0, numQubits).map((v, i) => ({
    vertexIndex: i,
    state: spinorFromBlochSphere(0, 0),  // Initialize to |0⟩
    blochCoords: { theta: 0, phi: 0 },
    groupElement: groupStructure?.[i]?.pauliRelation || `vertex_${i}`,
    entangledWith: []
  }));

  return {
    qubits,
    substrate,
    globalRotor: createBiRotor(ROTOR_IDENTITY, ROTOR_IDENTITY),
    time: 0,
    energy: 0,
    temperature: 1.0,
    history: []
  };
}

/**
 * Apply a quantum gate to a specific qubit
 */
export function applyGate(
  state: PhillipsGateState,
  gateIndex: number,
  qubitIndex: number
): PhillipsGateState {
  const gates = [GATE_I, GATE_X, GATE_Y, GATE_Z, GATE_H, GATE_S, GATE_T];
  const gate = gates[gateIndex % gates.length];

  const newQubits = [...state.qubits];
  const qubit = { ...newQubits[qubitIndex] };

  // Apply gate rotor to spinor
  qubit.state = normalizeSpinor(applyRotorToSpinor(gate.rotor, qubit.state));
  qubit.blochCoords = spinorToBlochSphere(qubit.state);

  newQubits[qubitIndex] = qubit;

  return {
    ...state,
    qubits: newQubits,
    time: state.time + 1
  };
}

/**
 * Apply a custom rotor gate to a qubit
 */
export function applyRotorGate(
  state: PhillipsGateState,
  rotor: Rotor3D,
  qubitIndex: number
): PhillipsGateState {
  const newQubits = [...state.qubits];
  const qubit = { ...newQubits[qubitIndex] };

  qubit.state = normalizeSpinor(applyRotorToSpinor(rotor, qubit.state));
  qubit.blochCoords = spinorToBlochSphere(qubit.state);

  newQubits[qubitIndex] = qubit;

  return {
    ...state,
    qubits: newQubits,
    time: state.time + 1
  };
}

/**
 * Apply a global 4D rotation to the entire substrate
 * This is the "Phillips Gate" operation - a coherent rotation of all qubits
 */
export function applyPhillipsGate(
  state: PhillipsGateState,
  leftRotor: Rotor3D,
  rightRotor: Rotor3D
): PhillipsGateState {
  const newBiRotor = createBiRotor(
    multiplyRotors(leftRotor, state.globalRotor.left),
    multiplyRotors(state.globalRotor.right, rightRotor)
  );

  // The global rotation affects all qubits through the 4D geometry
  // Each qubit picks up a phase based on its vertex position
  const newQubits = state.qubits.map((qubit, i) => {
    const vertices = state.substrate === '24cell'
      ? get24CellAsQuaternions()
      : get600CellVertices();

    const v = vertices[i];
    const rotated = applyBiRotor(newBiRotor, v.x, v.y, v.z, v.w);

    // The phase accumulated is related to the rotation of the vertex
    const phaseShift = Math.atan2(rotated.y, rotated.x) - Math.atan2(v.y, v.x);

    // Apply phase to the qubit
    const phaseRotor = rotorFromAxisAngle(0, 0, 1, phaseShift);
    const newState = normalizeSpinor(applyRotorToSpinor(phaseRotor, qubit.state));

    return {
      ...qubit,
      state: newState,
      blochCoords: spinorToBlochSphere(newState)
    };
  });

  return {
    ...state,
    qubits: newQubits,
    globalRotor: newBiRotor,
    time: state.time + 1
  };
}

/**
 * Measure a qubit, collapsing it to |0⟩ or |1⟩
 */
export function measureQubit(
  state: PhillipsGateState,
  qubitIndex: number
): { state: PhillipsGateState, result: 0 | 1 } {
  const qubit = state.qubits[qubitIndex];
  const { p0, p1 } = spinorProbabilities(qubit.state);

  // Probabilistic measurement
  const result: 0 | 1 = Math.random() < p0 ? 0 : 1;

  // Collapse the state
  const newQubits = [...state.qubits];
  newQubits[qubitIndex] = {
    ...qubit,
    state: result === 0
      ? { real0: 1, imag0: 0, real1: 0, imag1: 0 }
      : { real0: 0, imag0: 0, real1: 1, imag1: 0 },
    blochCoords: result === 0
      ? { theta: 0, phi: 0 }
      : { theta: Math.PI, phi: 0 }
  };

  // Record in history
  const newHistory = [...state.history, {
    time: state.time,
    measurement: [{ index: qubitIndex, result }]
  }];

  return {
    state: {
      ...state,
      qubits: newQubits,
      history: newHistory
    },
    result
  };
}

/**
 * Create entanglement between two qubits
 * Uses a CNOT-equivalent operation on the 24-cell
 */
export function entangleQubits(
  state: PhillipsGateState,
  controlIndex: number,
  targetIndex: number
): PhillipsGateState {
  const newQubits = [...state.qubits];

  // Mark as entangled
  newQubits[controlIndex] = {
    ...newQubits[controlIndex],
    entangledWith: [...newQubits[controlIndex].entangledWith, targetIndex]
  };
  newQubits[targetIndex] = {
    ...newQubits[targetIndex],
    entangledWith: [...newQubits[targetIndex].entangledWith, controlIndex]
  };

  // For simulation, apply conditional operation
  const control = newQubits[controlIndex];
  const { p1 } = spinorProbabilities(control.state);

  // If control is likely |1⟩, apply X to target
  if (p1 > 0.5) {
    newQubits[targetIndex] = {
      ...newQubits[targetIndex],
      state: normalizeSpinor(applyRotorToSpinor(GATE_X.rotor, newQubits[targetIndex].state)),
      blochCoords: spinorToBlochSphere(
        normalizeSpinor(applyRotorToSpinor(GATE_X.rotor, newQubits[targetIndex].state))
      )
    };
  }

  return {
    ...state,
    qubits: newQubits,
    time: state.time + 1
  };
}

// ═══════════════════════════════════════════════════════════════════════════
// CELLULAR AUTOMATON ON 600-CELL
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Initialize a cellular automaton on the 600-cell surface
 */
export function initializeCellularAutomaton(): CACell[] {
  const vertices = get600CellVertices();
  const edges = get600CellEdges(vertices);

  // Build neighbor map
  const neighbors: number[][] = Array(vertices.length).fill(null).map(() => []);
  for (const [i, j] of edges) {
    neighbors[i].push(j);
    neighbors[j].push(i);
  }

  return vertices.map((v, i) => ({
    vertexIndex: i,
    state: Math.random() < 0.1 ? 1 : 0,  // Sparse initial state
    neighbors: neighbors[i],
    energy: 0
  }));
}

/**
 * Step the cellular automaton forward
 * Uses a rule inspired by the Game of Life but adapted for the 600-cell topology
 */
export function stepCellularAutomaton(cells: CACell[]): CACell[] {
  return cells.map(cell => {
    const liveNeighbors = cell.neighbors.reduce(
      (count, ni) => count + cells[ni].state,
      0
    );

    // Rule: Similar to B3/S23 (Game of Life) but adapted for higher connectivity
    // 600-cell vertices have 12 neighbors each
    let newState = cell.state;

    if (cell.state === 1) {
      // Survival: need 4-7 live neighbors
      if (liveNeighbors < 4 || liveNeighbors > 7) {
        newState = 0;
      }
    } else {
      // Birth: need exactly 5 or 6 live neighbors
      if (liveNeighbors === 5 || liveNeighbors === 6) {
        newState = 1;
      }
    }

    return { ...cell, state: newState };
  });
}

/**
 * Create a "glider" pattern on the 600-cell
 * Returns the indices of cells to activate
 */
export function createGlider(startVertex: number, cells: CACell[]): number[] {
  // Find a local patch of connected vertices
  const gliderPattern: number[] = [startVertex];

  // Add some neighbors to form a moving pattern
  const neighbors = cells[startVertex].neighbors;
  if (neighbors.length >= 3) {
    gliderPattern.push(neighbors[0], neighbors[1], neighbors[2]);
  }

  return gliderPattern;
}

// ═══════════════════════════════════════════════════════════════════════════
// SIMULATED ANNEALING FOR OPTIMIZATION
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Compute the "energy" of the current quantum state
 * Lower energy = better solution for optimization problems
 */
export function computeEnergy(
  state: PhillipsGateState,
  costFunction: (probabilities: number[]) => number
): number {
  const probabilities = state.qubits.map(q => {
    const { p0, p1 } = spinorProbabilities(q.state);
    return p1;  // Probability of |1⟩
  });

  return costFunction(probabilities);
}

/**
 * Perform one step of simulated quantum annealing
 */
export function annealingStep(
  state: PhillipsGateState,
  costFunction: (probabilities: number[]) => number,
  coolingRate: number = 0.99
): PhillipsGateState {
  const currentEnergy = computeEnergy(state, costFunction);

  // Try a random perturbation
  const randomQubit = Math.floor(Math.random() * state.qubits.length);
  const randomAngle = (Math.random() - 0.5) * Math.PI * 0.1;
  const randomAxis = Math.floor(Math.random() * 3);

  const rotors = [
    rotorFromAxisAngle(1, 0, 0, randomAngle),
    rotorFromAxisAngle(0, 1, 0, randomAngle),
    rotorFromAxisAngle(0, 0, 1, randomAngle)
  ];

  const trialState = applyRotorGate(state, rotors[randomAxis], randomQubit);
  const trialEnergy = computeEnergy(trialState, costFunction);

  // Metropolis criterion
  const deltaE = trialEnergy - currentEnergy;
  const acceptProbability = deltaE < 0 ? 1 : Math.exp(-deltaE / state.temperature);

  if (Math.random() < acceptProbability) {
    return {
      ...trialState,
      energy: trialEnergy,
      temperature: state.temperature * coolingRate
    };
  }

  return {
    ...state,
    temperature: state.temperature * coolingRate
  };
}

/**
 * Run full annealing process for optimization
 */
export function runAnnealing(
  numQubits: number,
  costFunction: (probabilities: number[]) => number,
  iterations: number = 1000,
  initialTemp: number = 10.0
): { finalState: PhillipsGateState, solution: number[], energy: number } {
  let state = initializePhillipsGate(numQubits, '24cell');
  state = { ...state, temperature: initialTemp };

  // Start with random superpositions
  for (let i = 0; i < numQubits; i++) {
    state = applyGate(state, 4, i);  // Hadamard
  }

  // Annealing loop
  for (let i = 0; i < iterations; i++) {
    state = annealingStep(state, costFunction);
  }

  // Extract solution
  const solution = state.qubits.map(q => {
    const { p1 } = spinorProbabilities(q.state);
    return p1 > 0.5 ? 1 : 0;
  });

  return {
    finalState: state,
    solution,
    energy: state.energy
  };
}

// ═══════════════════════════════════════════════════════════════════════════
// VISUALIZATION HELPERS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Get the current state as a visualization-ready format
 */
export function getVisualizationData(state: PhillipsGateState): {
  vertices: { x: number, y: number, z: number, w: number }[],
  qubitStates: { theta: number, phi: number, p0: number, p1: number }[],
  projected2D: { x: number, y: number }[],
  edges: [number, number][],
  entanglements: [number, number][]
} {
  const baseVertices = state.substrate === '24cell'
    ? get24CellAsQuaternions()
    : get600CellVertices();

  // Apply global rotation
  const vertices = baseVertices.map(v =>
    applyBiRotor(state.globalRotor, v.x, v.y, v.z, v.w)
  );

  // Project to 2D via Hopf fibration
  const projected2D = vertices.map(v => {
    const hopf = hopfProjection(v.x, v.y, v.z, v.w);
    // Stereographic projection of base point
    const denom = 1 - hopf.basePoint.z;
    return {
      x: denom > 0.01 ? hopf.basePoint.x / denom : hopf.basePoint.x * 100,
      y: denom > 0.01 ? hopf.basePoint.y / denom : hopf.basePoint.y * 100
    };
  });

  const qubitStates = state.qubits.map(q => {
    const { p0, p1 } = spinorProbabilities(q.state);
    return {
      theta: q.blochCoords.theta,
      phi: q.blochCoords.phi,
      p0,
      p1
    };
  });

  // Get edges
  const edges = state.substrate === '24cell'
    ? get24CellEdges()
    : get600CellEdges(baseVertices).slice(0, 500);  // Limit for performance

  // Get entanglements
  const entanglements: [number, number][] = [];
  state.qubits.forEach((q, i) => {
    q.entangledWith.forEach(j => {
      if (i < j) entanglements.push([i, j]);
    });
  });

  return {
    vertices,
    qubitStates,
    projected2D,
    edges,
    entanglements
  };
}

/**
 * Get edges of the 24-cell
 */
function get24CellEdges(): [number, number][] {
  const vertices = get24CellAsQuaternions();
  const edges: [number, number][] = [];

  // Two vertices are connected if their quaternion distance is minimal
  const edgeLength = 1;  // For unit 24-cell

  for (let i = 0; i < vertices.length; i++) {
    for (let j = i + 1; j < vertices.length; j++) {
      const dx = vertices[i].x - vertices[j].x;
      const dy = vertices[i].y - vertices[j].y;
      const dz = vertices[i].z - vertices[j].z;
      const dw = vertices[i].w - vertices[j].w;
      const dist = Math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw);

      if (Math.abs(dist - edgeLength) < 0.01) {
        edges.push([i, j]);
      }
    }
  }

  return edges;
}

/**
 * Render the Phillips Gate state to canvas
 */
export function drawPhillipsGateState(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  state: PhillipsGateState
): void {
  ctx.save();
  ctx.translate(x, y);

  // Background
  ctx.fillStyle = '#0f172a';
  ctx.fillRect(0, 0, w, h);

  const viz = getVisualizationData(state);
  const cx = w / 2;
  const cy = h / 2;
  const scale = Math.min(w, h) * 0.35;

  // Draw edges (fainter)
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.2)';
  ctx.lineWidth = 1;
  for (const [i, j] of viz.edges) {
    if (i < viz.projected2D.length && j < viz.projected2D.length) {
      const p1 = viz.projected2D[i];
      const p2 = viz.projected2D[j];
      ctx.beginPath();
      ctx.moveTo(cx + p1.x * scale, cy + p1.y * scale);
      ctx.lineTo(cx + p2.x * scale, cy + p2.y * scale);
      ctx.stroke();
    }
  }

  // Draw entanglement links
  ctx.strokeStyle = 'rgba(244, 63, 94, 0.5)';
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 4]);
  for (const [i, j] of viz.entanglements) {
    const p1 = viz.projected2D[i];
    const p2 = viz.projected2D[j];
    ctx.beginPath();
    ctx.moveTo(cx + p1.x * scale, cy + p1.y * scale);
    ctx.lineTo(cx + p2.x * scale, cy + p2.y * scale);
    ctx.stroke();
  }
  ctx.setLineDash([]);

  // Draw qubits
  viz.qubitStates.forEach((qs, i) => {
    const p = viz.projected2D[i];
    const px = cx + p.x * scale;
    const py = cy + p.y * scale;

    // Color based on state (|0⟩ = blue, |1⟩ = red, superposition = purple)
    const r = Math.round(qs.p1 * 255);
    const b = Math.round(qs.p0 * 255);
    ctx.fillStyle = `rgb(${r}, 100, ${b})`;

    // Size based on certainty
    const certainty = Math.abs(qs.p0 - 0.5) * 2;
    const radius = 3 + certainty * 5;

    ctx.beginPath();
    ctx.arc(px, py, radius, 0, 2 * Math.PI);
    ctx.fill();

    // Draw Bloch vector direction as small line
    const vecLen = 10;
    const vx = Math.sin(qs.theta) * Math.cos(qs.phi);
    const vy = Math.sin(qs.theta) * Math.sin(qs.phi);

    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(px, py);
    ctx.lineTo(px + vx * vecLen, py - vy * vecLen);
    ctx.stroke();
  });

  // Title and stats
  ctx.fillStyle = '#ffffff';
  ctx.font = 'bold 11px monospace';
  ctx.fillText('PHILLIPS GATE SIMULATOR', 10, 15);

  ctx.font = '9px monospace';
  ctx.fillStyle = '#94a3b8';
  ctx.fillText(`Qubits: ${state.qubits.length}`, 10, 30);
  ctx.fillText(`Time: ${state.time}`, 10, 42);
  ctx.fillText(`Temp: ${state.temperature.toFixed(3)}`, 10, 54);
  ctx.fillText(`Energy: ${state.energy.toFixed(3)}`, 10, 66);

  ctx.restore();
}
