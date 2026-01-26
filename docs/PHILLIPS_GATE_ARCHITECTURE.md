# Phillips Gate Quantum Simulator Architecture

## Executive Summary

The Phillips Gate Simulator is a **visual quantum computing emulator** that uses:
- The **24-cell polytope** as a qubit register substrate
- **Geometric Algebra rotors** as quantum gate operations
- **Moiré visualization** for quantum state readout
- **Simulated annealing** for optimization problems

This is NOT a true quantum computer - it's a classical simulation designed to be **visually interpretable** by Vision LLMs.

---

## 1. Theoretical Foundation

### 1.1 The 24-Cell as Quantum Register

The **24-cell** (Icositetrachoron) is a regular 4D polytope with:
- 24 vertices
- 96 edges
- 96 faces (triangles)
- 24 cells (octahedra)

Its vertices correspond to the **24 Hurwitz quaternion units**:
```
±1, ±i, ±j, ±k                        (8 units)
(±1 ± i ± j ± k) / 2                   (16 units)
```

### 1.2 The Binary Tetrahedral Group (2T)

These 24 units form the **Binary Tetrahedral Group** 2T, which is isomorphic to SL(2,3) - the special linear group over the field with 3 elements.

**Critical insight**: 2T contains the **Pauli group** (up to phase):
- σ_x ↔ i
- σ_y ↔ j
- σ_z ↔ k
- Identity ↔ 1

This means the 24-cell vertices naturally encode quantum gate operations!

### 1.3 The Phillips Gate Concept

A **Phillips Gate** is a global 4D rotation of the entire 24-cell substrate:
```
|ψ'⟩ = L ⊗ R |ψ⟩
```

Where L and R are left and right isoclinic rotations.

Each qubit at vertex v picks up a phase based on how v moves under the rotation:
```
phase_v = arg(L × v × R̃)
```

This coherent phase accumulation is analogous to a **global unitary** acting on all qubits simultaneously.

---

## 2. System Architecture

### 2.1 Module Dependency Graph

```
┌─────────────────────────────────────────────────────────────┐
│                    PhillipsGateSimulator.ts                 │
│                    (Quantum Simulation)                     │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┴─────────────┐
        │                           │
        ▼                           ▼
┌───────────────────┐     ┌───────────────────┐
│ GeometricAlgebra  │     │   E8Projection    │
│ (Rotors, Spinors) │     │ (24-cell, 600-cell)│
└───────────────────┘     └───────────────────┘
        │                           │
        └───────────┬───────────────┘
                    │
                    ▼
          ┌───────────────────┐
          │   MoireEngine     │
          │ (Visualization)   │
          └───────────────────┘
                    │
                    ▼
          ┌───────────────────┐
          │ imageCompositor   │
          │ (Canvas Render)   │
          └───────────────────┘
```

### 2.2 Core Data Structures

```typescript
// A qubit stored at a 24-cell vertex
interface VertexQubit {
  vertexIndex: number;      // 0-23 for 24-cell
  state: Spinor;            // |ψ⟩ = α|0⟩ + β|1⟩
  blochCoords: {
    theta: number;          // Polar angle
    phi: number;            // Azimuthal angle
  };
  groupElement: string;     // e.g., "iσx", "T-gate class"
  entangledWith: number[];  // Indices of entangled qubits
}

// Complete simulation state
interface PhillipsGateState {
  qubits: VertexQubit[];
  substrate: '24cell' | '600cell';
  globalRotor: BiRotor4D;   // Current 4D orientation
  time: number;
  energy: number;
  temperature: number;      // For annealing
  history: MeasurementRecord[];
}
```

---

## 3. Quantum Gate Implementation

### 3.1 Standard Gates as Rotors

Each quantum gate corresponds to a rotation on the Bloch sphere:

```typescript
// Pauli-X (NOT gate): π rotation about X-axis
const GATE_X: QuantumGate = {
  name: 'X',
  rotor: rotorFromAxisAngle(1, 0, 0, Math.PI),
  axis: { x: 1, y: 0, z: 0 },
  angle: Math.PI
};

// Hadamard: π rotation about (X+Z)/√2 axis
const GATE_H: QuantumGate = {
  name: 'H',
  rotor: rotorFromAxisAngle(Math.SQRT1_2, 0, Math.SQRT1_2, Math.PI),
  axis: { x: Math.SQRT1_2, y: 0, z: Math.SQRT1_2 },
  angle: Math.PI
};

// T gate (π/8): π/4 rotation about Z-axis
const GATE_T: QuantumGate = {
  name: 'T',
  rotor: rotorFromAxisAngle(0, 0, 1, Math.PI / 4),
  axis: { x: 0, y: 0, z: 1 },
  angle: Math.PI / 4
};
```

### 3.2 Applying a Gate

```typescript
function applyGate(
  state: PhillipsGateState,
  gate: QuantumGate,
  qubitIndex: number
): PhillipsGateState {
  const newQubits = [...state.qubits];
  const qubit = { ...newQubits[qubitIndex] };

  // Apply the gate rotor to the spinor
  qubit.state = normalizeSpinor(
    applyRotorToSpinor(gate.rotor, qubit.state)
  );

  // Update Bloch coordinates
  qubit.blochCoords = spinorToBlochSphere(qubit.state);

  newQubits[qubitIndex] = qubit;
  return { ...state, qubits: newQubits, time: state.time + 1 };
}
```

### 3.3 The Global Phillips Gate

```typescript
function applyPhillipsGate(
  state: PhillipsGateState,
  leftRotor: Rotor3D,
  rightRotor: Rotor3D
): PhillipsGateState {
  // Compose with existing global rotation
  const newBiRotor = createBiRotor(
    multiplyRotors(leftRotor, state.globalRotor.left),
    multiplyRotors(state.globalRotor.right, rightRotor)
  );

  // Each qubit gets a phase based on vertex position
  const newQubits = state.qubits.map((qubit, i) => {
    const vertex = get24CellAsQuaternions()[i];

    // Rotate the vertex
    const rotated = applyBiRotor(newBiRotor, vertex.x, vertex.y, vertex.z, vertex.w);

    // Compute accumulated phase
    const phaseShift = Math.atan2(rotated.y, rotated.x) -
                       Math.atan2(vertex.y, vertex.x);

    // Apply phase to qubit
    const phaseRotor = rotorFromAxisAngle(0, 0, 1, phaseShift);
    const newState = normalizeSpinor(
      applyRotorToSpinor(phaseRotor, qubit.state)
    );

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
```

---

## 4. Measurement and Readout

### 4.1 Probabilistic Measurement

```typescript
function measureQubit(
  state: PhillipsGateState,
  qubitIndex: number
): { state: PhillipsGateState, result: 0 | 1 } {
  const qubit = state.qubits[qubitIndex];
  const { p0, p1 } = spinorProbabilities(qubit.state);

  // Probabilistic collapse
  const result: 0 | 1 = Math.random() < p0 ? 0 : 1;

  // Update state to collapsed basis state
  const newQubits = [...state.qubits];
  newQubits[qubitIndex] = {
    ...qubit,
    state: result === 0
      ? { real0: 1, imag0: 0, real1: 0, imag1: 0 }  // |0⟩
      : { real0: 0, imag0: 0, real1: 1, imag1: 0 }, // |1⟩
    blochCoords: result === 0
      ? { theta: 0, phi: 0 }
      : { theta: Math.PI, phi: 0 }
  };

  return { state: { ...state, qubits: newQubits }, result };
}
```

### 4.2 Visual State Readout

The simulator renders qubit states visually:

| Visual Feature | Quantum Meaning |
|----------------|-----------------|
| Vertex color (red↔blue) | |0⟩ vs |1⟩ probability |
| Vertex size | State certainty |
| Bloch vector line | Superposition direction |
| Rose dashed lines | Entanglement links |

---

## 5. The 600-Cell Extension

### 5.1 Scaling Up

The **600-cell** has 120 vertices, allowing for larger quantum registers:
- 120 qubits (vs 24 for the 24-cell)
- More complex entanglement topologies
- Better surface for cellular automaton

### 5.2 Cellular Automaton Mode

The 600-cell surface supports a **Game of Life** variant:

```typescript
function stepCellularAutomaton(cells: CACell[]): CACell[] {
  return cells.map(cell => {
    const liveNeighbors = cell.neighbors.reduce(
      (count, ni) => count + cells[ni].state, 0
    );

    // Modified rules for 12-connected 600-cell vertices
    let newState = cell.state;
    if (cell.state === 1) {
      // Survival: 4-7 neighbors
      if (liveNeighbors < 4 || liveNeighbors > 7) newState = 0;
    } else {
      // Birth: exactly 5 or 6 neighbors
      if (liveNeighbors === 5 || liveNeighbors === 6) newState = 1;
    }

    return { ...cell, state: newState };
  });
}
```

"Gliders" on the 600-cell surface can perform computation through their interactions.

---

## 6. Simulated Annealing

### 6.1 Energy Function

For optimization problems, define an energy (cost) function:

```typescript
function computeEnergy(
  state: PhillipsGateState,
  costFunction: (probabilities: number[]) => number
): number {
  const probabilities = state.qubits.map(q => {
    const { p1 } = spinorProbabilities(q.state);
    return p1;  // Probability of |1⟩
  });

  return costFunction(probabilities);
}
```

### 6.2 Metropolis-Hastings Step

```typescript
function annealingStep(
  state: PhillipsGateState,
  costFunction: (probs: number[]) => number,
  coolingRate: number = 0.99
): PhillipsGateState {
  const currentEnergy = computeEnergy(state, costFunction);

  // Random perturbation
  const randomQubit = Math.floor(Math.random() * state.qubits.length);
  const randomAngle = (Math.random() - 0.5) * Math.PI * 0.1;
  const randomRotor = rotorFromAxisAngle(
    Math.random(), Math.random(), Math.random(), randomAngle
  );

  const trialState = applyRotorGate(state, randomRotor, randomQubit);
  const trialEnergy = computeEnergy(trialState, costFunction);

  // Metropolis criterion
  const deltaE = trialEnergy - currentEnergy;
  const acceptProb = deltaE < 0 ? 1 : Math.exp(-deltaE / state.temperature);

  if (Math.random() < acceptProb) {
    return {
      ...trialState,
      energy: trialEnergy,
      temperature: state.temperature * coolingRate
    };
  }

  return { ...state, temperature: state.temperature * coolingRate };
}
```

### 6.3 Use Cases

1. **MaxCut Problem**: Partition graph vertices to maximize cut edges
2. **Traveling Salesman**: Find shortest tour (via binary encoding)
3. **Satisfiability**: Find truth assignments for Boolean formulas
4. **Logistics Optimization**: Resource allocation, scheduling

---

## 7. Visual Output

### 7.1 Rendering Pipeline

```typescript
function drawPhillipsGateState(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  state: PhillipsGateState
): void {
  // 1. Get visualization data
  const viz = getVisualizationData(state);

  // 2. Draw substrate edges (gray)
  for (const [i, j] of viz.edges) {
    drawLine(viz.projected2D[i], viz.projected2D[j], '#94a3b8', 0.2);
  }

  // 3. Draw entanglement links (rose dashed)
  for (const [i, j] of viz.entanglements) {
    drawDashedLine(viz.projected2D[i], viz.projected2D[j], '#F43F5E');
  }

  // 4. Draw qubits
  viz.qubitStates.forEach((qs, i) => {
    const color = `rgb(${qs.p1 * 255}, 100, ${qs.p0 * 255})`;
    const radius = 3 + Math.abs(qs.p0 - 0.5) * 10;
    drawCircle(viz.projected2D[i], radius, color);

    // Bloch vector
    drawBlochVector(viz.projected2D[i], qs.theta, qs.phi);
  });
}
```

### 7.2 Projection Method

The 4D polytope is projected to 2D using:
1. **4D Bi-Rotor rotation** (global orientation)
2. **Hopf fibration** projection (S³ → S²)
3. **Stereographic projection** (S² → ℝ²)

```typescript
function getVisualizationData(state: PhillipsGateState) {
  const vertices = get24CellAsQuaternions();

  // Apply global 4D rotation
  const rotatedVertices = vertices.map(v =>
    applyBiRotor(state.globalRotor, v.x, v.y, v.z, v.w)
  );

  // Project via Hopf fibration
  const projected2D = rotatedVertices.map(v => {
    const hopf = hopfProjection(v.x, v.y, v.z, v.w);
    const denom = 1 - hopf.basePoint.z;
    return {
      x: hopf.basePoint.x / denom,
      y: hopf.basePoint.y / denom
    };
  });

  return { vertices: rotatedVertices, projected2D, ... };
}
```

---

## 8. API Reference

### 8.1 Initialization

```typescript
// Create a new simulator with 24 qubits
const state = initializePhillipsGate(24, '24cell');

// Or with 120 qubits on 600-cell
const bigState = initializePhillipsGate(120, '600cell');
```

### 8.2 Gate Application

```typescript
// Apply Hadamard to qubit 0
state = applyGate(state, 4, 0);  // Gate index 4 = H

// Apply custom rotor
const myRotor = rotorFromAxisAngle(1, 1, 0, Math.PI / 3);
state = applyRotorGate(state, myRotor, 5);

// Apply global Phillips Gate
state = applyPhillipsGate(state, leftRotor, rightRotor);
```

### 8.3 Measurement

```typescript
// Measure single qubit
const { state: newState, result } = measureQubit(state, 0);
console.log(`Qubit 0 collapsed to |${result}⟩`);

// Get probabilities without collapsing
const { p0, p1 } = spinorProbabilities(state.qubits[3].state);
```

### 8.4 Entanglement

```typescript
// Create entanglement between qubits 0 and 1
state = entangleQubits(state, 0, 1);
```

### 8.5 Annealing

```typescript
// Define cost function
const cost = (probs: number[]) => {
  // Example: minimize sum of probabilities
  return probs.reduce((a, b) => a + b, 0);
};

// Run annealing
const result = runAnnealing(24, cost, 1000, 10.0);
console.log('Solution:', result.solution);
console.log('Energy:', result.energy);
```

---

## 9. Limitations and Future Work

### 9.1 Current Limitations

1. **Not real quantum**: Classical simulation with exponential state space collapse
2. **No true entanglement**: Simulated via conditional operations
3. **Measurement is simulated**: Uses pseudo-random collapse
4. **Limited gate set**: Only single-qubit gates implemented

### 9.2 Planned Enhancements

1. **Two-qubit gates**: CNOT, CZ, SWAP via edge operations
2. **Density matrices**: Mixed state representation
3. **Error simulation**: Depolarizing and dephasing noise
4. **Circuit compilation**: Parse OpenQASM or Cirq circuits
5. **VQE integration**: Variational quantum eigensolver

### 9.3 Research Directions

1. **Topological codes**: Use 600-cell surface for error correction
2. **Quantum walks**: Continuous-time evolution on polytope graph
3. **Geometric entanglement**: Exploit 4D geometry for entanglement patterns
4. **Visual quantum algorithms**: Grover/Shor with moiré readout

---

## 10. Example: Quantum Teleportation

```typescript
// Initialize 3-qubit system
let state = initializePhillipsGate(3, '24cell');

// Step 1: Create Bell pair between qubits 1 and 2
state = applyGate(state, 4, 1);  // H on qubit 1
state = entangleQubits(state, 1, 2);  // "CNOT" on 1→2

// Step 2: Prepare qubit 0 in some state to teleport
state = applyRotorGate(state, rotorFromAxisAngle(1, 0, 0, 0.7), 0);

// Step 3: Bell measurement on qubits 0 and 1
state = entangleQubits(state, 0, 1);
state = applyGate(state, 4, 0);  // H on qubit 0

const { state: s1, result: m0 } = measureQubit(state, 0);
const { state: s2, result: m1 } = measureQubit(s1, 1);

// Step 4: Classical correction on qubit 2
if (m1 === 1) state = applyGate(s2, 1, 2);  // X correction
if (m0 === 1) state = applyGate(state, 3, 2);  // Z correction

// Qubit 2 now has the original state of qubit 0!
console.log('Teleported state:', state.qubits[2].blochCoords);
```

---

*Document Version: 1.0.0*
*Last Updated: 2026-01-22*
*Module: PhillipsGateSimulator.ts*
