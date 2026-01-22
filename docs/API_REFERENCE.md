# API Reference: Visual Analog Computing Modules

## Table of Contents

1. [GeometricAlgebra.ts](#geometricalgebrats)
2. [E8Projection.ts](#e8projectionts)
3. [MoireEngine.ts](#moireenginets)
4. [PhillipsGateSimulator.ts](#phillipsgatesimulatorTs)
5. [Type Definitions](#type-definitions)

---

## GeometricAlgebra.ts

### Overview

Implements Clifford Algebra Cl(3,0) and Cl(4,0) for geometric transformations.

### Types

#### Multivector3D

```typescript
interface Multivector3D {
  s: number;       // Scalar (grade 0)
  e1: number;      // Vector x (grade 1)
  e2: number;      // Vector y (grade 1)
  e3: number;      // Vector z (grade 1)
  e12: number;     // Bivector xy (grade 2)
  e23: number;     // Bivector yz (grade 2)
  e31: number;     // Bivector zx (grade 2)
  e123: number;    // Pseudoscalar (grade 3)
}
```

#### Rotor3D

```typescript
interface Rotor3D {
  s: number;       // cos(θ/2)
  e12: number;     // sin(θ/2) * B_xy
  e23: number;     // sin(θ/2) * B_yz
  e31: number;     // sin(θ/2) * B_zx
}
```

#### Spinor

```typescript
interface Spinor {
  real0: number;   // Re(α) for |ψ⟩ = α|0⟩ + β|1⟩
  imag0: number;   // Im(α)
  real1: number;   // Re(β)
  imag1: number;   // Im(β)
}
```

#### BiRotor4D

```typescript
interface BiRotor4D {
  left: Rotor3D;   // Left-isoclinic rotation
  right: Rotor3D;  // Right-isoclinic rotation
}
```

### Constants

```typescript
const ROTOR_IDENTITY: Rotor3D;      // { s: 1, e12: 0, e23: 0, e31: 0 }
const MV3D_ZERO: Multivector3D;     // All zeros
const PSEUDOSCALAR: Multivector3D;  // { e123: 1, ... }
const PAULI_X: Rotor3D;             // Quarter-turn about X
const PAULI_Y: Rotor3D;             // Quarter-turn about Y
const PAULI_Z: Rotor3D;             // Quarter-turn about Z
```

### Functions

#### Rotor Construction

```typescript
function rotorFromAxisAngle(
  axisX: number,
  axisY: number,
  axisZ: number,
  angle: number
): Rotor3D
```
Creates a rotor for rotation by `angle` radians about the axis `(axisX, axisY, axisZ)`.

---

```typescript
function rotorFromBivectorAngle(
  b12: number,
  b23: number,
  b31: number,
  angle: number
): Rotor3D
```
Creates a rotor for rotation by `angle` in the plane defined by bivector `(b12, b23, b31)`.

#### Rotor Operations

```typescript
function multiplyRotors(r1: Rotor3D, r2: Rotor3D): Rotor3D
```
Composes two rotations. Result applies `r1` first, then `r2`.

---

```typescript
function reverseRotor(r: Rotor3D): Rotor3D
```
Returns the reverse (conjugate) of a rotor, representing the inverse rotation.

---

```typescript
function normalizeRotor(r: Rotor3D): Rotor3D
```
Normalizes rotor to unit magnitude.

---

```typescript
function slerpRotor(r1: Rotor3D, r2: Rotor3D, t: number): Rotor3D
```
Spherical linear interpolation between two rotors. `t` ∈ [0, 1].

#### Application Functions

```typescript
function applyRotorToVector(
  r: Rotor3D,
  vx: number,
  vy: number,
  vz: number
): { x: number, y: number, z: number }
```
Applies the sandwich product `R v R̃` to rotate a vector.

---

```typescript
function applyRotorToSpinor(r: Rotor3D, s: Spinor): Spinor
```
Applies a rotor as a quantum gate to a spinor state.

#### Conversion Functions

```typescript
function quaternionToRotor(q: Quaternion): Rotor3D
```
Converts a quaternion `(w, x, y, z)` to a rotor.

---

```typescript
function rotorToQuaternion(r: Rotor3D): Quaternion
```
Converts a rotor back to a quaternion.

---

```typescript
function rotorToAxisAngle(r: Rotor3D): {
  axis: { x: number, y: number, z: number },
  angle: number
}
```
Extracts the rotation axis and angle from a rotor.

#### Spinor Functions

```typescript
function spinorFromBlochSphere(theta: number, phi: number): Spinor
```
Creates a spinor from Bloch sphere coordinates.

---

```typescript
function spinorToBlochSphere(s: Spinor): { theta: number, phi: number }
```
Extracts Bloch sphere coordinates from a spinor.

---

```typescript
function spinorProbabilities(s: Spinor): { p0: number, p1: number }
```
Returns measurement probabilities for |0⟩ and |1⟩.

---

```typescript
function normalizeSpinor(s: Spinor): Spinor
```
Normalizes spinor to unit probability.

#### 4D Functions

```typescript
function createBiRotor(left: Rotor3D, right: Rotor3D): BiRotor4D
```
Creates a 4D double isoclinic rotation.

---

```typescript
function applyBiRotor(
  br: BiRotor4D,
  x: number,
  y: number,
  z: number,
  w: number
): { x: number, y: number, z: number, w: number }
```
Applies 4D rotation to a point.

---

```typescript
function cliffordRotation(
  angle: number,
  plane: 'xy' | 'xz' | 'xw' | 'yz' | 'yw' | 'zw'
): BiRotor4D
```
Creates a simple isoclinic rotation in the specified plane.

---

## E8Projection.ts

### Overview

High-dimensional geometry: E8 lattice, 600-cell, 24-cell, Hopf fibration, quasicrystals.

### Types

```typescript
interface Vector8 {
  x0: number; x1: number; x2: number; x3: number;
  x4: number; x5: number; x6: number; x7: number;
}

interface HopfFiber {
  basePoint: { x: number, y: number, z: number };
  fiberPhase: number;
  torusCoords: { u: number, v: number };
}

interface OctonionBasis {
  index: number;
  fanoPosition: { x: number, y: number };
}

interface FanoLine {
  points: [number, number, number];
  color: string;
}

interface QuasicrystalCell {
  vertices: { x: number, y: number }[];
  type: 'thin' | 'thick';
  phasonShift: number;
}
```

### Constants

```typescript
const PHI: number;        // Golden ratio ≈ 1.618
const PHI_INV: number;    // 1/φ ≈ 0.618
const SQRT5: number;      // √5 ≈ 2.236
```

### E8 Functions

```typescript
function getE8Roots(): Vector8[]
```
Returns all 240 root vectors of the E8 lattice.

---

```typescript
function projectE8ToCoxeterPlane(v: Vector8): { x: number, y: number }
```
Projects an E8 vector to 2D using the Coxeter plane projection.

### 600-Cell Functions

```typescript
function get600CellVertices(): Vector4[]
```
Returns all 120 vertices of the 600-cell (normalized to unit sphere).

---

```typescript
function get600CellEdges(vertices: Vector4[]): [number, number][]
```
Returns edge pairs for the 600-cell.

### 24-Cell Functions

```typescript
function get24CellAsQuaternions(): Vector4[]
```
Returns the 24 Hurwitz quaternion units.

---

```typescript
function get24CellGroupStructure(): {
  vertex: Vector4,
  pauliRelation: string,
  order: number
}[]
```
Maps each vertex to its group-theoretic role.

### Fano Plane

```typescript
function getFanoPlane(): {
  points: OctonionBasis[],
  lines: FanoLine[]
}
```
Returns the 7 points and 7 lines of the Fano plane.

---

```typescript
function octonionMultiply(i: number, j: number): {
  result: number,
  sign: number
}
```
Computes `eᵢ × eⱼ` using the Fano plane multiplication table.

### Hopf Fibration

```typescript
function hopfProjection(
  x: number,
  y: number,
  z: number,
  w: number
): { basePoint: Vector3, fiberPhase: number }
```
Projects a unit quaternion to S² via Hopf fibration.

---

```typescript
function inverseHopf(
  bx: number,
  by: number,
  bz: number,
  fiberPhase: number
): Vector4
```
Reconstructs a quaternion from Hopf coordinates.

---

```typescript
function generateCliffordTorus(
  uDivisions: number,
  vDivisions: number
): Vector4[][]
```
Generates the flat Clifford torus in S³.

---

```typescript
function generateVillarceauCircles(
  numCircles: number,
  pointsPerCircle: number,
  offset?: number
): Vector4[][]
```
Generates Villarceau circles on the Clifford torus.

### Quasicrystal Functions

```typescript
function generatePenroseTiling(
  size: number,
  resolution: number,
  phaseOffset?: { u: number, v: number }
): QuasicrystalCell[]
```
Generates a Penrose tiling via cut-and-project from 5D.

---

```typescript
function generateAmmannBeenkerTiling(
  size: number,
  resolution: number
): QuasicrystalCell[]
```
Generates an 8-fold symmetric Ammann-Beenker tiling from 4D.

### Projection Utilities

```typescript
function stereographicProjection4Dto3D(
  x: number,
  y: number,
  z: number,
  w: number
): { x: number, y: number, z: number }
```
Projects S³ to ℝ³ from the point (0,0,0,-1).

---

```typescript
function rotate600CellAndProject(
  vertices: Vector4[],
  biRotor: BiRotor4D,
  screenW: number,
  screenH: number,
  scale: number
): { x: number, y: number, depth: number }[]
```
Rotates 600-cell and projects to 2D screen coordinates.

---

## MoireEngine.ts

### Overview

Moiré interference patterns for visual analog computation.

### Types

```typescript
interface GratingParams {
  type: 'linear' | 'circular' | 'logPolar' | 'hyperbolic' | 'spiral';
  frequency: number;
  phase: number;
  orientation: number;
  centerX: number;
  centerY: number;
  dutyCycle: number;
}

interface MoireResult {
  fringeSpacing: number;
  fringeAngle: number;
  magnification: number;
  beatFrequency: number;
  phaseShift: number;
}

interface NomogramScale {
  label: string;
  values: number[];
  positions: number[];
  logarithmic: boolean;
}

interface Nomogram {
  scaleA: NomogramScale;
  scaleB: NomogramScale;
  scaleC: NomogramScale;
  currentA: number;
  currentB: number;
  computedC: number;
}
```

### Grating Generation

```typescript
function generateLinearGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color?: string
): void
```

---

```typescript
function generateCircularGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color?: string
): void
```

---

```typescript
function generateLogPolarGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color?: string
): void
```

---

```typescript
function generateHyperbolicGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color?: string
): void
```

### Moiré Computation

```typescript
function computeLinearMoire(
  g1: { frequency: number, angle: number },
  g2: { frequency: number, angle: number }
): MoireResult
```

---

```typescript
function computeCircularMoire(
  g1: { centerX: number, centerY: number, frequency: number },
  g2: { centerX: number, centerY: number, frequency: number }
): MoireResult
```

---

```typescript
function computeLogPolarMoire(
  g1: { phase: number, scale: number },
  g2: { phase: number, scale: number }
): {
  rotationShift: number,
  scaleShift: number,
  complexArg: number,
  complexMag: number
}
```

### Nomogram Functions

```typescript
function createMultiplicationNomogram(
  rangeA: [number, number],
  rangeB: [number, number]
): Nomogram
```

---

```typescript
function createVelocityNomogram(
  timeRangeMs: [number, number],
  distanceRangeM: [number, number]
): Nomogram
```

---

```typescript
function drawNomogram(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  nomogram: Nomogram,
  colors: { bg: string, scale: string, line: string, marker: string }
): void
```

### Visualization Functions

```typescript
function drawQuaternionMoire(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  qw: number, qx: number, qy: number, qz: number,
  referenceQ?: { w: number, x: number, y: number, z: number } | null
): void
```

---

```typescript
function drawSpinorMoire(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  theta: number,
  phi: number
): void
```

---

```typescript
function drawVernierScale(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  mainValue: number,
  precision?: number
): void
```

---

```typescript
function drawDifferentialFlowField(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  expectedFlow: { x: number, y: number },
  observedFlow: { x: number, y: number },
  gridSpacing?: number
): void
```

---

## PhillipsGateSimulator.ts

### Overview

Quantum computing simulation on 24-cell/600-cell substrate.

### Types

```typescript
interface VertexQubit {
  vertexIndex: number;
  state: Spinor;
  blochCoords: { theta: number, phi: number };
  groupElement: string;
  entangledWith: number[];
}

interface QuantumGate {
  name: string;
  rotor: Rotor3D;
  axis: { x: number, y: number, z: number };
  angle: number;
  matrix: [[number, number], [number, number]];
}

interface PhillipsGateState {
  qubits: VertexQubit[];
  substrate: '24cell' | '600cell';
  globalRotor: BiRotor4D;
  time: number;
  energy: number;
  temperature: number;
  history: { time: number, measurement: { index: number, result: 0 | 1 }[] }[];
}

interface CACell {
  vertexIndex: number;
  state: number;
  neighbors: number[];
  energy: number;
}
```

### Gate Constants

```typescript
const GATE_I: QuantumGate;   // Identity
const GATE_X: QuantumGate;   // Pauli-X (NOT)
const GATE_Y: QuantumGate;   // Pauli-Y
const GATE_Z: QuantumGate;   // Pauli-Z
const GATE_H: QuantumGate;   // Hadamard
const GATE_S: QuantumGate;   // Phase gate
const GATE_T: QuantumGate;   // T gate (π/8)
```

### Gate Factory Functions

```typescript
function createRxGate(theta: number): QuantumGate
function createRyGate(theta: number): QuantumGate
function createRzGate(theta: number): QuantumGate
```

### Simulation Functions

```typescript
function initializePhillipsGate(
  numQubits?: number,
  substrate?: '24cell' | '600cell'
): PhillipsGateState
```

---

```typescript
function applyGate(
  state: PhillipsGateState,
  gateIndex: number,
  qubitIndex: number
): PhillipsGateState
```

---

```typescript
function applyRotorGate(
  state: PhillipsGateState,
  rotor: Rotor3D,
  qubitIndex: number
): PhillipsGateState
```

---

```typescript
function applyPhillipsGate(
  state: PhillipsGateState,
  leftRotor: Rotor3D,
  rightRotor: Rotor3D
): PhillipsGateState
```

---

```typescript
function measureQubit(
  state: PhillipsGateState,
  qubitIndex: number
): { state: PhillipsGateState, result: 0 | 1 }
```

---

```typescript
function entangleQubits(
  state: PhillipsGateState,
  controlIndex: number,
  targetIndex: number
): PhillipsGateState
```

### Cellular Automaton

```typescript
function initializeCellularAutomaton(): CACell[]
function stepCellularAutomaton(cells: CACell[]): CACell[]
function createGlider(startVertex: number, cells: CACell[]): number[]
```

### Annealing Functions

```typescript
function computeEnergy(
  state: PhillipsGateState,
  costFunction: (probabilities: number[]) => number
): number
```

---

```typescript
function annealingStep(
  state: PhillipsGateState,
  costFunction: (probabilities: number[]) => number,
  coolingRate?: number
): PhillipsGateState
```

---

```typescript
function runAnnealing(
  numQubits: number,
  costFunction: (probabilities: number[]) => number,
  iterations?: number,
  initialTemp?: number
): {
  finalState: PhillipsGateState,
  solution: number[],
  energy: number
}
```

### Visualization

```typescript
function getVisualizationData(state: PhillipsGateState): {
  vertices: Vector4[],
  qubitStates: { theta: number, phi: number, p0: number, p1: number }[],
  projected2D: { x: number, y: number }[],
  edges: [number, number][],
  entanglements: [number, number][]
}
```

---

```typescript
function drawPhillipsGateState(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  state: PhillipsGateState
): void
```

---

## Type Definitions

### From types.ts

```typescript
// Core types (existing)
interface Vector3 { x: number; y: number; z: number; }
interface Quaternion { w: number; x: number; y: number; z: number; }
interface Point2D { x: number; y: number; }

// Geometric Algebra types
interface Vector4 { x: number; y: number; z: number; w: number; }
interface Rotor { s: number; e12: number; e23: number; e31: number; }
interface SpinorState { real0: number; imag0: number; real1: number; imag1: number; }
interface BlochCoords { theta: number; phi: number; }

// Moiré types
interface GratingConfig {
  type: 'linear' | 'circular' | 'logPolar' | 'hyperbolic' | 'spiral';
  frequency: number;
  phase: number;
  orientation: number;
  center: Point2D;
  dutyCycle: number;
  color: string;
}

interface MoireAnalysis {
  fringeSpacing: number;
  fringeAngle: number;
  magnification: number;
  beatFrequency: number;
  phaseShift: number;
  confidence: number;
}

interface NomogramConfig {
  scaleA: { label: string; range: [number, number]; logarithmic: boolean; };
  scaleB: { label: string; range: [number, number]; logarithmic: boolean; };
  scaleC: { label: string; operation: 'multiply' | 'divide' | 'add' | 'subtract'; };
}

// E8/Hopf types
interface HopfProjection {
  basePoint: Vector3;
  fiberPhase: number;
  villarceauAngle: number;
}

interface E8Vertex {
  coords: number[];
  coxeterProjection: Point2D;
  rootType: 'integer' | 'halfInteger';
}

interface FanoPoint {
  index: number;
  position: Point2D;
  quaternionicTriples: number[][];
}

// Phillips Gate types
interface VertexQubitState {
  vertexIndex: number;
  spinor: SpinorState;
  bloch: BlochCoords;
  entangledIndices: number[];
  pauliRelation: string;
}

interface PhillipsGateSnapshot {
  qubits: VertexQubitState[];
  substrate: '24cell' | '600cell';
  time: number;
  energy: number;
  temperature: number;
  globalRotation: { left: Rotor; right: Rotor; };
}

interface PolytopeCAState {
  cells: { vertexIndex: number; state: 0 | 1; neighbors: number[]; }[];
  generation: number;
  totalEnergy: number;
}

interface AnnealingResult {
  solution: number[];
  energy: number;
  iterations: number;
  temperatureHistory: number[];
  converged: boolean;
}
```

---

*Document Version: 1.0.0*
*Last Updated: 2026-01-22*
