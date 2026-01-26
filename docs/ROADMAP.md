# Roadmap: Evolution & Integration

This document outlines the strategy for evolving KineticLens from a passive analysis tool into an active, agentic system.

## Phase 1: Enhanced Visual Intelligence (Current)
- [x] Tri-Phase Photography
- [x] Parallax Gyro-Epitaxy
- [ ] **Next Step:** Stereo-Photogrammetry using Dual-Lens cameras (iPhone Pro / Android Flagships) to capture true depth maps instead of estimating via Epitaxy.

---

## Evolution Track 3: Phillips Gate Visual Quantum Computing (COMPLETE)

### Implemented Modules

#### 3.1 Geometric Algebra Engine (`GeometricAlgebra.ts`)
- [x] Multivector arithmetic in Cl(3,0)
- [x] Rotor construction (axis-angle, bivector-angle)
- [x] Rotor composition and interpolation (SLERP)
- [x] Spinor/qubit state representation
- [x] Rotor-to-spinor gate application
- [x] 4D Bi-Rotors for double isoclinic rotation
- [x] Quaternion ↔ Rotor conversion

#### 3.2 E8 Projection Engine (`E8Projection.ts`)
- [x] E8 root system (240 vertices)
- [x] Coxeter plane projection
- [x] 600-Cell generation (120 vertices, golden ratio coords)
- [x] 24-Cell as Hurwitz quaternion units
- [x] Binary Tetrahedral Group structure
- [x] Fano Plane for octonion multiplication
- [x] Hopf fibration (S³ → S²)
- [x] Clifford torus and Villarceau circles
- [x] Penrose tiling (5-fold quasicrystal)
- [x] Ammann-Beenker tiling (8-fold quasicrystal)

#### 3.3 Moiré Visual Computing (`MoireEngine.ts`)
- [x] Linear grating generation
- [x] Circular grating (Fresnel zone plate)
- [x] Log-polar grating (conformal mapping)
- [x] Hyperbolic grating (Lorentz-like)
- [x] Moiré fringe computation (spacing, magnification)
- [x] Nomogram generation and rendering
- [x] Quaternion moiré display
- [x] Spinor/Bloch sphere moiré
- [x] Vernier scale visualization
- [x] Differential flow field (gyro vs optical)

#### 3.4 Phillips Gate Simulator (`PhillipsGateSimulator.ts`)
- [x] 24-Cell qubit substrate
- [x] 600-Cell extended substrate
- [x] Standard quantum gates (I, X, Y, Z, H, S, T)
- [x] Parameterized rotation gates (Rx, Ry, Rz)
- [x] Global Phillips Gate operation
- [x] Qubit measurement with collapse
- [x] Entanglement simulation
- [x] Cellular automaton on 600-cell
- [x] Simulated annealing optimization
- [x] Visual state rendering

#### 3.5 Integration
- [x] imageCompositor.ts: Moiré panels (quaternion, log-polar, nomogram)
- [x] types.ts: Quantum and moiré type definitions
- [x] Documentation: Theory guides, API reference

### Architecture Philosophy
The Phillips Gate system implements "Visual Algebra" - replacing numerical computation with geometric pattern recognition. The core insight:

> **Vision LLMs excel at spatial relationships. By encoding calculations as moiré fringes, nomograms, and flow fields, we transform arithmetic into pattern matching.**

### Documentation
- [GEOMETRIC_ALGEBRA_PRIMER.md](./GEOMETRIC_ALGEBRA_PRIMER.md) - Mathematical foundations
- [MOIRE_VISUAL_COMPUTING.md](./MOIRE_VISUAL_COMPUTING.md) - Moiré theory and implementation
- [PHILLIPS_GATE_ARCHITECTURE.md](./PHILLIPS_GATE_ARCHITECTURE.md) - Quantum simulation design
- [API_REFERENCE.md](./API_REFERENCE.md) - Complete API documentation

---

## Phase 2: Agentic Integration (The "Watcher" Protocol)

We aim to reconfigure this system for **Autonomous Monitoring**.

### Use Case: Smart Security Sentry
Instead of recording 24/7, the system buffers sensor data.
1.  **Trigger:** Accelerometer detects vibration (e.g., door slamming) OR Vision detects specific pixel flux.
2.  **Agentic Decision:** The local model analyzes the `gyroStress` and `stabilityScore`.
3.  **Action:** If data is high-quality, it triggers the "Kinetic Burst" and sends to Gemini for intent analysis.
    *   *Prompt:* "Did this person approach the door with aggressive intent?"

### Architecture Changes Required:
*   **Loop:** Move `CaptureScreen` logic into a WebWorker.
*   **Buffer:** Increase ring buffer to 10 seconds.
*   **Local Inference:** Use TensorFlow.js (MobileNet) for pre-filtering frames before sending to Gemini (Cost reduction).

## Phase 3: Hardware Acceleration

### LiDAR Fusion
Integration with WebXR Device API to access LiDAR depth buffers.
*   **Impact:** `drawDepthLattice` becomes real measurement, not just a visual projection.
*   **Variable:** `depthMap` (Float32Array) added to `CapturePoint`.

### Edge-TPU Optimization
Compiling the "Holographic Compositor" into WebAssembly (Wasm) using Rust to reduce the 500ms encoding latency to <50ms.

## Phase 4: Swarm Telemetry

Connecting multiple KineticLens instances via WebSockets.
*   **Concept:** Two phones looking at the same object from different angles.
*   **Math:** The "Geometric Proof" expands to solve for X, Y, and Z velocity using triangulation between Device A and Device B.
