# KineticLens Development Roadmap
## From Visualization to Computation

---

## Current State Analysis

The codebase has powerful mathematical modules that are **disconnected from the measurement pipeline**:

| Module | Status | Problem |
|--------|--------|---------|
| GeometricAlgebra.ts | ~450 lines | Only `quaternionToRotor()` used |
| E8Projection.ts | ~650 lines | **Never imported** |
| PhillipsGateSimulator.ts | ~550 lines | **Never instantiated** |
| MoireEngine.ts | ~600 lines | Only visualization, no computation |

**Current data flow:**
```
Sensors → OrthogonalFilter → Draw Pretty Pictures → Send to Gemini → LLM Does Physics
```

**Problem:** The 24-cell, moiré patterns, and geometric algebra are *cosmetic overlays*, not computational tools. The LLM is doing 100% of the physics.

---

## The Vision: Geometric Computation

**Goal:** Use geometric structures as an **analog computing layer** that performs physics locally, with LLM as validator/interpreter.

```
Sensors → Geometric Computation (LOCAL) → Structured Data → LLM Validates/Interprets
           ↑                                    ↓
      24-cell quaternion sampling          "Your estimate of 2.3 m/s is consistent
      Moiré fringe measurement             with the visual parallax I observe"
      Rotor-based sensor fusion
```

---

## Core Insight: Why Geometry Matters

### 1. The 24-Cell as Quaternion Sampling Lattice

The 24-cell's 24 vertices are the **Hurwitz quaternion units**:
```
±1, ±i, ±j, ±k                           (8 vertices)
½(±1 ± i ± j ± k)                        (16 vertices)
```

**Why this matters for sensors:**
- Device orientation is a quaternion (4D unit sphere S³)
- The 24-cell provides **optimal discrete sampling** of S³
- Each vertex = a canonical orientation (like compass points, but in 4D)
- Sensor readings can be **snapped** to nearest vertex for noise reduction
- Interpolation between vertices uses **rotor SLERP** (geodesic, no gimbal lock)

**Application:**
```typescript
// Current: Raw quaternion (noisy)
orientation = deviceOrientation.quaternion

// Proposed: 24-cell quantized (denoised + regularized)
nearestVertex = findNearest24CellVertex(orientation)
orientationClean = slerpToVertex(orientation, nearestVertex, smoothingFactor)
```

### 2. Moiré as Analog Computer

Moiré patterns aren't just pretty - they perform **optical multiplication**:

```
Grating 1: f₁(x) = sin(2πx/d₁)
Grating 2: f₂(x) = sin(2πx/d₂)
Product:   f₁·f₂ = ½[cos(2π(1/d₁ - 1/d₂)x) - cos(2π(1/d₁ + 1/d₂)x)]
                              ↑ DIFFERENCE FREQUENCY (visible moiré)
```

**Why this matters:**
- Small angle rotation θ → Large fringe shift Δx = d/θ
- **Amplification factor:** d/θ can be 100× or more
- This is real measurement, not visualization

**Application:**
```
Camera motion: θ_camera (from gyroscope)
Optical flow:  θ_observed (from video frames)
Object motion: θ_object = θ_observed - θ_camera

Using moiré:
- Grating 1 phase: θ_camera (from IMU)
- Grating 2 phase: θ_observed (from frame differencing)
- Moiré fringe shift = θ_object × amplification
```

### 3. Log-Polar Transform for Rotation-Scale Separation

Log-polar coordinates: (r, θ) → (log r, θ)

**Key property:**
- **Rotation** in Cartesian → **Vertical translation** in log-polar
- **Scaling** in Cartesian → **Horizontal translation** in log-polar

**Application:**
```
Scene change types:
- Camera pan/tilt → Rotation → Vertical moiré shift
- Camera zoom/dolly → Scale → Horizontal moiré shift
- Object motion → Mixed → Diagonal pattern

By decomposing log-polar moiré patterns, we separate motion types.
```

### 4. Geometric Algebra for Sensor Fusion

Problem: Combining accelerometer + gyroscope + magnetometer without gimbal lock.

**Matrix approach (problematic):**
- Euler angles → Gimbal lock
- Rotation matrices → 9 numbers for 3 DOF, redundant

**Rotor approach (clean):**
```typescript
// Gyroscope gives angular velocity ω
angularVelocity = gyroReading  // Vec3

// Integrate using rotor derivative: dR/dt = ½ω·R
rotorDerivative = multiplyRotors(
  rotorFromBivector(angularVelocity),
  currentRotor
).scale(0.5)

// Update orientation
newRotor = addRotors(currentRotor, rotorDerivative.scale(dt))
newRotor = normalizeRotor(newRotor)  // Stay on unit sphere
```

**Advantage:** No gimbal lock, always valid rotation, minimal representation.

---

## Development Phases

### Phase 1: Foundation - Connect Existing Modules
**Goal:** Wire up the unused modules to the data pipeline

#### Milestone 1.1: 24-Cell Orientation Quantization
- [ ] Create `OrientationQuantizer` class using 24-cell vertices
- [ ] Implement `snapToNearestVertex(quaternion) → {vertex, distance}`
- [ ] Add rotor SLERP for smooth interpolation
- [ ] Replace raw device orientation with quantized orientation in CapturePoint

**Deliverable:** Noise-reduced orientation data using 24-cell structure

#### Milestone 1.2: Rotor-Based IMU Integration
- [ ] Replace quaternion math in cpeMath.ts with rotor operations
- [ ] Implement `integrateGyroscope(rotorState, angularVelocity, dt) → newRotor`
- [ ] Add complementary filter: fuse gyro integration with accelerometer gravity vector
- [ ] Output: Clean rotation estimate without LLM

**Deliverable:** Local rotation tracking using geometric algebra

#### Milestone 1.3: Optical Flow Computation
- [ ] Implement Lucas-Kanade optical flow in MoireEngine.ts
- [ ] Compute frame-to-frame displacement vectors
- [ ] Separate flow field into translation + rotation + zoom components
- [ ] Output: Flow field decomposition

**Deliverable:** Local motion estimation from video frames

---

### Phase 2: Moiré as Measurement

#### Milestone 2.1: Log-Polar Frame Transform
- [ ] Convert video frames to log-polar representation
- [ ] Implement efficient polar resampling on GPU (WebGL)
- [ ] Enable rotation detection via vertical correlation
- [ ] Enable zoom detection via horizontal correlation

**Deliverable:** Rotation/scale separated from video frames

#### Milestone 2.2: Moiré Differential Measurement
- [ ] Create reference grating from IMU rotation estimate
- [ ] Overlay on log-polar video frame
- [ ] Compute moiré fringe phase from interference pattern
- [ ] Phase difference = object motion (IMU-compensated)

**Deliverable:** IMU-corrected motion measurement using optical interference

#### Milestone 2.3: Velocity Nomogram Integration
- [ ] Connect nomogram scales to computed values (not just visualization)
- [ ] Input: optical flow magnitude, time delta, estimated distance
- [ ] Output: velocity in physical units (m/s)
- [ ] Render nomogram with computed answer highlighted

**Deliverable:** Visual calculation of velocity that's also computed locally

---

### Phase 3: Sensor Fusion Engine

#### Milestone 3.1: Kalman Filter with Rotor State
- [ ] Define state vector: [position, velocity, rotor, angular_velocity]
- [ ] Implement rotor-space Kalman update (multiplicative)
- [ ] Fuse: accelerometer, gyroscope, optical flow
- [ ] Output: Smoothed 6-DOF trajectory estimate

**Deliverable:** Local sensor fusion without LLM

#### Milestone 3.2: Parallax Depth Estimation
- [ ] Use multi-frame capture with known camera motion
- [ ] Compute disparity from optical flow + IMU
- [ ] Estimate depth per-pixel: z = baseline × focal_length / disparity
- [ ] Generate depth map overlay

**Deliverable:** Monocular depth estimation using motion parallax

#### Milestone 3.3: Object vs Camera Motion Separation
- [ ] Camera motion: from IMU integration
- [ ] Total scene motion: from optical flow
- [ ] Object motion: total - camera (vector subtraction)
- [ ] Identify independently moving objects

**Deliverable:** Motion decomposition computed locally

---

### Phase 4: LLM as Validator (Not Sole Compute)

#### Milestone 4.1: Structured Data Output
- [ ] Pack computed results into structured format:
  ```json
  {
    "camera_velocity": [0.2, 0.1, 0.0],
    "camera_rotation": {"axis": [0,1,0], "angle": 0.05},
    "objects": [
      {"bbox": [100,100,200,200], "velocity": [1.2, 0.0, 0.0]}
    ],
    "depth_map": "base64...",
    "confidence": 0.85
  }
  ```
- [ ] Include uncertainty estimates

**Deliverable:** Self-contained measurement result (LLM optional)

#### Milestone 4.2: LLM Validation Prompt
- [ ] Send computed data + composite image to LLM
- [ ] Prompt: "Verify these computed measurements against visual evidence"
- [ ] LLM checks for gross errors, not primary computation
- [ ] Output: confidence boost or warning flags

**Deliverable:** LLM as sanity checker, not sole physics engine

#### Milestone 4.3: Anomaly Detection
- [ ] If LLM disagrees significantly with local computation, flag for review
- [ ] Compare computed velocity vs LLM-estimated velocity
- [ ] Large discrepancy → possible sensor failure or unusual scene

**Deliverable:** Robust measurement with cross-validation

---

### Phase 5: Advanced Geometry Applications

#### Milestone 5.1: 600-Cell Extended Lattice
- [ ] Use 600-cell (120 vertices) for finer orientation quantization
- [ ] Enable sub-degree rotation precision
- [ ] Implement cellular automaton on 600-cell for pattern detection

**Deliverable:** Higher precision orientation using larger polytope

#### Milestone 5.2: E8 Embedding for Multi-Sensor Fusion
- [ ] 8D state vector: [x, y, z, roll, pitch, yaw, scale, time]
- [ ] E8 lattice provides optimal 8D sampling
- [ ] Use E8 projection for dimensionality reduction to 2D visualization

**Deliverable:** High-dimensional sensor embedding

#### Milestone 5.3: Hopf Fibration for Phase Coherence
- [ ] Camera orientation defines point on S² (Bloch sphere)
- [ ] Fiber over that point is S¹ (phase circle)
- [ ] Use fiber phase for time synchronization across sensors
- [ ] Enables sub-frame interpolation

**Deliverable:** Phase-coherent multi-sensor timing

---

## Success Metrics

### Phase 1 Complete When:
- [ ] Orientation uses 24-cell quantization
- [ ] Rotation computed locally via rotor integration
- [ ] Optical flow computed without LLM

### Phase 2 Complete When:
- [ ] Log-polar transform runs in real-time
- [ ] Moiré interference measures rotation to <0.5° precision
- [ ] Velocity computed locally via nomogram logic

### Phase 3 Complete When:
- [ ] Kalman filter produces smooth 6-DOF trajectory
- [ ] Depth map generated from parallax
- [ ] Object motion separated from camera motion locally

### Phase 4 Complete When:
- [ ] LLM receives computed data, not just images
- [ ] LLM validates rather than computes
- [ ] System works offline (with reduced confidence)

### Phase 5 Complete When:
- [ ] 600-cell provides sub-degree precision
- [ ] E8 embedding demonstrates benefit for sensor fusion
- [ ] Hopf fibration enables phase-locked multi-sensor

---

## Architecture Diagram (Target State)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          KINETICLENS TARGET ARCHITECTURE                │
└─────────────────────────────────────────────────────────────────────────┘

                              ┌──────────────────┐
                              │   SENSOR INPUT   │
                              │  Camera/IMU/GPS  │
                              └────────┬─────────┘
                                       │
         ┌─────────────────────────────┼─────────────────────────────┐
         │                             │                             │
         ▼                             ▼                             ▼
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│  ACCELEROMETER  │         │   GYROSCOPE     │         │     CAMERA      │
│                 │         │                 │         │                 │
│ gravity vector  │         │ angular rate    │         │ video frames    │
└────────┬────────┘         └────────┬────────┘         └────────┬────────┘
         │                           │                           │
         │                           │                           │
         ▼                           ▼                           ▼
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│ ORTHOGONAL      │         │ ROTOR           │         │ LOG-POLAR       │
│ FILTER          │         │ INTEGRATOR      │         │ TRANSFORM       │
│                 │         │                 │         │                 │
│ → clean accel   │         │ → orientation   │         │ → rot/scale     │
│ → efficiency    │         │   (no gimbal)   │         │   separated     │
└────────┬────────┘         └────────┬────────┘         └────────┬────────┘
         │                           │                           │
         └───────────────────────────┼───────────────────────────┘
                                     │
                                     ▼
                          ┌─────────────────────┐
                          │  24-CELL QUANTIZER  │
                          │                     │
                          │ snap orientation to │
                          │ nearest Hurwitz     │
                          │ quaternion unit     │
                          └──────────┬──────────┘
                                     │
                                     ▼
                          ┌─────────────────────┐
                          │  MOIRÉ COMPUTER     │
                          │                     │
                          │ IMU grating ⊗ video │
                          │ = object motion     │
                          └──────────┬──────────┘
                                     │
                                     ▼
                          ┌─────────────────────┐
                          │  KALMAN FUSION      │
                          │                     │
                          │ rotor-space filter  │
                          │ smooth trajectory   │
                          └──────────┬──────────┘
                                     │
                    ┌────────────────┴────────────────┐
                    │                                 │
                    ▼                                 ▼
         ┌─────────────────┐               ┌─────────────────┐
         │ LOCAL OUTPUT    │               │ LLM VALIDATION  │
         │                 │               │                 │
         │ velocity: 2.3m/s│  ──────────▶  │ "Measurement    │
         │ direction: 45°  │               │  appears        │
         │ confidence: 85% │  ◀──────────  │  consistent"    │
         └─────────────────┘               └─────────────────┘
```

---

## Immediate Next Steps

1. **Start with Milestone 1.1** - Wire up 24-cell orientation quantization
2. **Add optical flow** (Milestone 1.3) - Get local motion from video
3. **Build moiré measurement** (Milestone 2.2) - Use interference for precision
4. **Create Kalman filter** (Milestone 3.1) - Fuse everything locally

This transforms the system from "send pictures to LLM" to "compute locally, validate with LLM."

---

## Technical Dependencies

- **WebGL** for log-polar transform and moiré computation (GPU acceleration)
- **Web Workers** for Kalman filter (off main thread)
- **TypeScript strict mode** for rotor type safety
- **Canvas 2D** fallback for devices without WebGL

---

## File Structure (Proposed)

```
services/
├── GeometricAlgebra.ts      # ✅ EXISTS - rotors, spinors, bivectors
├── E8Projection.ts          # ✅ EXISTS - polytopes, lattices
├── MoireEngine.ts           # ✅ EXISTS - gratings, interference
├── PhillipsGateSimulator.ts # ✅ EXISTS - quantum-inspired ops
│
├── OrientationQuantizer.ts  # 🆕 NEW - 24-cell snap + SLERP
├── RotorIMU.ts              # 🆕 NEW - rotor-based gyro integration
├── OpticalFlow.ts           # 🆕 NEW - Lucas-Kanade + flow decomp
├── LogPolarTransform.ts     # 🆕 NEW - WebGL polar resampling
├── MoireMeasurement.ts      # 🆕 NEW - interference → angle
├── KalmanFusion.ts          # 🆕 NEW - rotor-space filter
└── MeasurementOutput.ts     # 🆕 NEW - structured JSON output
```

---

*This roadmap transforms KineticLens from a "visual prompt generator" into a "geometric computation engine with LLM validation."*
