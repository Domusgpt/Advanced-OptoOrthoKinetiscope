# KineticLens Sensor Fusion Architecture

## Hypersphere Lattice Quaternion Framework

This document outlines the development roadmap for integrating multi-sensor fusion with the 24-cell (icositetrachoron) Hurwitz quaternion lattice.

---

## Core Concept: Orientation Lives on S³

Orientations are unit quaternions → points on the 3-sphere S³.
The 24-cell's vertices form a **discrete sampling lattice** on S³.

```
    S³ (3-sphere of unit quaternions)
           ╱╲
          ╱  ╲
    Raw IMU   24-Cell Lattice
    (noisy)   (24 discrete orientations)
         ╲    ╱
          ╲  ╱
       Quantized
       Orientation
```

### Why 24-Cell?

1. **Hurwitz integers** - The 24 vertices are exactly the unit Hurwitz quaternions
2. **Optimal packing** - Maximum angular separation between neighbors (~60°)
3. **Group structure** - Forms the binary tetrahedral group 2T ≅ SL(2,3)
4. **Closed under multiplication** - Composing rotations stays on lattice

---

## Phase 1: Adaptive Resolution (Current Sprint)

### Problem
Fixed 0.5x downsampling works for normal motion but loses precision for small movements.

### Solution: Motion-Adaptive Scaling

```javascript
// Adaptive scale based on detected motion magnitude
function getAdaptiveScale() {
    const motionMagnitude = state.flowMagnitude;

    if (motionMagnitude < 1) {
        // Small motion: need high precision
        return 1.0;  // Full resolution
    } else if (motionMagnitude < 5) {
        // Medium motion: balanced
        return 0.75;
    } else {
        // Fast motion: speed matters more
        return 0.5;
    }
}
```

### Implementation Tasks
- [ ] Add `state.adaptiveScale` that updates each frame
- [ ] Smoothly transition between scales (avoid jarring changes)
- [ ] Display current processing resolution in UI

---

## Phase 2: Accelerometer Scale Calibration

### Problem
Optical flow gives motion **direction** but not absolute **scale**.
Current workaround: user enters scene distance manually.

### Solution: Use Accelerometer as Ground Truth

```
Accelerometer → absolute acceleration (m/s²)
                     ↓
              double integrate
                     ↓
              displacement (m)
                     ↓
              compare with flow
                     ↓
              calibrate pixels_per_meter
```

### Algorithm

```javascript
const accelCalibration = {
    velocityFromAccel: { x: 0, y: 0, z: 0 },
    displacement: { x: 0, y: 0, z: 0 },
    calibrationSamples: [],
    pixelsPerMeter: 400,  // Will be refined

    update(accel, dt) {
        // Remove gravity (use orientation to rotate gravity out)
        const gravity = rotateVectorByQuat([0, 0, 9.81], state.orientation);
        const linearAccel = {
            x: accel.x - gravity[0],
            y: accel.y - gravity[1],
            z: accel.z - gravity[2]
        };

        // Integrate to get velocity
        this.velocityFromAccel.x += linearAccel.x * dt;
        this.velocityFromAccel.y += linearAccel.y * dt;

        // High-pass filter to reduce drift
        this.velocityFromAccel.x *= 0.98;
        this.velocityFromAccel.y *= 0.98;
    },

    calibrate(flowVelocity, accelVelocity) {
        // When both sensors agree on direction, use accel magnitude
        // to calibrate flow's scale
        const flowMag = Math.sqrt(flowVelocity.x**2 + flowVelocity.y**2);
        const accelMag = Math.sqrt(accelVelocity.x**2 + accelVelocity.y**2);

        if (flowMag > 0.1 && accelMag > 0.1) {
            const ratio = accelMag / flowMag;
            this.calibrationSamples.push(ratio);

            // Use median for robustness
            if (this.calibrationSamples.length > 30) {
                this.calibrationSamples.sort((a, b) => a - b);
                this.pixelsPerMeter = this.calibrationSamples[15];
                this.calibrationSamples = [];
            }
        }
    }
};
```

### Implementation Tasks
- [ ] Add gravity compensation using current orientation
- [ ] Implement accelerometer velocity integration with drift correction
- [ ] Add calibration routine that runs during movement
- [ ] Store calibration in localStorage for persistence

---

## Phase 3: Quaternion-Space Kalman Filter

### Problem
Multiple sensors give different estimates. Need optimal fusion.

### Key Insight
Standard Kalman filter works in Euclidean space.
**Orientations live on S³** (a curved manifold).

### Solution: Multiplicative Extended Kalman Filter (MEKF)

```
State vector lives on S³ (quaternion)
Error/innovation lives in tangent space (3D vector)

        S³ (quaternion manifold)
            ↑
    exp map │ ↓ log map
            │
    R³ (tangent space at current estimate)
    └── Standard Kalman operations here
```

### State Representation

```javascript
const kalmanState = {
    // Orientation as quaternion (lives on S³)
    q: [1, 0, 0, 0],

    // Angular velocity (tangent space)
    omega: [0, 0, 0],

    // Gyroscope bias (tangent space)
    bias: [0, 0, 0],

    // Covariance (6x6 in tangent space)
    P: identity6x6(),

    // Process noise
    Q: {
        omega: 0.01,    // rad/s
        bias: 0.0001    // rad/s per second
    },

    // Measurement noise
    R: {
        gyro: 0.1,      // rad/s
        accel: 0.5,     // m/s² (for gravity direction)
        flow: 2.0       // pixels (for rotation from flow)
    }
};
```

### Quaternion Operations for Kalman

```javascript
// Exponential map: tangent vector → quaternion
function expMap(v) {
    const angle = Math.sqrt(v[0]**2 + v[1]**2 + v[2]**2);
    if (angle < 1e-6) return [1, 0, 0, 0];
    const s = Math.sin(angle/2) / angle;
    return [Math.cos(angle/2), v[0]*s, v[1]*s, v[2]*s];
}

// Log map: quaternion → tangent vector
function logMap(q) {
    const sinHalfAngle = Math.sqrt(q[1]**2 + q[2]**2 + q[3]**2);
    if (sinHalfAngle < 1e-6) return [0, 0, 0];
    const angle = 2 * Math.atan2(sinHalfAngle, q[0]);
    const s = angle / sinHalfAngle;
    return [q[1]*s, q[2]*s, q[3]*s];
}

// Prediction step
function kalmanPredict(dt) {
    // Predict orientation: q = q ⊗ exp(ω * dt)
    const deltaQ = expMap([
        (kalmanState.omega[0] - kalmanState.bias[0]) * dt,
        (kalmanState.omega[1] - kalmanState.bias[1]) * dt,
        (kalmanState.omega[2] - kalmanState.bias[2]) * dt
    ]);
    kalmanState.q = quatMultiply(kalmanState.q, deltaQ);
    kalmanState.q = quatNormalize(kalmanState.q);

    // Predict covariance: P = F*P*F' + Q
    // (F is identity for orientation, Jacobian for bias coupling)
}

// Update from gyroscope
function kalmanUpdateGyro(gyroMeasurement) {
    kalmanState.omega = gyroMeasurement;
    // Innovation in tangent space
    // Standard Kalman update for omega and bias
}

// Update from accelerometer (gravity direction)
function kalmanUpdateAccel(accelMeasurement) {
    // Expected gravity in body frame
    const expectedGravity = rotateVectorByQuatInverse([0, 0, 1], kalmanState.q);

    // Measured gravity direction (normalized accel)
    const accelNorm = normalize(accelMeasurement);

    // Innovation: rotation between expected and measured
    const innovation = crossProduct(expectedGravity, accelNorm);

    // Update orientation via tangent space correction
    const correction = expMap(kalmanGain.dot(innovation));
    kalmanState.q = quatMultiply(correction, kalmanState.q);
}
```

### Implementation Tasks
- [ ] Implement exp/log maps for S³
- [ ] Create 6x6 covariance matrix representation
- [ ] Implement prediction step with proper Jacobians
- [ ] Implement gyro update
- [ ] Implement accelerometer update (gravity)
- [ ] Implement optical flow rotation update

---

## Phase 4: 24-Cell Lattice Integration

### Concept: Quantized Orientation Filtering

Instead of continuous quaternion filtering, **snap to nearest 24-cell vertex** with hysteresis.

```
Continuous          Lattice              Filtered
Orientation    →    Quantize    →       Output
    ↓                   ↓                  ↓
[w,x,y,z]        vertex index         stable
                    (0-23)            orientation
```

### Benefits
1. **Noise rejection** - Small jitters don't change output
2. **Discrete states** - Easier to reason about
3. **Natural hysteresis** - Must cross ~30° to change vertex

### Algorithm with Hysteresis

```javascript
const latticeFilter = {
    currentVertex: 0,
    vertexConfidence: 1.0,
    switchThreshold: 0.8,  // Must be 80% closer to switch

    update(orientation) {
        const nearest = findNearestVertex(orientation);
        const distToNearest = quatDistance(orientation, HURWITZ[nearest]);
        const distToCurrent = quatDistance(orientation, HURWITZ[this.currentVertex]);

        // Hysteresis: only switch if significantly closer
        if (distToNearest < distToCurrent * this.switchThreshold) {
            this.currentVertex = nearest;
        }

        // Confidence based on distance to current vertex
        this.vertexConfidence = 1 - (distToCurrent / (Math.PI / 3));

        return this.currentVertex;
    }
};
```

### Lattice-Based Motion Classification

```javascript
// Each 24-cell vertex has 8 neighbors
// Motion between vertices = specific rotation
const VERTEX_TRANSITIONS = precomputeTransitions();

function classifyMotion(fromVertex, toVertex) {
    if (fromVertex === toVertex) return 'STABLE';

    const transition = VERTEX_TRANSITIONS[fromVertex][toVertex];
    if (transition.isNeighbor) {
        return {
            type: 'SMALL_ROTATION',
            axis: transition.axis,
            angle: 60  // degrees (approx)
        };
    } else {
        return {
            type: 'LARGE_ROTATION',
            path: transition.shortestPath  // sequence of vertices
        };
    }
};
```

### Implementation Tasks
- [ ] Precompute 24-cell adjacency graph
- [ ] Implement hysteresis-based vertex tracking
- [ ] Add vertex transition detection
- [ ] Classify motion types based on lattice traversal

---

## Phase 5: Multi-Rate Sensor Fusion

### Problem
- IMU: 60-100 Hz (fast but drifty)
- Camera: 30 Hz (slow but drift-free)
- Need to combine optimally

### Solution: Asynchronous Kalman Updates

```
Time →  |----|----|----|----|----|----|----|----|
IMU     ↓    ↓    ↓    ↓    ↓    ↓    ↓    ↓    (100 Hz)
Camera            ↓              ↓              (30 Hz)

Each sensor triggers its own Kalman update
Prediction runs continuously between updates
```

### Implementation

```javascript
const multiRateFusion = {
    lastIMUTime: 0,
    lastCameraTime: 0,
    imuBuffer: [],  // Buffer IMU between camera frames

    onIMU(gyro, accel, timestamp) {
        const dt = (timestamp - this.lastIMUTime) / 1000;
        this.lastIMUTime = timestamp;

        // Always run prediction
        kalmanPredict(dt);

        // Update with gyro
        kalmanUpdateGyro(gyro);

        // Update with accel (gravity)
        if (isStationary(gyro, accel)) {
            kalmanUpdateAccel(accel);
        }

        // Buffer for camera frame correlation
        this.imuBuffer.push({ gyro, accel, timestamp });
        if (this.imuBuffer.length > 10) this.imuBuffer.shift();
    },

    onCamera(flow, timestamp) {
        const dt = (timestamp - this.lastCameraTime) / 1000;
        this.lastCameraTime = timestamp;

        // Compute rotation from flow field divergence/curl
        const rotationFromFlow = estimateRotationFromFlow(flow);

        // Update Kalman with camera observation
        kalmanUpdateFlow(rotationFromFlow);

        // Correlate with buffered IMU to detect ego vs object motion
        const predictedFlow = integrateIMUBuffer(this.imuBuffer, dt);
        const flowResidual = subtractFlow(flow, predictedFlow);

        // flowResidual = object motion (not camera)
        state.objectMotion = flowResidual;
    }
};
```

### Smart Polling / Adaptive Sampling

```javascript
const adaptiveSampling = {
    motionState: 'STATIONARY',  // STATIONARY, MOVING, FAST
    skipFrames: 0,

    shouldProcessFrame() {
        if (this.motionState === 'FAST') {
            return true;  // Process every frame
        } else if (this.motionState === 'MOVING') {
            this.skipFrames = (this.skipFrames + 1) % 2;
            return this.skipFrames === 0;  // Every other frame
        } else {
            this.skipFrames = (this.skipFrames + 1) % 4;
            return this.skipFrames === 0;  // Every 4th frame
        }
    },

    updateMotionState(gyroMagnitude, flowMagnitude) {
        const motion = Math.max(gyroMagnitude, flowMagnitude);

        if (motion > 10) {
            this.motionState = 'FAST';
        } else if (motion > 2) {
            this.motionState = 'MOVING';
        } else {
            this.motionState = 'STATIONARY';
            // Use stationary time for bias calibration
            this.calibrateBias();
        }
    },

    calibrateBias() {
        // When stationary, learn gyro bias from raw readings
        // (true angular velocity should be zero)
    }
};
```

### Implementation Tasks
- [ ] Add IMU event buffering
- [ ] Implement asynchronous Kalman updates
- [ ] Add flow-based rotation estimation
- [ ] Implement ego-motion prediction from IMU
- [ ] Add motion state detection
- [ ] Implement adaptive frame skipping

---

## Phase 6: Flow Field Decomposition

### Problem
Raw optical flow mixes:
- Translation (camera moving sideways)
- Rotation (camera turning)
- Expansion (camera moving forward)
- Independent objects

### Solution: Decompose Flow Field

```
                    Flow Field
                        │
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
   Translation      Rotation        Expansion
   (uniform)        (curl)          (divergence)
        │               │               │
        ↓               ↓               ↓
   Lateral vel     Angular vel     Forward vel
```

### Mathematical Decomposition

```javascript
function decomposeFlowField(flowField, cx, cy, focalLength) {
    let sumTx = 0, sumTy = 0;     // Translation
    let sumCurl = 0;               // Rotation
    let sumDiv = 0;                // Expansion
    let count = 0;

    for (const f of flowField) {
        if (f.quality < 0.4) continue;

        const rx = f.x - cx;  // Relative to center
        const ry = f.y - cy;
        const r2 = rx*rx + ry*ry;

        // Translation component (uniform)
        sumTx += f.dx;
        sumTy += f.dy;

        // Curl (rotation around optical axis)
        // curl = (∂vy/∂x - ∂vx/∂y) ≈ (rx*fy - ry*fx) / r²
        if (r2 > 100) {
            sumCurl += (rx * f.dy - ry * f.dx) / r2;
        }

        // Divergence (expansion/contraction)
        // div = ∂vx/∂x + ∂vy/∂y ≈ (rx*fx + ry*fy) / r²
        if (r2 > 100) {
            sumDiv += (rx * f.dx + ry * f.dy) / r2;
        }

        count++;
    }

    if (count === 0) return null;

    return {
        // Lateral translation (m/s)
        translation: {
            x: (sumTx / count) / focalLength,
            y: (sumTy / count) / focalLength
        },
        // Rotation around Z (rad/s)
        rotation: sumCurl / count,
        // Forward velocity (m/s) - requires depth
        expansion: sumDiv / count,
        // Time to collision (if expanding)
        ttc: sumDiv > 0.01 ? focalLength / sumDiv : Infinity
    };
}
```

### Implementation Tasks
- [ ] Implement flow field decomposition
- [ ] Add rotation estimation from curl
- [ ] Add expansion/TTC calculation
- [ ] Separate translation from rotation in velocity estimate
- [ ] Display decomposed components in UI

---

## Development Timeline

### Sprint 1 (Current)
- [x] Basic optical flow
- [x] Feature tracking with trails
- [x] Ego-motion separation
- [x] Data logging/export
- [ ] Adaptive resolution

### Sprint 2
- [ ] Accelerometer calibration
- [ ] Gravity compensation
- [ ] Auto scale factor

### Sprint 3
- [ ] Quaternion Kalman filter
- [ ] Multi-rate fusion
- [ ] Smart polling

### Sprint 4
- [ ] 24-cell hysteresis
- [ ] Lattice motion classification
- [ ] Flow decomposition

### Sprint 5
- [ ] Full integration
- [ ] Performance optimization
- [ ] Calibration wizard UI

---

## File Structure

```
services/
├── OrientationQuantizer.ts    # 24-cell lattice operations
├── RotorIMU.ts                # Quaternion IMU integration
├── OpticalFlow.ts             # Flow computation
├── FlowDecomposition.ts       # NEW: Translation/rotation/expansion
├── KalmanFusion.ts            # Quaternion-space Kalman
├── MultiRateFusion.ts         # NEW: Async sensor fusion
├── AccelCalibration.ts        # NEW: Scale calibration
└── KineticLens.ts             # Unified API

docs/
├── DEVELOPMENT_ROADMAP.md     # Original roadmap
├── SENSOR_FUSION_ARCHITECTURE.md  # This document
└── kinetic-lens-test.html     # Test application
```

---

## Key Equations Reference

### Pinhole Camera Model
```
pixel_displacement = (world_velocity × focal_length) / distance
world_velocity = (pixel_displacement × distance) / focal_length
```

### Quaternion Kinematics
```
q̇ = ½ × q ⊗ [0, ω]
q(t+dt) = q(t) ⊗ exp(ω × dt / 2)
```

### Kalman Gain
```
K = P × H' × (H × P × H' + R)⁻¹
```

### 24-Cell Vertex Distance
```
d(q1, q2) = 2 × arccos(|q1 · q2|)
```

### Flow Decomposition
```
v(x,y) = T + ω×r + (Z/f)×r
       = [Tx, Ty] + ωz×[−y, x] + expansion×[x, y]
```
