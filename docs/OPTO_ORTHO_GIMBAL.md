# Protocol Specification: OptoOrtho-Gimbal Telemetry Stabilization
**Version 2.1 | Technical Reference Manual**

## 1. Executive Summary
The **OptoOrtho-Gimbal** (Optical-Orthogonal Software Gimbal) is a proprietary sensor fusion protocol implemented within the `OrthogonalFilter` class (`services/cpeMath.ts`). It addresses the fundamental stochastic nature of handheld telemetry by applying **Real-Time Principal Component Analysis (PCA)** to the accelerometer stream.

Unlike low-pass filters (Moving Average/Kalman) which introduce temporal latency ($>100ms$), the OptoOrtho protocol provides **Zero-Latency Signal Pass-Through** along the user's intended trajectory while selectively dampening orthogonal jitter.

## 2. Kernel Architecture
The core logic resides in `services/cpeMath.ts`. The system operates on a 60Hz loop driven by the `DeviceMotion` API.

### 2.1 State Variables
The `OrthogonalFilter` class maintains specific inertial state vectors:
*   **`gravity` (`Vector3`):** A low-pass filtered estimation of the gravity vector (down).
    *   *Code Reference:* `this.alphaGravity = 0.92`. This high decay rate ensures the system adapts quickly to orientation changes while filtering out transient acceleration.
*   **`principalAxis` (`Vector3`):** The computed "Rail" of intended motion.
    *   *Code Reference:* `this.alphaTrend = 0.15`. This aggressive learning rate allows the system to lock onto "Snap" motions (sudden directional changes) within ~100ms.

### 2.2 Vector Decomposition Algorithm (`update` method)
The raw acceleration vector $\vec{A}_{raw}$ is processed through a multi-stage pipeline:

**Stage 1: Gravity Subtraction**
We isolate linear acceleration by subtracting the gravity component.
```typescript
const linear = {
    x: rawAccel.x - this.gravity.x,
    y: rawAccel.y - this.gravity.y,
    z: rawAccel.z - this.gravity.z
};
```

**Stage 2: Principal Axis Determination**
We calculate the rolling average of the *direction* of acceleration, normalizing it to unit length. This creates the unit vector $\hat{u}$ (The Rail).
```typescript
this.principalAxis = (1 - alpha) * prevAxis + alpha * linear;
const axis = normalize(this.principalAxis);
```

**Stage 3: Scalar Projection (The Dot Product)**
We verify how much of the current force aligns with the historical intent.
$$ s = \vec{A}_{linear} \cdot \hat{u} $$
*   *Code Reference:* `const scalarProjection = this.dot(linear, axis);`

**Stage 4: Orthogonal Rejection**
We decompose the vector into Parallel ($\vec{A}_{\parallel}$) and Orthogonal ($\vec{A}_{\perp}$) components.
1.  **Parallel (Signal):** $\vec{A}_{\parallel} = s \times \hat{u}$. Passed through at **100% gain**.
2.  **Orthogonal (Noise):** $\vec{A}_{\perp} = \vec{A}_{linear} - \vec{A}_{\parallel}$. Dampened by **90%**.
    *   *Code Reference:* `const cleanOrthogonal = this.scale(orthogonalComponent, 0.1);`

## 3. Efficiency Metric Derivation
The system exposes a `currentEfficiency` float (0.0 - 1.0) used by the UI/HUD. This is not an arbitrary score; it is a direct energy ratio.

$$ E_{efficiency} = 1.0 - \left( \frac{||\vec{A}_{\perp}||}{||\vec{A}_{linear}||} \right) $$

*   **Implementation:** Located in `update()`.
*   **Semantic Meaning:** An efficiency of **0.8** implies that 80% of the kinetic energy exerted by the user contributed to the intended trajectory, while 20% was wasted on orthogonal vibration (jitter).
*   **Thresholds:**
    *   `> 0.8`: Professional Stability (Cyan HUD).
    *   `< 0.5`: High Noise (Red Spikes in HUD).

## 4. 4D Hyper-Lattice Projection
To visualize this data, we project a 4-Dimensional structure (The 24-Cell / Icositetrachoron) onto the 2D canvas.

*   **Rotation:** We rotate the 4D vertices using a standard 3D Quaternion (`orientation`) plus a "Hyper-Rotation" driven by the `cleanVector` acceleration.
    *   *Code Reference:* `rotate4D_Hyper` function.
    *   *Effect:* Physical acceleration bends the lattice in the W-dimension, causing visual distortion that mirrors the sensor stress.

## 5. Integration with Gemini
The `cleanVector` is NOT just for display. It is recorded in the `CapturePoint` struct (`types.ts`) and packaged into the JSON sent to the LLM. The AI uses this "Clean" vector to calculate the `Lattice Shift` variable in the Geometric Proof.
