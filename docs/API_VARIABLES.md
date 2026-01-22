# API & Variable Reference
**Physics Engine Constants & Primitives**

## 1. Physics Constants (`cpeMath.ts`)

| Constant | Value | Description |
| :--- | :--- | :--- |
| `alphaGravity` | `0.92` | Coefficient for the High-Pass Gravity Filter. Determines how "heavy" the virtual gimbal feels. |
| `alphaTrend` | `0.15` | Learning rate for the Principal Axis. Higher = Snappier rail; Lower = Smoother rail. |
| `orthogonalDampening` | `0.1` | Multiplier for noise vectors. We reject 90% of orthogonal force. |
| `FOV_CONSTANT` | `1000` | Perspective projection constant for the 4D Lattice. |

## 2. Capture Point Primitives (`types.ts`)

The `CapturePoint` interface is the atomic unit of the system.

```typescript
interface CapturePoint {
  // TIMING
  timestamp: number;        // Unix Epoch
  relativeTime: number;     // ms from T0
  shutterInterval: number;  // ms duration of this specific frame exposure

  // INERTIAL
  acceleration: Vector3;    // Filtered Linear Acceleration (m/s^2)
  rotationRate: Vector3;    // Angular Velocity (deg/s)
  orientation: Quaternion;  // Absolute orientation (Fused)
  tiltAngle: number;        // Pitch relative to gravity (Degrees)

  // QUALITY METRICS
  vectorEfficiency: number; // 0.0 - 1.0 (Signal-to-Noise Ratio)
  moireVariance: number;    // Derived Jerk metric (High Freq Vibration)

  // OPTICAL
  zoomLevel: number;        // 1.0 or 2.0
  spectralPhase: 'RED' | 'GREEN' | 'BLUE'; // Logic channel for timeline
}
```

## 3. Gemini Injection Variables (`geminiService.ts`)

These variables are dynamically computed and injected into the System Instruction prompt.

### `avgEff` (Average Efficiency)
*   **Source:** Average of `vectorEfficiency` across all burst frames.
*   **Prompt Logic:**
    *   `> 0.6`: "Condition: STABLE. The Gold Lattice is a RELIABLE inertial reference."
    *   `< 0.6`: "Condition: UNSTABLE... Rely on Cyan Epipolar Flow."
*   **Purpose:** Switches the AI's trust model between Inertial and Optical data.

### `zShift` (Z-Axis Delta)
*   **Source:** `abs(accel_z_end - accel_z_start)`
*   **Purpose:** Triggers "Volumetric" analysis mode if the user walked forward/backward during capture.

### `gyroStress` (Rotational Energy)
*   **Source:** Magnitude of `rotationRate`.
*   **Purpose:** If high, the AI is instructed to ignore "Ghosting" in the composite, attributing it to motion blur rather than structural vibration.
