# KineticLens Operational Field Manual
**Ref: KL-OPS-2.1**

## 1. System Overview
KineticLens is a precision motion analysis instrument. Unlike standard camera apps, it requires operator discipline to achieve valid results. The system relies on the **OptoOrtho-Gimbal** to synchronize optical data with inertial telemetry.

## 2. HUD Symbology & Operator Feedback
The Heads-Up Display (HUD) provides real-time feedback on the quality of the data being buffered.

### 2.1 The Principal Axis Rail (Dotted Line)
*   **Appearance:** A dotted line extending across the screen.
*   **Meaning:** This represents the **Computed Trajectory** of the device. The math engine has determined you are trying to move along this path.
*   **Operator Action:** Align your physical motion with this rail.

### 2.2 The Force Vector (Solid Line)
*   **Appearance:** A solid vector originating from the reticle center.
*   **Meaning:** Your **Instantaneous Acceleration**.
*   **Alignment:**
    *   **Cyan (Locked):** The Vector aligns with the Rail. Efficiency > 80%. *Proceed with capture.*
    *   **Red Spikes (Orthogonal Noise):** You are shaking the device perpendicular to the motion. *Stabilize grip.*

### 2.3 Status Indicators
*   **Lattice Integrity (Hexagon):** Represents the geometric consistency of the 4D projection. If this drops, your rotation sensors are drifting. Perform a figure-8 calibration.
*   **Vector Efficiency (Shield):** The ratio of Signal (Trajectory) to Noise (Jitter).
    *   **< 40%:** Haptic warning triggers. Capture is disabled to prevent data pollution.

## 3. Capture Protocols

### Mode A: TRAJECTORY (Standard)
**Use Case:** Linear motion analysis (Vehicles, Sprinters).
**Sequence:**
1.  System captures Frame 0 (Wide).
2.  System captures Frame 1 (Wide) at `t+delay`.
3.  System captures Frame 2 (Wide) at `t+2*delay`.
*   *Note:* The `delay` is dynamic, calculated based on the subject's estimated speed from the accelerometer. Fast movement = Shorter shutter intervals (down to 20ms).

### Mode B: MATRIX BURST (Forensic)
**Use Case:** Structural vibration, machinery analysis, complex interaction.
**Sequence (Automated 1.5s Scan):**
1.  **Phase 1 (Context):** 3x Wide frames. Establishes the scene.
2.  **Phase 2 (Interferometry):** 3x Telephoto (2x) frames. The lens physically zooms or crops. This amplifies vibration data for the AI.
3.  **Phase 3 (Structure):** 3x Active frames. Exposure is biased -1.5EV and Torch fires to highlight structural edges.

## 4. Troubleshooting

### "RAIL UNLOCKED" Warning
*   **Cause:** Orthogonal variance is too high. The system cannot determine your intended path.
*   **Fix:** Bring elbows in to torso. Pivot from the hips, not the shoulders.

### "HIGH MOIRE VARIANCE" in Analysis
*   **Cause:** High-frequency vibration (e.g., mounting on a running engine or vehicle dashboard).
*   **Fix:** Use a dampening mount. The AI has detected the vibration and flagged the data as "Noisy", reducing confidence scores.

### "LATTICE GHOSTING" in Composite
*   **Observation:** The Gold Grid in the final image appears double-exposed.
*   **Meaning:** The shutter speed was slower than the sensor readout latency.
*   **Fix:** Increase environmental lighting. The system requires >200 Lux to match sensor polling rates.
