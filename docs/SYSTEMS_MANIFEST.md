# KineticLens Systems Manifest
**Subsystem Technical Specifications**

## 1. Subsystem: Parallax Gyro-Epitaxy
**File:** `components/GhostView.tsx`

**Purpose:** To extract Z-axis (Depth) motion from a monocular camera sensor without LiDAR.

**Mechanism:**
1.  **Epitaxial Stacking:** We overlay frames T0, T1, and T2 with varying alpha channels (`0.3`, `0.5`, `1.0`).
2.  **Gyro Projection:** We project a virtual "Reticle" onto the canvas for each frame, driven *solely* by the gyroscope Quaternion.
3.  **Differential Analysis:**
    *   If the **Visual Image** scales up (Zoom) but the **Gyro Reticle** stays the same size -> **Subject approached Camera**.
    *   If the **Visual Image** scales up AND the **Gyro Reticle** scales up (due to convex projection math) -> **Camera moved closer to Subject**.

**Code Reference:** `drawReticle` function in `GhostView.tsx` implements the `convexityFactor = Math.sin(pitch) + (zForce * 0.4)` formula to simulate lens breathing induced by physical movement.

## 2. Subsystem: Hyper-Vernier Scales
**File:** `services/imageCompositor.ts`

**Purpose:** To provide the AI with a reliable measurement standard that adapts to sensor noise.

**Mechanism:**
*   **Grid Generation:** In `drawHyperLatticeInterference`, we draw the `get24CellVertices()` projection.
*   **Vernier Shift:** If the accelerometer detects "Jerk" (Change in acceleration), we calculate a `offsetMagnitude`.
*   **Visual Output:** We render a second, fainter grid offset by this magnitude.
*   **AI Logic:** The distance between the "Main Grid" line and the "Ghost Grid" line acts as a visual scale bar for **Uncertainty**. The AI measures this distance in pixels to determine its error margin.

## 3. Subsystem: Rational Dynamics Engine
**File:** `App.tsx` & `CaptureScreen.tsx`

**Purpose:** To optimize the timing of data capture.

**Mechanism:**
The system monitors the `stabilityScore` (derived from `kineticEnergy`).
*   **State:** `IDLE` -> `READY` threshold is `< 0.5 Joules` (normalized energy units).
*   **Burst Trigger:** When the user presses capture, the system checks `VectorEfficiency`.
    *   If `Efficiency < 40%`, it **Rejects** the trigger (and vibrates).
    *   If `Efficiency > 40%`, it fires the burst sequence.

This acts as a "Gatekeeper", ensuring that only high-quality data enters the expensive LLM analysis pipeline.
