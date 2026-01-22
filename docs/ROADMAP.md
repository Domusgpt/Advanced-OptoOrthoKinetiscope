# Roadmap: Evolution & Integration

This document outlines the strategy for evolving KineticLens from a passive analysis tool into an active, agentic system.

## Phase 1: Enhanced Visual Intelligence (Current)
- [x] Tri-Phase Photography
- [x] Parallax Gyro-Epitaxy
- [ ] **Next Step:** Stereo-Photogrammetry using Dual-Lens cameras (iPhone Pro / Android Flagships) to capture true depth maps instead of estimating via Epitaxy.

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
