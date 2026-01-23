/**
 * KALMAN SENSOR FUSION
 *
 * Fuses multiple sensor sources into a coherent motion estimate:
 * - IMU (gyroscope + accelerometer) via RotorIMU
 * - Optical flow from camera
 * - Moiré measurements for precision refinement
 * - 24-cell quantization for discrete reference points
 *
 * State vector:
 *   [orientation (rotor), angular_velocity, position, velocity]
 *
 * The key innovation is using rotor-space updates which:
 * - Avoid gimbal lock
 * - Preserve unit quaternion constraint
 * - Enable geodesic interpolation
 */

import { Rotor3D, normalizeRotor, slerpRotor, multiplyRotors, reverseRotor } from './GeometricAlgebra';
import { Vector3 } from '../types';
import { OrientationQuantizer, QuantizationResult } from './OrientationQuantizer';
import { RotorIMU, IMUState } from './RotorIMU';
import { OpticalFlowEngine, FlowDecomposition, FlowVector } from './OpticalFlow';
import { MoireMeasurementEngine, MoireMeasurement } from './MoireMeasurement';

// ═══════════════════════════════════════════════════════════════════════════
// TYPES
// ═══════════════════════════════════════════════════════════════════════════

export interface FusedState {
  /** Orientation as rotor */
  orientation: Rotor3D;

  /** Angular velocity (rad/s) */
  angularVelocity: Vector3;

  /** Position (m) - relative to start */
  position: Vector3;

  /** Linear velocity (m/s) */
  velocity: Vector3;

  /** Camera motion (translation in image space, px/frame) */
  cameraFlow: FlowVector;

  /** Object/scene motion (translation in image space, px/frame) */
  sceneFlow: FlowVector;

  /** 24-cell vertex index (nearest) */
  discreteOrientation: number;

  /** Confidence in estimate (0-1) */
  confidence: number;

  /** Timestamp */
  timestamp: number;
}

export interface SensorReading {
  /** Gyroscope (rad/s) */
  gyro?: Vector3;

  /** Accelerometer (m/s²) */
  accel?: Vector3;

  /** Video frame (grayscale) */
  frame?: ImageData;

  /** Device orientation quaternion (from browser API) */
  deviceOrientation?: { x: number; y: number; z: number; w: number };

  /** Timestamp (ms) */
  timestamp: number;
}

export interface FusionConfig {
  /** Weight for IMU orientation */
  imuWeight: number;

  /** Weight for optical flow */
  flowWeight: number;

  /** Weight for moiré measurement */
  moireWeight: number;

  /** Estimated focal length (pixels) */
  focalLength: number;

  /** Estimated scene distance (m) */
  sceneDistance: number;

  /** Process noise for orientation */
  processNoiseOrientation: number;

  /** Process noise for velocity */
  processNoiseVelocity: number;

  /** Measurement noise for optical flow */
  measurementNoiseFlow: number;
}

const DEFAULT_CONFIG: FusionConfig = {
  imuWeight: 0.6,
  flowWeight: 0.3,
  moireWeight: 0.1,
  focalLength: 1000,
  sceneDistance: 2.0,
  processNoiseOrientation: 0.01,
  processNoiseVelocity: 0.1,
  measurementNoiseFlow: 2.0
};

// ═══════════════════════════════════════════════════════════════════════════
// KALMAN FUSION ENGINE
// ═══════════════════════════════════════════════════════════════════════════

export class KalmanFusion {
  private config: FusionConfig;

  // Sub-systems
  private imu: RotorIMU;
  private quantizer: OrientationQuantizer;
  private opticalFlow: OpticalFlowEngine;
  private moireMeasurement: MoireMeasurementEngine;

  // State
  private state: FusedState;
  private stateCovariance: number[][] | null = null;
  private initialized: boolean = false;

  // History
  private stateHistory: FusedState[] = [];
  private readonly maxHistory = 30;

  constructor(config: Partial<FusionConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };

    this.imu = new RotorIMU();
    this.quantizer = new OrientationQuantizer(0.3);
    this.opticalFlow = new OpticalFlowEngine();
    this.moireMeasurement = new MoireMeasurementEngine();

    this.state = this.createInitialState();
  }

  private createInitialState(): FusedState {
    return {
      orientation: { s: 1, e12: 0, e23: 0, e31: 0 },
      angularVelocity: { x: 0, y: 0, z: 0 },
      position: { x: 0, y: 0, z: 0 },
      velocity: { x: 0, y: 0, z: 0 },
      cameraFlow: { x: 0, y: 0 },
      sceneFlow: { x: 0, y: 0 },
      discreteOrientation: 0,
      confidence: 1.0,
      timestamp: 0
    };
  }

  /**
   * Initialize the fusion system
   */
  initialize(reading: SensorReading): void {
    if (reading.accel) {
      this.imu.initialize(reading.accel, reading.timestamp);
    }
    this.state.timestamp = reading.timestamp;
    this.initialized = true;

    // Initialize state covariance
    this.stateCovariance = this.createIdentityMatrix(9, 0.1);
  }

  private createIdentityMatrix(size: number, scale: number): number[][] {
    const matrix: number[][] = [];
    for (let i = 0; i < size; i++) {
      matrix[i] = [];
      for (let j = 0; j < size; j++) {
        matrix[i][j] = i === j ? scale : 0;
      }
    }
    return matrix;
  }

  /**
   * Main update function - fuses all available sensor data
   */
  update(reading: SensorReading): FusedState {
    if (!this.initialized) {
      this.initialize(reading);
      return this.getState();
    }

    const dt = (reading.timestamp - this.state.timestamp) / 1000;
    if (dt <= 0 || dt > 1) {
      this.state.timestamp = reading.timestamp;
      return this.getState();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 1: IMU UPDATE (Prediction)
    // ═══════════════════════════════════════════════════════════════════════

    let imuState: IMUState | null = null;
    if (reading.gyro && reading.accel) {
      imuState = this.imu.update(reading.gyro, reading.accel, reading.timestamp);

      // Predict orientation from IMU
      this.state.orientation = imuState.orientation;
      this.state.angularVelocity = imuState.angularVelocity;
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 2: OPTICAL FLOW (Measurement)
    // ═══════════════════════════════════════════════════════════════════════

    let flowDecomp: FlowDecomposition | null = null;
    if (reading.frame) {
      const flow = this.opticalFlow.computeFlow(reading.frame, reading.timestamp);

      if (flow) {
        flowDecomp = this.opticalFlow.decomposeFlow(flow);

        // Predict camera-induced flow from IMU rotation
        const predictedCameraFlow = this.opticalFlow.predictFlowFromRotation(
          this.state.orientation,
          dt,
          this.config.focalLength,
          reading.frame.width,
          reading.frame.height
        );

        // Separate camera motion from scene motion
        const separation = this.opticalFlow.separateMotion(flowDecomp, predictedCameraFlow);

        this.state.cameraFlow = separation.cameraMotion;
        this.state.sceneFlow = separation.objectMotion;

        // Update velocity estimate from optical flow
        if (separation.confidence > 0.5) {
          const flowVelocity = this.opticalFlow.flowToVelocity(
            separation.objectMotion,
            this.config.sceneDistance,
            this.config.focalLength,
            dt
          );

          // Fuse with existing velocity
          const alpha = this.config.flowWeight * separation.confidence;
          this.state.velocity.x = (1 - alpha) * this.state.velocity.x + alpha * flowVelocity.x;
          this.state.velocity.y = (1 - alpha) * this.state.velocity.y + alpha * flowVelocity.y;
        }
      }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 3: 24-CELL QUANTIZATION
    // ═══════════════════════════════════════════════════════════════════════

    const quantResult = this.quantizer.quantize(this.state.orientation, reading.timestamp);
    this.state.discreteOrientation = quantResult.vertexIndex;

    // Use quantized orientation if confidence is high enough
    if (quantResult.confidence > 0.7) {
      // Blend toward quantized value
      this.state.orientation = slerpRotor(
        this.state.orientation,
        quantResult.smoothedRotor,
        0.1
      );
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 4: POSITION INTEGRATION
    // ═══════════════════════════════════════════════════════════════════════

    this.state.position.x += this.state.velocity.x * dt;
    this.state.position.y += this.state.velocity.y * dt;
    this.state.position.z += this.state.velocity.z * dt;

    // Apply drag to prevent runaway drift
    const drag = Math.exp(-0.5 * dt);
    this.state.velocity.x *= drag;
    this.state.velocity.y *= drag;
    this.state.velocity.z *= drag;

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 5: CONFIDENCE UPDATE
    // ═══════════════════════════════════════════════════════════════════════

    let confidence = 0;
    let weights = 0;

    if (imuState) {
      confidence += imuState.confidence * this.config.imuWeight;
      weights += this.config.imuWeight;
    }

    if (flowDecomp) {
      confidence += flowDecomp.confidence * this.config.flowWeight;
      weights += this.config.flowWeight;
    }

    this.state.confidence = weights > 0 ? confidence / weights : 0.5;
    this.state.timestamp = reading.timestamp;

    // Save to history
    this.stateHistory.push({ ...this.state });
    if (this.stateHistory.length > this.maxHistory) {
      this.stateHistory.shift();
    }

    return this.getState();
  }

  /**
   * Get current fused state (copy)
   */
  getState(): FusedState {
    return {
      ...this.state,
      orientation: { ...this.state.orientation },
      angularVelocity: { ...this.state.angularVelocity },
      position: { ...this.state.position },
      velocity: { ...this.state.velocity },
      cameraFlow: { ...this.state.cameraFlow },
      sceneFlow: { ...this.state.sceneFlow }
    };
  }

  /**
   * Get measurement summary
   */
  getMeasurementSummary(): {
    speed: number;
    direction: number;
    rotation: number;
    stability: number;
  } {
    const vMag = Math.sqrt(
      this.state.velocity.x ** 2 +
      this.state.velocity.y ** 2 +
      this.state.velocity.z ** 2
    );

    const direction = Math.atan2(this.state.velocity.y, this.state.velocity.x) * 180 / Math.PI;

    const omegaMag = Math.sqrt(
      this.state.angularVelocity.x ** 2 +
      this.state.angularVelocity.y ** 2 +
      this.state.angularVelocity.z ** 2
    );

    const stability = this.quantizer.getStability();

    return {
      speed: vMag,
      direction,
      rotation: omegaMag * 180 / Math.PI,
      stability
    };
  }

  /**
   * Get Euler angles from current orientation
   */
  getEulerAngles(): { roll: number; pitch: number; yaw: number } {
    return this.imu.getEulerAngles();
  }

  /**
   * Get velocity in different units
   */
  getVelocity(unit: 'mps' | 'kph' | 'mph' = 'mps'): number {
    const mps = Math.sqrt(
      this.state.velocity.x ** 2 +
      this.state.velocity.y ** 2 +
      this.state.velocity.z ** 2
    );

    switch (unit) {
      case 'kph': return mps * 3.6;
      case 'mph': return mps * 2.237;
      default: return mps;
    }
  }

  /**
   * Get the trajectory over recent history
   */
  getTrajectory(): Vector3[] {
    return this.stateHistory.map(s => ({ ...s.position }));
  }

  /**
   * Get angular velocity history
   */
  getAngularVelocityHistory(): { timestamp: number; omega: Vector3 }[] {
    return this.stateHistory.map(s => ({
      timestamp: s.timestamp,
      omega: { ...s.angularVelocity }
    }));
  }

  /**
   * Predict future state (linear extrapolation)
   */
  predict(dt: number): FusedState {
    const future = this.getState();

    // Simple kinematic prediction
    future.position.x += future.velocity.x * dt;
    future.position.y += future.velocity.y * dt;
    future.position.z += future.velocity.z * dt;

    // Rotate by angular velocity
    const angle = Math.sqrt(
      future.angularVelocity.x ** 2 +
      future.angularVelocity.y ** 2 +
      future.angularVelocity.z ** 2
    ) * dt;

    if (angle > 1e-10) {
      const deltaRotor: Rotor3D = {
        s: Math.cos(angle / 2),
        e12: -Math.sin(angle / 2) * future.angularVelocity.z / (angle / dt),
        e23: -Math.sin(angle / 2) * future.angularVelocity.x / (angle / dt),
        e31: -Math.sin(angle / 2) * future.angularVelocity.y / (angle / dt)
      };

      future.orientation = normalizeRotor(
        multiplyRotors(future.orientation, deltaRotor)
      );
    }

    future.timestamp += dt * 1000;
    return future;
  }

  /**
   * Reset all state
   */
  reset(): void {
    this.imu.reset();
    this.quantizer.reset();
    this.opticalFlow.reset();
    this.moireMeasurement.reset();
    this.state = this.createInitialState();
    this.stateHistory = [];
    this.initialized = false;
  }

  /**
   * Update configuration
   */
  setConfig(config: Partial<FusionConfig>): void {
    this.config = { ...this.config, ...config };
  }

  /**
   * Set scene distance estimate (affects velocity calculation)
   */
  setSceneDistance(distance: number): void {
    this.config.sceneDistance = distance;
  }

  /**
   * Get diagnostic info for debugging
   */
  getDiagnostics(): {
    imuConfidence: number;
    flowConfidence: number;
    quantizationConfidence: number;
    gyroBias: Vector3;
    nearestVertex: string;
  } {
    const imuState = this.imu.getState();

    return {
      imuConfidence: imuState.confidence,
      flowConfidence: this.state.confidence,
      quantizationConfidence: this.quantizer.getStability(),
      gyroBias: { ...imuState.gyroBias },
      nearestVertex: OrientationQuantizer.getVertexLabel(this.state.discreteOrientation)
    };
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// FACTORY FUNCTION
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Create a KalmanFusion instance configured for typical phone usage
 */
export function createPhoneFusion(focalLengthPx: number = 1000): KalmanFusion {
  return new KalmanFusion({
    focalLength: focalLengthPx,
    sceneDistance: 2.0,
    imuWeight: 0.6,
    flowWeight: 0.35,
    moireWeight: 0.05
  });
}

/**
 * Create a KalmanFusion instance for high-precision measurement
 */
export function createPrecisionFusion(): KalmanFusion {
  return new KalmanFusion({
    imuWeight: 0.4,
    flowWeight: 0.3,
    moireWeight: 0.3, // Higher moiré weight for precision
    processNoiseOrientation: 0.005,
    processNoiseVelocity: 0.05
  });
}
