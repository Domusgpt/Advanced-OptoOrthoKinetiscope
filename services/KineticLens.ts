/**
 * KINETIC LENS - Main Entry Point
 *
 * Unified motion measurement system using:
 * - Geometric Algebra rotors for gimbal-free orientation
 * - 24-cell lattice for discrete orientation sampling
 * - Moiré interference for sub-degree precision
 * - Kalman fusion for sensor integration
 *
 * Usage:
 *   const lens = new KineticLens();
 *   lens.initialize(initialReading);
 *
 *   // In animation loop:
 *   const state = lens.update({ gyro, accel, frame, timestamp });
 *   console.log(`Speed: ${state.speed} m/s`);
 */

import { KalmanFusion, FusedState, SensorReading, FusionConfig, createPhoneFusion } from './KalmanFusion';
import { OrientationQuantizer, HURWITZ_ROTORS, QuantizationResult } from './OrientationQuantizer';
import { RotorIMU, IMUState, createPhoneIMU, dpsToRad } from './RotorIMU';
import { OpticalFlowEngine, FlowField, FlowDecomposition, toLogPolar } from './OpticalFlow';
import { MoireMeasurementEngine, MoireMeasurement, imageDataToGrayscale, calculateAmplification } from './MoireMeasurement';
import { Rotor3D, quaternionToRotor, rotorToQuaternion, applyRotorToVector, slerpRotor, rotorFromAxisAngle } from './GeometricAlgebra';
import { Vector3, Quaternion } from '../types';

// ═══════════════════════════════════════════════════════════════════════════
// RE-EXPORTS
// ═══════════════════════════════════════════════════════════════════════════

export {
  // Fusion
  KalmanFusion,
  FusedState,
  SensorReading,
  FusionConfig,
  createPhoneFusion,

  // 24-Cell Quantizer
  OrientationQuantizer,
  HURWITZ_ROTORS,
  QuantizationResult,

  // IMU
  RotorIMU,
  IMUState,
  createPhoneIMU,
  dpsToRad,

  // Optical Flow
  OpticalFlowEngine,
  FlowField,
  FlowDecomposition,
  toLogPolar,

  // Moiré
  MoireMeasurementEngine,
  MoireMeasurement,
  imageDataToGrayscale,
  calculateAmplification,

  // Geometric Algebra
  Rotor3D,
  quaternionToRotor,
  rotorToQuaternion,
  applyRotorToVector,
  slerpRotor,
  rotorFromAxisAngle
};

// ═══════════════════════════════════════════════════════════════════════════
// TYPES
// ═══════════════════════════════════════════════════════════════════════════

export interface KineticLensConfig {
  /** Focal length in pixels (for velocity calculation) */
  focalLength: number;

  /** Estimated scene distance in meters */
  sceneDistance: number;

  /** Enable moiré-based precision measurement */
  enableMoire: boolean;

  /** Enable 24-cell quantization */
  enable24Cell: boolean;

  /** Enable optical flow */
  enableOpticalFlow: boolean;

  /** Smoothing factor (0 = no smoothing, 1 = max smoothing) */
  smoothing: number;
}

export interface KineticLensOutput {
  /** Estimated speed (m/s) */
  speed: number;

  /** Estimated speed (km/h) */
  speedKmh: number;

  /** Movement direction (degrees, 0 = right, 90 = down) */
  direction: number;

  /** Rotation rate (degrees/second) */
  rotationRate: number;

  /** Roll, pitch, yaw angles (degrees) */
  euler: { roll: number; pitch: number; yaw: number };

  /** Nearest 24-cell vertex label */
  discreteOrientation: string;

  /** Stability score (0-1) */
  stability: number;

  /** Overall confidence (0-1) */
  confidence: number;

  /** Camera ego-motion (px/frame) */
  cameraMotion: { x: number; y: number };

  /** Scene/object motion (px/frame) */
  sceneMotion: { x: number; y: number };

  /** Raw fused state (for advanced use) */
  rawState: FusedState;

  /** Timestamp */
  timestamp: number;
}

// ═══════════════════════════════════════════════════════════════════════════
// KINETIC LENS CLASS
// ═══════════════════════════════════════════════════════════════════════════

export class KineticLens {
  private fusion: KalmanFusion;
  private config: KineticLensConfig;
  private initialized: boolean = false;

  constructor(config: Partial<KineticLensConfig> = {}) {
    this.config = {
      focalLength: 1000,
      sceneDistance: 2.0,
      enableMoire: true,
      enable24Cell: true,
      enableOpticalFlow: true,
      smoothing: 0.3,
      ...config
    };

    this.fusion = new KalmanFusion({
      focalLength: this.config.focalLength,
      sceneDistance: this.config.sceneDistance
    });
  }

  /**
   * Initialize with first sensor reading
   */
  initialize(reading: SensorReading): void {
    this.fusion.initialize(reading);
    this.initialized = true;
  }

  /**
   * Update with new sensor data
   */
  update(reading: SensorReading): KineticLensOutput {
    if (!this.initialized) {
      this.initialize(reading);
    }

    const state = this.fusion.update(reading);
    return this.formatOutput(state);
  }

  /**
   * Convenience method: update with raw sensor values
   */
  updateRaw(
    gyroX: number, gyroY: number, gyroZ: number,
    accelX: number, accelY: number, accelZ: number,
    frame: ImageData | null,
    timestamp: number
  ): KineticLensOutput {
    const reading: SensorReading = {
      gyro: { x: gyroX, y: gyroY, z: gyroZ },
      accel: { x: accelX, y: accelY, z: accelZ },
      frame: frame || undefined,
      timestamp
    };

    return this.update(reading);
  }

  /**
   * Update with browser DeviceMotion/DeviceOrientation events
   */
  updateFromDeviceEvents(
    motionEvent: DeviceMotionEvent | null,
    orientationEvent: DeviceOrientationEvent | null,
    frame: ImageData | null,
    timestamp: number
  ): KineticLensOutput {
    const reading: SensorReading = { timestamp };

    if (motionEvent) {
      if (motionEvent.rotationRate) {
        reading.gyro = dpsToRad({
          x: motionEvent.rotationRate.beta || 0,
          y: motionEvent.rotationRate.gamma || 0,
          z: motionEvent.rotationRate.alpha || 0
        });
      }

      if (motionEvent.accelerationIncludingGravity) {
        reading.accel = {
          x: motionEvent.accelerationIncludingGravity.x || 0,
          y: motionEvent.accelerationIncludingGravity.y || 0,
          z: motionEvent.accelerationIncludingGravity.z || 0
        };
      }
    }

    if (orientationEvent && orientationEvent.alpha !== null) {
      // Convert Euler to quaternion (approximate)
      const alpha = (orientationEvent.alpha || 0) * Math.PI / 180;
      const beta = (orientationEvent.beta || 0) * Math.PI / 180;
      const gamma = (orientationEvent.gamma || 0) * Math.PI / 180;

      reading.deviceOrientation = this.eulerToQuaternion(alpha, beta, gamma);
    }

    if (frame) {
      reading.frame = frame;
    }

    return this.update(reading);
  }

  private eulerToQuaternion(yaw: number, pitch: number, roll: number): { x: number; y: number; z: number; w: number } {
    const cy = Math.cos(yaw / 2);
    const sy = Math.sin(yaw / 2);
    const cp = Math.cos(pitch / 2);
    const sp = Math.sin(pitch / 2);
    const cr = Math.cos(roll / 2);
    const sr = Math.sin(roll / 2);

    return {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    };
  }

  private formatOutput(state: FusedState): KineticLensOutput {
    const summary = this.fusion.getMeasurementSummary();
    const euler = this.fusion.getEulerAngles();
    const diagnostics = this.fusion.getDiagnostics();

    return {
      speed: summary.speed,
      speedKmh: summary.speed * 3.6,
      direction: summary.direction,
      rotationRate: summary.rotation,
      euler,
      discreteOrientation: diagnostics.nearestVertex,
      stability: summary.stability,
      confidence: state.confidence,
      cameraMotion: state.cameraFlow,
      sceneMotion: state.sceneFlow,
      rawState: state,
      timestamp: state.timestamp
    };
  }

  /**
   * Get the current speed in specified units
   */
  getSpeed(unit: 'mps' | 'kph' | 'mph' = 'mps'): number {
    return this.fusion.getVelocity(unit);
  }

  /**
   * Get recent trajectory
   */
  getTrajectory(): Vector3[] {
    return this.fusion.getTrajectory();
  }

  /**
   * Predict state at future time
   */
  predict(dt: number): FusedState {
    return this.fusion.predict(dt);
  }

  /**
   * Set estimated scene distance (affects velocity calculation)
   */
  setSceneDistance(meters: number): void {
    this.config.sceneDistance = meters;
    this.fusion.setSceneDistance(meters);
  }

  /**
   * Reset all state
   */
  reset(): void {
    this.fusion.reset();
    this.initialized = false;
  }

  /**
   * Get diagnostic info
   */
  getDiagnostics(): ReturnType<typeof KalmanFusion.prototype.getDiagnostics> {
    return this.fusion.getDiagnostics();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// CONVENIENCE FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Create a KineticLens instance with default phone settings
 */
export function createKineticLens(): KineticLens {
  return new KineticLens({
    focalLength: 1000,
    sceneDistance: 2.0,
    enableMoire: true,
    enable24Cell: true,
    enableOpticalFlow: true,
    smoothing: 0.3
  });
}

/**
 * Create a KineticLens for measuring fast-moving objects
 */
export function createFastObjectLens(): KineticLens {
  return new KineticLens({
    focalLength: 1000,
    sceneDistance: 5.0,
    enableMoire: false, // Moiré can't track very fast motion
    enable24Cell: true,
    enableOpticalFlow: true,
    smoothing: 0.1 // Less smoothing for responsiveness
  });
}

/**
 * Create a KineticLens for high-precision slow measurements
 */
export function createPrecisionLens(): KineticLens {
  return new KineticLens({
    focalLength: 1000,
    sceneDistance: 1.0,
    enableMoire: true,
    enable24Cell: true,
    enableOpticalFlow: true,
    smoothing: 0.5 // More smoothing for stability
  });
}

// ═══════════════════════════════════════════════════════════════════════════
// VERSION INFO
// ═══════════════════════════════════════════════════════════════════════════

export const VERSION = '1.0.0';
export const CAPABILITIES = {
  rotorIMU: true,
  opticalFlow: true,
  moireMeasurement: true,
  cellQuantization24: true,
  kalmanFusion: true
};
