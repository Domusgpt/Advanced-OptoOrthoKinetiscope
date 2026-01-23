/**
 * ROTOR-BASED IMU INTEGRATION
 *
 * Integrates gyroscope readings using Geometric Algebra rotors instead of
 * Euler angles or rotation matrices. This avoids gimbal lock and provides
 * minimal representation (4 numbers for 3 DOF rotation).
 *
 * Key operations:
 * - Gyroscope integration: dR/dt = ½ω·R where ω is angular velocity bivector
 * - Complementary filter: fuses gyro (fast, drifty) with accelerometer (slow, stable)
 * - Gravity estimation: extracts gravity vector to correct orientation
 */

import {
  Rotor3D,
  normalizeRotor,
  multiplyRotors,
  reverseRotor,
  applyRotorToVector,
  rotorFromAxisAngle,
  slerpRotor,
  ROTOR_IDENTITY
} from './GeometricAlgebra';
import { Vector3 } from '../types';

// ═══════════════════════════════════════════════════════════════════════════
// TYPES
// ═══════════════════════════════════════════════════════════════════════════

export interface IMUState {
  /** Current orientation as a rotor */
  orientation: Rotor3D;

  /** Current angular velocity (rad/s) */
  angularVelocity: Vector3;

  /** Estimated gravity vector in body frame */
  gravityBody: Vector3;

  /** Estimated gravity vector in world frame */
  gravityWorld: Vector3;

  /** Linear acceleration (gravity removed) in body frame */
  linearAccelBody: Vector3;

  /** Linear acceleration in world frame */
  linearAccelWorld: Vector3;

  /** Accumulated linear velocity (by integration) */
  velocity: Vector3;

  /** Gyroscope bias estimate */
  gyroBias: Vector3;

  /** Confidence in orientation estimate (0-1) */
  confidence: number;

  /** Timestamp of last update */
  timestamp: number;
}

export interface IMUConfig {
  /** Complementary filter alpha for accelerometer correction (0-1) */
  accelCorrectionAlpha: number;

  /** Gyroscope bias learning rate */
  biasLearningRate: number;

  /** Gravity magnitude (m/s²) */
  gravityMagnitude: number;

  /** Minimum acceleration magnitude to trust for gravity estimation */
  minAccelForGravity: number;

  /** Maximum acceleration magnitude to trust for gravity estimation */
  maxAccelForGravity: number;

  /** Enable velocity integration (accumulates error over time) */
  integrateVelocity: boolean;
}

const DEFAULT_CONFIG: IMUConfig = {
  accelCorrectionAlpha: 0.02,
  biasLearningRate: 0.001,
  gravityMagnitude: 9.81,
  minAccelForGravity: 9.0,
  maxAccelForGravity: 10.5,
  integrateVelocity: false
};

// ═══════════════════════════════════════════════════════════════════════════
// ROTOR IMU CLASS
// ═══════════════════════════════════════════════════════════════════════════

export class RotorIMU {
  private state: IMUState;
  private config: IMUConfig;
  private initialized: boolean = false;

  constructor(config: Partial<IMUConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };

    this.state = {
      orientation: ROTOR_IDENTITY,
      angularVelocity: { x: 0, y: 0, z: 0 },
      gravityBody: { x: 0, y: 0, z: -this.config.gravityMagnitude },
      gravityWorld: { x: 0, y: 0, z: -this.config.gravityMagnitude },
      linearAccelBody: { x: 0, y: 0, z: 0 },
      linearAccelWorld: { x: 0, y: 0, z: 0 },
      velocity: { x: 0, y: 0, z: 0 },
      gyroBias: { x: 0, y: 0, z: 0 },
      confidence: 1.0,
      timestamp: 0
    };
  }

  /**
   * Initialize with accelerometer reading (assumes device is stationary)
   */
  initialize(accel: Vector3, timestamp: number): void {
    // Use accelerometer to determine initial orientation
    // Assumes accel ≈ -gravity when stationary

    const accelMag = Math.sqrt(accel.x ** 2 + accel.y ** 2 + accel.z ** 2);

    if (accelMag > 0.1) {
      // Normalize
      const gx = -accel.x / accelMag;
      const gy = -accel.y / accelMag;
      const gz = -accel.z / accelMag;

      // Find rotation from world-frame gravity (0,0,-1) to measured gravity
      this.state.orientation = this.rotationFromVectors(
        { x: 0, y: 0, z: -1 },
        { x: gx, y: gy, z: gz }
      );

      this.state.gravityBody = { x: -gx * this.config.gravityMagnitude,
                                  y: -gy * this.config.gravityMagnitude,
                                  z: -gz * this.config.gravityMagnitude };
    }

    this.state.timestamp = timestamp;
    this.initialized = true;
  }

  /**
   * Compute rotation that takes vector 'from' to vector 'to'
   */
  private rotationFromVectors(from: Vector3, to: Vector3): Rotor3D {
    // Normalize inputs
    const fromMag = Math.sqrt(from.x ** 2 + from.y ** 2 + from.z ** 2);
    const toMag = Math.sqrt(to.x ** 2 + to.y ** 2 + to.z ** 2);

    if (fromMag < 1e-10 || toMag < 1e-10) return ROTOR_IDENTITY;

    const fx = from.x / fromMag, fy = from.y / fromMag, fz = from.z / fromMag;
    const tx = to.x / toMag, ty = to.y / toMag, tz = to.z / toMag;

    // Dot product
    const dot = fx * tx + fy * ty + fz * tz;

    if (dot > 0.9999) {
      return ROTOR_IDENTITY;
    }

    if (dot < -0.9999) {
      // Opposite vectors: rotate 180° about any perpendicular axis
      let ax = 1, ay = 0, az = 0;
      if (Math.abs(fx) > 0.9) {
        ax = 0; ay = 1;
      }
      return rotorFromAxisAngle(ax, ay, az, Math.PI);
    }

    // Cross product gives rotation axis
    const cx = fy * tz - fz * ty;
    const cy = fz * tx - fx * tz;
    const cz = fx * ty - fy * tx;

    // Angle from dot product
    const angle = Math.acos(Math.max(-1, Math.min(1, dot)));

    return rotorFromAxisAngle(cx, cy, cz, angle);
  }

  /**
   * Update IMU state with new sensor readings
   */
  update(gyro: Vector3, accel: Vector3, timestamp: number): IMUState {
    if (!this.initialized) {
      this.initialize(accel, timestamp);
      return this.getState();
    }

    const dt = (timestamp - this.state.timestamp) / 1000; // seconds
    if (dt <= 0 || dt > 1) {
      // Invalid time delta, just update timestamp
      this.state.timestamp = timestamp;
      return this.getState();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 1: GYROSCOPE INTEGRATION
    // ═══════════════════════════════════════════════════════════════════════

    // Remove bias from gyroscope readings
    const gyroCorrected: Vector3 = {
      x: gyro.x - this.state.gyroBias.x,
      y: gyro.y - this.state.gyroBias.y,
      z: gyro.z - this.state.gyroBias.z
    };

    this.state.angularVelocity = gyroCorrected;

    // Convert angular velocity to rotation increment
    // For small dt: R_new ≈ R_old * exp(ω*dt/2)
    // Using first-order approximation: exp(ω*dt/2) ≈ 1 + ω*dt/2

    const angle = Math.sqrt(
      gyroCorrected.x ** 2 + gyroCorrected.y ** 2 + gyroCorrected.z ** 2
    ) * dt;

    if (angle > 1e-10) {
      const deltaRotor = rotorFromAxisAngle(
        gyroCorrected.x,
        gyroCorrected.y,
        gyroCorrected.z,
        angle
      );

      // Apply rotation: R_new = R_old * deltaR
      this.state.orientation = multiplyRotors(this.state.orientation, deltaRotor);
      this.state.orientation = normalizeRotor(this.state.orientation);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 2: ACCELEROMETER CORRECTION (Complementary Filter)
    // ═══════════════════════════════════════════════════════════════════════

    const accelMag = Math.sqrt(accel.x ** 2 + accel.y ** 2 + accel.z ** 2);

    // Only trust accelerometer when magnitude is close to gravity
    // (i.e., device is not accelerating significantly)
    if (accelMag >= this.config.minAccelForGravity &&
        accelMag <= this.config.maxAccelForGravity) {

      // Normalize accelerometer reading
      const measuredGravity: Vector3 = {
        x: -accel.x / accelMag,
        y: -accel.y / accelMag,
        z: -accel.z / accelMag
      };

      // Predict gravity direction based on current orientation
      // World gravity is (0, 0, -1), transform to body frame
      const R_inv = reverseRotor(this.state.orientation);
      const predictedGravity = applyRotorToVector(R_inv, 0, 0, -1);

      // Compute correction rotation
      const correctionRotor = this.rotationFromVectors(
        { x: predictedGravity.x, y: predictedGravity.y, z: predictedGravity.z },
        measuredGravity
      );

      // Apply partial correction (complementary filter)
      const alpha = this.config.accelCorrectionAlpha;
      const corrected = slerpRotor(ROTOR_IDENTITY, correctionRotor, alpha);

      this.state.orientation = multiplyRotors(corrected, this.state.orientation);
      this.state.orientation = normalizeRotor(this.state.orientation);

      // Update gravity estimate in body frame
      this.state.gravityBody = {
        x: measuredGravity.x * this.config.gravityMagnitude,
        y: measuredGravity.y * this.config.gravityMagnitude,
        z: measuredGravity.z * this.config.gravityMagnitude
      };

      // High confidence when accel matches gravity
      this.state.confidence = Math.min(1, this.state.confidence + 0.1);

      // Update gyro bias estimate when stationary
      const gyroMag = Math.sqrt(gyro.x ** 2 + gyro.y ** 2 + gyro.z ** 2);
      if (gyroMag < 0.05) {
        // Very little rotation - learn bias
        this.state.gyroBias.x += this.config.biasLearningRate * gyro.x;
        this.state.gyroBias.y += this.config.biasLearningRate * gyro.y;
        this.state.gyroBias.z += this.config.biasLearningRate * gyro.z;
      }
    } else {
      // Can't trust accelerometer (device is accelerating)
      this.state.confidence = Math.max(0.3, this.state.confidence - 0.05);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 3: COMPUTE LINEAR ACCELERATION
    // ═══════════════════════════════════════════════════════════════════════

    // Transform gravity estimate to body frame
    const R_inv = reverseRotor(this.state.orientation);
    const gravityInBody = applyRotorToVector(R_inv, 0, 0, -this.config.gravityMagnitude);

    // Linear acceleration = total accel - gravity
    this.state.linearAccelBody = {
      x: accel.x - gravityInBody.x,
      y: accel.y - gravityInBody.y,
      z: accel.z - gravityInBody.z
    };

    // Transform linear acceleration to world frame
    const worldAccel = applyRotorToVector(
      this.state.orientation,
      this.state.linearAccelBody.x,
      this.state.linearAccelBody.y,
      this.state.linearAccelBody.z
    );
    this.state.linearAccelWorld = worldAccel;

    // Update gravity in world frame (should be close to (0,0,-g))
    const worldGravity = applyRotorToVector(
      this.state.orientation,
      this.state.gravityBody.x,
      this.state.gravityBody.y,
      this.state.gravityBody.z
    );
    this.state.gravityWorld = worldGravity;

    // ═══════════════════════════════════════════════════════════════════════
    // STEP 4: VELOCITY INTEGRATION (optional, accumulates error)
    // ═══════════════════════════════════════════════════════════════════════

    if (this.config.integrateVelocity) {
      this.state.velocity.x += this.state.linearAccelWorld.x * dt;
      this.state.velocity.y += this.state.linearAccelWorld.y * dt;
      this.state.velocity.z += this.state.linearAccelWorld.z * dt;

      // Apply drag to prevent runaway drift
      const drag = 0.98;
      this.state.velocity.x *= drag;
      this.state.velocity.y *= drag;
      this.state.velocity.z *= drag;
    }

    this.state.timestamp = timestamp;

    return this.getState();
  }

  /**
   * Get current state (copy to prevent mutation)
   */
  getState(): IMUState {
    return {
      orientation: { ...this.state.orientation },
      angularVelocity: { ...this.state.angularVelocity },
      gravityBody: { ...this.state.gravityBody },
      gravityWorld: { ...this.state.gravityWorld },
      linearAccelBody: { ...this.state.linearAccelBody },
      linearAccelWorld: { ...this.state.linearAccelWorld },
      velocity: { ...this.state.velocity },
      gyroBias: { ...this.state.gyroBias },
      confidence: this.state.confidence,
      timestamp: this.state.timestamp
    };
  }

  /**
   * Get Euler angles (roll, pitch, yaw) from current orientation
   * Note: These can have gimbal lock, use sparingly
   */
  getEulerAngles(): { roll: number; pitch: number; yaw: number } {
    const r = this.state.orientation;

    // Convert rotor to Euler angles (ZYX convention)
    const sinp = 2 * (r.s * r.e31 - r.e12 * r.e23);

    let roll: number, pitch: number, yaw: number;

    if (Math.abs(sinp) >= 0.999) {
      // Gimbal lock
      pitch = Math.sign(sinp) * Math.PI / 2;
      roll = 0;
      yaw = 2 * Math.atan2(r.e12, r.s);
    } else {
      pitch = Math.asin(sinp);
      roll = Math.atan2(
        2 * (r.s * r.e23 + r.e31 * r.e12),
        1 - 2 * (r.e23 ** 2 + r.e31 ** 2)
      );
      yaw = Math.atan2(
        2 * (r.s * r.e12 + r.e23 * r.e31),
        1 - 2 * (r.e31 ** 2 + r.e12 ** 2)
      );
    }

    return {
      roll: roll * 180 / Math.PI,
      pitch: pitch * 180 / Math.PI,
      yaw: yaw * 180 / Math.PI
    };
  }

  /**
   * Transform a vector from body frame to world frame
   */
  bodyToWorld(v: Vector3): Vector3 {
    return applyRotorToVector(this.state.orientation, v.x, v.y, v.z);
  }

  /**
   * Transform a vector from world frame to body frame
   */
  worldToBody(v: Vector3): Vector3 {
    const invRotor = reverseRotor(this.state.orientation);
    return applyRotorToVector(invRotor, v.x, v.y, v.z);
  }

  /**
   * Reset to identity orientation
   */
  reset(): void {
    this.state.orientation = ROTOR_IDENTITY;
    this.state.angularVelocity = { x: 0, y: 0, z: 0 };
    this.state.velocity = { x: 0, y: 0, z: 0 };
    this.state.gyroBias = { x: 0, y: 0, z: 0 };
    this.state.confidence = 1.0;
    this.initialized = false;
  }

  /**
   * Set orientation directly (e.g., from external source)
   */
  setOrientation(rotor: Rotor3D): void {
    this.state.orientation = normalizeRotor(rotor);
  }

  /**
   * Update configuration
   */
  setConfig(config: Partial<IMUConfig>): void {
    this.config = { ...this.config, ...config };
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Convert degrees per second to radians per second
 */
export function dpsToRad(dps: Vector3): Vector3 {
  const scale = Math.PI / 180;
  return {
    x: dps.x * scale,
    y: dps.y * scale,
    z: dps.z * scale
  };
}

/**
 * Create a RotorIMU configured for typical phone sensors
 */
export function createPhoneIMU(): RotorIMU {
  return new RotorIMU({
    accelCorrectionAlpha: 0.03,
    biasLearningRate: 0.002,
    gravityMagnitude: 9.81,
    minAccelForGravity: 9.2,
    maxAccelForGravity: 10.4,
    integrateVelocity: false
  });
}
