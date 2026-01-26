/**
 * ORIENTATION QUANTIZER
 *
 * Uses the 24-cell (icositetrachoron) as a discrete sampling lattice for S³ (unit quaternions).
 * The 24 vertices are the Hurwitz quaternion units, which provide optimal discrete sampling
 * of the 3-sphere.
 *
 * Key insight: Instead of using raw noisy sensor data, we snap orientations to the nearest
 * vertex of the 24-cell and interpolate using geodesic SLERP. This provides:
 * - Noise reduction through quantization
 * - Smooth interpolation without gimbal lock
 * - A discrete basis for representing rotations
 */

import { Rotor3D, normalizeRotor, slerpRotor, quaternionToRotor, rotorToQuaternion } from './GeometricAlgebra';
import { Quaternion } from '../types';

// ═══════════════════════════════════════════════════════════════════════════
// 24-CELL VERTICES AS ROTORS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * The 24 Hurwitz quaternion units, pre-converted to rotors.
 * These form the vertices of the 24-cell and represent the
 * Binary Tetrahedral Group (2T) ≅ SL(2,3).
 */
export const HURWITZ_ROTORS: Rotor3D[] = (() => {
  const rotors: Rotor3D[] = [];

  // Set A: 8 unit quaternions (±1, ±i, ±j, ±k)
  // Identity and half-turns about principal axes
  rotors.push({ s: 1, e12: 0, e23: 0, e31: 0 });   // 1
  rotors.push({ s: -1, e12: 0, e23: 0, e31: 0 });  // -1
  rotors.push({ s: 0, e12: 0, e23: 1, e31: 0 });   // i → e23
  rotors.push({ s: 0, e12: 0, e23: -1, e31: 0 });  // -i
  rotors.push({ s: 0, e12: 0, e23: 0, e31: 1 });   // j → e31
  rotors.push({ s: 0, e12: 0, e23: 0, e31: -1 });  // -j
  rotors.push({ s: 0, e12: 1, e23: 0, e31: 0 });   // k → e12
  rotors.push({ s: 0, e12: -1, e23: 0, e31: 0 });  // -k

  // Set B: 16 half-integer quaternions (±1±i±j±k)/2
  // These are 120° rotations about body diagonals
  const h = 0.5;
  for (let sw = -1; sw <= 1; sw += 2) {
    for (let sx = -1; sx <= 1; sx += 2) {
      for (let sy = -1; sy <= 1; sy += 2) {
        for (let sz = -1; sz <= 1; sz += 2) {
          rotors.push({
            s: sw * h,
            e23: sx * h,  // i component
            e31: sy * h,  // j component
            e12: sz * h   // k component
          });
        }
      }
    }
  }

  return rotors;
})();

// ═══════════════════════════════════════════════════════════════════════════
// QUANTIZER CLASS
// ═══════════════════════════════════════════════════════════════════════════

export interface QuantizationResult {
  /** Index of nearest 24-cell vertex (0-23) */
  vertexIndex: number;

  /** The nearest vertex as a rotor */
  nearestRotor: Rotor3D;

  /** Angular distance to nearest vertex (radians) */
  distance: number;

  /** Smoothed output rotor (SLERP between input and vertex) */
  smoothedRotor: Rotor3D;

  /** Confidence: 1.0 when exactly on vertex, decreases with distance */
  confidence: number;
}

export class OrientationQuantizer {
  private smoothingFactor: number;
  private previousRotor: Rotor3D = { s: 1, e12: 0, e23: 0, e31: 0 };
  private previousVertexIndex: number = 0;
  private history: { rotor: Rotor3D; timestamp: number }[] = [];
  private readonly maxHistoryLength = 10;

  /**
   * @param smoothingFactor 0.0 = snap exactly to vertex, 1.0 = no smoothing
   */
  constructor(smoothingFactor: number = 0.3) {
    this.smoothingFactor = Math.max(0, Math.min(1, smoothingFactor));
  }

  /**
   * Compute the angular distance between two rotors (geodesic distance on S³)
   */
  private rotorDistance(r1: Rotor3D, r2: Rotor3D): number {
    // Dot product gives cos(θ) where θ is half the rotation angle between them
    let dot = r1.s * r2.s + r1.e12 * r2.e12 + r1.e23 * r2.e23 + r1.e31 * r2.e31;

    // Handle antipodal points (same rotation, different sign)
    dot = Math.abs(dot);
    dot = Math.min(1, Math.max(-1, dot));

    // Full rotation angle
    return 2 * Math.acos(dot);
  }

  /**
   * Find the nearest 24-cell vertex to a given rotor
   */
  findNearestVertex(r: Rotor3D): { index: number; rotor: Rotor3D; distance: number } {
    let bestIndex = 0;
    let bestDistance = Infinity;

    for (let i = 0; i < HURWITZ_ROTORS.length; i++) {
      const dist = this.rotorDistance(r, HURWITZ_ROTORS[i]);
      if (dist < bestDistance) {
        bestDistance = dist;
        bestIndex = i;
      }
    }

    return {
      index: bestIndex,
      rotor: HURWITZ_ROTORS[bestIndex],
      distance: bestDistance
    };
  }

  /**
   * Quantize an orientation from a quaternion input
   */
  quantizeQuaternion(q: Quaternion, timestamp?: number): QuantizationResult {
    const inputRotor = quaternionToRotor(q);
    return this.quantize(inputRotor, timestamp);
  }

  /**
   * Main quantization method
   */
  quantize(inputRotor: Rotor3D, timestamp?: number): QuantizationResult {
    // Normalize input
    const normalized = normalizeRotor(inputRotor);

    // Find nearest vertex
    const nearest = this.findNearestVertex(normalized);

    // Calculate confidence based on distance
    // Maximum distance between adjacent 24-cell vertices is about 60°
    const maxExpectedDistance = Math.PI / 3; // 60 degrees
    const confidence = Math.max(0, 1 - (nearest.distance / maxExpectedDistance));

    // Smooth toward vertex using SLERP
    // Blend between staying on input (smoothingFactor=1) and snapping to vertex (smoothingFactor=0)
    let smoothedRotor: Rotor3D;

    if (this.smoothingFactor >= 0.99) {
      // No smoothing, just use input
      smoothedRotor = normalized;
    } else if (this.smoothingFactor <= 0.01) {
      // Full snap to vertex
      smoothedRotor = nearest.rotor;
    } else {
      // SLERP between input and vertex
      // Lower smoothing = closer to vertex
      smoothedRotor = slerpRotor(nearest.rotor, normalized, this.smoothingFactor);
    }

    // Apply temporal smoothing with previous frame
    smoothedRotor = slerpRotor(this.previousRotor, smoothedRotor, 0.3);

    // Update state
    this.previousRotor = smoothedRotor;
    this.previousVertexIndex = nearest.index;

    // Update history
    if (timestamp !== undefined) {
      this.history.push({ rotor: smoothedRotor, timestamp });
      if (this.history.length > this.maxHistoryLength) {
        this.history.shift();
      }
    }

    return {
      vertexIndex: nearest.index,
      nearestRotor: nearest.rotor,
      distance: nearest.distance,
      smoothedRotor,
      confidence
    };
  }

  /**
   * Get the angular velocity from recent history (radians per second)
   */
  getAngularVelocity(): { x: number; y: number; z: number; magnitude: number } | null {
    if (this.history.length < 2) return null;

    const recent = this.history[this.history.length - 1];
    const prev = this.history[this.history.length - 2];

    const dt = (recent.timestamp - prev.timestamp) / 1000; // seconds
    if (dt <= 0) return null;

    // Compute rotation from prev to recent
    const prevReverse: Rotor3D = {
      s: prev.rotor.s,
      e12: -prev.rotor.e12,
      e23: -prev.rotor.e23,
      e31: -prev.rotor.e31
    };

    // delta = recent * prev^-1
    const delta: Rotor3D = {
      s: recent.rotor.s * prevReverse.s - recent.rotor.e12 * prevReverse.e12 -
         recent.rotor.e23 * prevReverse.e23 - recent.rotor.e31 * prevReverse.e31,
      e12: recent.rotor.s * prevReverse.e12 + recent.rotor.e12 * prevReverse.s +
           recent.rotor.e23 * prevReverse.e31 - recent.rotor.e31 * prevReverse.e23,
      e23: recent.rotor.s * prevReverse.e23 - recent.rotor.e12 * prevReverse.e31 +
           recent.rotor.e23 * prevReverse.s + recent.rotor.e31 * prevReverse.e12,
      e31: recent.rotor.s * prevReverse.e31 + recent.rotor.e12 * prevReverse.e23 -
           recent.rotor.e23 * prevReverse.e12 + recent.rotor.e31 * prevReverse.s
    };

    // Extract angle and axis
    const angle = 2 * Math.acos(Math.min(1, Math.max(-1, delta.s)));
    const sinHalf = Math.sin(angle / 2);

    if (Math.abs(sinHalf) < 1e-10) {
      return { x: 0, y: 0, z: 0, magnitude: 0 };
    }

    // Angular velocity = angle / dt in direction of axis
    const scale = angle / (dt * sinHalf);

    return {
      x: -delta.e23 * scale,  // Negate to match coordinate conventions
      y: -delta.e31 * scale,
      z: -delta.e12 * scale,
      magnitude: angle / dt
    };
  }

  /**
   * Get the average rotation rate over the history window
   */
  getAverageAngularSpeed(): number {
    if (this.history.length < 2) return 0;

    let totalAngle = 0;
    const first = this.history[0];
    const last = this.history[this.history.length - 1];

    for (let i = 1; i < this.history.length; i++) {
      const dist = this.rotorDistance(
        this.history[i - 1].rotor,
        this.history[i].rotor
      );
      totalAngle += dist;
    }

    const totalTime = (last.timestamp - first.timestamp) / 1000;
    if (totalTime <= 0) return 0;

    return totalAngle / totalTime;
  }

  /**
   * Get the stability score (0 = very unstable, 1 = perfectly still)
   */
  getStability(): number {
    const avgSpeed = this.getAverageAngularSpeed();
    // Map speed to stability: 0 rad/s = 1.0 stability, 2 rad/s = 0.0 stability
    return Math.max(0, 1 - avgSpeed / 2);
  }

  /**
   * Reset the quantizer state
   */
  reset(): void {
    this.previousRotor = { s: 1, e12: 0, e23: 0, e31: 0 };
    this.previousVertexIndex = 0;
    this.history = [];
  }

  /**
   * Set the smoothing factor
   */
  setSmoothingFactor(factor: number): void {
    this.smoothingFactor = Math.max(0, Math.min(1, factor));
  }

  /**
   * Get the name/label of a vertex (for debugging/display)
   */
  static getVertexLabel(index: number): string {
    if (index < 0 || index >= 24) return 'invalid';

    // First 8 are unit quaternions
    const unitLabels = ['1', '-1', 'i', '-i', 'j', '-j', 'k', '-k'];
    if (index < 8) return unitLabels[index];

    // Remaining 16 are half-integer quaternions
    const halfIndex = index - 8;
    const signs = [
      ['+', '+', '+', '+'], ['+', '+', '+', '-'],
      ['+', '+', '-', '+'], ['+', '+', '-', '-'],
      ['+', '-', '+', '+'], ['+', '-', '+', '-'],
      ['+', '-', '-', '+'], ['+', '-', '-', '-'],
      ['-', '+', '+', '+'], ['-', '+', '+', '-'],
      ['-', '+', '-', '+'], ['-', '+', '-', '-'],
      ['-', '-', '+', '+'], ['-', '-', '+', '-'],
      ['-', '-', '-', '+'], ['-', '-', '-', '-']
    ];

    const s = signs[halfIndex];
    return `(${s[0]}1${s[1]}i${s[2]}j${s[3]}k)/2`;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Get the 24-cell adjacency: which vertices are connected by edges
 * Each vertex has 8 neighbors (the 24-cell has 96 edges)
 */
export function get24CellAdjacency(): number[][] {
  const adjacency: number[][] = Array(24).fill(null).map(() => []);

  // Two vertices are adjacent if their angular distance is about 60° (π/3)
  const edgeThreshold = Math.PI / 3 + 0.1; // Allow small tolerance

  for (let i = 0; i < 24; i++) {
    for (let j = i + 1; j < 24; j++) {
      const dist = 2 * Math.acos(Math.abs(
        HURWITZ_ROTORS[i].s * HURWITZ_ROTORS[j].s +
        HURWITZ_ROTORS[i].e12 * HURWITZ_ROTORS[j].e12 +
        HURWITZ_ROTORS[i].e23 * HURWITZ_ROTORS[j].e23 +
        HURWITZ_ROTORS[i].e31 * HURWITZ_ROTORS[j].e31
      ));

      if (dist > 0.1 && dist < edgeThreshold) {
        adjacency[i].push(j);
        adjacency[j].push(i);
      }
    }
  }

  return adjacency;
}

/**
 * Find the shortest path between two vertices on the 24-cell graph
 * Uses BFS since all edges have equal weight
 */
export function shortestPath24Cell(start: number, end: number): number[] {
  if (start === end) return [start];

  const adjacency = get24CellAdjacency();
  const visited = new Set<number>();
  const parent = new Map<number, number>();
  const queue: number[] = [start];
  visited.add(start);

  while (queue.length > 0) {
    const current = queue.shift()!;

    for (const neighbor of adjacency[current]) {
      if (!visited.has(neighbor)) {
        visited.add(neighbor);
        parent.set(neighbor, current);

        if (neighbor === end) {
          // Reconstruct path
          const path: number[] = [end];
          let node = end;
          while (parent.has(node)) {
            node = parent.get(node)!;
            path.unshift(node);
          }
          return path;
        }

        queue.push(neighbor);
      }
    }
  }

  return []; // No path found (shouldn't happen for connected graph)
}
