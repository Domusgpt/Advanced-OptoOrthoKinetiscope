/**
 * E8 PROJECTION ENGINE - High-Dimensional Geometric Structures
 *
 * Implements the mathematical machinery for visualizing E8 symmetries
 * and their projections onto lower-dimensional spaces.
 *
 * Key structures:
 * - E8 Root System (240 vertices in 8D)
 * - 600-Cell (120 vertices in 4D) - Hexacosichoron
 * - 24-Cell (24 vertices in 4D) - Icositetrachoron
 * - Fano Plane (7 points, 7 lines) - Octonion multiplication table
 * - Hopf Fibration (S³ → S² with S¹ fibers) - Quaternion visualization
 *
 * Mathematical relationships:
 * - E8 ⊃ D8 ⊃ D4 × D4 ≅ SO(8) × SO(8)
 * - 600-cell is the convex hull of 120 unit quaternions
 * - 24-cell vertices ≅ Binary Tetrahedral Group (2T) ≅ SL(2,3)
 * - 2T contains the 24 elements of the Hurwitz quaternion units
 */

import { Vector4 } from "./cpeMath";
import { BiRotor4D, Rotor3D, cliffordRotation, applyBiRotor } from "./GeometricAlgebra";

// ═══════════════════════════════════════════════════════════════════════════
// TYPE DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

export interface Vector8 {
  x0: number; x1: number; x2: number; x3: number;
  x4: number; x5: number; x6: number; x7: number;
}

export interface HopfFiber {
  basePoint: { x: number, y: number, z: number };  // Point on S²
  fiberPhase: number;  // Phase angle on the S¹ fiber
  torusCoords: { u: number, v: number };  // Coordinates on the Clifford torus
}

export interface FanoLine {
  points: [number, number, number];  // Three points on the line
  color: string;  // Visual identifier
}

export interface OctonionBasis {
  index: number;  // 0-7 (e0=1, e1-e7 imaginary)
  fanoPosition: { x: number, y: number };  // Position in Fano plane visualization
}

export interface QuasicrystalCell {
  vertices: { x: number, y: number }[];
  type: 'thin' | 'thick';  // Penrose rhombus types
  phasonShift: number;  // Displacement in the perpendicular space
}

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTS: THE GOLDEN RATIO AND RELATED
// ═══════════════════════════════════════════════════════════════════════════

/** The Golden Ratio φ = (1 + √5) / 2 ≈ 1.618 */
export const PHI = (1 + Math.sqrt(5)) / 2;

/** Inverse golden ratio 1/φ = φ - 1 ≈ 0.618 */
export const PHI_INV = PHI - 1;

/** √5 for various formulas */
export const SQRT5 = Math.sqrt(5);

// ═══════════════════════════════════════════════════════════════════════════
// E8 ROOT SYSTEM (240 vertices in 8D)
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Generate the 240 root vectors of E8
 * The E8 lattice is the densest sphere packing in 8 dimensions
 *
 * Construction: E8 = D8 ∪ D8 + (½,½,½,½,½,½,½,½)
 * - D8: All integer vectors with even sum (112 roots)
 * - Half-integer: Vectors in (Z + ½)^8 with even sum (128 roots)
 */
export function getE8Roots(): Vector8[] {
  const roots: Vector8[] = [];

  // TYPE 1: Permutations of (±1, ±1, 0, 0, 0, 0, 0, 0)
  // These are 112 vectors (C(8,2) * 2^2 = 28 * 4 = 112)
  for (let i = 0; i < 8; i++) {
    for (let j = i + 1; j < 8; j++) {
      for (const si of [-1, 1]) {
        for (const sj of [-1, 1]) {
          const v: number[] = [0, 0, 0, 0, 0, 0, 0, 0];
          v[i] = si;
          v[j] = sj;
          roots.push({
            x0: v[0], x1: v[1], x2: v[2], x3: v[3],
            x4: v[4], x5: v[5], x6: v[6], x7: v[7]
          });
        }
      }
    }
  }

  // TYPE 2: Half-integer vectors (±½, ±½, ..., ±½) with even number of minus signs
  // These are 128 vectors (2^7 = 128, half of 2^8)
  for (let bits = 0; bits < 256; bits++) {
    let negCount = 0;
    const v: number[] = [];
    for (let i = 0; i < 8; i++) {
      if ((bits >> i) & 1) {
        v.push(-0.5);
        negCount++;
      } else {
        v.push(0.5);
      }
    }
    // Keep only vectors with even number of negative components
    if (negCount % 2 === 0) {
      roots.push({
        x0: v[0], x1: v[1], x2: v[2], x3: v[3],
        x4: v[4], x5: v[5], x6: v[6], x7: v[7]
      });
    }
  }

  return roots; // Returns 240 roots
}

/**
 * Project E8 roots to 2D using the Coxeter plane
 * The Coxeter plane is the unique plane where all roots appear as a regular pattern
 */
export function projectE8ToCoxeterPlane(v: Vector8): { x: number, y: number } {
  // The Coxeter projection uses specific eigenvectors of the E8 Coxeter element
  // These create a 30-fold symmetric pattern

  const angle1 = Math.PI / 30;
  const angle2 = 11 * Math.PI / 30;
  const angle3 = 7 * Math.PI / 30;
  const angle4 = 13 * Math.PI / 30;

  // Projection matrix rows (simplified for 8D → 2D)
  const px = v.x0 * Math.cos(angle1) + v.x1 * Math.cos(angle2) +
             v.x2 * Math.cos(angle3) + v.x3 * Math.cos(angle4) +
             v.x4 * Math.cos(5*angle1) + v.x5 * Math.cos(5*angle2) +
             v.x6 * Math.cos(5*angle3) + v.x7 * Math.cos(5*angle4);

  const py = v.x0 * Math.sin(angle1) + v.x1 * Math.sin(angle2) +
             v.x2 * Math.sin(angle3) + v.x3 * Math.sin(angle4) +
             v.x4 * Math.sin(5*angle1) + v.x5 * Math.sin(5*angle2) +
             v.x6 * Math.sin(5*angle3) + v.x7 * Math.sin(5*angle4);

  return { x: px, y: py };
}

// ═══════════════════════════════════════════════════════════════════════════
// 600-CELL (HEXACOSICHORON) - 120 vertices in 4D
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Generate the 120 vertices of the 600-cell
 * The 600-cell is the 4D analog of the icosahedron
 * Its vertices form the binary icosahedral group (2I)
 *
 * Construction uses three types of coordinates:
 * 1. 8 vertices: permutations of (±1, 0, 0, 0)
 * 2. 16 vertices: (±½, ±½, ±½, ±½)
 * 3. 96 vertices: even permutations of (0, ±½, ±φ/2, ±1/(2φ))
 */
export function get600CellVertices(): Vector4[] {
  const vertices: Vector4[] = [];

  // TYPE 1: Axis-aligned (8 vertices)
  const axes = [
    { x: 1, y: 0, z: 0, w: 0 }, { x: -1, y: 0, z: 0, w: 0 },
    { x: 0, y: 1, z: 0, w: 0 }, { x: 0, y: -1, z: 0, w: 0 },
    { x: 0, y: 0, z: 1, w: 0 }, { x: 0, y: 0, z: -1, w: 0 },
    { x: 0, y: 0, z: 0, w: 1 }, { x: 0, y: 0, z: 0, w: -1 }
  ];
  vertices.push(...axes);

  // TYPE 2: 16-cell vertices (16 vertices)
  for (const sx of [-0.5, 0.5]) {
    for (const sy of [-0.5, 0.5]) {
      for (const sz of [-0.5, 0.5]) {
        for (const sw of [-0.5, 0.5]) {
          vertices.push({ x: sx, y: sy, z: sz, w: sw });
        }
      }
    }
  }

  // TYPE 3: Golden ratio coordinates (96 vertices)
  // Even permutations of (0, ±1/2, ±φ/2, ±1/(2φ))
  const half = 0.5;
  const phiHalf = PHI / 2;
  const phiInvHalf = PHI_INV / 2;

  // Generate all even permutations
  const baseCoords = [0, half, phiHalf, phiInvHalf];
  const evenPerms = getEvenPermutations([0, 1, 2, 3]);

  for (const perm of evenPerms) {
    // For each permutation, apply all sign combinations
    for (let signs = 0; signs < 8; signs++) { // 2^3 = 8 (one coord is 0)
      const coords = [
        baseCoords[perm[0]],
        baseCoords[perm[1]],
        baseCoords[perm[2]],
        baseCoords[perm[3]]
      ];

      // Apply signs to non-zero coordinates
      let signIdx = 0;
      for (let i = 0; i < 4; i++) {
        if (coords[i] !== 0) {
          if ((signs >> signIdx) & 1) {
            coords[i] = -coords[i];
          }
          signIdx++;
        }
      }

      vertices.push({ x: coords[0], y: coords[1], z: coords[2], w: coords[3] });
    }
  }

  // Normalize all vertices to unit sphere
  return vertices.map(v => {
    const mag = Math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z + v.w*v.w);
    return { x: v.x/mag, y: v.y/mag, z: v.z/mag, w: v.w/mag };
  });
}

/**
 * Get the edges of the 600-cell
 * Two vertices are connected if their distance equals the "edge length"
 */
export function get600CellEdges(vertices: Vector4[]): [number, number][] {
  const edges: [number, number][] = [];
  const edgeLength = 1 / PHI; // The 600-cell edge length for unit radius

  for (let i = 0; i < vertices.length; i++) {
    for (let j = i + 1; j < vertices.length; j++) {
      const dx = vertices[i].x - vertices[j].x;
      const dy = vertices[i].y - vertices[j].y;
      const dz = vertices[i].z - vertices[j].z;
      const dw = vertices[i].w - vertices[j].w;
      const dist = Math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw);

      if (Math.abs(dist - edgeLength) < 0.01) {
        edges.push([i, j]);
      }
    }
  }

  return edges;
}

// ═══════════════════════════════════════════════════════════════════════════
// 24-CELL (ICOSITETRACHORON) - For Phillips Gate Substrate
// ═══════════════════════════════════════════════════════════════════════════

/**
 * The 24-cell vertices correspond to the 24 Hurwitz quaternion units
 * These form the binary tetrahedral group 2T ≅ SL(2,3)
 * This group contains the Pauli matrices (up to phase)!
 *
 * The 24 units are:
 * - 8 quaternion units: ±1, ±i, ±j, ±k
 * - 16 Hurwitz units: (±1±i±j±k)/2
 */
export function get24CellAsQuaternions(): Vector4[] {
  const vertices: Vector4[] = [];

  // 8 basic quaternion units
  vertices.push(
    { x: 0, y: 0, z: 0, w: 1 },   // 1
    { x: 0, y: 0, z: 0, w: -1 },  // -1
    { x: 1, y: 0, z: 0, w: 0 },   // i
    { x: -1, y: 0, z: 0, w: 0 },  // -i
    { x: 0, y: 1, z: 0, w: 0 },   // j
    { x: 0, y: -1, z: 0, w: 0 },  // -j
    { x: 0, y: 0, z: 1, w: 0 },   // k
    { x: 0, y: 0, z: -1, w: 0 }   // -k
  );

  // 16 Hurwitz quaternion units
  const half = 0.5;
  for (const sw of [-half, half]) {
    for (const sx of [-half, half]) {
      for (const sy of [-half, half]) {
        for (const sz of [-half, half]) {
          vertices.push({ x: sx, y: sy, z: sz, w: sw });
        }
      }
    }
  }

  return vertices;
}

/**
 * Map 24-cell vertices to their role in the binary tetrahedral group
 * Returns the group element index and its relation to Pauli matrices
 */
export function get24CellGroupStructure(): {
  vertex: Vector4,
  pauliRelation: string,
  order: number  // Order of the group element
}[] {
  const vertices = get24CellAsQuaternions();

  return vertices.map((v, i) => {
    // Determine the relation to Pauli matrices
    let pauliRelation = '';
    let order = 1;

    if (Math.abs(v.w) > 0.9) {
      pauliRelation = v.w > 0 ? 'I (Identity)' : '-I';
      order = v.w > 0 ? 1 : 2;
    } else if (Math.abs(v.x) > 0.9) {
      pauliRelation = v.x > 0 ? 'iσx' : '-iσx';
      order = 4;
    } else if (Math.abs(v.y) > 0.9) {
      pauliRelation = v.y > 0 ? 'iσy' : '-iσy';
      order = 4;
    } else if (Math.abs(v.z) > 0.9) {
      pauliRelation = v.z > 0 ? 'iσz' : '-iσz';
      order = 4;
    } else {
      // Hurwitz unit - these generate order-3 and order-6 rotations
      pauliRelation = `T-gate class (ω rotation)`;
      order = 3;
    }

    return { vertex: v, pauliRelation, order };
  });
}

// ═══════════════════════════════════════════════════════════════════════════
// FANO PLANE - OCTONION MULTIPLICATION TABLE
// ═══════════════════════════════════════════════════════════════════════════

/**
 * The Fano plane is the smallest finite projective plane PG(2,2)
 * It has 7 points and 7 lines, with each line containing 3 points
 *
 * The 7 imaginary octonion units e1...e7 live on the 7 points
 * Each line represents a quaternionic triple: ei*ej = ek (cyclic)
 */
export function getFanoPlane(): {
  points: OctonionBasis[],
  lines: FanoLine[]
} {
  // Position the 7 points in a hexagonal arrangement
  // Point 7 (e7) is at the center
  const points: OctonionBasis[] = [
    { index: 1, fanoPosition: { x: 0, y: 1 } },
    { index: 2, fanoPosition: { x: Math.sqrt(3)/2, y: 0.5 } },
    { index: 4, fanoPosition: { x: Math.sqrt(3)/2, y: -0.5 } },
    { index: 3, fanoPosition: { x: 0, y: -1 } },
    { index: 6, fanoPosition: { x: -Math.sqrt(3)/2, y: -0.5 } },
    { index: 5, fanoPosition: { x: -Math.sqrt(3)/2, y: 0.5 } },
    { index: 7, fanoPosition: { x: 0, y: 0 } }  // Center
  ];

  // The 7 lines of the Fano plane
  // Each line defines a quaternionic subalgebra
  const lines: FanoLine[] = [
    { points: [1, 2, 4], color: '#F43F5E' },  // Rose - e1*e2=e4
    { points: [2, 3, 5], color: '#F97316' },  // Orange - e2*e3=e5
    { points: [3, 4, 6], color: '#FACC15' },  // Yellow - e3*e4=e6
    { points: [4, 5, 7], color: '#22C55E' },  // Green - e4*e5=e7
    { points: [5, 6, 1], color: '#22D3EE' },  // Cyan - e5*e6=e1
    { points: [6, 7, 2], color: '#3B82F6' },  // Blue - e6*e7=e2
    { points: [7, 1, 3], color: '#A855F7' }   // Purple - e7*e1=e3
  ];

  return { points, lines };
}

/**
 * Octonion multiplication using the Fano plane
 * Returns ei * ej and the sign
 */
export function octonionMultiply(i: number, j: number): { result: number, sign: number } {
  if (i === 0) return { result: j, sign: 1 };
  if (j === 0) return { result: i, sign: 1 };
  if (i === j) return { result: 0, sign: -1 };  // ei² = -1

  // The Fano plane multiplication table
  const table: { [key: string]: { result: number, sign: number } } = {
    '1,2': { result: 4, sign: 1 },
    '2,4': { result: 1, sign: 1 },
    '4,1': { result: 2, sign: 1 },
    '2,3': { result: 5, sign: 1 },
    '3,5': { result: 2, sign: 1 },
    '5,2': { result: 3, sign: 1 },
    '3,4': { result: 6, sign: 1 },
    '4,6': { result: 3, sign: 1 },
    '6,3': { result: 4, sign: 1 },
    '4,5': { result: 7, sign: 1 },
    '5,7': { result: 4, sign: 1 },
    '7,4': { result: 5, sign: 1 },
    '5,6': { result: 1, sign: 1 },
    '6,1': { result: 5, sign: 1 },
    '1,5': { result: 6, sign: 1 },
    '6,7': { result: 2, sign: 1 },
    '7,2': { result: 6, sign: 1 },
    '2,6': { result: 7, sign: 1 },
    '7,1': { result: 3, sign: 1 },
    '1,3': { result: 7, sign: 1 },
    '3,7': { result: 1, sign: 1 }
  };

  const key = `${i},${j}`;
  if (table[key]) {
    return table[key];
  }

  // Reverse order has opposite sign
  const revKey = `${j},${i}`;
  if (table[revKey]) {
    return { result: table[revKey].result, sign: -table[revKey].sign };
  }

  // Fallback
  return { result: 0, sign: 0 };
}

// ═══════════════════════════════════════════════════════════════════════════
// HOPF FIBRATION - S³ → S² with S¹ fibers
// ═══════════════════════════════════════════════════════════════════════════

/**
 * The Hopf fibration maps each point on S³ (unit quaternions) to S² (Bloch sphere)
 * The fiber over each point of S² is a great circle (S¹)
 *
 * This is fundamental for visualizing quaternion rotations!
 * h: S³ → S²
 * h(q) = q * k * q̄  (map quaternion to its rotation axis on the z-sphere)
 */
export function hopfProjection(
  x: number, y: number, z: number, w: number
): { basePoint: { x: number, y: number, z: number }, fiberPhase: number } {
  // The quaternion (w, x, y, z) maps to the point on S² via stereographic projection
  // Base point: (2(xz + yw), 2(yz - xw), x² + y² - z² - w²)

  // Apply the Hopf map
  const bx = 2 * (x * z + y * w);
  const by = 2 * (y * z - x * w);
  const bz = x * x + y * y - z * z - w * w;

  // Normalize to S²
  const mag = Math.sqrt(bx * bx + by * by + bz * bz);
  const basePoint = mag > 1e-10
    ? { x: bx / mag, y: by / mag, z: bz / mag }
    : { x: 0, y: 0, z: 1 };

  // The fiber phase is the "rotation about the base axis"
  // Computed from the original quaternion's phase
  const fiberPhase = Math.atan2(y, x) - Math.atan2(w, z);

  return { basePoint, fiberPhase };
}

/**
 * Inverse Hopf: Given a point on S² and a fiber phase, reconstruct a quaternion
 */
export function inverseHopf(
  bx: number, by: number, bz: number,
  fiberPhase: number
): { x: number, y: number, z: number, w: number } {
  // Normalize the base point
  const bmag = Math.sqrt(bx*bx + by*by + bz*bz);
  const nx = bx / bmag;
  const ny = by / bmag;
  const nz = bz / bmag;

  // Compute the quaternion from axis and phase
  // The axis (nx, ny, nz) determines the base point
  // The fiberPhase determines the rotation about that axis

  const halfTheta = Math.acos(Math.max(-1, Math.min(1, nz)));
  const phi = Math.atan2(ny, nx);

  const cosHalfTheta = Math.cos(halfTheta / 2);
  const sinHalfTheta = Math.sin(halfTheta / 2);

  return {
    x: sinHalfTheta * Math.cos(phi + fiberPhase),
    y: sinHalfTheta * Math.sin(phi + fiberPhase),
    z: cosHalfTheta * Math.sin(fiberPhase),
    w: cosHalfTheta * Math.cos(fiberPhase)
  };
}

/**
 * Generate the Clifford torus - the inverse image of the equator under Hopf
 * This is a flat torus embedded in S³, perfect for moiré visualization!
 */
export function generateCliffordTorus(
  uDivisions: number,
  vDivisions: number
): Vector4[][] {
  const torus: Vector4[][] = [];

  for (let i = 0; i <= uDivisions; i++) {
    const row: Vector4[] = [];
    const u = (2 * Math.PI * i) / uDivisions;

    for (let j = 0; j <= vDivisions; j++) {
      const v = (2 * Math.PI * j) / vDivisions;

      // Clifford torus parametrization
      // (cos(u)/√2, sin(u)/√2, cos(v)/√2, sin(v)/√2)
      const r = Math.SQRT1_2;
      row.push({
        x: r * Math.cos(u),
        y: r * Math.sin(u),
        z: r * Math.cos(v),
        w: r * Math.sin(v)
      });
    }
    torus.push(row);
  }

  return torus;
}

/**
 * Generate Villarceau circles - circles on the Clifford torus
 * When stereographically projected to ℝ³, these form linked torus knots
 */
export function generateVillarceauCircles(
  numCircles: number,
  pointsPerCircle: number,
  offset: number = 0
): Vector4[][] {
  const circles: Vector4[][] = [];

  for (let c = 0; c < numCircles; c++) {
    const circle: Vector4[] = [];
    const phi = (2 * Math.PI * c) / numCircles + offset;

    for (let i = 0; i <= pointsPerCircle; i++) {
      const theta = (2 * Math.PI * i) / pointsPerCircle;

      // Villarceau circle parametrization
      const r = Math.SQRT1_2;
      circle.push({
        x: r * Math.cos(theta) * Math.cos(phi) - r * Math.sin(theta) * Math.sin(phi),
        y: r * Math.cos(theta) * Math.sin(phi) + r * Math.sin(theta) * Math.cos(phi),
        z: r * Math.cos(theta),
        w: r * Math.sin(theta)
      });
    }
    circles.push(circle);
  }

  return circles;
}

// ═══════════════════════════════════════════════════════════════════════════
// QUASICRYSTAL PROJECTIONS - Penrose & Ammann-Beenker Tilings
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Generate a Penrose tiling using the "cut and project" method from 5D
 * The tiling is a 2D slice of a 5D hypercubic lattice
 */
export function generatePenroseTiling(
  size: number,
  resolution: number,
  phaseOffset: { u: number, v: number } = { u: 0, v: 0 }
): QuasicrystalCell[] {
  const cells: QuasicrystalCell[] = [];

  // The projection matrix from 5D to 2D (physical space)
  // Uses angles based on the pentagon
  const angles = [0, 1, 2, 3, 4].map(i => (2 * Math.PI * i) / 5);
  const projX = angles.map(a => Math.cos(a));
  const projY = angles.map(a => Math.sin(a));

  // The perpendicular projection (for determining phason shifts)
  const perpX = angles.map(a => Math.cos(2 * a));
  const perpY = angles.map(a => Math.sin(2 * a));

  // Scan through lattice points
  for (let n0 = -resolution; n0 <= resolution; n0++) {
    for (let n1 = -resolution; n1 <= resolution; n1++) {
      for (let n2 = -resolution; n2 <= resolution; n2++) {
        for (let n3 = -resolution; n3 <= resolution; n3++) {
          // Project to physical space
          const x = n0 * projX[0] + n1 * projX[1] + n2 * projX[2] + n3 * projX[3];
          const y = n0 * projY[0] + n1 * projY[1] + n2 * projY[2] + n3 * projY[3];

          // Project to perpendicular space
          const px = n0 * perpX[0] + n1 * perpX[1] + n2 * perpX[2] + n3 * perpX[3] + phaseOffset.u;
          const py = n0 * perpY[0] + n1 * perpY[1] + n2 * perpY[2] + n3 * perpY[3] + phaseOffset.v;

          // Acceptance window: pentagons in perp space
          const perpDist = Math.sqrt(px * px + py * py);

          if (Math.abs(x) <= size && Math.abs(y) <= size && perpDist < 1.5) {
            // Determine rhombus type from local geometry
            const type: 'thin' | 'thick' = perpDist < 1.0 ? 'thick' : 'thin';

            // Generate rhombus vertices (simplified)
            const s = 0.1;
            const vertices = [
              { x: x - s, y: y },
              { x: x, y: y - s * (type === 'thick' ? 1.618 : 0.618) },
              { x: x + s, y: y },
              { x: x, y: y + s * (type === 'thick' ? 1.618 : 0.618) }
            ];

            cells.push({
              vertices,
              type,
              phasonShift: perpDist
            });
          }
        }
      }
    }
  }

  return cells;
}

/**
 * Generate an Ammann-Beenker tiling (8-fold symmetric) from 4D
 * This is particularly relevant as it relates to the octonion structure
 */
export function generateAmmannBeenkerTiling(
  size: number,
  resolution: number
): QuasicrystalCell[] {
  const cells: QuasicrystalCell[] = [];
  const sqrt2 = Math.SQRT2;

  // Projection from 4D to 2D physical space
  // Uses 8-fold symmetry angles
  for (let n0 = -resolution; n0 <= resolution; n0++) {
    for (let n1 = -resolution; n1 <= resolution; n1++) {
      for (let n2 = -resolution; n2 <= resolution; n2++) {
        for (let n3 = -resolution; n3 <= resolution; n3++) {
          // 4D → 2D projection using octagonal angles
          const x = n0 + n1 * Math.cos(Math.PI/4) + n2 * Math.cos(Math.PI/2) + n3 * Math.cos(3*Math.PI/4);
          const y = n1 * Math.sin(Math.PI/4) + n2 * Math.sin(Math.PI/2) + n3 * Math.sin(3*Math.PI/4);

          // Perpendicular space projection
          const px = n0 - n2;
          const py = n1 - n3;
          const perpDist = Math.sqrt(px * px + py * py);

          if (Math.abs(x) <= size && Math.abs(y) <= size && perpDist < sqrt2) {
            const type: 'thin' | 'thick' = (n0 + n1 + n2 + n3) % 2 === 0 ? 'thick' : 'thin';
            const s = 0.08;

            // Square or rhombus vertices
            const angle = Math.atan2(py, px);
            const vertices = [
              { x: x + s * Math.cos(angle), y: y + s * Math.sin(angle) },
              { x: x + s * Math.cos(angle + Math.PI/2), y: y + s * Math.sin(angle + Math.PI/2) },
              { x: x + s * Math.cos(angle + Math.PI), y: y + s * Math.sin(angle + Math.PI) },
              { x: x + s * Math.cos(angle + 3*Math.PI/2), y: y + s * Math.sin(angle + 3*Math.PI/2) }
            ];

            cells.push({
              vertices,
              type,
              phasonShift: perpDist
            });
          }
        }
      }
    }
  }

  return cells;
}

// ═══════════════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Generate all even permutations of [0,1,2,3]
 * There are 12 even permutations (half of 4! = 24)
 */
function getEvenPermutations(arr: number[]): number[][] {
  const result: number[][] = [];

  function permute(arr: number[], start: number, parity: number) {
    if (start === arr.length - 1) {
      if (parity === 0) {
        result.push([...arr]);
      }
      return;
    }

    for (let i = start; i < arr.length; i++) {
      [arr[start], arr[i]] = [arr[i], arr[start]];
      permute(arr, start + 1, i === start ? parity : 1 - parity);
      [arr[start], arr[i]] = [arr[i], arr[start]];
    }
  }

  permute([...arr], 0, 0);
  return result;
}

/**
 * Stereographic projection from S³ to ℝ³
 * Projects from the point (0,0,0,-1)
 */
export function stereographicProjection4Dto3D(
  x: number, y: number, z: number, w: number
): { x: number, y: number, z: number } {
  const denom = 1 + w;
  if (Math.abs(denom) < 1e-10) {
    // Point at infinity
    return { x: 0, y: 0, z: 1000 };
  }

  return {
    x: x / denom,
    y: y / denom,
    z: z / denom
  };
}

/**
 * Apply a 4D bi-rotor to a 600-cell and project to 2D
 * Used for animated visualization of 4D rotations
 */
export function rotate600CellAndProject(
  vertices: Vector4[],
  biRotor: BiRotor4D,
  screenW: number,
  screenH: number,
  scale: number
): { x: number, y: number, depth: number }[] {
  return vertices.map(v => {
    // Apply 4D rotation
    const rotated = applyBiRotor(biRotor, v.x, v.y, v.z, v.w);

    // Stereographic projection to 3D
    const s3 = stereographicProjection4Dto3D(rotated.x, rotated.y, rotated.z, rotated.w);

    // Perspective projection to 2D
    const fov = 4;
    const pz = s3.z + fov;
    const pScale = scale / pz;

    return {
      x: s3.x * pScale + screenW / 2,
      y: s3.y * pScale + screenH / 2,
      depth: s3.z
    };
  });
}
