/**
 * GEOMETRIC ALGEBRA ENGINE - Cl(3,0) and Cl(4,0)
 *
 * Implements Clifford Algebra for visual analog computation.
 * Core concepts:
 * - Multivectors: Complete geometric objects (scalars, vectors, bivectors, trivectors)
 * - Rotors: Even-grade elements encoding rotations without gimbal lock
 * - Spinors: Half-angle representations for quantum state evolution
 * - Versors: Generalized reflections and rotations
 *
 * The mathematical isomorphism:
 *   Cl(3,0)_even ≅ ℍ (Quaternions)
 *   Cl(4,0)_even ≅ ℍ⊗ℍ (Biquaternions - Left/Right isoclinic)
 */

// ═══════════════════════════════════════════════════════════════════════════
// TYPE DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Multivector in Cl(3,0) - 8 basis elements
 * e0 = 1 (scalar)
 * e1, e2, e3 (vectors)
 * e12, e23, e31 (bivectors - oriented planes)
 * e123 = I (pseudoscalar - oriented volume)
 */
export interface Multivector3D {
  s: number;      // Scalar (grade 0)
  e1: number;     // Vector x (grade 1)
  e2: number;     // Vector y (grade 1)
  e3: number;     // Vector z (grade 1)
  e12: number;    // Bivector xy (grade 2)
  e23: number;    // Bivector yz (grade 2)
  e31: number;    // Bivector zx (grade 2)
  e123: number;   // Pseudoscalar (grade 3)
}

/**
 * Rotor in Cl(3,0) - Even subalgebra (scalar + bivector)
 * R = cos(θ/2) + sin(θ/2) * B̂
 * where B̂ is a unit bivector representing the rotation plane
 */
export interface Rotor3D {
  s: number;      // cos(θ/2)
  e12: number;    // sin(θ/2) * B_xy
  e23: number;    // sin(θ/2) * B_yz
  e31: number;    // sin(θ/2) * B_zx
}

/**
 * Spinor for quantum state representation
 * |ψ⟩ = α|0⟩ + β|1⟩ where α, β ∈ ℂ
 * Encoded as: ψ = α + β * e12 (complex subalgebra of Cl(3,0))
 */
export interface Spinor {
  real0: number;  // Re(α)
  imag0: number;  // Im(α)
  real1: number;  // Re(β)
  imag1: number;  // Im(β)
}

/**
 * Multivector in Cl(4,0) - 16 basis elements
 * Extends Cl(3,0) with e4 for 4D double isoclinic rotations
 */
export interface Multivector4D {
  // Grade 0
  s: number;
  // Grade 1
  e1: number; e2: number; e3: number; e4: number;
  // Grade 2
  e12: number; e13: number; e14: number;
  e23: number; e24: number; e34: number;
  // Grade 3
  e123: number; e124: number; e134: number; e234: number;
  // Grade 4
  e1234: number;
}

/**
 * Bi-rotor for 4D double isoclinic rotation
 * Decomposes into left-isoclinic (L) and right-isoclinic (R) components
 * Total rotation = L ⊗ R
 */
export interface BiRotor4D {
  left: Rotor3D;   // Left-isoclinic rotation
  right: Rotor3D;  // Right-isoclinic rotation
}

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════

/** Identity rotor (no rotation) */
export const ROTOR_IDENTITY: Rotor3D = { s: 1, e12: 0, e23: 0, e31: 0 };

/** Null multivector */
export const MV3D_ZERO: Multivector3D = {
  s: 0, e1: 0, e2: 0, e3: 0, e12: 0, e23: 0, e31: 0, e123: 0
};

/** Pseudoscalar (unit oriented volume) */
export const PSEUDOSCALAR: Multivector3D = {
  s: 0, e1: 0, e2: 0, e3: 0, e12: 0, e23: 0, e31: 0, e123: 1
};

/** Pauli matrices encoded as rotors (quarter-turn generators) */
export const PAULI_X: Rotor3D = { s: Math.SQRT1_2, e12: 0, e23: Math.SQRT1_2, e31: 0 };
export const PAULI_Y: Rotor3D = { s: Math.SQRT1_2, e12: 0, e23: 0, e31: Math.SQRT1_2 };
export const PAULI_Z: Rotor3D = { s: Math.SQRT1_2, e12: Math.SQRT1_2, e23: 0, e31: 0 };

// ═══════════════════════════════════════════════════════════════════════════
// MULTIVECTOR ARITHMETIC
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Geometric product of two 3D multivectors
 * This is the fundamental operation of Geometric Algebra
 * AB = A·B + A∧B (inner + outer product combined)
 */
export function geometricProduct3D(a: Multivector3D, b: Multivector3D): Multivector3D {
  // This is the full 8x8 multiplication table for Cl(3,0)
  // Using the relations: e1² = e2² = e3² = 1, eiej = -ejei for i≠j

  return {
    s: a.s*b.s + a.e1*b.e1 + a.e2*b.e2 + a.e3*b.e3
       - a.e12*b.e12 - a.e23*b.e23 - a.e31*b.e31 - a.e123*b.e123,

    e1: a.s*b.e1 + a.e1*b.s - a.e2*b.e12 + a.e3*b.e31
        + a.e12*b.e2 - a.e23*b.e123 - a.e31*b.e3 - a.e123*b.e23,

    e2: a.s*b.e2 + a.e1*b.e12 + a.e2*b.s - a.e3*b.e23
        - a.e12*b.e1 + a.e23*b.e3 - a.e31*b.e123 - a.e123*b.e31,

    e3: a.s*b.e3 - a.e1*b.e31 + a.e2*b.e23 + a.e3*b.s
        - a.e12*b.e123 - a.e23*b.e2 + a.e31*b.e1 - a.e123*b.e12,

    e12: a.s*b.e12 + a.e1*b.e2 - a.e2*b.e1 + a.e3*b.e123
         + a.e12*b.s - a.e23*b.e31 + a.e31*b.e23 + a.e123*b.e3,

    e23: a.s*b.e23 + a.e1*b.e123 + a.e2*b.e3 - a.e3*b.e2
         + a.e12*b.e31 + a.e23*b.s - a.e31*b.e12 + a.e123*b.e1,

    e31: a.s*b.e31 - a.e1*b.e3 + a.e2*b.e123 + a.e3*b.e1
         - a.e12*b.e23 + a.e23*b.e12 + a.e31*b.s + a.e123*b.e2,

    e123: a.s*b.e123 + a.e1*b.e23 + a.e2*b.e31 + a.e3*b.e12
          + a.e12*b.e3 + a.e23*b.e1 + a.e31*b.e2 + a.e123*b.s
  };
}

/**
 * Add two multivectors
 */
export function addMV3D(a: Multivector3D, b: Multivector3D): Multivector3D {
  return {
    s: a.s + b.s,
    e1: a.e1 + b.e1, e2: a.e2 + b.e2, e3: a.e3 + b.e3,
    e12: a.e12 + b.e12, e23: a.e23 + b.e23, e31: a.e31 + b.e31,
    e123: a.e123 + b.e123
  };
}

/**
 * Scale a multivector by a scalar
 */
export function scaleMV3D(a: Multivector3D, k: number): Multivector3D {
  return {
    s: a.s * k,
    e1: a.e1 * k, e2: a.e2 * k, e3: a.e3 * k,
    e12: a.e12 * k, e23: a.e23 * k, e31: a.e31 * k,
    e123: a.e123 * k
  };
}

/**
 * Reverse (reversion) of a multivector
 * Reverses the order of basis vectors in each term
 * For rotors: R̃ = cos(θ/2) - sin(θ/2)B̂ (the inverse rotation)
 */
export function reverseMV3D(a: Multivector3D): Multivector3D {
  return {
    s: a.s,
    e1: a.e1, e2: a.e2, e3: a.e3,
    e12: -a.e12, e23: -a.e23, e31: -a.e31,  // Grade 2 reverses sign
    e123: -a.e123  // Grade 3 reverses sign
  };
}

/**
 * Conjugate of a multivector (grade involution + reversion)
 */
export function conjugateMV3D(a: Multivector3D): Multivector3D {
  return {
    s: a.s,
    e1: -a.e1, e2: -a.e2, e3: -a.e3,
    e12: -a.e12, e23: -a.e23, e31: -a.e31,
    e123: a.e123
  };
}

/**
 * Magnitude squared of a multivector
 */
export function normSquaredMV3D(a: Multivector3D): number {
  return a.s*a.s + a.e1*a.e1 + a.e2*a.e2 + a.e3*a.e3
       + a.e12*a.e12 + a.e23*a.e23 + a.e31*a.e31 + a.e123*a.e123;
}

// ═══════════════════════════════════════════════════════════════════════════
// ROTOR OPERATIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Create a rotor from an axis-angle representation
 * R = cos(θ/2) + sin(θ/2) * (axis × e123)
 * Note: The bivector is the dual of the axis (axis wedge product with pseudoscalar)
 */
export function rotorFromAxisAngle(
  axisX: number, axisY: number, axisZ: number,
  angle: number
): Rotor3D {
  // Normalize axis
  const mag = Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);
  if (mag < 1e-10) return ROTOR_IDENTITY;

  const nx = axisX / mag;
  const ny = axisY / mag;
  const nz = axisZ / mag;

  const halfAngle = angle / 2;
  const c = Math.cos(halfAngle);
  const s = Math.sin(halfAngle);

  // The bivector dual to axis (n) is: n·I = n1*e23 + n2*e31 + n3*e12
  // But the rotation bivector is actually -n·I for conventional handedness
  return {
    s: c,
    e12: -s * nz,  // Rotation about z-axis
    e23: -s * nx,  // Rotation about x-axis
    e31: -s * ny   // Rotation about y-axis
  };
}

/**
 * Create a rotor directly from a bivector plane and angle
 * More geometrically intuitive: specify the plane of rotation directly
 */
export function rotorFromBivectorAngle(
  b12: number, b23: number, b31: number,
  angle: number
): Rotor3D {
  // Normalize bivector
  const mag = Math.sqrt(b12*b12 + b23*b23 + b31*b31);
  if (mag < 1e-10) return ROTOR_IDENTITY;

  const halfAngle = angle / 2;
  const c = Math.cos(halfAngle);
  const s = Math.sin(halfAngle) / mag;

  return {
    s: c,
    e12: s * b12,
    e23: s * b23,
    e31: s * b31
  };
}

/**
 * Multiply two rotors (compose rotations)
 * R_total = R2 * R1 (apply R1 first, then R2)
 */
export function multiplyRotors(r1: Rotor3D, r2: Rotor3D): Rotor3D {
  // Rotor multiplication in the even subalgebra of Cl(3,0)
  // Equivalent to quaternion multiplication
  return {
    s: r1.s*r2.s - r1.e12*r2.e12 - r1.e23*r2.e23 - r1.e31*r2.e31,
    e12: r1.s*r2.e12 + r1.e12*r2.s + r1.e23*r2.e31 - r1.e31*r2.e23,
    e23: r1.s*r2.e23 - r1.e12*r2.e31 + r1.e23*r2.s + r1.e31*r2.e12,
    e31: r1.s*r2.e31 + r1.e12*r2.e23 - r1.e23*r2.e12 + r1.e31*r2.s
  };
}

/**
 * Rotor reverse (conjugate) - gives the inverse rotation
 */
export function reverseRotor(r: Rotor3D): Rotor3D {
  return { s: r.s, e12: -r.e12, e23: -r.e23, e31: -r.e31 };
}

/**
 * Normalize a rotor to unit magnitude
 */
export function normalizeRotor(r: Rotor3D): Rotor3D {
  const mag = Math.sqrt(r.s*r.s + r.e12*r.e12 + r.e23*r.e23 + r.e31*r.e31);
  if (mag < 1e-10) return ROTOR_IDENTITY;
  return { s: r.s/mag, e12: r.e12/mag, e23: r.e23/mag, e31: r.e31/mag };
}

/**
 * Apply a rotor to a vector using the sandwich product: v' = R v R̃
 * This is the fundamental rotation operation
 */
export function applyRotorToVector(
  r: Rotor3D,
  vx: number, vy: number, vz: number
): { x: number, y: number, z: number } {
  // Expand v as a multivector (grade-1 only)
  // Then compute R * v * R̃

  // First: R * v
  const rv_s = -r.e12*vz - r.e23*vx - r.e31*vy;
  const rv_e1 = r.s*vx + r.e12*vy - r.e31*vz;
  const rv_e2 = r.s*vy - r.e12*vx + r.e23*vz;
  const rv_e3 = r.s*vz + r.e31*vx - r.e23*vy;
  const rv_e123 = r.e12*vz + r.e23*vx + r.e31*vy;

  // Second: (R*v) * R̃
  const rr = reverseRotor(r);

  return {
    x: rv_s*(-rr.e23) + rv_e1*rr.s + rv_e2*(-rr.e12) + rv_e3*rr.e31 + rv_e123*(-rr.e23),
    y: rv_s*(-rr.e31) + rv_e1*rr.e12 + rv_e2*rr.s + rv_e3*(-rr.e23) + rv_e123*(-rr.e31),
    z: rv_s*(-rr.e12) + rv_e1*(-rr.e31) + rv_e2*rr.e23 + rv_e3*rr.s + rv_e123*(-rr.e12)
  };
}

/**
 * Extract axis and angle from a rotor
 */
export function rotorToAxisAngle(r: Rotor3D): {
  axis: { x: number, y: number, z: number },
  angle: number
} {
  // Clamp scalar to valid range for acos
  const s = Math.max(-1, Math.min(1, r.s));
  const halfAngle = Math.acos(s);
  const angle = 2 * halfAngle;

  const sinHalf = Math.sin(halfAngle);
  if (Math.abs(sinHalf) < 1e-10) {
    return { axis: { x: 0, y: 0, z: 1 }, angle: 0 };
  }

  // The axis is the dual of the bivector part
  return {
    axis: {
      x: -r.e23 / sinHalf,
      y: -r.e31 / sinHalf,
      z: -r.e12 / sinHalf
    },
    angle
  };
}

/**
 * Spherical Linear Interpolation between two rotors
 * Smooth interpolation along the shortest arc on S³
 */
export function slerpRotor(r1: Rotor3D, r2: Rotor3D, t: number): Rotor3D {
  // Compute dot product (cosine of angle between rotors)
  let dot = r1.s*r2.s + r1.e12*r2.e12 + r1.e23*r2.e23 + r1.e31*r2.e31;

  // If negative dot, negate one rotor for shortest path
  let r2n = r2;
  if (dot < 0) {
    r2n = { s: -r2.s, e12: -r2.e12, e23: -r2.e23, e31: -r2.e31 };
    dot = -dot;
  }

  // If very close, use linear interpolation
  if (dot > 0.9995) {
    return normalizeRotor({
      s: r1.s + t * (r2n.s - r1.s),
      e12: r1.e12 + t * (r2n.e12 - r1.e12),
      e23: r1.e23 + t * (r2n.e23 - r1.e23),
      e31: r1.e31 + t * (r2n.e31 - r1.e31)
    });
  }

  // Standard SLERP
  const theta = Math.acos(dot);
  const sinTheta = Math.sin(theta);
  const w1 = Math.sin((1 - t) * theta) / sinTheta;
  const w2 = Math.sin(t * theta) / sinTheta;

  return {
    s: w1 * r1.s + w2 * r2n.s,
    e12: w1 * r1.e12 + w2 * r2n.e12,
    e23: w1 * r1.e23 + w2 * r2n.e23,
    e31: w1 * r1.e31 + w2 * r2n.e31
  };
}

// ═══════════════════════════════════════════════════════════════════════════
// SPINOR / QUANTUM STATE OPERATIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Create a spinor from Bloch sphere coordinates (θ, φ)
 * |ψ⟩ = cos(θ/2)|0⟩ + e^(iφ)sin(θ/2)|1⟩
 */
export function spinorFromBlochSphere(theta: number, phi: number): Spinor {
  const cosHalf = Math.cos(theta / 2);
  const sinHalf = Math.sin(theta / 2);

  return {
    real0: cosHalf,
    imag0: 0,
    real1: sinHalf * Math.cos(phi),
    imag1: sinHalf * Math.sin(phi)
  };
}

/**
 * Convert spinor back to Bloch sphere coordinates
 */
export function spinorToBlochSphere(s: Spinor): { theta: number, phi: number } {
  const mag0 = Math.sqrt(s.real0*s.real0 + s.imag0*s.imag0);
  const mag1 = Math.sqrt(s.real1*s.real1 + s.imag1*s.imag1);

  const theta = 2 * Math.atan2(mag1, mag0);

  // Phase difference
  const phase0 = Math.atan2(s.imag0, s.real0);
  const phase1 = Math.atan2(s.imag1, s.real1);
  const phi = phase1 - phase0;

  return { theta, phi };
}

/**
 * Apply a rotor to a spinor (quantum gate operation)
 * This is the key insight: GA rotors ARE quantum gates!
 */
export function applyRotorToSpinor(r: Rotor3D, s: Spinor): Spinor {
  // The rotor acts on the spinor via left multiplication
  // In the isomorphism, this corresponds to SU(2) action on ℂ²

  // Map rotor to 2x2 complex matrix and apply
  // [a -b*] [α]   where a = r.s + i*r.e12
  // [b  a*] [β]         b = r.e31 + i*r.e23

  const ar = r.s;
  const ai = r.e12;
  const br = r.e31;
  const bi = r.e23;

  // Complex multiplication for 2x2 matrix-vector product
  return {
    real0: ar*s.real0 - ai*s.imag0 - br*s.real1 - bi*s.imag1,
    imag0: ar*s.imag0 + ai*s.real0 - br*s.imag1 + bi*s.real1,
    real1: br*s.real0 - bi*s.imag0 + ar*s.real1 + ai*s.imag1,
    imag1: br*s.imag0 + bi*s.real0 + ar*s.imag1 - ai*s.real1
  };
}

/**
 * Normalize a spinor to unit probability
 */
export function normalizeSpinor(s: Spinor): Spinor {
  const mag = Math.sqrt(
    s.real0*s.real0 + s.imag0*s.imag0 +
    s.real1*s.real1 + s.imag1*s.imag1
  );
  if (mag < 1e-10) return { real0: 1, imag0: 0, real1: 0, imag1: 0 };

  return {
    real0: s.real0 / mag,
    imag0: s.imag0 / mag,
    real1: s.real1 / mag,
    imag1: s.imag1 / mag
  };
}

/**
 * Compute measurement probabilities for a spinor
 */
export function spinorProbabilities(s: Spinor): { p0: number, p1: number } {
  const p0 = s.real0*s.real0 + s.imag0*s.imag0;
  const p1 = s.real1*s.real1 + s.imag1*s.imag1;
  return { p0, p1 };
}

// ═══════════════════════════════════════════════════════════════════════════
// 4D DOUBLE ISOCLINIC ROTATIONS (For 24-Cell/600-Cell)
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Create a 4D double isoclinic rotation from two quaternion-like rotors
 * In 4D, a general rotation decomposes into left and right isoclinic parts
 * This is the key to the "Phillips Gate" - each isoclinic half is a quantum operation
 */
export function createBiRotor(left: Rotor3D, right: Rotor3D): BiRotor4D {
  return {
    left: normalizeRotor(left),
    right: normalizeRotor(right)
  };
}

/**
 * Apply a 4D double isoclinic rotation to a 4D point
 * v' = L * v * R̃ (quaternion sandwich from both sides)
 */
export function applyBiRotor(
  br: BiRotor4D,
  x: number, y: number, z: number, w: number
): { x: number, y: number, z: number, w: number } {
  // Encode the 4D point as a quaternion
  // Then apply left multiplication by L and right multiplication by R̃

  const L = br.left;
  const R = reverseRotor(br.right);

  // First: L * q (left quaternion multiplication)
  // q = w + x*i + y*j + z*k
  const lw = L.s*w - L.e23*x - L.e31*y - L.e12*z;
  const lx = L.s*x + L.e23*w + L.e31*z - L.e12*y;
  const ly = L.s*y - L.e23*z + L.e31*w + L.e12*x;
  const lz = L.s*z + L.e23*y - L.e31*x + L.e12*w;

  // Then: (L*q) * R̃ (right quaternion multiplication)
  const rw = lw*R.s - lx*R.e23 - ly*R.e31 - lz*R.e12;
  const rx = lw*R.e23 + lx*R.s - ly*R.e12 + lz*R.e31;
  const ry = lw*R.e31 + lx*R.e12 + ly*R.s - lz*R.e23;
  const rz = lw*R.e12 - lx*R.e31 + ly*R.e23 + lz*R.s;

  return { x: rx, y: ry, z: rz, w: rw };
}

/**
 * Compose two bi-rotors
 */
export function multiplyBiRotors(br1: BiRotor4D, br2: BiRotor4D): BiRotor4D {
  return {
    left: multiplyRotors(br2.left, br1.left),
    right: multiplyRotors(br1.right, br2.right)
  };
}

/**
 * Create a simple isoclinic rotation (same angle in both isoclinic subspaces)
 * This creates a "Clifford rotation" - the simplest 4D rotation
 */
export function cliffordRotation(angle: number, plane: 'xy' | 'xz' | 'xw' | 'yz' | 'yw' | 'zw'): BiRotor4D {
  const halfAngle = angle / 2;
  const c = Math.cos(halfAngle);
  const s = Math.sin(halfAngle);

  // Different planes correspond to different isoclinic decompositions
  switch (plane) {
    case 'xy':
      return createBiRotor(
        { s: c, e12: s, e23: 0, e31: 0 },
        { s: c, e12: s, e23: 0, e31: 0 }
      );
    case 'xz':
      return createBiRotor(
        { s: c, e12: 0, e23: 0, e31: s },
        { s: c, e12: 0, e23: 0, e31: s }
      );
    case 'xw':
      return createBiRotor(
        { s: c, e12: 0, e23: s, e31: 0 },
        { s: c, e12: 0, e23: -s, e31: 0 }
      );
    case 'yz':
      return createBiRotor(
        { s: c, e12: 0, e23: s, e31: 0 },
        { s: c, e12: 0, e23: s, e31: 0 }
      );
    case 'yw':
      return createBiRotor(
        { s: c, e12: 0, e23: 0, e31: s },
        { s: c, e12: 0, e23: 0, e31: -s }
      );
    case 'zw':
      return createBiRotor(
        { s: c, e12: s, e23: 0, e31: 0 },
        { s: c, e12: -s, e23: 0, e31: 0 }
      );
    default:
      return createBiRotor(ROTOR_IDENTITY, ROTOR_IDENTITY);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY: CONVERSION TO/FROM QUATERNIONS
// ═══════════════════════════════════════════════════════════════════════════

import { Quaternion } from "../types";

/**
 * Convert a quaternion to a rotor
 * Quaternion: q = w + xi + yj + zk
 * Rotor: R = s + e12*xy + e23*yz + e31*zx
 * The isomorphism is: i↔e23, j↔e31, k↔e12
 */
export function quaternionToRotor(q: Quaternion): Rotor3D {
  return {
    s: q.w,
    e12: q.z,   // k component
    e23: q.x,   // i component
    e31: q.y    // j component
  };
}

/**
 * Convert a rotor back to a quaternion
 */
export function rotorToQuaternion(r: Rotor3D): Quaternion {
  return {
    w: r.s,
    x: r.e23,
    y: r.e31,
    z: r.e12
  };
}
