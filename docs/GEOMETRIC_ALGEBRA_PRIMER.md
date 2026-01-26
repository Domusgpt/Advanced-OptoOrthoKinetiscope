# Geometric Algebra Primer for KineticLens

## Introduction

This document provides the mathematical foundation for the Geometric Algebra (GA) implementation in `GeometricAlgebra.ts`. GA unifies and generalizes complex numbers, quaternions, and matrix algebra into a single coherent framework.

---

## 1. Why Geometric Algebra?

### 1.1 The Problem with Quaternions

Quaternions are powerful for 3D rotations but have limitations:
- Non-intuitive multiplication rules (i² = j² = k² = ijk = -1)
- No clear geometric interpretation of the "w" component
- Difficult to extend to higher dimensions
- The "double cover" property is mysterious without context

### 1.2 The GA Solution

Geometric Algebra provides:
- **Clear geometric meaning** for every element
- **Dimension-independent** formulation
- **Direct physical interpretation** of rotations as "rotating in a plane"
- **Natural extension** to 4D, 8D, and beyond

---

## 2. Clifford Algebra Cl(3,0)

### 2.1 Basis Elements

In 3D Euclidean space, Cl(3,0) has 2³ = 8 basis elements:

| Grade | Elements | Geometric Meaning |
|-------|----------|-------------------|
| 0 | 1 | Scalar (magnitude) |
| 1 | e₁, e₂, e₃ | Vectors (directed lengths) |
| 2 | e₁₂, e₂₃, e₃₁ | Bivectors (oriented planes) |
| 3 | e₁₂₃ | Pseudoscalar (oriented volume) |

### 2.2 The Geometric Product

The fundamental operation is the **geometric product**, which combines:
- **Inner product** (dot product) - measures alignment
- **Outer product** (wedge product) - measures "perpendicularity"

For two vectors a and b:
```
ab = a·b + a∧b
```
Where:
- `a·b` is a scalar (grade 0)
- `a∧b` is a bivector (grade 2)

### 2.3 Key Identities

For basis vectors:
```
e₁² = e₂² = e₃² = 1    (they square to +1)
eᵢeⱼ = -eⱼeᵢ           (anti-commute when i ≠ j)
```

For bivectors:
```
e₁₂² = e₂₃² = e₃₁² = -1   (they square to -1, like imaginary units!)
```

This is why quaternions "work" - the bivectors ARE the imaginary units.

---

## 3. Rotors: The Rotation Operators

### 3.1 Definition

A **rotor** is an even-grade element of the algebra:
```
R = s + B
```
Where:
- `s` is a scalar: cos(θ/2)
- `B` is a bivector: sin(θ/2) × B̂

The bivector B̂ represents the **plane of rotation** (not the axis!).

### 3.2 Constructing a Rotor

To rotate by angle θ in the plane of bivector B:
```typescript
function rotorFromBivectorAngle(
  b12: number, b23: number, b31: number,  // Bivector components
  angle: number
): Rotor3D {
  const mag = Math.sqrt(b12*b12 + b23*b23 + b31*b31);
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
```

### 3.3 The Axis-Angle Connection

The "rotation axis" n is the **dual** of the rotation bivector:
```
B = n × I = n₁e₂₃ + n₂e₃₁ + n₃e₁₂
```

This is why:
- Rotation about Z-axis (n = e₃) → Bivector e₁₂ (XY plane)
- Rotation about X-axis (n = e₁) → Bivector e₂₃ (YZ plane)
- Rotation about Y-axis (n = e₂) → Bivector e₃₁ (ZX plane)

### 3.4 Applying a Rotor

To rotate a vector v:
```
v' = R v R̃
```

Where R̃ (R-tilde) is the **reverse** of R:
```
R̃ = s - B    (negate the bivector part)
```

```typescript
function applyRotorToVector(
  r: Rotor3D,
  vx: number, vy: number, vz: number
): Vector3 {
  // Compute R * v
  const rv = geometricProductRotorVector(r, { x: vx, y: vy, z: vz });

  // Compute (R * v) * R̃
  const rr = reverseRotor(r);
  return geometricProductMultivectorRotor(rv, rr);
}
```

### 3.5 Composing Rotations

To apply rotation R₁ then R₂:
```
R_total = R₂ × R₁
```

Note the order: R₂ comes first because we're "sandwiching" the vector:
```
v'' = R₂(R₁ v R̃₁)R̃₂ = (R₂R₁) v (R̃₁R̃₂) = R_total v R̃_total
```

---

## 4. The Quaternion Isomorphism

### 4.1 Mapping

The even subalgebra of Cl(3,0) is isomorphic to the quaternions ℍ:

| Quaternion | GA Rotor |
|------------|----------|
| 1 | 1 |
| i | e₂₃ |
| j | e₃₁ |
| k | e₁₂ |

### 4.2 Conversion Functions

```typescript
function quaternionToRotor(q: Quaternion): Rotor3D {
  return {
    s: q.w,
    e12: q.z,   // k → e₁₂
    e23: q.x,   // i → e₂₃
    e31: q.y    // j → e₃₁
  };
}

function rotorToQuaternion(r: Rotor3D): Quaternion {
  return {
    w: r.s,
    x: r.e23,
    y: r.e31,
    z: r.e12
  };
}
```

### 4.3 Why This Matters

With this isomorphism, we can:
1. Use quaternion data from device sensors
2. Convert to rotors for geometric manipulation
3. Apply GA operations (composition, interpolation)
4. Convert back for storage/transmission

---

## 5. Spinors: Quantum States

### 5.1 The Spinor as Half-Rotor

A **spinor** transforms under rotations with **half** the angle:
```
ψ' = R ψ    (single-sided action, not sandwich)
```

This is why spinors are "square roots of vectors" - applying the same spinor twice gives a full rotation.

### 5.2 Qubit Representation

A qubit state |ψ⟩ = α|0⟩ + β|1⟩ maps to:
```typescript
interface Spinor {
  real0: number;  // Re(α)
  imag0: number;  // Im(α)
  real1: number;  // Re(β)
  imag1: number;  // Im(β)
}
```

### 5.3 Bloch Sphere Coordinates

The Bloch sphere parametrizes pure qubit states:
```
|ψ⟩ = cos(θ/2)|0⟩ + e^(iφ)sin(θ/2)|1⟩
```

Where:
- θ = polar angle (0 = north pole |0⟩, π = south pole |1⟩)
- φ = azimuthal angle (phase)

```typescript
function spinorFromBlochSphere(theta: number, phi: number): Spinor {
  const cosHalf = Math.cos(theta / 2);
  const sinHalf = Math.sin(theta / 2);

  return {
    real0: cosHalf,
    imag0: 0,
    real1: sinHalf * Math.cos(phi),
    imag1: sinHalf * Math.sin(phi)
  };
}
```

### 5.4 Quantum Gates as Rotors

The key insight: **Quantum gates ARE rotors!**

| Gate | Rotation | Rotor |
|------|----------|-------|
| X (NOT) | π about X | R = (1/√2)(1 + e₂₃) |
| Y | π about Y | R = (1/√2)(1 + e₃₁) |
| Z | π about Z | R = (1/√2)(1 + e₁₂) |
| H | π about (X+Z)/√2 | R = (1/√2)(1 + (e₂₃+e₁₂)/√2) |

```typescript
function applyRotorToSpinor(r: Rotor3D, s: Spinor): Spinor {
  // The rotor acts on the spinor via SU(2) representation
  // R = a + b·σ where σ are Pauli matrices
  const ar = r.s;
  const ai = r.e12;
  const br = r.e31;
  const bi = r.e23;

  return {
    real0: ar*s.real0 - ai*s.imag0 - br*s.real1 - bi*s.imag1,
    imag0: ar*s.imag0 + ai*s.real0 - br*s.imag1 + bi*s.real1,
    real1: br*s.real0 - bi*s.imag0 + ar*s.real1 + ai*s.imag1,
    imag1: br*s.imag0 + bi*s.real0 + ar*s.imag1 - ai*s.real1
  };
}
```

---

## 6. Four Dimensions: Cl(4,0) and Bi-Rotors

### 6.1 The Problem in 4D

In 3D, a general rotation has 3 degrees of freedom (axis + angle).
In 4D, a general rotation has **6** degrees of freedom.

The key insight: 4D rotations decompose into **two independent isoclinic rotations**.

### 6.2 Left and Right Isoclinic

Every 4D rotation R can be written as:
```
R = L ⊗ R
```

Where:
- L is a "left-isoclinic" rotation (rotates one pair of planes)
- R is a "right-isoclinic" rotation (rotates the orthogonal pair)

### 6.3 The Bi-Rotor Structure

```typescript
interface BiRotor4D {
  left: Rotor3D;   // Left-isoclinic rotation
  right: Rotor3D;  // Right-isoclinic rotation
}
```

### 6.4 Applying a 4D Rotation

For a 4D point q = (x, y, z, w), treating it as a quaternion:
```
q' = L × q × R̃
```

This is the "double quaternion" action:
```typescript
function applyBiRotor(
  br: BiRotor4D,
  x: number, y: number, z: number, w: number
): Vector4 {
  // Encode point as quaternion
  // Apply L from left, R̃ from right
  const L = br.left;
  const R = reverseRotor(br.right);

  // L * q
  const lw = L.s*w - L.e23*x - L.e31*y - L.e12*z;
  const lx = L.s*x + L.e23*w + L.e31*z - L.e12*y;
  const ly = L.s*y - L.e23*z + L.e31*w + L.e12*x;
  const lz = L.s*z + L.e23*y - L.e31*x + L.e12*w;

  // (L*q) * R̃
  return {
    x: lw*R.e23 + lx*R.s - ly*R.e12 + lz*R.e31,
    y: lw*R.e31 + lx*R.e12 + ly*R.s - lz*R.e23,
    z: lw*R.e12 - lx*R.e31 + ly*R.e23 + lz*R.s,
    w: lw*R.s - lx*R.e23 - ly*R.e31 - lz*R.e12
  };
}
```

### 6.5 Clifford Rotations

A **Clifford rotation** is a simple isoclinic where L = R:
```typescript
function cliffordRotation(angle: number, plane: string): BiRotor4D {
  const halfAngle = angle / 2;
  const c = Math.cos(halfAngle);
  const s = Math.sin(halfAngle);

  // XY plane rotation
  const rotor = { s: c, e12: s, e23: 0, e31: 0 };
  return { left: rotor, right: rotor };
}
```

---

## 7. Spherical Linear Interpolation (SLERP)

### 7.1 Why Not Linear Interpolation?

Linear interpolation (LERP) between rotors produces incorrect results:
- The "speed" varies (slow at start, fast in middle, slow at end)
- The path doesn't follow the shortest arc on the rotation manifold

### 7.2 SLERP Formula

For rotors R₁ and R₂, interpolation parameter t ∈ [0,1]:
```
SLERP(R₁, R₂, t) = R₁ × (R₁⁻¹ × R₂)^t
```

Practically:
```typescript
function slerpRotor(r1: Rotor3D, r2: Rotor3D, t: number): Rotor3D {
  // Compute dot product (cosine of angle)
  let dot = r1.s*r2.s + r1.e12*r2.e12 + r1.e23*r2.e23 + r1.e31*r2.e31;

  // Ensure shortest path
  let r2n = r2;
  if (dot < 0) {
    r2n = negateRotor(r2);
    dot = -dot;
  }

  // If very close, use LERP
  if (dot > 0.9995) {
    return normalizeRotor(lerpRotor(r1, r2n, t));
  }

  // Standard SLERP
  const theta = Math.acos(dot);
  const sinTheta = Math.sin(theta);
  const w1 = Math.sin((1-t) * theta) / sinTheta;
  const w2 = Math.sin(t * theta) / sinTheta;

  return {
    s: w1*r1.s + w2*r2n.s,
    e12: w1*r1.e12 + w2*r2n.e12,
    e23: w1*r1.e23 + w2*r2n.e23,
    e31: w1*r1.e31 + w2*r2n.e31
  };
}
```

---

## 8. Practical Usage Patterns

### 8.1 Device Orientation to Rotor

```typescript
// From DeviceOrientation event
function handleOrientation(event: DeviceOrientationEvent) {
  const { alpha, beta, gamma } = event;

  // Convert to quaternion (standard Web API method)
  const q = eulerToQuaternion(alpha, beta, gamma);

  // Convert to rotor for GA operations
  const rotor = quaternionToRotor(q);

  // Now we can compose, interpolate, etc.
}
```

### 8.2 Relative Rotation

To find the rotation FROM orientation A TO orientation B:
```typescript
const rotorA = quaternionToRotor(orientationA);
const rotorB = quaternionToRotor(orientationB);

// R_diff such that B = R_diff × A
const rotorDiff = multiplyRotors(rotorB, reverseRotor(rotorA));

// Extract axis and angle
const { axis, angle } = rotorToAxisAngle(rotorDiff);
```

### 8.3 Smooth Camera Stabilization

```typescript
// Exponential smoothing with rotors
let smoothedRotor = ROTOR_IDENTITY;
const alpha = 0.1;  // Smoothing factor

function updateStabilization(newOrientation: Quaternion) {
  const newRotor = quaternionToRotor(newOrientation);

  // SLERP towards new value
  smoothedRotor = slerpRotor(smoothedRotor, newRotor, alpha);

  // Apply inverse to cancel camera motion
  const stabilizingRotor = reverseRotor(smoothedRotor);

  return stabilizingRotor;
}
```

---

## 9. Performance Considerations

### 9.1 Operation Costs

| Operation | Multiplications | Additions | Notes |
|-----------|-----------------|-----------|-------|
| Rotor × Rotor | 16 | 12 | Like quaternion multiply |
| Rotor × Vector × Rotor̃ | 48 | 32 | Full sandwich product |
| SLERP (one step) | 20 | 16 | Plus trig functions |
| Normalize | 4 | 3 | Plus sqrt |

### 9.2 Optimizations

1. **Pre-compute reverses**: If applying the same rotation many times, compute R̃ once.

2. **Batch transforms**: When rotating many vectors by the same rotor, precompute the rotation matrix.

3. **Use LERP when possible**: For small interpolation steps (t < 0.1), LERP + normalize is faster and nearly identical.

---

## 10. Connections to the Codebase

### 10.1 GeometricAlgebra.ts Functions

| Function | Purpose |
|----------|---------|
| `rotorFromAxisAngle` | Create rotor from traditional axis-angle |
| `rotorFromBivectorAngle` | Create rotor from plane + angle (more GA-native) |
| `multiplyRotors` | Compose two rotations |
| `applyRotorToVector` | Rotate a 3D vector |
| `applyRotorToSpinor` | Apply quantum gate to qubit |
| `slerpRotor` | Smooth interpolation |
| `quaternionToRotor` | Convert from sensor data |
| `createBiRotor` | Create 4D double rotation |
| `applyBiRotor` | Rotate 4D point |

### 10.2 Integration Points

- **cpeMath.ts**: Uses quaternions from sensors, converts to rotors for 24-cell rotation
- **MoireEngine.ts**: Uses rotor angle extraction for grating phase calculation
- **PhillipsGateSimulator.ts**: Uses rotors as quantum gates on spinor states
- **E8Projection.ts**: Uses bi-rotors for 4D polytope rotation

---

## Appendix: Recommended Reading

1. **"Geometric Algebra for Physicists"** by Doran & Lasenby - Comprehensive textbook
2. **"Geometric Algebra for Computer Science"** by Dorst, Fontijne, Mann - Practical applications
3. **David Hestenes' papers** - The inventor of modern GA
4. **bivector.net** - Online GA calculator and tutorials

---

*Document Version: 1.0.0*
*Last Updated: 2026-01-22*
*Module: GeometricAlgebra.ts*
