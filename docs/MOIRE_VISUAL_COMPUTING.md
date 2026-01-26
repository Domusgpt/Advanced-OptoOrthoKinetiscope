# Moiré Visual Computing Engine

## Theory of Operation

### Executive Summary

The Moiré Visual Computing Engine replaces textual data readouts with **geometric visual patterns** that Vision Language Models (Vision LLMs) can interpret more reliably. This approach leverages the fundamental insight that:

> **Vision Transformers excel at spatial relationships but struggle with arithmetic and small text.**

By encoding numerical values as geometric constructs (gratings, nomograms, flow fields), we transform computational problems into pattern recognition tasks.

---

## 1. Mathematical Foundations

### 1.1 The Moiré Phenomenon

When two periodic patterns with slightly different spatial frequencies are superimposed, a low-frequency "beat" pattern emerges. This is the **moiré effect**.

#### Indicial Equation

For two families of curves defined by:
- Pattern 1: ψ₁(x, y) = k (integer)
- Pattern 2: ψ₂(x, y) = l (integer)

The moiré fringes occur where:

```
ψ₁(x, y) - ψ₂(x, y) = m    (m is an integer)
```

#### Linear Grating Example

Two linear gratings with periods p₁ and p₂, separated by angle θ:

```
Moiré spacing D = (p₁ × p₂) / √(p₁² + p₂² - 2p₁p₂cos(θ))
```

**Key insight**: As θ → 0, the moiré spacing D → ∞. This means **infinitesimal rotations produce large fringe shifts**.

### 1.2 Mechanical Amplification

The magnification factor M is defined as:

```
M = D / min(p₁, p₂) = p / Δp
```

Where Δp is the difference in periods. For a 1% difference in periods:

```
M = 1 / 0.01 = 100×
```

This 100× magnification allows sub-pixel precision in displacement detection.

### 1.3 The Log-Polar Conformal Map

The transformation w = ln(z) maps the complex plane to a cylinder:

```
z = r × e^(iθ)  →  w = ln(r) + iθ
```

Under this mapping:
- **Rotation** (multiplication by e^(iφ)) → **Vertical translation** by φ
- **Scaling** (multiplication by k) → **Horizontal translation** by ln(k)

This transforms multiplicative operations into additive ones, enabling **visual multiplication** through grating superposition.

---

## 2. Implementation Architecture

### 2.1 Module Structure

```
services/
├── MoireEngine.ts          # Core moiré computation
├── GeometricAlgebra.ts     # Rotor/spinor math (enables moiré interpretation)
├── E8Projection.ts         # High-dimensional structures for complex moiré
├── PhillipsGateSimulator.ts # Quantum simulation using moiré visualization
└── imageCompositor.ts      # Renders moiré overlays to canvas
```

### 2.2 Grating Types

| Type | Equation | Encodes | Use Case |
|------|----------|---------|----------|
| Linear | y = mx + b | Translation, shear | Flow field visualization |
| Circular | r² = x² + y² | Focus, distance | Depth measurement |
| Log-Polar | w = ln(z) | Rotation AND scale | Quaternion visualization |
| Hyperbolic | x² - y² = c | Squeeze/boost | Lorentz transformations |
| Spiral | r = ae^(bθ) | Complex phase | Spinor state display |

### 2.3 Data Flow

```
Sensor Data (Quaternion, Acceleration, Gyro)
    ↓
GeometricAlgebra.ts (Convert to Rotor)
    ↓
MoireEngine.ts (Generate gratings based on rotor parameters)
    ↓
imageCompositor.ts (Superimpose gratings on camera feed)
    ↓
Composite Image with Moiré Patterns
    ↓
Vision LLM (Reads fringe positions instead of numbers)
```

---

## 3. Core Functions Reference

### 3.1 Grating Generation

```typescript
// Generate a log-polar grating for quaternion visualization
generateLogPolarGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number,      // Position
  w: number, h: number,      // Dimensions
  params: {
    frequency: number,       // Radial lines count
    phase: number,           // Rotation offset (encodes quaternion angle)
    centerX: number,
    centerY: number
  },
  color: string
): void
```

**How it works:**
1. Draws radial lines from center (encode rotation angle)
2. Draws logarithmic spirals (encode scale factor)
3. When two gratings overlap, moiré reveals the difference

### 3.2 Moiré Computation

```typescript
// Compute moiré fringe parameters from two linear gratings
computeLinearMoire(
  g1: { frequency: number, angle: number },
  g2: { frequency: number, angle: number }
): MoireResult

interface MoireResult {
  fringeSpacing: number;    // Distance between visible fringes
  fringeAngle: number;      // Orientation of fringe pattern
  magnification: number;    // Amplification factor (can be 100×+)
  beatFrequency: number;    // Low-frequency envelope
  phaseShift: number;       // Detected phase difference
}
```

### 3.3 Quaternion Moiré Display

```typescript
// Visualize a quaternion as overlapping toroidal gratings
drawQuaternionMoire(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  qw: number, qx: number, qy: number, qz: number,
  referenceQ: Quaternion | null  // If provided, shows difference
): void
```

**Visual Output:**
- Gold grating: Reference orientation (identity or specified)
- Cyan grating: Current orientation
- Rose marker: Intersection point indicating rotation magnitude
- Moiré fringes: Beat pattern reveals rotation axis/angle

---

## 4. Nomograms: Visual Analog Calculators

### 4.1 Concept

A **nomogram** is a graphical calculating device. Instead of computing:

```
Speed = Distance / Time
```

The AI visually connects points on two scales and reads the answer where the line crosses a third scale.

### 4.2 Implementation

```typescript
// Create a velocity nomogram
const nomogram = createVelocityNomogram(
  [16, 1000],   // Time range (ms)
  [0.1, 10]     // Distance range (m)
);

// Set current values
nomogram.currentA = 100;  // 100ms
nomogram.currentB = 1.5;  // 1.5m

// The computation is automatic
nomogram.computedC = 15;  // 15 m/s
```

### 4.3 Visual Rendering

```typescript
drawNomogram(ctx, x, y, w, h, nomogram, {
  bg: '#0f172a',       // Background color
  scale: '#94a3b8',    // Scale line color
  line: '#FACC15',     // Computation line (gold)
  marker: '#F43F5E'    // Data point markers (rose)
});
```

**How the AI reads it:**
1. Locate the red dot on the TIME scale (left)
2. Locate the cyan dot on the DISTANCE scale (right)
3. Draw an imaginary line between them
4. Read where it crosses the SPEED scale (center)
5. The gold dot marks the answer

---

## 5. Differential Flow Field

### 5.1 Purpose

Isolate **subject motion** from **camera motion** by comparing:
- Expected motion (from gyroscope)
- Observed motion (from optical flow)

```
Subject_Motion = Observed_Flow - Expected_Flow
```

### 5.2 Visual Encoding

| Color | Meaning | Source |
|-------|---------|--------|
| Gold/Yellow | Expected flow | Gyroscope rotation rate |
| Cyan | Observed flow | Optical flow analysis |
| Rose | Subject motion | Difference (the answer) |

### 5.3 Usage

```typescript
drawDifferentialFlowField(
  ctx, x, y, w, h,
  expectedFlow: { x: number, y: number },   // From gyro
  observedFlow: { x: number, y: number },   // From optical
  gridSpacing: number
);
```

---

## 6. Integration with Vision LLMs

### 6.1 Prompt Engineering

Instead of:
```
"The rotation is 45 degrees and the speed is 2.5 m/s"
```

The prompt becomes:
```
"READ THE ANALOG INSTRUMENTS:
1. NOMOGRAM: Draw a line between the red and cyan dots.
   Where does it cross the center scale?
2. MOIRÉ: Count the fringe shift from reference.
   Each fringe = 15 degrees rotation.
3. FLOW FIELD: The rose arrows show isolated subject motion.
   Length indicates speed."
```

### 6.2 Why This Works

Vision Transformers (ViT) are trained on:
- Recognizing patterns and shapes
- Spatial relationships between objects
- Color differentiation
- Line following and intersection detection

They are NOT optimized for:
- Parsing small text
- Performing arithmetic
- Understanding numerical notation

By converting numbers to geometry, we play to the model's strengths.

---

## 7. Calibration and Accuracy

### 7.1 Grating Calibration

```typescript
// The fringe-to-angle conversion depends on grating frequency
const degreesPerFringe = 360 / gratingFrequency;

// Example: 12 radial lines = 30° per fringe
const rotation = fringeCount * 30;
```

### 7.2 Nomogram Scale Calibration

Scales are logarithmic by default (for multiplication/division). For addition/subtraction, use linear scales:

```typescript
const linearScale: NomogramScale = {
  label: 'DISPLACEMENT',
  values: [0, 1, 2, 3, 4, 5],
  positions: [0, 0.2, 0.4, 0.6, 0.8, 1.0],  // Evenly spaced
  logarithmic: false
};
```

### 7.3 Error Sources

| Source | Mitigation |
|--------|------------|
| Grating rendering resolution | Use anti-aliasing, minimum 2px line spacing |
| Phase aliasing | Keep frequency < Nyquist limit (half pixel density) |
| Color bleeding | Use high-contrast colors with sufficient separation |
| Perspective distortion | Apply correction based on camera FOV |

---

## 8. Advanced Topics

### 8.1 Zone Plate Interferometry

Circular gratings (Fresnel zone plates) create hyperbolic moiré:

```typescript
generateCircularGrating(ctx, x, y, w, h, {
  frequency: 10,      // Number of zones
  phase: 0,           // Focus offset
  centerX: cx,
  centerY: cy
}, '#22D3EE');
```

Two zone plates with separated centers create a moiré of hyperbolas, encoding the **separation vector**.

### 8.2 Vernier Scales

For ultra-high precision angle reading:

```typescript
drawVernierScale(
  ctx, x, y, w, h,
  mainValue: 127.3,   // Degrees
  precision: 10       // Vernier divisions
);
```

The vernier principle: N divisions on the vernier span N-1 divisions on the main scale, providing 1/N subdivision precision.

### 8.3 Hopf Fibration Moiré

Quaternions live on S³ (the 3-sphere). The Hopf fibration projects S³ → S² with S¹ fibers:

```
Quaternion q → (Base point on Bloch sphere, Phase angle on fiber)
```

Moiré patterns on the projected Clifford torus directly visualize quaternion multiplication.

---

## 9. Performance Considerations

### 9.1 Rendering Cost

| Operation | Relative Cost | Optimization |
|-----------|---------------|--------------|
| Linear grating | 1× | Use `ctx.setLineDash()` for efficiency |
| Circular grating | 2× | Precompute zone radii |
| Log-polar grating | 5× | Cache spiral paths, limit to visible area |
| Full nomogram | 3× | Render scale ticks only on change |

### 9.2 Recommended Settings

```typescript
// For real-time (60fps) rendering
const REALTIME_CONFIG = {
  gratingFrequency: 8,      // Fewer lines
  spiralResolution: 0.05,   // Coarser spiral
  nomogramTicks: 7          // Fewer scale marks
};

// For high-quality export
const EXPORT_CONFIG = {
  gratingFrequency: 24,
  spiralResolution: 0.01,
  nomogramTicks: 20
};
```

---

## 10. Future Development

### 10.1 Planned Enhancements

1. **Adaptive Frequency**: Auto-adjust grating frequency based on detected rotation rate
2. **Multi-scale Moiré**: Nested gratings for coarse + fine measurement
3. **3D Moiré**: Project gratings onto detected surfaces for volumetric measurement
4. **Learned Decoding**: Train a small CNN to read moiré patterns numerically

### 10.2 Research Directions

- **Photonic Computing**: Physical moiré patterns for optical neural networks
- **Quasicrystal Moiré**: Non-periodic tilings for higher-dimensional encoding
- **Topological Moiré**: Using winding numbers for robust phase detection

---

## Appendix A: Mathematical Notation

| Symbol | Meaning |
|--------|---------|
| ψ | Grating phase function |
| p | Grating period |
| D | Moiré fringe spacing |
| M | Magnification factor |
| φ | Golden ratio (1.618...) |
| θ | Angle (context-dependent) |
| q | Quaternion (w, x, y, z) |
| R | Rotor (s, e12, e23, e31) |

## Appendix B: Color Semantics

| Color | Hex | Semantic Meaning |
|-------|-----|------------------|
| Gold | #FACC15 | Inertial/sensor truth |
| Cyan | #22D3EE | Optical/observed reality |
| Rose | #F43F5E | Difference/subject motion |
| Emerald | #10B981 | Quality/confidence |
| Slate | #94a3b8 | Neutral/scale marks |

---

*Document Version: 1.0.0*
*Last Updated: 2026-01-22*
*Module: MoireEngine.ts*
