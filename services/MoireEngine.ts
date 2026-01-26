/**
 * MOIRÉ ENGINE - Visual Analog Computation
 *
 * Implements moiré interference patterns as a high-precision visual calculator
 * for Vision LLM interpretation. The core principle:
 *
 *   Moiré patterns magnify microscopic changes into macroscopic shifts
 *
 * Mathematical basis:
 * - Indicial Equation: ψ₁(x,y) - ψ₂(x,y) = m (integer fringe order)
 * - Beat Frequency: f_moiré = |f₁ - f₂| (visible low-frequency envelope)
 * - Magnification: M = p / Δp (approaches infinity as Δp → 0)
 *
 * Key grating types:
 * 1. Linear gratings - Encode translation, detect shear
 * 2. Circular (Fresnel zone plates) - Encode focus, detect distance
 * 3. Log-polar gratings - Encode rotation AND scale (conformal map)
 * 4. Hyperbolic gratings - Encode squeeze/boost (Lorentz-like)
 *
 * For quaternion/octonion visualization:
 * - Log-polar gratings transform complex multiplication to fringe translation
 * - Nested toroidal gratings visualize Hopf fibration structure
 * - Fano-symmetric gratings encode octonion multiplication rules
 */

// ═══════════════════════════════════════════════════════════════════════════
// TYPE DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════

export interface GratingParams {
  type: 'linear' | 'circular' | 'logPolar' | 'hyperbolic' | 'spiral';
  frequency: number;        // Spatial frequency (lines per unit)
  phase: number;            // Phase offset (0 to 2π)
  orientation: number;      // Rotation angle (radians)
  centerX: number;          // Center x coordinate
  centerY: number;          // Center y coordinate
  dutyCycle: number;        // Ratio of opaque to transparent (0-1)
}

export interface MoireResult {
  fringeSpacing: number;    // Distance between moiré fringes
  fringeAngle: number;      // Orientation of moiré fringes
  magnification: number;    // Mechanical amplification factor
  beatFrequency: number;    // Low-frequency envelope
  phaseShift: number;       // Detected phase difference
}

export interface NomogramScale {
  label: string;
  values: number[];         // Tick mark values
  positions: number[];      // Y positions (0-1 normalized)
  logarithmic: boolean;     // Linear or log scale
}

export interface Nomogram {
  scaleA: NomogramScale;
  scaleB: NomogramScale;
  scaleC: NomogramScale;    // Result scale
  currentA: number;         // Current value on scale A
  currentB: number;         // Current value on scale B
  computedC: number;        // Computed result
}

export interface VisualGrating {
  width: number;
  height: number;
  data: Float32Array;       // Intensity values 0-1
}

// ═══════════════════════════════════════════════════════════════════════════
// GRATING GENERATION - Canvas Rendering
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Generate a linear grating pattern
 * T(x) = (1 + cos(2πfx + φ)) / 2
 */
export function generateLinearGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color: string = '#ffffff'
): void {
  const freq = params.frequency || 20;
  const phase = params.phase || 0;
  const angle = params.orientation || 0;
  const duty = params.dutyCycle || 0.5;

  ctx.save();
  ctx.translate(x + w/2, y + h/2);
  ctx.rotate(angle);

  const period = w / freq;
  const lineWidth = period * duty;

  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;

  ctx.beginPath();
  for (let i = -freq * 2; i <= freq * 2; i++) {
    const lx = i * period + (phase / (2 * Math.PI)) * period;
    ctx.moveTo(lx, -h);
    ctx.lineTo(lx, h);
  }
  ctx.stroke();

  ctx.restore();
}

/**
 * Generate a circular grating (Fresnel zone plate)
 * Creates concentric rings with radii proportional to √n
 * Used for focusing and distance measurement
 */
export function generateCircularGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color: string = '#ffffff'
): void {
  const freq = params.frequency || 10;
  const phase = params.phase || 0;
  const cx = params.centerX ?? w/2;
  const cy = params.centerY ?? h/2;

  ctx.save();
  ctx.translate(x, y);

  const maxRadius = Math.sqrt(w*w + h*h) / 2;

  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;

  // Zone plate radii: r_n = √(nλf) ≈ √n for visualization
  for (let n = 1; n <= freq * 4; n++) {
    const r = Math.sqrt(n + phase / Math.PI) * (maxRadius / Math.sqrt(freq * 4));

    if (n % 2 === 0) {  // Draw only even zones for contrast
      ctx.beginPath();
      ctx.arc(cx, cy, r, 0, 2 * Math.PI);
      ctx.stroke();
    }
  }

  ctx.restore();
}

/**
 * Generate a log-polar grating
 * This is the KEY grating for quaternion visualization!
 *
 * The conformal map w = ln(z) transforms:
 * - Rotation (multiplication by e^iθ) → Vertical translation
 * - Scaling (multiplication by r) → Horizontal translation
 *
 * Pattern: Lines of constant ln|z| (radii) and arg(z) (rays)
 */
export function generateLogPolarGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color: string = '#ffffff'
): void {
  const freq = params.frequency || 8;
  const phase = params.phase || 0;
  const cx = params.centerX ?? w/2;
  const cy = params.centerY ?? h/2;

  ctx.save();
  ctx.translate(x, y);

  ctx.strokeStyle = color;
  ctx.lineWidth = 1;

  // Radial lines (constant θ) - encode rotation
  const angularFreq = freq * 2;
  for (let i = 0; i < angularFreq; i++) {
    const angle = (2 * Math.PI * i / angularFreq) + phase;
    const len = Math.max(w, h);

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(angle) * len, cy + Math.sin(angle) * len);
    ctx.stroke();
  }

  // Logarithmic spirals (constant ln|z|/θ ratio)
  // r = e^(bθ) where b determines the spiral tightness
  const spiralFreq = freq;
  const b = 0.2;  // Spiral growth rate

  for (let s = 0; s < spiralFreq; s++) {
    const phaseOffset = (2 * Math.PI * s / spiralFreq);

    ctx.beginPath();
    for (let theta = 0; theta < 6 * Math.PI; theta += 0.02) {
      const r = 5 * Math.exp(b * (theta + phaseOffset + phase));
      if (r > Math.max(w, h)) break;

      const px = cx + r * Math.cos(theta);
      const py = cy + r * Math.sin(theta);

      if (theta === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    }
    ctx.stroke();
  }

  ctx.restore();
}

/**
 * Generate hyperbolic gratings
 * Encodes Lorentz-like squeeze transformations
 * Moiré of two hyperbolas → visualizes rapidity/boost
 */
export function generateHyperbolicGrating(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  params: Partial<GratingParams>,
  color: string = '#ffffff'
): void {
  const freq = params.frequency || 10;
  const phase = params.phase || 0;
  const angle = params.orientation || 0;

  ctx.save();
  ctx.translate(x + w/2, y + h/2);
  ctx.rotate(angle);

  ctx.strokeStyle = color;
  ctx.lineWidth = 1;

  // Draw hyperbolas: x² - y² = c for various c
  for (let n = 1; n <= freq; n++) {
    const c = n * (w / freq / 4) + phase * 5;

    // Right branch
    ctx.beginPath();
    for (let y = -h; y <= h; y += 2) {
      const xSq = c * c + y * y;
      if (xSq < 0) continue;
      const xVal = Math.sqrt(xSq);
      if (y === -h) ctx.moveTo(xVal, y);
      else ctx.lineTo(xVal, y);
    }
    ctx.stroke();

    // Left branch
    ctx.beginPath();
    for (let y = -h; y <= h; y += 2) {
      const xSq = c * c + y * y;
      if (xSq < 0) continue;
      const xVal = -Math.sqrt(xSq);
      if (y === -h) ctx.moveTo(xVal, y);
      else ctx.lineTo(xVal, y);
    }
    ctx.stroke();
  }

  ctx.restore();
}

// ═══════════════════════════════════════════════════════════════════════════
// MOIRÉ SUPERPOSITION - Fringe Computation
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Compute moiré fringe parameters from two linear gratings
 * The indicial equation: fringes appear where grating lines coincide
 */
export function computeLinearMoire(
  g1: { frequency: number, angle: number },
  g2: { frequency: number, angle: number }
): MoireResult {
  const f1 = g1.frequency;
  const f2 = g2.frequency;
  const theta1 = g1.angle;
  const theta2 = g2.angle;

  // Period of each grating
  const p1 = 1 / f1;
  const p2 = 1 / f2;

  // Angle difference
  const dTheta = theta2 - theta1;

  // Moiré fringe spacing (derived from indicial equation)
  // D = p1 * p2 / √(p1² + p2² - 2*p1*p2*cos(Δθ))
  const denomSq = p1*p1 + p2*p2 - 2*p1*p2*Math.cos(dTheta);
  const fringeSpacing = denomSq > 1e-10 ? (p1 * p2) / Math.sqrt(denomSq) : Infinity;

  // Moiré fringe angle
  // tan(β) = (p1*sin(θ2) - p2*sin(θ1)) / (p1*cos(θ2) - p2*cos(θ1))
  const numerator = p1 * Math.sin(theta2) - p2 * Math.sin(theta1);
  const denominator = p1 * Math.cos(theta2) - p2 * Math.cos(theta1);
  const fringeAngle = Math.atan2(numerator, denominator);

  // Beat frequency
  const beatFrequency = Math.abs(f1 - f2);

  // Magnification factor
  const magnification = fringeSpacing / Math.min(p1, p2);

  // Phase shift (from grating alignment)
  const phaseShift = 0;  // Would be computed from actual grating overlap

  return {
    fringeSpacing,
    fringeAngle,
    magnification,
    beatFrequency,
    phaseShift
  };
}

/**
 * Compute moiré from two circular gratings (zone plates)
 * Creates hyperbolic fringe patterns
 */
export function computeCircularMoire(
  g1: { centerX: number, centerY: number, frequency: number },
  g2: { centerX: number, centerY: number, frequency: number }
): MoireResult {
  // Distance between centers
  const dx = g2.centerX - g1.centerX;
  const dy = g2.centerY - g1.centerY;
  const separation = Math.sqrt(dx*dx + dy*dy);

  // The moiré of two zone plates creates hyperbolic fringes
  // with foci at the two centers

  const f1 = g1.frequency;
  const f2 = g2.frequency;

  const beatFrequency = Math.abs(f1 - f2);
  const fringeSpacing = separation > 0 ? separation / Math.max(1, beatFrequency) : Infinity;
  const fringeAngle = Math.atan2(dy, dx);
  const magnification = fringeSpacing * Math.min(f1, f2);

  return {
    fringeSpacing,
    fringeAngle,
    magnification,
    beatFrequency,
    phaseShift: 0
  };
}

/**
 * Compute moiré from two log-polar gratings
 * This is the key for quaternion visualization!
 *
 * When one grating is rotated: fringe shift ∝ rotation angle
 * When one grating is scaled: fringe shift ∝ log(scale)
 */
export function computeLogPolarMoire(
  g1: { phase: number, scale: number },
  g2: { phase: number, scale: number }
): {
  rotationShift: number,   // Fringe shift due to relative rotation
  scaleShift: number,      // Fringe shift due to relative scaling
  complexArg: number,      // Argument of the complex ratio z2/z1
  complexMag: number       // Magnitude of the complex ratio
} {
  // In log-polar coordinates:
  // z = r * e^(iθ) → w = ln(r) + iθ

  // Rotation difference → phase shift in moiré
  const rotationShift = g2.phase - g1.phase;

  // Scale difference → log-shift in moiré
  const scaleShift = Math.log(g2.scale / g1.scale);

  // These directly give us the complex multiplication result!
  // z2/z1 = (r2/r1) * e^(i(θ2-θ1))

  return {
    rotationShift,
    scaleShift,
    complexArg: rotationShift,
    complexMag: g2.scale / g1.scale
  };
}

// ═══════════════════════════════════════════════════════════════════════════
// NOMOGRAM GENERATION - Visual Analog Calculator
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Create a multiplication nomogram
 * A line connecting points on scales A and B intersects scale C at A*B
 */
export function createMultiplicationNomogram(
  rangeA: [number, number],
  rangeB: [number, number]
): Nomogram {
  // Logarithmic scales for multiplication
  const generateLogScale = (range: [number, number], label: string): NomogramScale => {
    const [min, max] = range;
    const logMin = Math.log10(min);
    const logMax = Math.log10(max);

    const values: number[] = [];
    const positions: number[] = [];

    // Generate logarithmically spaced tick marks
    for (let exp = Math.floor(logMin); exp <= Math.ceil(logMax); exp++) {
      for (const mantissa of [1, 2, 5]) {
        const val = mantissa * Math.pow(10, exp);
        if (val >= min && val <= max) {
          values.push(val);
          positions.push((Math.log10(val) - logMin) / (logMax - logMin));
        }
      }
    }

    return { label, values, positions, logarithmic: true };
  };

  const scaleA = generateLogScale(rangeA, 'A');
  const scaleB = generateLogScale(rangeB, 'B');

  // Result scale range
  const resultRange: [number, number] = [
    rangeA[0] * rangeB[0],
    rangeA[1] * rangeB[1]
  ];
  const scaleC = generateLogScale(resultRange, 'A × B');

  return {
    scaleA,
    scaleB,
    scaleC,
    currentA: (rangeA[0] + rangeA[1]) / 2,
    currentB: (rangeB[0] + rangeB[1]) / 2,
    computedC: ((rangeA[0] + rangeA[1]) / 2) * ((rangeB[0] + rangeB[1]) / 2)
  };
}

/**
 * Create a velocity nomogram (Distance / Time = Speed)
 * The classic three-scale parallel nomogram
 */
export function createVelocityNomogram(
  timeRangeMs: [number, number],
  distanceRangeM: [number, number]
): Nomogram {
  const scaleA: NomogramScale = {
    label: 'TIME (ms)',
    values: [16, 33, 50, 100, 200, 500, 1000],
    positions: [0.1, 0.2, 0.3, 0.45, 0.6, 0.8, 1.0],
    logarithmic: true
  };

  const scaleB: NomogramScale = {
    label: 'DISTANCE (m)',
    values: [0.1, 0.2, 0.5, 1, 2, 5, 10],
    positions: [0.1, 0.18, 0.35, 0.5, 0.65, 0.82, 1.0],
    logarithmic: true
  };

  const scaleC: NomogramScale = {
    label: 'SPEED (m/s)',
    values: [0.1, 0.5, 1, 2, 5, 10, 20, 50],
    positions: [0.05, 0.2, 0.32, 0.45, 0.62, 0.75, 0.87, 1.0],
    logarithmic: true
  };

  return {
    scaleA,
    scaleB,
    scaleC,
    currentA: 100,
    currentB: 1,
    computedC: 10  // 1m / 0.1s = 10 m/s
  };
}

/**
 * Render a nomogram to canvas
 * Draws three parallel scales with a "computation line" connecting current values
 */
export function drawNomogram(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  nomogram: Nomogram,
  colors: { bg: string, scale: string, line: string, marker: string }
): void {
  ctx.save();
  ctx.translate(x, y);

  // Background
  ctx.fillStyle = colors.bg;
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = colors.scale;
  ctx.lineWidth = 1;
  ctx.strokeRect(0, 0, w, h);

  const margin = 15;
  const scaleH = h - 2 * margin;

  // Scale positions (left, center, right)
  const scaleXs = [margin + 10, w / 2, w - margin - 10];

  // Draw each scale
  const scales = [nomogram.scaleA, nomogram.scaleB, nomogram.scaleC];

  scales.forEach((scale, i) => {
    const sx = scaleXs[i];

    // Vertical axis
    ctx.strokeStyle = colors.scale;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sx, margin);
    ctx.lineTo(sx, margin + scaleH);
    ctx.stroke();

    // Tick marks and labels
    ctx.fillStyle = colors.scale;
    ctx.font = '8px monospace';
    ctx.textAlign = i === 0 ? 'right' : (i === 2 ? 'left' : 'center');

    scale.values.forEach((val, j) => {
      const py = margin + (1 - scale.positions[j]) * scaleH;

      // Tick mark
      ctx.beginPath();
      ctx.moveTo(sx - 4, py);
      ctx.lineTo(sx + 4, py);
      ctx.stroke();

      // Label
      const labelX = i === 0 ? sx - 6 : (i === 2 ? sx + 6 : sx);
      const labelY = i === 1 ? py - 8 : py + 3;
      ctx.fillText(val.toString(), labelX, labelY);
    });

    // Scale title
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText(scale.label, sx, margin - 3);
  });

  // Compute positions of current values
  const getYPosition = (value: number, scale: NomogramScale): number => {
    const logVal = Math.log10(value);
    const logMin = Math.log10(scale.values[0]);
    const logMax = Math.log10(scale.values[scale.values.length - 1]);
    const norm = (logVal - logMin) / (logMax - logMin);
    return margin + (1 - norm) * scaleH;
  };

  const yA = getYPosition(nomogram.currentA, nomogram.scaleA);
  const yB = getYPosition(nomogram.currentB, nomogram.scaleB);
  const yC = getYPosition(nomogram.computedC, nomogram.scaleC);

  // Draw computation line (connects A and B, passes through C)
  ctx.strokeStyle = colors.line;
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(scaleXs[0], yA);
  ctx.lineTo(scaleXs[2], yB);
  ctx.stroke();
  ctx.setLineDash([]);

  // Draw markers at current values
  ctx.fillStyle = colors.marker;
  ctx.beginPath();
  ctx.arc(scaleXs[0], yA, 5, 0, 2 * Math.PI);
  ctx.fill();

  ctx.beginPath();
  ctx.arc(scaleXs[2], yB, 5, 0, 2 * Math.PI);
  ctx.fill();

  // Result marker (where line crosses center scale)
  ctx.fillStyle = colors.line;
  ctx.beginPath();
  ctx.arc(scaleXs[1], yC, 6, 0, 2 * Math.PI);
  ctx.fill();

  ctx.restore();
}

// ═══════════════════════════════════════════════════════════════════════════
// QUATERNION VISUALIZATION via MOIRÉ
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Draw a quaternion as a moiré pattern on the Clifford torus projection
 * The torus is parametrized by (u, v) where:
 * - u controls rotation in the XY plane
 * - v controls rotation in the ZW plane
 */
export function drawQuaternionMoire(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  qw: number, qx: number, qy: number, qz: number,
  referenceQ: { w: number, x: number, y: number, z: number } | null = null
): void {
  ctx.save();
  ctx.translate(x, y);

  const cx = w / 2;
  const cy = h / 2;
  const radius = Math.min(w, h) * 0.4;

  // Convert quaternion to torus coordinates
  // Using Hopf fibration: (φ, θ, ψ) where φ,θ parametrize S² and ψ is fiber
  const quaternionToTorus = (w: number, x: number, y: number, z: number) => {
    // Stereographic projection style
    const u = Math.atan2(y, x);  // XY plane angle
    const v = Math.atan2(z, w);  // ZW plane angle
    return { u, v };
  };

  const current = quaternionToTorus(qw, qx, qy, qz);

  // Draw the "reference" grating (fixed or identity)
  ctx.strokeStyle = 'rgba(250, 204, 21, 0.4)';  // Gold
  ctx.lineWidth = 1;

  const refQ = referenceQ || { w: 1, x: 0, y: 0, z: 0 };
  const ref = quaternionToTorus(refQ.w, refQ.x, refQ.y, refQ.z);

  // Draw toroidal grid centered on reference
  const gridLines = 12;
  for (let i = 0; i < gridLines; i++) {
    const angle = (2 * Math.PI * i / gridLines) + ref.u;

    // Radial line (u = constant)
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(angle) * radius, cy + Math.sin(angle) * radius);
    ctx.stroke();
  }

  // Concentric circles (v = constant)
  for (let i = 1; i <= 4; i++) {
    const r = radius * i / 4;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, 2 * Math.PI);
    ctx.stroke();
  }

  // Draw the "current" grating (based on actual quaternion)
  ctx.strokeStyle = 'rgba(34, 211, 238, 0.4)';  // Cyan

  for (let i = 0; i < gridLines; i++) {
    const angle = (2 * Math.PI * i / gridLines) + current.u;

    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(angle) * radius, cy + Math.sin(angle) * radius);
    ctx.stroke();
  }

  // The moiré effect happens naturally when these overlap!
  // The Vision LLM will see the beat pattern

  // Draw a marker at the "effective" position
  const markerAngle = current.u - ref.u;  // Relative rotation
  const markerRadius = radius * 0.6;

  ctx.fillStyle = '#F43F5E';  // Rose
  ctx.beginPath();
  ctx.arc(
    cx + Math.cos(markerAngle) * markerRadius,
    cy + Math.sin(markerAngle) * markerRadius,
    6, 0, 2 * Math.PI
  );
  ctx.fill();

  // Label the rotation
  ctx.fillStyle = '#ffffff';
  ctx.font = 'bold 10px monospace';
  ctx.textAlign = 'center';

  const degrees = (markerAngle * 180 / Math.PI).toFixed(1);
  ctx.fillText(`Δθ = ${degrees}°`, cx, h - 5);

  ctx.restore();
}

/**
 * Draw a spinor/qubit state using moiré on the Bloch sphere projection
 */
export function drawSpinorMoire(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  theta: number,  // Polar angle (0 = |0⟩, π = |1⟩)
  phi: number     // Azimuthal angle (phase)
): void {
  ctx.save();
  ctx.translate(x, y);

  const cx = w / 2;
  const cy = h / 2;
  const radius = Math.min(w, h) * 0.4;

  // Reference grating (aligned with |0⟩ state)
  ctx.strokeStyle = 'rgba(250, 204, 21, 0.3)';
  ctx.lineWidth = 1;

  // Draw reference latitude lines
  for (let lat = -4; lat <= 4; lat++) {
    const latAngle = lat * Math.PI / 8;
    const r = radius * Math.cos(latAngle);
    const yOffset = radius * Math.sin(latAngle) * 0.5;  // Perspective compression

    ctx.beginPath();
    ctx.ellipse(cx, cy + yOffset, r, r * 0.3, 0, 0, 2 * Math.PI);
    ctx.stroke();
  }

  // Draw reference longitude lines
  for (let lon = 0; lon < 8; lon++) {
    const lonAngle = lon * Math.PI / 4;
    ctx.beginPath();
    ctx.ellipse(cx, cy, radius * Math.abs(Math.cos(lonAngle)), radius, lonAngle, 0, Math.PI);
    ctx.stroke();
  }

  // Actual state grating (rotated based on θ and φ)
  ctx.strokeStyle = 'rgba(34, 211, 238, 0.3)';

  // Rotate the grid based on state
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(phi);

  // Draw rotated latitude lines
  for (let lat = -4; lat <= 4; lat++) {
    const latAngle = lat * Math.PI / 8 + theta * 0.5;
    const r = radius * Math.cos(latAngle);
    const yOffset = radius * Math.sin(latAngle) * 0.5;

    if (r > 0) {
      ctx.beginPath();
      ctx.ellipse(0, yOffset, r, r * 0.3, 0, 0, 2 * Math.PI);
      ctx.stroke();
    }
  }

  ctx.restore();

  // Draw the Bloch vector
  const bx = Math.sin(theta) * Math.cos(phi);
  const by = Math.sin(theta) * Math.sin(phi);
  const bz = Math.cos(theta);

  // Project to 2D (oblique view)
  const projX = cx + bx * radius;
  const projY = cy - bz * radius * 0.8 + by * radius * 0.3;

  ctx.strokeStyle = '#F43F5E';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(projX, projY);
  ctx.stroke();

  ctx.fillStyle = '#F43F5E';
  ctx.beginPath();
  ctx.arc(projX, projY, 5, 0, 2 * Math.PI);
  ctx.fill();

  // Probability annotations
  ctx.fillStyle = '#ffffff';
  ctx.font = '9px monospace';
  ctx.textAlign = 'left';

  const p0 = Math.cos(theta / 2) ** 2;
  const p1 = Math.sin(theta / 2) ** 2;

  ctx.fillText(`|0⟩: ${(p0 * 100).toFixed(0)}%`, 5, 12);
  ctx.fillText(`|1⟩: ${(p1 * 100).toFixed(0)}%`, 5, 24);

  ctx.restore();
}

// ═══════════════════════════════════════════════════════════════════════════
// VERNIER & PRECISION SCALE VISUALIZATION
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Draw a Vernier scale for high-precision angle reading
 * The moiré between main and vernier scales gives sub-division precision
 */
export function drawVernierScale(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  mainValue: number,      // Main scale value (0-360 degrees)
  precision: number = 10  // Vernier divisions
): void {
  ctx.save();
  ctx.translate(x, y);

  const margin = 10;
  const scaleW = w - 2 * margin;

  // Main scale (top)
  const mainDivisions = 36;  // 10° each
  ctx.strokeStyle = '#FACC15';
  ctx.lineWidth = 1;

  for (let i = 0; i <= mainDivisions; i++) {
    const px = margin + (i / mainDivisions) * scaleW;
    const tickH = i % 9 === 0 ? 15 : (i % 3 === 0 ? 10 : 5);

    ctx.beginPath();
    ctx.moveTo(px, 5);
    ctx.lineTo(px, 5 + tickH);
    ctx.stroke();

    if (i % 9 === 0) {
      ctx.fillStyle = '#FACC15';
      ctx.font = '8px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`${i * 10}°`, px, 5 + tickH + 10);
    }
  }

  // Vernier scale (bottom, slightly different spacing)
  // 10 vernier divisions span 9 main divisions
  const vernierOffset = (mainValue / 360) * scaleW;
  const vernierDivisions = precision;
  const vernierSpan = (9 / mainDivisions) * scaleW;

  ctx.strokeStyle = '#22D3EE';

  for (let i = 0; i <= vernierDivisions; i++) {
    const px = margin + vernierOffset + (i / vernierDivisions) * vernierSpan - vernierSpan / 2;

    if (px < margin || px > margin + scaleW) continue;

    const tickH = i === vernierDivisions / 2 ? 15 : (i % 5 === 0 ? 10 : 5);

    ctx.beginPath();
    ctx.moveTo(px, h - 5);
    ctx.lineTo(px, h - 5 - tickH);
    ctx.stroke();
  }

  // Find aligned tick (the moiré "bright fringe")
  // This is where the Vernier precision reading is taken
  const fractionalPart = (mainValue % 10);
  const alignedIdx = Math.round(fractionalPart);

  // Highlight the aligned position
  ctx.fillStyle = '#F43F5E';
  ctx.font = 'bold 10px monospace';
  ctx.textAlign = 'center';
  ctx.fillText(`${mainValue.toFixed(1)}°`, w / 2, h / 2 + 4);

  ctx.restore();
}

// ═══════════════════════════════════════════════════════════════════════════
// INTERFERENCE FIELD FOR FLOW VISUALIZATION
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Draw a differential flow field using moiré principles
 * Yellow: Expected motion (from gyro)
 * Cyan: Observed motion (from optical flow)
 * The difference reveals subject motion
 */
export function drawDifferentialFlowField(
  ctx: CanvasRenderingContext2D,
  x: number, y: number, w: number, h: number,
  expectedFlow: { x: number, y: number },   // From sensors
  observedFlow: { x: number, y: number },   // From optical analysis
  gridSpacing: number = 40
): void {
  ctx.save();
  ctx.translate(x, y);

  // Draw expected flow vectors (yellow/gold)
  ctx.strokeStyle = 'rgba(250, 204, 21, 0.6)';
  ctx.lineWidth = 1.5;

  for (let gx = gridSpacing / 2; gx < w; gx += gridSpacing) {
    for (let gy = gridSpacing / 2; gy < h; gy += gridSpacing) {
      const scale = 15;
      ctx.beginPath();
      ctx.moveTo(gx, gy);
      ctx.lineTo(gx + expectedFlow.x * scale, gy + expectedFlow.y * scale);
      ctx.stroke();

      // Arrow head
      const angle = Math.atan2(expectedFlow.y, expectedFlow.x);
      ctx.beginPath();
      ctx.moveTo(gx + expectedFlow.x * scale, gy + expectedFlow.y * scale);
      ctx.lineTo(
        gx + expectedFlow.x * scale - 4 * Math.cos(angle - 0.5),
        gy + expectedFlow.y * scale - 4 * Math.sin(angle - 0.5)
      );
      ctx.stroke();
    }
  }

  // Draw observed flow vectors (cyan)
  ctx.strokeStyle = 'rgba(34, 211, 238, 0.6)';

  for (let gx = gridSpacing / 2; gx < w; gx += gridSpacing) {
    for (let gy = gridSpacing / 2; gy < h; gy += gridSpacing) {
      const scale = 15;
      ctx.beginPath();
      ctx.moveTo(gx, gy);
      ctx.lineTo(gx + observedFlow.x * scale, gy + observedFlow.y * scale);
      ctx.stroke();
    }
  }

  // Draw difference vectors (rose) - THE SUBJECT MOTION
  ctx.strokeStyle = '#F43F5E';
  ctx.lineWidth = 2;

  const diffX = observedFlow.x - expectedFlow.x;
  const diffY = observedFlow.y - expectedFlow.y;

  for (let gx = gridSpacing / 2; gx < w; gx += gridSpacing * 2) {
    for (let gy = gridSpacing / 2; gy < h; gy += gridSpacing * 2) {
      const scale = 15;
      ctx.beginPath();
      ctx.moveTo(gx, gy);
      ctx.lineTo(gx + diffX * scale, gy + diffY * scale);
      ctx.stroke();
    }
  }

  // Legend
  ctx.font = '9px monospace';
  ctx.fillStyle = '#FACC15';
  ctx.fillText('GYRO FLOW', 5, h - 25);
  ctx.fillStyle = '#22D3EE';
  ctx.fillText('OPTICAL FLOW', 5, h - 15);
  ctx.fillStyle = '#F43F5E';
  ctx.fillText('SUBJECT MOTION', 5, h - 5);

  ctx.restore();
}
