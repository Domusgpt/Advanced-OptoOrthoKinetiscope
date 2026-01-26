/**
 * MOIRÉ MEASUREMENT ENGINE
 *
 * Uses moiré interference patterns for precision angle and displacement measurement.
 * Key insight: Small rotations cause large fringe shifts, providing mechanical amplification.
 *
 * Measurement formula:
 *   Fringe spacing D = d / (2 * sin(θ/2)) ≈ d/θ for small θ
 *   Amplification M = D/d = 1/θ
 *
 * For d = 10px grating and θ = 1°, D = 573px (57× amplification)
 *
 * This module:
 * - Generates reference gratings based on IMU rotation
 * - Overlays on video frames
 * - Measures fringe phase to determine precise rotation
 * - Provides sub-degree accuracy through fringe interpolation
 */

import { Rotor3D } from './GeometricAlgebra';

// ═══════════════════════════════════════════════════════════════════════════
// TYPES
// ═══════════════════════════════════════════════════════════════════════════

export interface GratingParams {
  /** Grating spacing in pixels */
  spacing: number;

  /** Rotation angle in radians */
  rotation: number;

  /** Phase offset (0-2π) */
  phase: number;

  /** Grating type */
  type: 'linear' | 'circular' | 'logpolar';
}

export interface MoireMeasurement {
  /** Measured angle difference (radians) */
  angleDifference: number;

  /** Fringe spacing (pixels) */
  fringeSpacing: number;

  /** Fringe phase (0-2π) */
  fringePhase: number;

  /** Amplification factor */
  amplification: number;

  /** Measurement confidence (0-1) */
  confidence: number;

  /** Estimated error (radians) */
  error: number;
}

export interface MoireConfig {
  /** Base grating spacing */
  gratingSpacing: number;

  /** Number of angular samples for phase measurement */
  angularSamples: number;

  /** Smoothing factor for temporal filtering */
  smoothingFactor: number;

  /** Minimum fringe contrast for valid measurement */
  minContrast: number;
}

const DEFAULT_CONFIG: MoireConfig = {
  gratingSpacing: 12,
  angularSamples: 36,
  smoothingFactor: 0.3,
  minContrast: 0.1
};

// ═══════════════════════════════════════════════════════════════════════════
// MOIRÉ MEASUREMENT ENGINE
// ═══════════════════════════════════════════════════════════════════════════

export class MoireMeasurementEngine {
  private config: MoireConfig;
  private referenceGrating: Float32Array | null = null;
  private referenceRotation: number = 0;
  private previousMeasurement: MoireMeasurement | null = null;

  constructor(config: Partial<MoireConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
  }

  /**
   * Generate a linear grating pattern
   */
  generateLinearGrating(
    width: number,
    height: number,
    params: GratingParams
  ): Float32Array {
    const grating = new Float32Array(width * height);
    const { spacing, rotation, phase } = params;

    const cosR = Math.cos(rotation);
    const sinR = Math.sin(rotation);
    const freq = (2 * Math.PI) / spacing;

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        // Rotate coordinates
        const xr = x * cosR - y * sinR;

        // Sinusoidal grating
        const value = 0.5 + 0.5 * Math.sin(freq * xr + phase);
        grating[y * width + x] = value;
      }
    }

    return grating;
  }

  /**
   * Generate a log-polar grating (rotation becomes translation)
   */
  generateLogPolarGrating(
    width: number,
    height: number,
    centerX: number,
    centerY: number,
    angularFreq: number,
    radialFreq: number,
    rotationOffset: number
  ): Float32Array {
    const grating = new Float32Array(width * height);

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const dx = x - centerX;
        const dy = y - centerY;
        const r = Math.sqrt(dx * dx + dy * dy);
        const theta = Math.atan2(dy, dx);

        if (r < 5) {
          grating[y * width + x] = 0.5;
          continue;
        }

        const logR = Math.log(r);
        const value = 0.5 + 0.5 * Math.sin(
          angularFreq * (theta + rotationOffset) + radialFreq * logR
        );
        grating[y * width + x] = value;
      }
    }

    return grating;
  }

  /**
   * Compute moiré pattern from two gratings
   */
  computeMoire(
    grating1: Float32Array,
    grating2: Float32Array,
    width: number,
    height: number
  ): Float32Array {
    const moire = new Float32Array(width * height);

    for (let i = 0; i < moire.length; i++) {
      // Multiplicative superposition
      moire[i] = grating1[i] * grating2[i];
    }

    return moire;
  }

  /**
   * Extract fringe parameters from moiré pattern
   */
  analyzeFringes(
    moire: Float32Array,
    width: number,
    height: number
  ): { spacing: number; angle: number; phase: number; contrast: number } {
    // Use 2D Fourier analysis to find dominant frequency
    // Simplified: sample along multiple angles and find peak

    const { angularSamples } = this.config;
    const centerX = width / 2;
    const centerY = height / 2;
    const sampleRadius = Math.min(width, height) / 3;

    let bestAngle = 0;
    let bestSpacing = Infinity;
    let bestContrast = 0;
    let bestPhase = 0;

    // Sample at different angles to find fringe direction
    for (let ai = 0; ai < angularSamples; ai++) {
      const angle = (ai / angularSamples) * Math.PI;
      const cosA = Math.cos(angle);
      const sinA = Math.sin(angle);

      // Sample along this direction
      const samples: number[] = [];
      for (let r = -sampleRadius; r <= sampleRadius; r += 2) {
        const x = Math.round(centerX + r * cosA);
        const y = Math.round(centerY + r * sinA);

        if (x >= 0 && x < width && y >= 0 && y < height) {
          samples.push(moire[y * width + x]);
        }
      }

      if (samples.length < 10) continue;

      // Find dominant frequency using autocorrelation
      const { spacing, phase, contrast } = this.analyzeProfile(samples);

      if (contrast > bestContrast) {
        bestContrast = contrast;
        bestSpacing = spacing * 2; // Convert sample spacing to pixels
        bestAngle = angle;
        bestPhase = phase;
      }
    }

    return {
      spacing: bestSpacing,
      angle: bestAngle,
      phase: bestPhase,
      contrast: bestContrast
    };
  }

  /**
   * Analyze a 1D profile to extract frequency and phase
   */
  private analyzeProfile(samples: number[]): { spacing: number; phase: number; contrast: number } {
    const n = samples.length;

    // Compute mean and normalize
    const mean = samples.reduce((a, b) => a + b, 0) / n;
    const normalized = samples.map(s => s - mean);

    // Find zero crossings to estimate frequency
    let zeroCrossings = 0;
    for (let i = 1; i < n; i++) {
      if ((normalized[i - 1] >= 0 && normalized[i] < 0) ||
          (normalized[i - 1] < 0 && normalized[i] >= 0)) {
        zeroCrossings++;
      }
    }

    // Spacing = samples per period
    const spacing = zeroCrossings > 0 ? (2 * n) / zeroCrossings : Infinity;

    // Estimate phase using first samples
    let sumSin = 0, sumCos = 0;
    const freq = (2 * Math.PI) / spacing;
    for (let i = 0; i < n; i++) {
      sumSin += normalized[i] * Math.sin(freq * i);
      sumCos += normalized[i] * Math.cos(freq * i);
    }
    const phase = Math.atan2(sumSin, sumCos);

    // Contrast = peak-to-peak / mean
    const max = Math.max(...samples);
    const min = Math.min(...samples);
    const contrast = mean > 0.01 ? (max - min) / (2 * mean) : 0;

    return { spacing, phase, contrast };
  }

  /**
   * Set reference grating based on IMU orientation
   */
  setReference(
    width: number,
    height: number,
    rotation: number
  ): void {
    this.referenceGrating = this.generateLinearGrating(width, height, {
      spacing: this.config.gratingSpacing,
      rotation: rotation,
      phase: 0,
      type: 'linear'
    });
    this.referenceRotation = rotation;
  }

  /**
   * Measure rotation from video frame relative to reference
   */
  measureRotation(
    frameGrayscale: Float32Array,
    width: number,
    height: number,
    expectedRotation: number
  ): MoireMeasurement {
    // Generate test grating at expected rotation
    const testGrating = this.generateLinearGrating(width, height, {
      spacing: this.config.gratingSpacing,
      rotation: expectedRotation,
      phase: 0,
      type: 'linear'
    });

    // Multiply with frame (treating frame as a "natural" grating)
    // Edges and textures in the frame act as irregular gratings
    const moire = this.computeMoire(frameGrayscale, testGrating, width, height);

    // Analyze resulting moiré pattern
    const fringes = this.analyzeFringes(moire, width, height);

    // Calculate angle difference from fringe spacing
    // D = d / (2 * sin(Δθ/2))
    // For small angles: Δθ ≈ d / D
    const d = this.config.gratingSpacing;
    let angleDifference = 0;

    if (fringes.spacing > d && fringes.spacing < width) {
      angleDifference = d / fringes.spacing;
    }

    // Determine sign from fringe angle
    if (fringes.angle > Math.PI / 4 && fringes.angle < 3 * Math.PI / 4) {
      angleDifference = -angleDifference;
    }

    // Amplification factor
    const amplification = fringes.spacing / d;

    // Error estimate (inversely proportional to contrast and amplification)
    const error = fringes.contrast > 0.01
      ? (1 / amplification) * (1 / fringes.contrast) * 0.01
      : 0.1;

    // Confidence
    const confidence = Math.min(1, fringes.contrast * amplification / 10);

    const measurement: MoireMeasurement = {
      angleDifference,
      fringeSpacing: fringes.spacing,
      fringePhase: fringes.phase,
      amplification,
      confidence,
      error
    };

    // Temporal smoothing
    if (this.previousMeasurement && this.config.smoothingFactor < 1) {
      const alpha = this.config.smoothingFactor;
      measurement.angleDifference =
        alpha * measurement.angleDifference +
        (1 - alpha) * this.previousMeasurement.angleDifference;
    }

    this.previousMeasurement = measurement;
    return measurement;
  }

  /**
   * Use log-polar moiré to measure rotation precisely
   * Key advantage: rotation becomes pure translation in log-polar space
   */
  measureRotationLogPolar(
    frame1: Float32Array,
    frame2: Float32Array,
    width: number,
    height: number
  ): { rotation: number; scale: number; confidence: number } {
    const cx = width / 2;
    const cy = height / 2;

    // Generate log-polar gratings for each frame
    const lp1 = this.generateLogPolarGrating(width, height, cx, cy, 12, 4, 0);
    const lp2 = this.generateLogPolarGrating(width, height, cx, cy, 12, 4, 0);

    // Modulate with frames
    const mod1 = new Float32Array(width * height);
    const mod2 = new Float32Array(width * height);

    for (let i = 0; i < mod1.length; i++) {
      mod1[i] = frame1[i] * lp1[i];
      mod2[i] = frame2[i] * lp2[i];
    }

    // Cross-correlate in angular direction (vertical in log-polar)
    // Rotation shows up as vertical shift

    let bestShift = 0;
    let bestCorr = -Infinity;

    const maxShift = height / 4;
    for (let shift = -maxShift; shift <= maxShift; shift++) {
      let corr = 0;
      let count = 0;

      for (let y = 0; y < height; y++) {
        const y2 = (y + shift + height) % height;
        for (let x = 0; x < width; x++) {
          corr += mod1[y * width + x] * mod2[y2 * width + x];
          count++;
        }
      }

      corr /= count;
      if (corr > bestCorr) {
        bestCorr = corr;
        bestShift = shift;
      }
    }

    // Convert shift to rotation angle
    const rotation = (bestShift / height) * 2 * Math.PI;

    // Scale detection (horizontal shift in log-polar)
    // Simplified: assume scale = 1 for now
    const scale = 1;

    const confidence = Math.min(1, Math.max(0, bestCorr * 2));

    return { rotation, scale, confidence };
  }

  /**
   * Generate visualization of moiré measurement
   */
  visualize(
    width: number,
    height: number,
    imuRotation: number,
    measuredRotation: number
  ): ImageData {
    const imageData = new ImageData(width, height);
    const data = imageData.data;

    // Generate reference grating (based on IMU)
    const refGrating = this.generateLinearGrating(width, height, {
      spacing: this.config.gratingSpacing,
      rotation: imuRotation,
      phase: 0,
      type: 'linear'
    });

    // Generate "observed" grating (based on measurement)
    const obsGrating = this.generateLinearGrating(width, height, {
      spacing: this.config.gratingSpacing,
      rotation: measuredRotation,
      phase: 0,
      type: 'linear'
    });

    // Compute moiré
    const moire = this.computeMoire(refGrating, obsGrating, width, height);

    // Render
    for (let i = 0; i < moire.length; i++) {
      const v = Math.floor(moire[i] * 255);
      const j = i * 4;
      data[j] = v;         // R
      data[j + 1] = v;     // G
      data[j + 2] = v;     // B
      data[j + 3] = 255;   // A
    }

    return imageData;
  }

  /**
   * Reset state
   */
  reset(): void {
    this.referenceGrating = null;
    this.previousMeasurement = null;
  }

  /**
   * Update configuration
   */
  setConfig(config: Partial<MoireConfig>): void {
    this.config = { ...this.config, ...config };
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Convert ImageData to grayscale Float32Array (0-1 range)
 */
export function imageDataToGrayscale(imageData: ImageData): Float32Array {
  const gray = new Float32Array(imageData.width * imageData.height);
  const data = imageData.data;

  for (let i = 0; i < gray.length; i++) {
    const j = i * 4;
    gray[i] = (0.299 * data[j] + 0.587 * data[j + 1] + 0.114 * data[j + 2]) / 255;
  }

  return gray;
}

/**
 * Calculate theoretical fringe spacing for given grating and angle
 */
export function calculateFringeSpacing(gratingSpacing: number, angle: number): number {
  if (Math.abs(angle) < 1e-10) return Infinity;
  return gratingSpacing / (2 * Math.sin(Math.abs(angle) / 2));
}

/**
 * Calculate angle from observed fringe spacing
 */
export function calculateAngleFromFringes(
  gratingSpacing: number,
  fringeSpacing: number
): number {
  if (fringeSpacing <= gratingSpacing) return Math.PI / 2;
  if (!isFinite(fringeSpacing)) return 0;
  return 2 * Math.asin(gratingSpacing / (2 * fringeSpacing));
}

/**
 * Calculate amplification factor
 */
export function calculateAmplification(gratingSpacing: number, angle: number): number {
  const fringeSpacing = calculateFringeSpacing(gratingSpacing, angle);
  return fringeSpacing / gratingSpacing;
}
