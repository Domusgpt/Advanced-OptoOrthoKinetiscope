/**
 * OPTICAL FLOW ENGINE
 *
 * Computes motion from video frames using Lucas-Kanade optical flow.
 * Decomposes total flow into:
 * - Translation (pan/tilt)
 * - Rotation (roll)
 * - Expansion/contraction (zoom/dolly)
 * - Residual (independent object motion)
 *
 * Key insight: By comparing optical flow with IMU-predicted flow, we can
 * separate camera ego-motion from scene motion.
 */

import { Vector3 } from '../types';
import { Rotor3D, applyRotorToVector, reverseRotor } from './GeometricAlgebra';

// ═══════════════════════════════════════════════════════════════════════════
// TYPES
// ═══════════════════════════════════════════════════════════════════════════

export interface FlowVector {
  x: number;
  y: number;
}

export interface FlowField {
  /** Width in pixels */
  width: number;

  /** Height in pixels */
  height: number;

  /** Grid cell size */
  cellSize: number;

  /** Flow vectors at each grid point */
  vectors: FlowVector[][];

  /** Confidence at each grid point (0-1) */
  confidence: number[][];

  /** Timestamp */
  timestamp: number;
}

export interface FlowDecomposition {
  /** Average translation (px/frame) */
  translation: FlowVector;

  /** Rotation about image center (radians/frame) */
  rotation: number;

  /** Expansion factor (1.0 = no change, >1 = zooming in) */
  expansion: number;

  /** Focus of Expansion (for forward/backward motion) */
  foe: { x: number; y: number } | null;

  /** Residual flow after removing ego-motion */
  residual: FlowField | null;

  /** Overall confidence */
  confidence: number;
}

export interface OpticalFlowConfig {
  /** Block size for matching (pixels) */
  blockSize: number;

  /** Search radius for matching (pixels) */
  searchRadius: number;

  /** Grid spacing for flow computation */
  gridSpacing: number;

  /** Minimum confidence threshold */
  minConfidence: number;

  /** Enable subpixel refinement */
  subpixel: boolean;
}

const DEFAULT_CONFIG: OpticalFlowConfig = {
  blockSize: 16,
  searchRadius: 16,
  gridSpacing: 24,
  minConfidence: 0.3,
  subpixel: true
};

// ═══════════════════════════════════════════════════════════════════════════
// OPTICAL FLOW ENGINE
// ═══════════════════════════════════════════════════════════════════════════

export class OpticalFlowEngine {
  private config: OpticalFlowConfig;
  private previousFrame: ImageData | null = null;
  private previousGray: Uint8Array | null = null;
  private flowHistory: FlowDecomposition[] = [];
  private readonly maxHistory = 10;

  constructor(config: Partial<OpticalFlowConfig> = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
  }

  /**
   * Convert RGBA ImageData to grayscale Uint8Array
   */
  private toGrayscale(imageData: ImageData): Uint8Array {
    const gray = new Uint8Array(imageData.width * imageData.height);
    const data = imageData.data;

    for (let i = 0; i < gray.length; i++) {
      const j = i * 4;
      // Luminance formula
      gray[i] = Math.round(0.299 * data[j] + 0.587 * data[j + 1] + 0.114 * data[j + 2]);
    }

    return gray;
  }

  /**
   * Compute Sum of Absolute Differences between two blocks
   */
  private computeSAD(
    gray1: Uint8Array, gray2: Uint8Array,
    width: number,
    x1: number, y1: number,
    x2: number, y2: number,
    blockSize: number
  ): number {
    let sad = 0;
    const halfBlock = Math.floor(blockSize / 2);

    for (let dy = -halfBlock; dy <= halfBlock; dy++) {
      for (let dx = -halfBlock; dx <= halfBlock; dx++) {
        const idx1 = (y1 + dy) * width + (x1 + dx);
        const idx2 = (y2 + dy) * width + (x2 + dx);

        if (idx1 >= 0 && idx1 < gray1.length && idx2 >= 0 && idx2 < gray2.length) {
          sad += Math.abs(gray1[idx1] - gray2[idx2]);
        }
      }
    }

    return sad;
  }

  /**
   * Find best matching block using block matching
   */
  private findBestMatch(
    prevGray: Uint8Array,
    currGray: Uint8Array,
    width: number,
    height: number,
    x: number, y: number
  ): { dx: number; dy: number; confidence: number } {
    const { blockSize, searchRadius } = this.config;
    const halfBlock = Math.floor(blockSize / 2);

    // Skip if too close to edge
    if (x < searchRadius + halfBlock || x >= width - searchRadius - halfBlock ||
        y < searchRadius + halfBlock || y >= height - searchRadius - halfBlock) {
      return { dx: 0, dy: 0, confidence: 0 };
    }

    let bestDx = 0;
    let bestDy = 0;
    let bestSAD = Infinity;
    let secondBestSAD = Infinity;

    // Search in neighborhood
    for (let dy = -searchRadius; dy <= searchRadius; dy += 2) {
      for (let dx = -searchRadius; dx <= searchRadius; dx += 2) {
        const sad = this.computeSAD(
          prevGray, currGray,
          width,
          x, y,
          x + dx, y + dy,
          blockSize
        );

        if (sad < bestSAD) {
          secondBestSAD = bestSAD;
          bestSAD = sad;
          bestDx = dx;
          bestDy = dy;
        } else if (sad < secondBestSAD) {
          secondBestSAD = sad;
        }
      }
    }

    // Subpixel refinement
    if (this.config.subpixel && bestSAD < Infinity) {
      // Refine with smaller steps around best match
      for (let dy = bestDy - 1; dy <= bestDy + 1; dy++) {
        for (let dx = bestDx - 1; dx <= bestDx + 1; dx++) {
          if (dx === bestDx && dy === bestDy) continue;

          const sad = this.computeSAD(
            prevGray, currGray,
            width,
            x, y,
            x + dx, y + dy,
            blockSize
          );

          if (sad < bestSAD) {
            secondBestSAD = bestSAD;
            bestSAD = sad;
            bestDx = dx;
            bestDy = dy;
          }
        }
      }
    }

    // Confidence based on distinctiveness
    const maxSAD = blockSize * blockSize * 255;
    const matchQuality = 1 - (bestSAD / maxSAD);
    const distinctiveness = secondBestSAD > 0 ? (secondBestSAD - bestSAD) / secondBestSAD : 0;
    const confidence = matchQuality * (0.5 + 0.5 * distinctiveness);

    return { dx: bestDx, dy: bestDy, confidence };
  }

  /**
   * Compute optical flow between two frames
   */
  computeFlow(currentFrame: ImageData, timestamp: number): FlowField | null {
    const currGray = this.toGrayscale(currentFrame);
    const width = currentFrame.width;
    const height = currentFrame.height;

    if (!this.previousGray || !this.previousFrame) {
      this.previousFrame = currentFrame;
      this.previousGray = currGray;
      return null;
    }

    const { gridSpacing } = this.config;

    const gridWidth = Math.floor(width / gridSpacing);
    const gridHeight = Math.floor(height / gridSpacing);

    const vectors: FlowVector[][] = [];
    const confidence: number[][] = [];

    for (let gy = 0; gy < gridHeight; gy++) {
      vectors[gy] = [];
      confidence[gy] = [];

      for (let gx = 0; gx < gridWidth; gx++) {
        const x = Math.floor(gx * gridSpacing + gridSpacing / 2);
        const y = Math.floor(gy * gridSpacing + gridSpacing / 2);

        const match = this.findBestMatch(
          this.previousGray,
          currGray,
          width, height,
          x, y
        );

        vectors[gy][gx] = { x: match.dx, y: match.dy };
        confidence[gy][gx] = match.confidence;
      }
    }

    // Update previous frame
    this.previousFrame = currentFrame;
    this.previousGray = currGray;

    return {
      width,
      height,
      cellSize: gridSpacing,
      vectors,
      confidence,
      timestamp
    };
  }

  /**
   * Decompose flow field into translation, rotation, and expansion
   */
  decomposeFlow(flow: FlowField): FlowDecomposition {
    const cx = flow.width / 2;
    const cy = flow.height / 2;

    let totalDx = 0, totalDy = 0;
    let totalRotation = 0;
    let totalExpansion = 0;
    let totalWeight = 0;

    const gridHeight = flow.vectors.length;
    const gridWidth = flow.vectors[0]?.length || 0;

    // First pass: compute weighted averages
    for (let gy = 0; gy < gridHeight; gy++) {
      for (let gx = 0; gx < gridWidth; gx++) {
        const conf = flow.confidence[gy][gx];
        if (conf < this.config.minConfidence) continue;

        const v = flow.vectors[gy][gx];
        const x = (gx + 0.5) * flow.cellSize - cx;
        const y = (gy + 0.5) * flow.cellSize - cy;
        const r = Math.sqrt(x * x + y * y);

        totalDx += v.x * conf;
        totalDy += v.y * conf;
        totalWeight += conf;

        // Rotation: tangential component of flow
        if (r > 10) {
          const tangentX = -y / r;
          const tangentY = x / r;
          const tangentialFlow = v.x * tangentX + v.y * tangentY;
          totalRotation += (tangentialFlow / r) * conf;
        }

        // Expansion: radial component of flow
        if (r > 10) {
          const radialX = x / r;
          const radialY = y / r;
          const radialFlow = v.x * radialX + v.y * radialY;
          totalExpansion += (radialFlow / r) * conf;
        }
      }
    }

    if (totalWeight < 0.1) {
      return {
        translation: { x: 0, y: 0 },
        rotation: 0,
        expansion: 1,
        foe: null,
        residual: null,
        confidence: 0
      };
    }

    const translation: FlowVector = {
      x: totalDx / totalWeight,
      y: totalDy / totalWeight
    };

    const rotation = totalRotation / totalWeight;
    const expansion = 1 + totalExpansion / totalWeight;

    // Find Focus of Expansion (FOE)
    let foe: { x: number; y: number } | null = null;
    if (Math.abs(expansion - 1) > 0.01) {
      // FOE is where radial flow converges to zero
      // Approximate: center + translation / (expansion - 1)
      const scale = 1 / (expansion - 1);
      if (Math.abs(scale) < 1000) {
        foe = {
          x: cx - translation.x * scale,
          y: cy - translation.y * scale
        };
      }
    }

    // Compute residual flow (subtract ego-motion model)
    const residual = this.computeResidual(flow, translation, rotation, expansion);

    // Update history
    const decomposition: FlowDecomposition = {
      translation,
      rotation,
      expansion,
      foe,
      residual,
      confidence: totalWeight / (gridWidth * gridHeight)
    };

    this.flowHistory.push(decomposition);
    if (this.flowHistory.length > this.maxHistory) {
      this.flowHistory.shift();
    }

    return decomposition;
  }

  /**
   * Compute residual flow after removing ego-motion
   */
  private computeResidual(
    flow: FlowField,
    translation: FlowVector,
    rotation: number,
    expansion: number
  ): FlowField {
    const cx = flow.width / 2;
    const cy = flow.height / 2;

    const gridHeight = flow.vectors.length;
    const gridWidth = flow.vectors[0]?.length || 0;

    const residualVectors: FlowVector[][] = [];
    const residualConfidence: number[][] = [];

    for (let gy = 0; gy < gridHeight; gy++) {
      residualVectors[gy] = [];
      residualConfidence[gy] = [];

      for (let gx = 0; gx < gridWidth; gx++) {
        const v = flow.vectors[gy][gx];
        const conf = flow.confidence[gy][gx];

        const x = (gx + 0.5) * flow.cellSize - cx;
        const y = (gy + 0.5) * flow.cellSize - cy;

        // Expected flow from ego-motion model
        // Translation component
        let expectedX = translation.x;
        let expectedY = translation.y;

        // Rotation component (tangential)
        const r = Math.sqrt(x * x + y * y);
        if (r > 1) {
          expectedX += -y * rotation;
          expectedY += x * rotation;
        }

        // Expansion component (radial)
        expectedX += x * (expansion - 1);
        expectedY += y * (expansion - 1);

        // Residual = observed - expected
        residualVectors[gy][gx] = {
          x: v.x - expectedX,
          y: v.y - expectedY
        };
        residualConfidence[gy][gx] = conf;
      }
    }

    return {
      width: flow.width,
      height: flow.height,
      cellSize: flow.cellSize,
      vectors: residualVectors,
      confidence: residualConfidence,
      timestamp: flow.timestamp
    };
  }

  /**
   * Predict optical flow from camera rotation (using IMU)
   */
  predictFlowFromRotation(
    rotor: Rotor3D,
    dt: number,
    focalLength: number,
    width: number,
    height: number
  ): FlowVector {
    // Extract rotation axis and angle from rotor change
    const angle = 2 * Math.acos(Math.min(1, Math.abs(rotor.s)));
    const sinHalf = Math.sin(angle / 2);

    if (Math.abs(sinHalf) < 1e-10) {
      return { x: 0, y: 0 };
    }

    // Angular velocity about each axis
    const omegaX = -rotor.e23 / sinHalf * angle / dt;
    const omegaY = -rotor.e31 / sinHalf * angle / dt;
    const omegaZ = -rotor.e12 / sinHalf * angle / dt;

    // Predicted flow at image center from pure rotation
    // Flow_x ≈ f * omega_y (yaw causes horizontal flow)
    // Flow_y ≈ -f * omega_x (pitch causes vertical flow)
    // Roll causes rotational flow pattern, not translation

    return {
      x: focalLength * omegaY,
      y: -focalLength * omegaX
    };
  }

  /**
   * Separate camera motion from object motion using IMU
   */
  separateMotion(
    observedFlow: FlowDecomposition,
    predictedCameraFlow: FlowVector
  ): {
    cameraMotion: FlowVector;
    objectMotion: FlowVector;
    confidence: number;
  } {
    // Object motion = observed - predicted camera motion
    const objectMotion: FlowVector = {
      x: observedFlow.translation.x - predictedCameraFlow.x,
      y: observedFlow.translation.y - predictedCameraFlow.y
    };

    // Confidence based on how well the model fits
    const residualMag = Math.sqrt(objectMotion.x ** 2 + objectMotion.y ** 2);
    const observedMag = Math.sqrt(
      observedFlow.translation.x ** 2 + observedFlow.translation.y ** 2
    );

    const confidence = observedMag > 0.1
      ? Math.max(0, 1 - residualMag / (2 * observedMag))
      : observedFlow.confidence;

    return {
      cameraMotion: predictedCameraFlow,
      objectMotion,
      confidence
    };
  }

  /**
   * Estimate velocity from flow (requires distance estimate)
   */
  flowToVelocity(
    flow: FlowVector,
    distance: number,
    focalLength: number,
    dt: number
  ): Vector3 {
    // Velocity = (flow * distance) / (focal_length * dt)
    const scale = distance / (focalLength * dt);

    return {
      x: flow.x * scale,
      y: flow.y * scale,
      z: 0 // Can't determine Z velocity without stereo or depth
    };
  }

  /**
   * Get average flow over recent history
   */
  getAverageFlow(): FlowDecomposition | null {
    if (this.flowHistory.length === 0) return null;

    let avgTx = 0, avgTy = 0;
    let avgRot = 0, avgExp = 0;
    let totalConf = 0;

    for (const f of this.flowHistory) {
      avgTx += f.translation.x * f.confidence;
      avgTy += f.translation.y * f.confidence;
      avgRot += f.rotation * f.confidence;
      avgExp += f.expansion * f.confidence;
      totalConf += f.confidence;
    }

    if (totalConf < 0.01) return null;

    return {
      translation: { x: avgTx / totalConf, y: avgTy / totalConf },
      rotation: avgRot / totalConf,
      expansion: avgExp / totalConf,
      foe: null,
      residual: null,
      confidence: totalConf / this.flowHistory.length
    };
  }

  /**
   * Reset state
   */
  reset(): void {
    this.previousFrame = null;
    this.previousGray = null;
    this.flowHistory = [];
  }

  /**
   * Update configuration
   */
  setConfig(config: Partial<OpticalFlowConfig>): void {
    this.config = { ...this.config, ...config };
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// LOG-POLAR TRANSFORM (Rotation/Scale invariant)
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Transform image to log-polar coordinates
 * Rotation becomes vertical translation
 * Scale becomes horizontal translation
 */
export function toLogPolar(
  imageData: ImageData,
  centerX: number,
  centerY: number,
  outputWidth: number,
  outputHeight: number
): ImageData {
  const input = imageData.data;
  const inW = imageData.width;
  const inH = imageData.height;

  const output = new ImageData(outputWidth, outputHeight);
  const outData = output.data;

  const maxRadius = Math.sqrt(centerX ** 2 + centerY ** 2);
  const logMaxRadius = Math.log(maxRadius);

  for (let outY = 0; outY < outputHeight; outY++) {
    const theta = (outY / outputHeight) * 2 * Math.PI;

    for (let outX = 0; outX < outputWidth; outX++) {
      const logR = (outX / outputWidth) * logMaxRadius;
      const r = Math.exp(logR);

      const inX = Math.round(centerX + r * Math.cos(theta));
      const inY = Math.round(centerY + r * Math.sin(theta));

      const outIdx = (outY * outputWidth + outX) * 4;

      if (inX >= 0 && inX < inW && inY >= 0 && inY < inH) {
        const inIdx = (inY * inW + inX) * 4;
        outData[outIdx] = input[inIdx];
        outData[outIdx + 1] = input[inIdx + 1];
        outData[outIdx + 2] = input[inIdx + 2];
        outData[outIdx + 3] = input[inIdx + 3];
      } else {
        outData[outIdx + 3] = 0; // Transparent
      }
    }
  }

  return output;
}

/**
 * Measure rotation and scale by cross-correlating log-polar images
 */
export function measureRotationScale(
  logPolar1: ImageData,
  logPolar2: ImageData
): { rotation: number; scale: number; confidence: number } {
  // Convert to grayscale
  const gray1 = new Float32Array(logPolar1.width * logPolar1.height);
  const gray2 = new Float32Array(logPolar2.width * logPolar2.height);

  for (let i = 0; i < gray1.length; i++) {
    const j = i * 4;
    gray1[i] = 0.299 * logPolar1.data[j] + 0.587 * logPolar1.data[j + 1] + 0.114 * logPolar1.data[j + 2];
    gray2[i] = 0.299 * logPolar2.data[j] + 0.587 * logPolar2.data[j + 1] + 0.114 * logPolar2.data[j + 2];
  }

  // Simple phase correlation (FFT would be faster for large images)
  const width = logPolar1.width;
  const height = logPolar1.height;

  let bestDx = 0, bestDy = 0, bestCorr = -Infinity;

  // Search for peak correlation
  const searchRangeX = Math.min(width / 4, 50);
  const searchRangeY = Math.min(height / 4, 50);

  for (let dy = -searchRangeY; dy <= searchRangeY; dy++) {
    for (let dx = -searchRangeX; dx <= searchRangeX; dx++) {
      let corr = 0;
      let count = 0;

      for (let y = 0; y < height; y++) {
        const y2 = (y + dy + height) % height;
        for (let x = 0; x < width; x++) {
          const x2 = (x + dx + width) % width;
          corr += gray1[y * width + x] * gray2[y2 * width + x2];
          count++;
        }
      }

      corr /= count;
      if (corr > bestCorr) {
        bestCorr = corr;
        bestDx = dx;
        bestDy = dy;
      }
    }
  }

  // Convert to rotation and scale
  const rotation = (bestDy / height) * 2 * Math.PI;
  const logMaxRadius = Math.log(Math.sqrt((width / 2) ** 2 + (height / 2) ** 2));
  const scale = Math.exp((bestDx / width) * logMaxRadius);

  // Confidence from correlation strength
  const maxCorr = 255 * 255; // Max possible correlation
  const confidence = Math.max(0, Math.min(1, bestCorr / (maxCorr * 0.5)));

  return { rotation, scale, confidence };
}
