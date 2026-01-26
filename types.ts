
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

export type CaptureMode = 'TRAJECTORY' | 'VIBRATION' | 'STRUCTURAL' | 'VIDEO_STREAM' | 'MATRIX_BURST';

export interface CapturePoint {
  id: number;
  timestamp: number; // Unix epoch
  relativeTime: number; // ms from start
  imageUri: string; // Base64 data URI
  acceleration: Vector3;
  rotationRate: Vector3; // degrees per second
  orientation: Quaternion; // Derived from deviceorientation alpha/beta/gamma
  tiltAngle: number; // Pitch in degrees (derived from gravity vector)
  
  // Advanced Sensor Data
  isoEstimate?: number; // 100-3200 (Estimated from noise variance)
  exposureTime?: number; // ms
  zoomLevel: number; // 1.0 = Wide, 2.0 = Tele
  focusDistance?: number; // meters (if supported)
  
  // New: Chroma & Timing Logic
  spectralPhase?: 'RED' | 'GREEN' | 'BLUE'; // For channel filtering
  shutterInterval?: number; // ms since last frame (dynamic based on speed)

  // Hyper-Burst Matrix Data
  matrixPosition?: { row: number, col: number }; // 0-2, 0-2
  lensGroup?: 'WIDE' | 'TELE' | 'ACTIVE';
  exposureBias?: number; // EV step
}

export type UnitSystem = 'metric' | 'imperial';

export interface UserMetrics {
  referenceName: string; 
  referenceSize: number; 
  unitSystem: UnitSystem; 
  subjectType: string; 
  deviceHeightCm: number; 
}

export interface CalibrationData {
  baselineAccel: Vector3; 
  baselineGyro: Vector3;  
  motionSignature: Vector3[]; 
  scaleDelta: number; 
  lightLevel: 'low' | 'optimal' | 'bright'; 
  luxEstimate: number;
  userMetrics: UserMetrics;
  
  // Smart Metrics
  pixelsPerUnit: number; 
  gridScale: number; 
  fieldOfView: number; 
  focalLengthPx: number;
  zoomCalibrationCurve?: { zoomLevel: number, pxWidth: number }[]; // New: Lens Calibration
  
  // Adaptive Polling Metrics
  sensorReadoutLatency: number; // ms
  maxStableBurstRate: number; 
  jitterVariance: number; // 0.0 to 1.0
  isoNoiseFloor: number; // 0-255 variance
  
  // Rational Dynamics Coefficients
  gyroConfidence: number; 
  accelConfidence: number; 
  opticalConfidence: number; 
}

export interface SessionData {
  sessionId: string;
  mode: CaptureMode;
  captures: CapturePoint[];
  calibration: CalibrationData | null;
  startTime: number;
  endTime: number;
}

export interface BoundingBox {
  ymin: number;
  xmin: number;
  ymax: number;
  xmax: number;
}

export interface Point2D {
  x: number;
  y: number;
}

export interface KinematicAnalysis {
  cameraMotionDescription: string; 
  subjectMotionVector: string; 
  isolationConfidence: number; 
  depthCues: string; 
  estimatedPhysicalDisplacement: string; 
  floorMapAnalysis: string; 
  holographicInterferenceAnalysis: string;
  structuralIntegrity?: string; 
  occlusionMatrixReading?: string;
  parallaxEpitaxyAnalysis?: string;
  geometricProof?: string; // The mathematical deduction step
}

export interface AnalysisResult {
  estimatedSpeed: string;
  trajectoryDescription: string;
  confidenceScore: number;
  reasoning: string;
  objectDetected: string;
  boundingBox?: BoundingBox;
  trajectoryPoints?: Point2D[]; 
  kinematics: KinematicAnalysis; 
}

export enum AppState {
  IDLE,
  INSTRUCTIONS,
  CALIBRATION,
  CAPTURING,
  ENCODING,
  PROCESSING,
  ANALYZING,
  RESULTS,
  ERROR
}

// ═══════════════════════════════════════════════════════════════════════════
// GEOMETRIC ALGEBRA & QUANTUM TYPES
// ═══════════════════════════════════════════════════════════════════════════

/**
 * 4D Vector for hyperdimensional projections
 */
export interface Vector4 {
  x: number;
  y: number;
  z: number;
  w: number;
}

/**
 * Rotor in Clifford Algebra Cl(3,0)
 * Represents a rotation without gimbal lock
 * R = s + e12*xy + e23*yz + e31*zx
 */
export interface Rotor {
  s: number;    // Scalar part (cos(θ/2))
  e12: number;  // XY bivector
  e23: number;  // YZ bivector
  e31: number;  // ZX bivector
}

/**
 * Spinor for quantum state representation
 * |ψ⟩ = α|0⟩ + β|1⟩ where α, β ∈ ℂ
 */
export interface SpinorState {
  real0: number;  // Re(α)
  imag0: number;  // Im(α)
  real1: number;  // Re(β)
  imag1: number;  // Im(β)
}

/**
 * Bloch sphere coordinates for qubit visualization
 */
export interface BlochCoords {
  theta: number;  // Polar angle (0 = |0⟩, π = |1⟩)
  phi: number;    // Azimuthal angle (phase)
}

/**
 * Quantum gate definition
 */
export interface QuantumGateDefinition {
  name: string;
  symbol: string;
  rotorParams: {
    axis: Vector3;
    angle: number;
  };
  description: string;
}

// ═══════════════════════════════════════════════════════════════════════════
// MOIRÉ VISUALIZATION TYPES
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Grating pattern parameters for moiré generation
 */
export interface GratingConfig {
  type: 'linear' | 'circular' | 'logPolar' | 'hyperbolic' | 'spiral';
  frequency: number;
  phase: number;
  orientation: number;
  center: Point2D;
  dutyCycle: number;
  color: string;
}

/**
 * Moiré fringe analysis result
 */
export interface MoireAnalysis {
  fringeSpacing: number;
  fringeAngle: number;
  magnification: number;
  beatFrequency: number;
  phaseShift: number;
  confidence: number;
}

/**
 * Nomogram configuration for visual calculation
 */
export interface NomogramConfig {
  scaleA: {
    label: string;
    range: [number, number];
    logarithmic: boolean;
  };
  scaleB: {
    label: string;
    range: [number, number];
    logarithmic: boolean;
  };
  scaleC: {
    label: string;
    operation: 'multiply' | 'divide' | 'add' | 'subtract';
  };
}

/**
 * Hopf fibration projection data
 */
export interface HopfProjection {
  basePoint: Vector3;      // Point on S² (Bloch sphere)
  fiberPhase: number;      // Phase on S¹ fiber
  villarceauAngle: number; // Angle on the Clifford torus
}

/**
 * E8 lattice vertex for high-dimensional visualization
 */
export interface E8Vertex {
  coords: number[];  // 8-dimensional coordinates
  coxeterProjection: Point2D;
  rootType: 'integer' | 'halfInteger';
}

/**
 * Fano plane point for octonion visualization
 */
export interface FanoPoint {
  index: number;           // 1-7 for imaginary units
  position: Point2D;       // Position in 2D diagram
  quaternionicTriples: number[][]; // Lines through this point
}

// ═══════════════════════════════════════════════════════════════════════════
// PHILLIPS GATE SIMULATION TYPES
// ═══════════════════════════════════════════════════════════════════════════

/**
 * Qubit state on a 24-cell vertex
 */
export interface VertexQubitState {
  vertexIndex: number;
  spinor: SpinorState;
  bloch: BlochCoords;
  entangledIndices: number[];
  pauliRelation: string;
}

/**
 * Phillips Gate simulation snapshot
 */
export interface PhillipsGateSnapshot {
  qubits: VertexQubitState[];
  substrate: '24cell' | '600cell';
  time: number;
  energy: number;
  temperature: number;
  globalRotation: {
    left: Rotor;
    right: Rotor;
  };
}

/**
 * Cellular automaton state on polytope surface
 */
export interface PolytopeCAState {
  cells: {
    vertexIndex: number;
    state: 0 | 1;
    neighbors: number[];
  }[];
  generation: number;
  totalEnergy: number;
}

/**
 * Annealing optimization result
 */
export interface AnnealingResult {
  solution: number[];
  energy: number;
  iterations: number;
  temperatureHistory: number[];
  converged: boolean;
}
