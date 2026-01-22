
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
