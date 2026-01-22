import { CapturePoint, CalibrationData, Vector3, Quaternion } from "../types";
import { get24CellVertices, rotate4D_Hyper, project4Dto2D } from "./cpeMath";
import {
  createVelocityNomogram, drawNomogram,
  generateLogPolarGrating, generateCircularGrating,
  drawQuaternionMoire, drawDifferentialFlowField,
  computeLinearMoire, Nomogram
} from "./MoireEngine";
import { quaternionToRotor, rotorToAxisAngle } from "./GeometricAlgebra";

// --- COLOR PALETTE (Strict Data Semantics) ---
const C_SENSOR = "#FACC15"; // Gold (Inertial/Lattice) - The "Dead Reckoning"
const C_OPTIC  = "#22D3EE"; // Cyan (Epipolar/Visual) - The "Observed Reality"
const C_DELTA  = "#F43F5E"; // Rose (Difference/Error) - The "Subject Motion"
const C_VOID   = "#020617"; // Slate 950
const C_QUALITY = "#10B981"; // Emerald (Quality Control)

/**
 * SYSTEM 1: THE EPIPOLAR PLANE (Row 1)
 * Visualizes the "Focus of Expansion" (FOE) based on accelerometer data.
 * Static objects must flow along these lines.
 */
function drawEpipolarGeometry(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number, h: number,
    cap: CapturePoint
) {
    ctx.save();
    ctx.translate(x, y);
    
    // 1. Calculate the Focus of Expansion (FOE)
    const sensitivity = 80;
    const foeX = (w / 2) - (cap.acceleration.x * sensitivity);
    const foeY = (h / 2) + (cap.acceleration.y * sensitivity);
    
    // 2. Draw the FOE Reticle (if on screen or close)
    ctx.fillStyle = C_OPTIC;
    ctx.shadowColor = C_OPTIC;
    ctx.shadowBlur = 10;
    ctx.beginPath(); ctx.arc(foeX, foeY, 4, 0, Math.PI*2); ctx.fill();
    ctx.shadowBlur = 0;
    
    // 3. Draw Epipolar Rays (The "Flow Lines")
    ctx.strokeStyle = "rgba(34, 211, 238, 0.4)";
    ctx.lineWidth = 1;
    
    const rayCount = 12;
    for(let i=0; i<rayCount; i++) {
        const angle = (Math.PI * 2 * i) / rayCount;
        const rayLen = Math.max(w, h) * 2.0;
        const endX = foeX + Math.cos(angle) * rayLen;
        const endY = foeY + Math.sin(angle) * rayLen;
        
        ctx.beginPath();
        ctx.moveTo(foeX, foeY);
        ctx.lineTo(endX, endY);
        ctx.stroke();
    }

    // Label
    ctx.fillStyle = C_OPTIC;
    ctx.font = "bold 9px monospace";
    ctx.fillText("EPIPOLAR FIELD [CYAN_RAY]", 5, 12);
    ctx.fillStyle = "rgba(34, 211, 238, 0.6)";
    ctx.fillText("FLOW DEVIATION = MOTION", 5, 22);

    ctx.restore();
}

/**
 * SYSTEM 2: THE HYPER-LATTICE INTERFEROMETER (Row 2)
 * Projects the 4D Dead-Reckoning Mesh onto the Telephoto image.
 * Uses VECTORIZED OFFSET LOGIC based on Jerk Direction + Magnitude.
 */
function drawHyperLatticeInterference(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number, h: number,
    cap: CapturePoint
) {
    ctx.save();
    ctx.translate(x, y);

    // 1. Calculate Lattice State (Sensor Truth)
    const vertices = get24CellVertices();
    const scale = 200 * cap.zoomLevel; 
    
    const projected = vertices.map(v => {
        const rotated = rotate4D_Hyper(v, cap.orientation, cap.acceleration);
        return project4Dto2D(rotated, w, h, scale, 2.5);
    });

    // 2. Calculate Vectorized Uncertainty
    // Previously we just used magnitude. Now we use Direction.
    // X-Accel drives horizontal shear, Y-Accel drives vertical shear.
    const jerkX = cap.acceleration.x;
    const jerkY = cap.acceleration.y;
    const interval = cap.shutterInterval || 33;
    
    // Calculate shear vector
    const shearX = -jerkX * (interval * 0.2); 
    const shearY = jerkY * (interval * 0.2);
    
    const totalShear = Math.sqrt(shearX**2 + shearY**2);
    const isSignificant = totalShear > 2.0;

    // 3. Draw the Lattice (The "Sensor Mesh")
    // Base Thickness varies by stability
    ctx.lineWidth = isSignificant ? 2.5 : 1.5;
    ctx.strokeStyle = "rgba(250, 204, 21, 0.6)"; // Gold (Sensor)

    // Helper to draw lines
    const drawMesh = (offsetX = 0, offsetY = 0) => {
        ctx.beginPath();
        for (let i = 0; i < projected.length; i++) {
            for (let j = i + 1; j < projected.length; j++) {
                const p1 = projected[i]; const p2 = projected[j];
                if (p1.depth === -1 || p2.depth === -1) continue;
                const dist = Math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2);
                if (dist < scale * 1.3 && dist > scale * 0.1) {
                     ctx.moveTo(p1.x + offsetX, p1.y + offsetY); 
                     ctx.lineTo(p2.x + offsetX, p2.y + offsetY);
                }
            }
        }
        ctx.stroke();
    };

    // Draw Main Mesh (Reference)
    drawMesh(0,0);

    // Render "Ghost" mesh if uncertainty is significant
    // CHROMODYNAMIC SHIFT: Color indicates direction of the error
    if (isSignificant) {
        ctx.save();
        
        // Z-Accel determines if the ghost is Red (Forward error) or Blue (Backward/Drag error)
        // or we use horizontal direction. Let's use Z for Temperature, but position for Shear.
        const zForce = cap.acceleration.z;
        const ghostColor = zForce > 0 ? "rgba(239, 68, 68, 0.5)" : "rgba(59, 130, 246, 0.5)";
        
        ctx.strokeStyle = ghostColor;
        ctx.lineWidth = 1.0 + (totalShear * 0.1); // Thicker ghost = more error mass
        
        // Apply the vector offset
        ctx.translate(shearX, shearY); 
        drawMesh(0,0);
        ctx.restore();
    }

    // 4. Draw "Anchor Nodes"
    projected.forEach((p, i) => {
        if(i % 6 === 0 && p.depth !== -1) { 
            ctx.fillStyle = C_SENSOR;
            ctx.shadowColor = C_SENSOR;
            ctx.shadowBlur = 5;
            ctx.beginPath(); ctx.arc(p.x, p.y, 2.5, 0, Math.PI*2); ctx.fill();
            ctx.shadowBlur = 0;
        }
    });

    // Label
    ctx.fillStyle = C_SENSOR;
    ctx.font = "bold 9px monospace";
    ctx.fillText("INERTIAL LATTICE [GOLD_NODE]", 5, 12);
    ctx.fillStyle = isSignificant ? (cap.acceleration.z > 0 ? "#ef4444" : "#3b82f6") : "rgba(250, 204, 21, 0.6)";
    ctx.fillText(`SHEAR VECTOR: [${shearX.toFixed(1)}, ${shearY.toFixed(1)}]`, 5, 22);

    ctx.restore();
}

/**
 * SYSTEM 3: THE CHRONO-ABACUS (The Velocity Rail)
 */
function drawVelocityRail(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number,
    captures: CapturePoint[]
) {
    ctx.save();
    ctx.translate(x, y);

    // The Rail
    ctx.strokeStyle = "#334155";
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(w, 0); ctx.stroke();

    if(captures.length < 2) { ctx.restore(); return; }

    const tStart = captures[0].relativeTime;
    const tEnd = captures[captures.length-1].relativeTime;
    const tDuration = tEnd - tStart || 1;
    
    captures.forEach((cap, i) => {
        const pct = (cap.relativeTime - tStart) / tDuration;
        const px = pct * w;

        const energy = Math.sqrt(cap.acceleration.x**2 + cap.acceleration.y**2 + cap.acceleration.z**2);
        const radius = 3 + (energy * 0.5);

        // Color coding frames by time
        ctx.fillStyle = cap.spectralPhase === 'RED' ? '#F87171' : cap.spectralPhase === 'GREEN' ? '#4ADE80' : '#60A5FA';
        ctx.shadowColor = ctx.fillStyle;
        ctx.shadowBlur = 5;
        
        ctx.beginPath(); ctx.arc(px, 0, radius, 0, Math.PI*2); ctx.fill();
        ctx.shadowBlur = 0;

        ctx.fillStyle = "#fff";
        ctx.font = "9px monospace";
        ctx.textAlign = "center";
        const yOffset = i % 2 === 0 ? 15 : -10;
        ctx.fillText(`+${cap.relativeTime}ms`, px, yOffset);
    });
    
    ctx.fillStyle = "#64748b";
    ctx.font = "9px monospace";
    ctx.textAlign = "right";
    ctx.fillText("TIMING/JERK RAIL [ROSE_BEAD]", w, -10);

    ctx.restore();
}

/**
 * SYSTEM 4: VELOCITY NOMOGRAM (Analog Calculator for Vision LLM)
 * Replaces textual speed estimates with a geometric readout
 * The AI draws a mental line between Time and Distance to read Speed
 */
function drawVelocityNomogramPanel(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number, h: number,
    captures: CapturePoint[]
) {
    ctx.save();
    ctx.translate(x, y);

    // Create nomogram with current data
    const nomogram = createVelocityNomogram([16, 1000], [0.1, 10]);

    // Calculate time delta from captures
    if (captures.length >= 2) {
        const timeDeltaMs = captures[captures.length - 1].relativeTime - captures[0].relativeTime;
        nomogram.currentA = Math.max(16, Math.min(1000, timeDeltaMs));

        // Estimate distance from acceleration magnitude
        const avgAccel = captures.reduce((sum, c) => {
            const mag = Math.sqrt(c.acceleration.x**2 + c.acceleration.y**2 + c.acceleration.z**2);
            return sum + mag;
        }, 0) / captures.length;

        // Very rough distance estimate: d = 0.5 * a * t^2
        const tSec = timeDeltaMs / 1000;
        const estDistance = 0.5 * avgAccel * tSec * tSec;
        nomogram.currentB = Math.max(0.1, Math.min(10, estDistance));

        // Compute result
        nomogram.computedC = (nomogram.currentB / (nomogram.currentA / 1000));
    }

    // Draw the nomogram
    drawNomogram(ctx, 0, 0, w, h, nomogram, {
        bg: 'rgba(15, 23, 42, 0.9)',
        scale: '#94a3b8',
        line: '#FACC15',
        marker: '#F43F5E'
    });

    // Title
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('VELOCITY NOMOGRAM', w/2, h - 5);

    ctx.restore();
}

/**
 * SYSTEM 5: MOIRÉ QUATERNION DISPLAY
 * Visualizes orientation as overlapping gratings
 * The beat pattern encodes rotation - Vision LLM reads fringe shift
 */
function drawMoireQuaternionDisplay(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number, h: number,
    orientation: Quaternion,
    referenceOrientation: Quaternion | null
) {
    ctx.save();
    ctx.translate(x, y);

    // Background
    ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = '#334155';
    ctx.strokeRect(0, 0, w, h);

    // Draw quaternion moiré
    drawQuaternionMoire(
        ctx, 5, 15, w - 10, h - 30,
        orientation.w, orientation.x, orientation.y, orientation.z,
        referenceOrientation
    );

    // Extract rotation info for label
    const rotor = quaternionToRotor(orientation);
    const { axis, angle } = rotorToAxisAngle(rotor);
    const degrees = (angle * 180 / Math.PI).toFixed(1);

    // Title
    ctx.fillStyle = '#22D3EE';
    ctx.font = 'bold 9px monospace';
    ctx.textAlign = 'center';
    ctx.fillText('ORIENTATION MOIRÉ', w/2, 12);

    ctx.restore();
}

/**
 * SYSTEM 6: DIFFERENTIAL FLOW FIELD
 * Yellow = Expected (gyro), Cyan = Observed (optical)
 * Rose difference = Subject Motion isolated from camera motion
 */
function drawDifferentialFlow(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number, h: number,
    cap: CapturePoint
) {
    ctx.save();
    ctx.translate(x, y);

    // Compute expected flow from gyro (rotation rate)
    // Rotation causes apparent motion opposite to rotation direction
    const expectedFlow = {
        x: -cap.rotationRate.y * 0.01,  // Yaw causes horizontal flow
        y: cap.rotationRate.x * 0.01    // Pitch causes vertical flow
    };

    // For now, observed flow is simulated as expected + noise
    // In real implementation, this would come from optical flow analysis
    const observedFlow = {
        x: expectedFlow.x + (Math.random() - 0.5) * 0.5,
        y: expectedFlow.y + (Math.random() - 0.5) * 0.5
    };

    drawDifferentialFlowField(ctx, 0, 0, w, h, expectedFlow, observedFlow, 30);

    // Title
    ctx.fillStyle = '#ffffff';
    ctx.font = 'bold 9px monospace';
    ctx.fillText('DIFFERENTIAL FLOW', 5, 12);

    ctx.restore();
}

/**
 * SYSTEM 7: LOG-POLAR GRATING OVERLAY
 * Encodes rotation/scale as overlapping spiral patterns
 * Fringe shift = complex multiplication result
 */
function drawLogPolarOverlay(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number, h: number,
    orientation: Quaternion,
    zoomLevel: number
) {
    ctx.save();
    ctx.translate(x, y);

    // Reference grating (identity orientation)
    ctx.globalAlpha = 0.3;
    generateLogPolarGrating(ctx, 0, 0, w, h, {
        frequency: 6,
        phase: 0,
        centerX: w/2,
        centerY: h/2
    }, '#FACC15');

    // Current orientation grating (phase shifted by quaternion)
    const rotor = quaternionToRotor(orientation);
    const { angle } = rotorToAxisAngle(rotor);

    generateLogPolarGrating(ctx, 0, 0, w, h, {
        frequency: 6,
        phase: angle,  // Phase encodes rotation
        centerX: w/2,
        centerY: h/2
    }, '#22D3EE');

    ctx.globalAlpha = 1.0;

    // Moiré beat information
    const moire = computeLinearMoire(
        { frequency: 6, angle: 0 },
        { frequency: 6, angle: angle }
    );

    ctx.fillStyle = '#F43F5E';
    ctx.font = '9px monospace';
    ctx.fillText(`MAG: ${moire.magnification.toFixed(1)}x`, 5, h - 5);

    ctx.restore();
}

/**
 * SYSTEM 8: STADIMETRIC FLOOR GRID
 * Projects a reference grid on the floor plane
 * Grid cell size = 1 meter (calibrated to user height)
 * Vision LLM counts cells to measure distance
 */
function drawStadimetricGrid(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, w: number, h: number,
    cap: CapturePoint,
    deviceHeightCm: number
) {
    ctx.save();
    ctx.translate(x, y);

    // Compute pitch from quaternion
    const q = cap.orientation;
    const pitch = Math.asin(2 * (q.w * q.y - q.z * q.x));

    // Only draw if looking down (negative pitch)
    if (pitch > 0.1) {
        ctx.fillStyle = '#94a3b8';
        ctx.font = '9px monospace';
        ctx.fillText('LOOKING UP - NO FLOOR', w/2 - 50, h/2);
        ctx.restore();
        return;
    }

    const deviceHeightM = deviceHeightCm / 100;
    const fov = 60 * Math.PI / 180;  // Assume 60° FOV

    // Draw perspective grid
    ctx.strokeStyle = 'rgba(250, 204, 21, 0.4)';
    ctx.lineWidth = 1;

    // Horizon line (based on pitch)
    const horizonY = h/2 + pitch * (h / fov);

    // Draw converging lines to horizon
    const vanishingX = w/2;
    const numLines = 10;

    for (let i = -numLines; i <= numLines; i++) {
        const baseX = w/2 + i * (w / numLines);
        ctx.beginPath();
        ctx.moveTo(baseX, h);
        ctx.lineTo(vanishingX, Math.max(0, horizonY));
        ctx.stroke();
    }

    // Horizontal lines (perspective scaled)
    for (let d = 1; d <= 5; d++) {
        // Distance d meters
        const screenY = horizonY + (h - horizonY) * (deviceHeightM / d);

        if (screenY > 0 && screenY < h) {
            ctx.beginPath();
            ctx.moveTo(0, screenY);
            ctx.lineTo(w, screenY);
            ctx.stroke();

            // Label distance
            ctx.fillStyle = '#FACC15';
            ctx.font = '8px monospace';
            ctx.fillText(`${d}m`, 5, screenY - 2);
        }
    }

    // Title
    ctx.fillStyle = '#FACC15';
    ctx.font = 'bold 9px monospace';
    ctx.fillText('STADIMETRIC GRID', 5, 12);

    ctx.restore();
}

/**
 * MAIN COMPOSITOR
 */
async function createTrileticComposite(
    images: HTMLImageElement[],
    captures: CapturePoint[],
    outW: number, 
    outH: number, 
    ctx: CanvasRenderingContext2D
) {
    const cellW = outW / 3;
    const cellH = outH / 3; 

    // Background
    ctx.fillStyle = C_VOID;
    ctx.fillRect(0, 0, outW, outH);

    captures.forEach((cap, i) => {
        if (!images[i]) return;
        const row = Math.floor(i / 3);
        const col = i % 3;
        const x = col * cellW;
        const y = row * cellH;

        // 1. Draw Image
        ctx.save();
        ctx.beginPath(); ctx.rect(x, y, cellW, cellH); ctx.clip();
        
        if (row === 2) {
            ctx.filter = "contrast(1.4) brightness(1.1)";
        }
        ctx.drawImage(images[i], x, y, cellW, cellH);
        ctx.filter = "none";

        // 2. APPLY THE TRILETIC OVERLAYS
        if (row === 0) {
            drawEpipolarGeometry(ctx, x, y, cellW, cellH, cap);
        }
        else if (row === 1) {
            drawHyperLatticeInterference(ctx, x, y, cellW, cellH, cap);
        }
        else if (row === 2) {
            ctx.fillStyle = C_DELTA;
            ctx.font = "bold 9px monospace";
            ctx.fillText("SPECTRAL STRUCTURE", x + 5, y + 12);
        }

        ctx.strokeStyle = "rgba(255,255,255,0.1)";
        ctx.lineWidth = 1;
        ctx.strokeRect(x, y, cellW, cellH);

        ctx.restore();
    });

    // 3. THE OPTIC LOFT
    for(let c=0; c<3; c++) {
        const xBase = c * cellW;
        ctx.save();
        ctx.strokeStyle = C_OPTIC;
        ctx.setLineDash([3, 5]);
        ctx.lineWidth = 1.5;
        ctx.globalAlpha = 0.6;
        
        const r0CenterX = xBase + cellW/2;
        const r0BottomY = cellH;
        
        const r1TopLeftX = xBase + 5; 
        const r1TopRightX = xBase + cellW - 5;
        const r1TopY = cellH; 
        
        ctx.beginPath();
        ctx.moveTo(r0CenterX, r0BottomY - 20); 
        ctx.lineTo(r1TopLeftX, r1TopY + 20);   
        ctx.stroke();
        
        ctx.beginPath();
        ctx.moveTo(r0CenterX, r0BottomY - 20);
        ctx.lineTo(r1TopRightX, r1TopY + 20);
        ctx.stroke();
        
        ctx.restore();
    }

    // 4. THE VELOCITY RAIL
    drawVelocityRail(ctx, 60, outH - 30, outW - 120, captures);

    // 5. MOIRÉ VISUALIZATION PANELS (Right side strip)
    const moirePanelW = 120;
    const moirePanelH = (outH - 60) / 3;

    // Reference orientation (first capture) for moiré comparison
    const refOrientation = captures.length > 0 ? captures[0].orientation : null;
    const currentOrientation = captures.length > 0 ? captures[captures.length - 1].orientation : { w: 1, x: 0, y: 0, z: 0 };
    const currentCapture = captures.length > 0 ? captures[captures.length - 1] : null;

    // Draw the moiré panels on the right edge
    if (currentCapture) {
        // Panel 1: Quaternion Moiré
        drawMoireQuaternionDisplay(
            ctx, outW - moirePanelW - 5, 5,
            moirePanelW, moirePanelH,
            currentOrientation, refOrientation
        );

        // Panel 2: Log-Polar Overlay
        drawLogPolarOverlay(
            ctx, outW - moirePanelW - 5, moirePanelH + 10,
            moirePanelW, moirePanelH,
            currentOrientation, currentCapture.zoomLevel
        );

        // Panel 3: Velocity Nomogram
        drawVelocityNomogramPanel(
            ctx, outW - moirePanelW - 5, 2 * moirePanelH + 15,
            moirePanelW, moirePanelH - 20,
            captures
        );
    }

    // 5. SIDE LABELS
    const labels = [
        { text: "WIDE [CONTEXT]", color: C_OPTIC, y: cellH * 0.5 },
        { text: "TELE [DETAIL]", color: C_SENSOR, y: cellH * 1.5 },
        { text: "ACTIVE [STRUCT]", color: C_DELTA, y: cellH * 2.5 }
    ];

    ctx.save();
    ctx.font = "bold 11px monospace";
    labels.forEach(l => {
        ctx.save();
        ctx.translate(15, l.y);
        ctx.rotate(-Math.PI/2);
        ctx.fillStyle = l.color;
        ctx.shadowColor = l.color; ctx.shadowBlur = 5;
        ctx.fillText(l.text, -40, 0); 
        ctx.restore();
    });
    ctx.restore();
}

/**
 * LEGEND RENDERER with QUALITY CONTROL
 */
function drawVisualKey(
    ctx: CanvasRenderingContext2D, 
    x: number, y: number, w: number, h: number,
    avgEfficiency: number, avgRejection: number
) {
    ctx.save();
    ctx.fillStyle = C_VOID;
    ctx.fillRect(x, y, w, h);
    ctx.strokeStyle = "#334155"; ctx.strokeRect(x, y, w, h);

    const zoneW = w / 4; // Split into 4 zones now
    const centerY = y + h/2;
    const labelY = y + h - 15;

    // KEY 1: EPIPOLAR
    const z1x = x + zoneW * 0.5;
    ctx.save(); ctx.translate(z1x, centerY - 10);
    ctx.strokeStyle = C_OPTIC; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(-15, -10); ctx.moveTo(0,0); ctx.lineTo(-15, 10); ctx.moveTo(0,0); ctx.lineTo(15, 0); ctx.stroke();
    ctx.restore();
    ctx.fillStyle = "#fff"; ctx.font = "10px monospace"; ctx.textAlign = "center";
    ctx.fillText("CYAN_RAY = FLOW", z1x, labelY);

    // KEY 2: LATTICE
    const z2x = x + zoneW * 1.5;
    ctx.save(); ctx.translate(z2x, centerY - 10);
    ctx.strokeStyle = C_SENSOR; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(-10, -10); ctx.lineTo(10, 10); ctx.moveTo(10, -10); ctx.lineTo(-10, 10); ctx.stroke();
    ctx.restore();
    ctx.fillText("GOLD_NODE = INERTIA", z2x, labelY);

    // KEY 3: INTERFERENCE
    const z3x = x + zoneW * 2.5;
    ctx.fillStyle = "#94a3b8";
    ctx.font = "9px monospace";
    ctx.fillText("RED/BLUE SHIFT", z3x, centerY);
    ctx.fillText("DIR. OF ERROR", z3x, centerY + 12);

    // KEY 4: QUALITY CONTROL (New)
    const z4x = x + zoneW * 3.5;
    const effColor = avgEfficiency > 0.8 ? C_QUALITY : C_DELTA;
    
    ctx.fillStyle = effColor;
    ctx.font = "bold 14px monospace";
    ctx.fillText(`${(avgEfficiency * 100).toFixed(0)}%`, z4x, centerY - 5);
    
    ctx.fillStyle = "#94a3b8";
    ctx.font = "9px monospace";
    ctx.fillText("VEC. EFFICIENCY", z4x, centerY + 10);
    
    // Draw micro bar
    ctx.fillStyle = "#334155";
    ctx.fillRect(z4x - 20, centerY + 15, 40, 4);
    ctx.fillStyle = effColor;
    ctx.fillRect(z4x - 20, centerY + 15, 40 * avgEfficiency, 4);

    ctx.restore();
}

/**
 * PUBLIC EXPORT
 */
export const generateCompositeFrame = async (
  captures: CapturePoint[],
  calibration: CalibrationData
): Promise<string> => {
  const images: HTMLImageElement[] = await Promise.all(
    captures.map((c) =>
      new Promise<HTMLImageElement>((resolve, reject) => {
        const img = new Image();
        img.onload = () => resolve(img);
        img.onerror = reject;
        img.src = c.imageUri;
      })
    )
  );

  const rawW = 1200;
  const rawH = 675; // Main content area
  const legendH = 120; 
  const totalH = rawH + legendH;

  const canvas = document.createElement("canvas");
  canvas.width = rawW;
  canvas.height = totalH;
  const ctx = canvas.getContext("2d");
  if (!ctx) throw new Error("Context failed");

  await createTrileticComposite(images, captures, rawW, rawH, ctx);
  
  // Calculate Avg Efficiency for the Legend
  const totalEff = captures.reduce((acc, c) => acc + (c as any).vectorEfficiency || 1.0, 0);
  const avgEff = totalEff / captures.length;
  
  const totalRej = captures.reduce((acc, c) => acc + (c as any).orthogonalNoise || 0, 0);
  const avgRej = totalRej / captures.length;

  drawVisualKey(ctx, 0, rawH, rawW, legendH, avgEff, avgRej);

  return canvas.toDataURL("image/jpeg", 0.95);
};