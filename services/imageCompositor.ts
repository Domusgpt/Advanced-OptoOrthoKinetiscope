import { CapturePoint, CalibrationData, Vector3, Quaternion } from "../types";
import { get24CellVertices, rotate4D_Hyper, project4Dto2D } from "./cpeMath";

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