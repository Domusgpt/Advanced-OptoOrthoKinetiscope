import React, { useEffect, useRef } from 'react';
import { CapturePoint, BoundingBox, Point2D } from '../types';
import { Download } from 'lucide-react';

interface GhostViewProps {
  captures: CapturePoint[];
  boundingBox?: BoundingBox;
  trajectoryPoints?: Point2D[];
}

const GhostView: React.FC<GhostViewProps> = ({ captures, boundingBox, trajectoryPoints }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Helper: Get color temperature based on value (0=White, -1=Blue, 1=Red)
  const getDopplerColor = (val: number, intensity: number = 1) => {
      // Clamp val between -1 and 1
      const n = Math.max(-1, Math.min(1, val));
      if (n > 0.1) return `rgba(239, 68, 68, ${intensity})`; // Red-500
      if (n < -0.1) return `rgba(59, 130, 246, ${intensity})`; // Blue-500
      return `rgba(255, 255, 255, ${intensity})`; // White
  };

  // Helper to draw the 3D Reticle with CHROMODYNAMIC PHYSICS
  const drawReticle = (ctx: CanvasRenderingContext2D, cap: CapturePoint, baseAlpha: number, cx: number, cy: number) => {
      ctx.save();
      ctx.translate(cx, cy);
      
      const q = cap.orientation;
      
      // PHYSICS EXTRACTION
      // 1. Kinetic Energy (Magnitude) -> Line Thickness
      const energy = Math.sqrt(cap.acceleration.x**2 + cap.acceleration.y**2 + cap.acceleration.z**2);
      const lineThickness = 1.0 + Math.min(3.0, energy * 0.5); // "Fat" lines for high energy
      
      // 2. Rotational Vectors -> Directional Split
      // We use gyro rates to determine the X/Y offset of the "Ghost" channels
      const yawRate = cap.rotationRate.y || 0;   // X-axis shift
      const pitchRate = cap.rotationRate.x || 0; // Y-axis shift
      const shiftScale = 2.0;
      const offX = -yawRate * shiftScale;
      const offY = pitchRate * shiftScale;

      // 3. Z-Force -> Color Temperature (Doppler)
      // +Z (Forward) = Red (Hot), -Z (Backward) = Blue (Cool)
      const zForce = cap.acceleration.z / 9.8; 
      
      // 4. Orientation Math
      const roll = Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
      const pitch = Math.asin(2*(q.w*q.y - q.z*q.x));
      const convexityFactor = Math.sin(pitch) + (zForce * 0.4); 
      
      // Dynamic Scaling 
      const scale = (cap.zoomLevel || 1.0) * (1.0 + (zForce * 0.1));
      const r = 80 * scale;

      // --- RENDER PASS ---
      
      // Apply Matrix Transform for the main reticle orientation
      ctx.rotate(roll);
      ctx.scale(1, Math.cos(pitch)); 

      const drawShape = (offsetX: number, offsetY: number, color: string, width: number) => {
         ctx.strokeStyle = color;
         ctx.lineWidth = width;
         ctx.beginPath();
         // Top Half Convexity
         ctx.moveTo(-r + offsetX, 0 + offsetY);
         ctx.bezierCurveTo(
             -r + offsetX, -r * (1 + convexityFactor) + offsetY, 
             r + offsetX, -r * (1 + convexityFactor) + offsetY, 
             r + offsetX, 0 + offsetY
         );
         // Bottom Half Convexity
         ctx.bezierCurveTo(
             r + offsetX, r * (1 - convexityFactor) + offsetY, 
             -r + offsetX, r * (1 - convexityFactor) + offsetY, 
             -r + offsetX, 0 + offsetY
         );
         ctx.stroke();
      };

      // 1. Draw "Past" Ghost (Lag) - Negative Vector
      // If moving RIGHT, drag is LEFT.
      if (Math.abs(offX) > 2 || Math.abs(offY) > 2) {
          ctx.globalAlpha = baseAlpha * 0.5;
          // Color is inverted from the force direction to signify drag
          const dragColor = getDopplerColor(-zForce, 0.8); 
          drawShape(-offX, -offY, dragColor, lineThickness * 0.8);
      }

      // 2. Draw "Future" Ghost (Lead) - Positive Vector
      // Projects where the reticle is going based on current momentum
      if (Math.abs(offX) > 2 || Math.abs(offY) > 2) {
          ctx.globalAlpha = baseAlpha * 0.5;
          const leadColor = getDopplerColor(zForce, 0.8);
          drawShape(offX, offY, leadColor, lineThickness * 0.8);
      }

      // 3. Draw MAIN RETICLE (Neutral / White-Gold)
      ctx.globalAlpha = baseAlpha;
      // If highly unstable, main reticle turns yellow to warn user
      const stabilityColor = energy > 5.0 ? '#facc15' : '#ffffff';
      drawShape(0, 0, stabilityColor, lineThickness);

      // 4. Draw Convex Crosshair
      ctx.beginPath();
      ctx.lineWidth = 1;
      const curveStrength = convexityFactor * 30;
      ctx.moveTo(-r, 0); ctx.quadraticCurveTo(0, curveStrength, r, 0); // Horiz
      ctx.moveTo(0, -r * (1 - convexityFactor)); ctx.lineTo(0, r * (1 + convexityFactor)); // Vert
      ctx.stroke();

      // 5. Pulsating Ring (Lens Breathing)
      // Visualizes Z-Depth Ambiguity
      if (Math.abs(zForce) > 0.1) {
         ctx.beginPath();
         // Ring color matches Doppler direction
         ctx.strokeStyle = getDopplerColor(zForce, 0.6);
         ctx.lineWidth = 0.5 + (Math.abs(zForce) * 2); // Thicker if moving fast Z
         ctx.setLineDash([2, 4]);
         ctx.arc(0, 0, r * (0.8 + Math.abs(zForce)*0.4), 0, Math.PI*2);
         ctx.stroke();
         ctx.setLineDash([]);
      }

      ctx.restore();
  };

  useEffect(() => {
    if (!canvasRef.current || captures.length < 1) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Use a subset of frames (start, middle, end) if we have many
    const displayIndices = captures.length >= 3 
        ? [0, Math.floor(captures.length/2), captures.length-1] 
        : captures.map((_, i) => i);
        
    const displayCaptures = displayIndices.map(i => captures[i]).filter(Boolean);

    const images = displayCaptures.map(c => {
      const img = new Image();
      img.src = c.imageUri;
      return img;
    });

    Promise.all(images.map(img => new Promise(resolve => {
       if (img.complete) resolve(true);
       else img.onload = () => resolve(true);
    }))).then(() => {
      canvas.width = images[0].naturalWidth || 1920;
      canvas.height = images[0].naturalHeight || 1080;
      const w = canvas.width;
      const h = canvas.height;

      // 1. DRAW GHOST LAYERS (Background Images)
      displayCaptures.forEach((cap, idx) => {
         const img = images[idx];
         // Opacity ramps up: Oldest = Faint, Newest = Opaque
         ctx.globalAlpha = 0.3 + (idx / displayCaptures.length) * 0.7;
         
         if (cap.zoomLevel > 1.0) {
             // Handle software crop for visual consistency if mixed zoom
             const zoom = cap.zoomLevel;
             const cropW = w / zoom;
             const cropH = h / zoom;
             const cropX = (w - cropW) / 2;
             const cropY = (h - cropH) / 2;
             ctx.drawImage(img, 0, 0, w, h, cropX, cropY, cropW, cropH); 
         } else {
             ctx.drawImage(img, 0, 0);
         }
      });
      ctx.globalAlpha = 1.0;

      // 2. DRAW KINETIC VOLUME GRID
      // Grid line thickness varies by overall energy
      const avgEnergy = displayCaptures.reduce((acc, c) => acc + Math.sqrt(c.acceleration.x**2 + c.acceleration.y**2), 0) / displayCaptures.length;
      
      // If high energy, sparse grid. If low energy, dense grid.
      let gridSpacing = 100 + (avgEnergy * 50);
      
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      
      // Grid offset by the FIRST frame's accel to lock it to world-space roughly
      const offsetX = (displayCaptures[0].acceleration.x * 20) % gridSpacing;
      const offsetY = (displayCaptures[0].acceleration.y * 20) % gridSpacing;

      for(let x=offsetX; x<w; x+=gridSpacing) { ctx.moveTo(x, 0); ctx.lineTo(x, h); }
      for(let y=offsetY; y<h; y+=gridSpacing) { ctx.moveTo(0, y); ctx.lineTo(w, y); }
      ctx.stroke();

      // 3. DRAW PARALLAX EPITAXY RETICLES
      const cx = w / 2;
      const cy = h / 2;
      
      // Draw oldest first, newest on top
      displayCaptures.forEach((cap, idx) => {
          // Alpha logic: T0 is faint, T-End is bright
          const alpha = 0.4 + (idx / displayCaptures.length) * 0.6;
          drawReticle(ctx, cap, alpha, cx, cy);
      });

      // 4. DRAW TRAJECTORY
      if (trajectoryPoints && trajectoryPoints.length >= 2) {
         ctx.beginPath();
         trajectoryPoints.forEach((p, i) => {
             const px = (p.x / 1000) * w;
             const py = (p.y / 1000) * h;
             if (i===0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
         });
         ctx.strokeStyle = '#facc15'; 
         ctx.lineWidth = 6;
         ctx.lineCap = 'round';
         ctx.lineJoin = 'round';
         ctx.stroke();
      }

      // 5. DRAW HUD
      const padding = 20;
      const boxW = w / 3 - padding * 2;
      const boxH = 150;
      const yPos = h - boxH - padding;

      displayCaptures.forEach((cap, idx) => {
         const xPos = padding + idx * (w / 3);
         
         // Box BG color shifts slightly based on energy
         const energy = Math.sqrt(cap.acceleration.x**2 + cap.acceleration.y**2 + cap.acceleration.z**2);
         const bgAlpha = 0.6 + Math.min(0.3, energy * 0.1);
         
         ctx.fillStyle = `rgba(15, 23, 42, ${bgAlpha})`;
         ctx.fillRect(xPos, yPos, boxW, boxH);
         ctx.strokeStyle = idx === displayCaptures.length-1 ? '#22d3ee' : '#334155';
         ctx.lineWidth = 2;
         ctx.strokeRect(xPos, yPos, boxW, boxH);
         
         // Text
         ctx.fillStyle = '#94a3b8'; 
         ctx.font = '24px monospace';
         ctx.fillText(`FRAME ${cap.id} [${cap.lensGroup || 'STD'}]`, xPos + 10, yPos + 35);
         
         // Z-Force Indicator
         const zF = cap.acceleration.z;
         ctx.fillStyle = zF > 0.5 ? '#f87171' : zF < -0.5 ? '#60a5fa' : '#ffffff';
         ctx.font = 'bold 28px monospace';
         ctx.fillText(`Z-FORCE: ${zF.toFixed(2)}g`, xPos + 10, yPos + 75);
         
         // Rotation Rate (Yaw)
         ctx.fillStyle = '#ffffff';
         ctx.font = '18px monospace';
         ctx.fillText(`YAW RATE: ${cap.rotationRate.y.toFixed(1)}°/s`, xPos + 10, yPos + 115);
      });
      
      // Legend
      ctx.fillStyle = 'rgba(255,255,255,0.7)';
      ctx.font = '14px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(`DOPPLER KEY: RED=(+)FWD/CW | BLUE=(-)BCK/CCW | THICKNESS=ENERGY`, w - 30, 30);

    });

  }, [captures, boundingBox, trajectoryPoints]);

  const handleExport = () => {
     if (!canvasRef.current) return;
     const link = document.createElement('a');
     link.download = `kinetic_lens_analysis_${Date.now()}.jpg`;
     link.href = canvasRef.current.toDataURL('image/jpeg', 0.9);
     link.click();
  };

  return (
    <div className="space-y-2">
      <div className="relative w-full aspect-video bg-slate-900 rounded-lg overflow-hidden border border-slate-700 shadow-xl">
        <canvas ref={canvasRef} className="w-full h-full object-contain" />
        <div className="absolute top-4 left-4 bg-black/60 px-3 py-1 rounded text-xs font-mono text-cyan-400 border border-cyan-900 pointer-events-none">
           CHROMODYNAMIC DOPPLER VISUALIZATION
        </div>
      </div>
      <button 
        onClick={handleExport}
        className="text-xs flex items-center gap-2 text-slate-400 hover:text-white transition-colors"
      >
        <Download size={14} /> Export Visualization Image
      </button>
    </div>
  );
};

export default GhostView;