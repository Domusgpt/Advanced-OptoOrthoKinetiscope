import React, { useEffect, useRef, useState } from 'react';
import { RefreshCw, Zap, Image as ImageIcon, Activity, Crosshair, ZoomIn, Layers, Box, Hexagon, Video, Disc, Grid3X3, ShieldCheck, ArrowUpRight } from 'lucide-react';
import { Vector3, CapturePoint, Quaternion, CalibrationData, CaptureMode } from '../types';
import { get24CellVertices, rotate4D_Hyper, project4Dto2D, Vector4, OrthogonalFilter } from '../services/cpeMath';

interface CaptureScreenProps {
  calibration: CalibrationData | null;
  onCaptureComplete: (captures: CapturePoint[], mode: CaptureMode) => void;
  onCancel: () => void;
  isEncoding: boolean;
}

function getQuaternion(alpha: number, beta: number, gamma: number): Quaternion {
  const _x = beta ? beta * Math.PI / 180 : 0;
  const _y = gamma ? gamma * Math.PI / 180 : 0;
  const _z = alpha ? alpha * Math.PI / 180 : 0;
  const cX = Math.cos(_x / 2); const cY = Math.cos(_y / 2); const cZ = Math.cos(_z / 2);
  const sX = Math.sin(_x / 2); const sY = Math.sin(_y / 2); const sZ = Math.sin(_z / 2);
  return {
    w: cX * cY * cZ - sX * sY * sZ,
    x: sX * cY * cZ - cX * sY * sZ,
    y: cX * sY * cZ + sX * cY * sZ,
    z: cX * cY * sZ + sX * sY * cZ,
  };
}

const CaptureScreen: React.FC<CaptureScreenProps> = ({ calibration, onCaptureComplete, onCancel, isEncoding }) => {
  const videoRef = useRef<HTMLVideoElement>(null);
  const overlayRef = useRef<HTMLCanvasElement>(null);
  
  const [captureMode, setCaptureMode] = useState<CaptureMode>('TRAJECTORY');
  const [useMultiScale, setUseMultiScale] = useState(false);
  const [isVideoRecording, setIsVideoRecording] = useState(false); 
  const [streamReady, setStreamReady] = useState(false);
  
  const captureBuffer = useRef<CapturePoint[]>([]); 

  const [stream, setStream] = useState<MediaStream | null>(null);
  const [track, setTrack] = useState<MediaStreamTrack | null>(null);
  const [isCountingDown, setIsCountingDown] = useState(false);
  const [count, setCount] = useState(3);
  const [flash, setFlash] = useState(false);
  const [hasTorch, setHasTorch] = useState(false);
  const [kineticEnergy, setKineticEnergy] = useState(0); 
  const [stabilityScore, setStabilityScore] = useState(100);
  const [vectorEfficiency, setVectorEfficiency] = useState(100); 
  
  const [capturedCount, setCapturedCount] = useState(0);
  const [burstTotal, setBurstTotal] = useState(3);
  const [activeLens, setActiveLens] = useState<'1x' | '2x'>('1x');
  
  // SENSORS & FILTERING
  const orthogonalFilter = useRef(new OrthogonalFilter()); 
  const currentAccel = useRef<Vector3>({ x: 0, y: 0, z: 0 }); 
  const rawAccel = useRef<Vector3>({ x: 0, y: 0, z: 0 }); 
  const prevAccel = useRef<Vector3>({ x: 0, y: 0, z: 0 }); 
  const currentGyro = useRef<Vector3>({ x: 0, y: 0, z: 0 });
  const currentOrientation = useRef<Quaternion>({ w: 1, x: 0, y: 0, z: 0 });

  const requestRef = useRef<number>(0);

  // --- INIT SENSORS ---
  useEffect(() => {
    const startCamera = async () => {
      try {
        const constraints: MediaStreamConstraints = {
          video: { 
            facingMode: 'environment', 
            width: { ideal: 1920 }, 
            height: { ideal: 1080 }, 
            frameRate: { ideal: 60 },
            zoom: 1.0 
          } as any,
          audio: false,
        };

        const mediaStream = await navigator.mediaDevices.getUserMedia(constraints);
        
        const videoTrack = mediaStream.getVideoTracks()[0];
        setTrack(videoTrack);
        setStream(mediaStream);
        
        const capabilities = videoTrack.getCapabilities();
        if ((capabilities as any).torch) setHasTorch(true);

        if (videoRef.current) {
            videoRef.current.srcObject = mediaStream;
            videoRef.current.onloadedmetadata = () => {
              videoRef.current?.play().then(() => setStreamReady(true)).catch(console.error);
            };
        }
      } catch (err) {
        console.error("Camera access denied", err);
      }
    };

    const handleMotion = (event: DeviceMotionEvent) => {
      if (event.accelerationIncludingGravity) {
        // Shift history
        prevAccel.current = { ...currentAccel.current };
        
        const raw = {
          x: event.accelerationIncludingGravity.x || 0,
          y: event.accelerationIncludingGravity.y || 0,
          z: event.accelerationIncludingGravity.z || 0,
        };
        rawAccel.current = raw;

        // ORTHOGONAL DECOMPOSITION FILTER
        const { clean, orthogonalNoise } = orthogonalFilter.current.update(raw);
        
        currentAccel.current = clean;
        
        // Calculate Efficiency for UI
        const eff = orthogonalFilter.current.getEfficiency() * 100;
        
        // Haptic Feedback for poor form (if available)
        if (eff < 40 && navigator.vibrate) {
            navigator.vibrate(5); // Tiny tick to warn user
        }

        setVectorEfficiency(prev => prev + (eff - prev) * 0.1); 
      }
      if (event.rotationRate) {
        currentGyro.current = {
          x: event.rotationRate.alpha || 0, 
          y: event.rotationRate.beta || 0, 
          z: event.rotationRate.gamma || 0
        };
      }
    };

    const handleOrientation = (event: DeviceOrientationEvent) => {
       const alpha = event.alpha || 0;
       const beta = event.beta || 0;
       const gamma = event.gamma || 0;
       currentOrientation.current = getQuaternion(alpha, beta, gamma);
    };

    startCamera();
    window.addEventListener('devicemotion', handleMotion);
    window.addEventListener('deviceorientation', handleOrientation);

    // --- CPE LIVE HUD RENDER LOOP ---
    const animateHUD = () => {
      const cvs = overlayRef.current;
      if (!cvs || !videoRef.current) {
          requestRef.current = requestAnimationFrame(animateHUD);
          return;
      }
      
      const ctx = cvs.getContext('2d');
      if (!ctx) return;

      if (cvs.width !== cvs.clientWidth) {
         cvs.width = cvs.clientWidth;
         cvs.height = cvs.clientHeight;
      }
      const w = cvs.width;
      const h = cvs.height;
      const cx = w/2;
      const cy = h/2;

      ctx.clearRect(0, 0, w, h);

      // 1. Calculate Stability & Energy (Using Cleaned Accel)
      const accMag = Math.sqrt(currentAccel.current.x**2 + currentAccel.current.y**2 + currentAccel.current.z**2);
      const rotMag = Math.sqrt(currentGyro.current.x**2 + currentGyro.current.y**2 + currentGyro.current.z**2) / 100;
      const energy = accMag + rotMag;
      setKineticEnergy(energy);
      
      setStabilityScore(prev => {
          const target = Math.max(0, 100 - (energy * 50));
          return prev + (target - prev) * 0.1;
      });

      // 2. VISUALIZE "THE RAIL" (Principal Axis)
      // This is the core visual feedback for OptoOrtho-Gimbal
      const rail = orthogonalFilter.current.getPrincipalAxis();
      
      // Project the 3D rail vector onto 2D screen space
      // We amplify it to stretch across the screen
      const railLen = 300;
      const railX = rail.x * railLen;
      const railY = -rail.y * railLen; // Invert Y for screen coords
      
      ctx.save();
      ctx.translate(cx, cy);
      
      // Draw The Infinite Rail (Intended Path)
      ctx.beginPath();
      ctx.moveTo(-railX, -railY);
      ctx.lineTo(railX, railY);
      ctx.lineWidth = 2;
      // If efficiency is high, rail is Cyan (Locked). If low, it fades to slate.
      const eff = orthogonalFilter.current.getEfficiency();
      const railColor = eff > 0.8 ? `rgba(34, 211, 238, ${eff})` : `rgba(148, 163, 184, ${eff})`;
      ctx.strokeStyle = railColor;
      ctx.setLineDash([5, 5]);
      ctx.stroke();

      // Draw The Current Force Vector (Actual Motion)
      const forceX = currentAccel.current.x * 50; 
      const forceY = -currentAccel.current.y * 50;
      
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(forceX, forceY);
      ctx.lineWidth = 4;
      ctx.strokeStyle = eff > 0.7 ? '#22d3ee' : '#f87171'; // Cyan vs Red
      ctx.setLineDash([]);
      ctx.stroke();

      // If orthogonal noise is high, draw "Jitter Spikes" perpendicular to rail
      if (eff < 0.6) {
          const jitterMag = (1.0 - eff) * 40;
          ctx.beginPath();
          // Draw a rough perpendicular line
          ctx.moveTo(forceX - jitterMag, forceY + jitterMag);
          ctx.lineTo(forceX + jitterMag, forceY - jitterMag);
          ctx.lineWidth = 2;
          ctx.strokeStyle = '#ef4444';
          ctx.stroke();
      }

      ctx.restore();


      // 3. Draw 4D Hyper-Lattice (Live)
      const vertices = get24CellVertices();
      const scale = captureMode === 'VIDEO_STREAM' ? 200 : 150; 
      
      const projected = vertices.map(v => {
          const rotated = rotate4D_Hyper(v, currentOrientation.current, currentAccel.current);
          return project4Dto2D(rotated, w, h, scale, 2.5);
      });

      ctx.beginPath();
      ctx.strokeStyle = energy > 1.5 ? 'rgba(239, 68, 68, 0.6)' : 'rgba(34, 211, 238, 0.4)'; 
      ctx.lineWidth = 1;
      
      for (let i = 0; i < projected.length; i++) {
        for (let j = i + 1; j < projected.length; j++) {
            const p1 = projected[i]; const p2 = projected[j];
            if (p1.depth === -1 || p2.depth === -1) continue;
            const dx = p1.x - p2.x; const dy = p1.y - p2.y;
            const dist = Math.sqrt(dx*dx + dy*dy);
            if (dist < scale * 1.5 && dist > scale * 0.2) {
                 ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y);
            }
        }
      }
      ctx.stroke();

      projected.forEach(p => {
          if (p.depth !== -1) {
              const r = 2 + (p.wFactor * 2);
              ctx.beginPath();
              ctx.arc(p.x, p.y, r, 0, Math.PI*2);
              ctx.fillStyle = energy > 1.5 ? '#fca5a5' : '#a5f3fc';
              ctx.fill();
          }
      });

      // 4. Matrix Burst Grid Overlay
      if (captureMode === 'MATRIX_BURST') {
          ctx.strokeStyle = 'rgba(250, 204, 21, 0.5)'; // Yellow
          ctx.lineWidth = 1;
          const cw = w/3; const ch = h/3;
          // Draw 3x3 Grid
          ctx.beginPath();
          ctx.moveTo(cw, 0); ctx.lineTo(cw, h);
          ctx.moveTo(cw*2, 0); ctx.lineTo(cw*2, h);
          ctx.moveTo(0, ch); ctx.lineTo(w, ch);
          ctx.moveTo(0, ch*2); ctx.lineTo(w, ch*2);
          ctx.stroke();
          
          ctx.fillStyle = '#facc15';
          ctx.font = '10px monospace';
          ctx.fillText("HYPER-SPECTRAL MATRIX READY", 10, h - 20);
      }

      requestRef.current = requestAnimationFrame(animateHUD);
    };
    requestRef.current = requestAnimationFrame(animateHUD);

    return () => {
      if (stream) stream.getTracks().forEach(track => track.stop());
      window.removeEventListener('devicemotion', handleMotion);
      window.removeEventListener('deviceorientation', handleOrientation);
      cancelAnimationFrame(requestRef.current);
    };
  }, [useMultiScale, captureMode, activeLens]);

  const setTorchState = async (on: boolean) => {
    if (track && hasTorch) {
      try { await track.applyConstraints({ advanced: [{ torch: on } as any] }); } catch (e) {}
    }
  };

  const setZoomState = async (zoom: number) => {
      if (track && (track.getCapabilities() as any).zoom) {
          try { 
            await track.applyConstraints({ advanced: [{ zoom: zoom } as any] }); 
            setActiveLens(zoom >= 2 ? '2x' : '1x');
          } catch(e) {}
      }
  };
  
  const setExposureBias = async (bias: number) => {
     if (track) {
         const caps = track.getCapabilities() as any;
         if (caps.exposureCompensation) {
             try { await track.applyConstraints({ advanced: [{ exposureCompensation: bias } as any] }); } catch(e) {}
         }
     }
  };

  // Helper: Calculate instantaneous Moire Variance (Jerk magnitude)
  const getMoireVariance = () => {
      const dx = currentAccel.current.x - prevAccel.current.x;
      const dy = currentAccel.current.y - prevAccel.current.y;
      const dz = currentAccel.current.z - prevAccel.current.z;
      return Math.sqrt(dx*dx + dy*dy + dz*dz); // Jerk Magnitude (m/s^3)
  };

  const grabFrame = async (overrideZoom: number, phase: 'RED' | 'GREEN' | 'BLUE', interval: number, metadata?: any) => {
    const video = videoRef.current;
    
    // Safety check: The image source is not usable if width is 0 or readyState is not adequate
    if (!video || video.readyState < 2 || video.videoWidth === 0 || video.videoHeight === 0) {
       console.warn("Video stream not ready for frame capture. Skipping.");
       return;
    }

    try {
      const bitmap = await createImageBitmap(video);
      const canvas = document.createElement('canvas');
      canvas.width = bitmap.width;
      canvas.height = bitmap.height;
      const ctx = canvas.getContext('2d');
      if(ctx) {
          ctx.drawImage(bitmap, 0, 0);
      }
      const uri = canvas.toDataURL('image/jpeg', 0.85);

      const t0 = captureBuffer.current.length > 0 ? captureBuffer.current[0].timestamp : Date.now();
      const tilt = Math.atan2(rawAccel.current.y, rawAccel.current.z) * (180 / Math.PI); // Use RAW for tilt (requires gravity)

      const point: CapturePoint = {
        id: captureBuffer.current.length,
        timestamp: Date.now(),
        relativeTime: Date.now() - t0,
        imageUri: uri,
        acceleration: { ...currentAccel.current }, // Filtered Linear Accel
        rotationRate: { ...currentGyro.current },
        orientation: { ...currentOrientation.current },
        tiltAngle: tilt,
        zoomLevel: overrideZoom,
        isoEstimate: calibration?.isoNoiseFloor || 100,
        spectralPhase: phase,
        shutterInterval: interval,
        ...metadata,
        // Capture the efficiency at the moment of shutter
        vectorEfficiency: orthogonalFilter.current.getEfficiency() 
      };

      captureBuffer.current.push(point);
      setCapturedCount(prev => prev + 1);
      setFlash(true);
      setTimeout(() => setFlash(false), 50);
    } catch (e) {
      console.error("Frame grab failed", e);
    }
  };

  const startSequence = async () => {
    if (!streamReady && !videoRef.current) {
        console.warn("Stream not ready");
        return;
    }

    if (captureMode === 'VIDEO_STREAM') {
        setIsVideoRecording(!isVideoRecording);
        return;
    }

    captureBuffer.current = [];
    setCapturedCount(0);
    
    // Set Total Frames based on Mode
    if (captureMode === 'MATRIX_BURST') setBurstTotal(9);
    else setBurstTotal(3);

    // Initial State
    await setTorchState(false); 
    await setZoomState(1.0); 

    setIsCountingDown(true);
    let counter = 3;
    setCount(counter);
    const timer = setInterval(() => {
      counter--;
      setCount(counter);
      if (counter === 0) {
        clearInterval(timer);
        setIsCountingDown(false);
        if (captureMode === 'MATRIX_BURST') executeMatrixBurst();
        else executeKineticBurst();
      }
    }, 1000);
  };

  // --- STANDARD 3-FRAME BURST ---
  const executeKineticBurst = async () => {
    const capturePoints = 3;
    for(let i=0; i<capturePoints; i++) {
        const velocity = Math.sqrt(currentAccel.current.x**2 + currentAccel.current.y**2 + currentAccel.current.z**2);
        const dynamicDelay = Math.max(20, 100 - (velocity * 5)); 
        
        let currentShotZoom = 1.0;
        if (useMultiScale && i === 1) currentShotZoom = 2.0;

        await setZoomState(currentShotZoom);
        if (useMultiScale && i === 1) await new Promise(r => setTimeout(r, 150));
        else if (i > 0) await new Promise(r => setTimeout(r, dynamicDelay));

        const phase = i === 0 ? 'RED' : i === 1 ? 'GREEN' : 'BLUE';
        await grabFrame(currentShotZoom, phase, dynamicDelay);
    }
    await setTorchState(false);
    await setZoomState(1.0); 
    
    // Only complete if we actually captured frames
    if (captureBuffer.current.length > 0) {
        onCaptureComplete(captureBuffer.current, captureMode);
    } else {
        console.error("Burst capture failed to obtain any frames");
        onCancel();
    }
  };

  // --- HYPER 9-FRAME MATRIX BURST ---
  const executeMatrixBurst = async () => {
      // (Same logic as before, just refined timing)
      const baseLatency = calibration?.sensorReadoutLatency || 33;
      const getRationalDelay = (zoom: number) => {
         const jerk = getMoireVariance(); 
         const intensity = jerk * zoom; 
         const adaptive = Math.max(baseLatency, 200 * Math.exp(-0.5 * intensity));
         return { delay: Math.round(adaptive), jerk };
      };

      // PHASE 1: WIDE
      await setZoomState(1.0); await setExposureBias(0);
      for(let i=0; i<3; i++) {
          const startT = performance.now();
          const { delay, jerk } = getRationalDelay(1.0);
          await new Promise(r => setTimeout(r, delay));
          const endT = performance.now();
          await grabFrame(1.0, i===0?'RED':i===1?'GREEN':'BLUE', Math.round(endT - startT), { matrixPosition: { row: 0, col: i }, lensGroup: 'WIDE', exposureBias: 0, moireVariance: jerk });
      }

      // PHASE 2: TELE
      await setZoomState(2.0); await new Promise(r => setTimeout(r, 200)); 
      for(let i=0; i<3; i++) {
        const startT = performance.now();
        const { delay, jerk } = getRationalDelay(2.0);
        await new Promise(r => setTimeout(r, delay));
        const endT = performance.now();
        await grabFrame(2.0, i===0?'RED':i===1?'GREEN':'BLUE', Math.round(endT - startT), { matrixPosition: { row: 1, col: i }, lensGroup: 'TELE', exposureBias: 0, moireVariance: jerk });
      }

      // PHASE 3: ACTIVE
      await setZoomState(1.0); await setExposureBias(-1.5); await setTorchState(true); await new Promise(r => setTimeout(r, 150)); 
      for(let i=0; i<3; i++) {
        const startT = performance.now();
        const { delay, jerk } = getRationalDelay(1.0);
        await new Promise(r => setTimeout(r, delay));
        const endT = performance.now();
        await grabFrame(1.0, i===0?'RED':i===1?'GREEN':'BLUE', Math.round(endT - startT), { matrixPosition: { row: 2, col: i }, lensGroup: 'ACTIVE', exposureBias: -1.5, moireVariance: jerk });
      }

      await setTorchState(false); await setExposureBias(0); await setZoomState(1.0);
      
      if (captureBuffer.current.length > 0) {
          onCaptureComplete(captureBuffer.current, captureMode);
      } else {
          onCancel();
      }
  };

  if (isEncoding) {
     return (
       <div className="w-full h-full bg-slate-900 flex flex-col items-center justify-center">
         <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-cyan-400 mb-4"></div>
         <h2 className="text-white font-mono">CONSTRUCTING TEMPORAL TRUSS...</h2>
         <p className="text-slate-500 text-xs mt-2">Integrating Multi-Lens Parallax</p>
       </div>
     );
  }

  return (
    <div className="relative w-full h-full flex flex-col landscape:flex-row bg-black">
      <div className="flex-1 relative overflow-hidden bg-black">
        <video ref={videoRef} autoPlay playsInline className="w-full h-full object-cover"/>
        <canvas ref={overlayRef} className="absolute inset-0 w-full h-full pointer-events-none" />
        
        {/* HUD Elements */}
        <div className="absolute top-4 right-4 flex flex-col items-end z-20 space-y-3">
           {/* Stability */}
           <div className="flex flex-col items-end">
             <div className="flex items-center gap-2 mb-1">
               <Hexagon size={16} className={stabilityScore > 80 ? 'text-cyan-400' : 'text-red-400'} />
               <span className="font-mono text-xs text-white">LATTICE INTEGRITY</span>
             </div>
             <div className="w-32 h-2 bg-slate-800 rounded-full overflow-hidden">
               <div 
                  className={`h-full transition-all duration-300 ${stabilityScore > 80 ? 'bg-cyan-500' : 'bg-red-500'}`}
                  style={{ width: `${stabilityScore}%` }}
               ></div>
             </div>
           </div>

           {/* Orthogonal Efficiency (New) */}
           <div className="flex flex-col items-end">
             <div className="flex items-center gap-2 mb-1">
               <ShieldCheck size={16} className={vectorEfficiency > 80 ? 'text-green-400' : 'text-yellow-400'} />
               <span className="font-mono text-xs text-white">VECTOR EFFICIENCY</span>
             </div>
             <div className="w-32 h-2 bg-slate-800 rounded-full overflow-hidden">
               <div 
                  className={`h-full transition-all duration-300 ${vectorEfficiency > 80 ? 'bg-green-500' : 'bg-yellow-500'}`}
                  style={{ width: `${vectorEfficiency}%` }}
               ></div>
             </div>
             <span className="text-[9px] text-slate-500 font-mono mt-1">ORTHOGONAL REJECTION: {(100-vectorEfficiency).toFixed(0)}%</span>
           </div>
        </div>

        {/* Capture Burst Indicator */}
        <div className="absolute top-4 left-1/2 -translate-x-1/2 flex gap-1 z-20 flex-wrap w-64 justify-center">
           {Array.from({ length: burstTotal }).map((_, idx) => (
             <div key={idx} className={`w-8 h-6 border border-white/30 rounded flex items-center justify-center bg-black/40 transition-all duration-200 ${capturedCount > idx ? 'bg-cyan-500/50 border-cyan-400' : ''}`}>
               {capturedCount > idx && (
                 <span className="text-[8px] font-mono text-white">
                   {idx < 3 ? '1x' : idx < 6 ? '2x' : 'Act'}
                 </span>
               )}
             </div>
           ))}
        </div>
        
        {/* Helper Message for "The Rail" */}
        <div className="absolute bottom-4 left-1/2 -translate-x-1/2 z-20 text-center pointer-events-none opacity-80">
            {vectorEfficiency > 80 ? (
                <div className="text-cyan-400 text-[10px] font-mono animate-pulse">RAIL LOCKED - TRAJECTORY CLEAN</div>
            ) : (
                <div className="text-red-400 text-[10px] font-mono bg-red-900/40 px-2 py-1 rounded">ALIGN MOTION TO DOTTED RAIL</div>
            )}
        </div>

        {flash && <div className="absolute inset-0 bg-white z-50 transition-opacity duration-75 ease-out" />}
        {isCountingDown && (
          <div className="absolute inset-0 bg-black/40 flex items-center justify-center z-40 backdrop-blur-sm">
             <span className="text-[10rem] font-black text-white italic animate-ping">{count}</span>
          </div>
        )}
      </div>

      {/* Control Panel */}
      <div className="h-32 landscape:h-full landscape:w-32 bg-slate-900 flex landscape:flex-col items-center justify-around px-8 landscape:py-8 landscape:px-0 z-10 border-t landscape:border-t-0 landscape:border-l border-slate-800">
        <button onClick={onCancel} className="p-4 rounded-full bg-slate-800 text-slate-400 hover:bg-slate-700 transition-colors">
          <RefreshCw size={24} />
        </button>

        <div className="flex flex-col gap-2 items-center">
            <button 
                onClick={() => setCaptureMode('TRAJECTORY')}
                className={`text-[10px] w-16 py-1 rounded ${captureMode==='TRAJECTORY' ? 'bg-cyan-600 text-white' : 'bg-slate-800 text-slate-400'}`}
            >TRAJ</button>
            <button 
                onClick={() => setCaptureMode('MATRIX_BURST')}
                className={`text-[10px] w-16 py-1 rounded ${captureMode==='MATRIX_BURST' ? 'bg-yellow-600 text-white' : 'bg-slate-800 text-slate-400'}`}
            >MATRIX</button>
            
            <button 
                onClick={() => setUseMultiScale(!useMultiScale)}
                disabled={captureMode === 'MATRIX_BURST'}
                className={`flex flex-col items-center justify-center mt-2 w-12 h-12 rounded-lg border ${useMultiScale ? 'bg-green-900/30 border-green-500 text-green-400' : 'bg-slate-800 border-slate-700 text-slate-500'} ${captureMode === 'MATRIX_BURST' ? 'opacity-30' : ''}`}
            >
                <Layers size={16} />
                <span className="text-[9px] font-bold mt-1">MULTI</span>
            </button>
        </div>

        <button 
            onClick={startSequence}
            disabled={isCountingDown || stabilityScore < 30 || !streamReady}
            className={`w-20 h-20 rounded-full border-4 flex items-center justify-center relative group shadow-lg transition-all ${stabilityScore < 30 || !streamReady ? 'border-red-900 bg-red-900/20 grayscale' : 'border-slate-200 shadow-cyan-900/50 hover:scale-105'}`}
        >
             {captureMode === 'MATRIX_BURST' ? (
                <Grid3X3 className={`transition-all duration-200 ${kineticEnergy > 1 ? 'text-yellow-300' : 'text-yellow-100'}`} size={32} />
             ) : (
                <Box className={`transition-all duration-200 ${kineticEnergy > 1 ? 'scale-125 text-red-300' : 'text-cyan-200'}`} size={32} />
             )}
        </button>

        <div className="flex flex-col gap-2 items-center">
             <button 
                onClick={() => setCaptureMode('VIDEO_STREAM')}
                className={`text-[10px] w-16 py-1 rounded ${captureMode==='VIDEO_STREAM' ? 'bg-red-600 text-white' : 'bg-slate-800 text-slate-400'}`}
            >VIDEO</button>
        </div>
      </div>
    </div>
  );
};

export default CaptureScreen;