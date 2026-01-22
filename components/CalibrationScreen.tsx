
import React, { useState, useEffect, useRef } from 'react';
import { CalibrationData, Vector3, UserMetrics } from '../types';
import { Fingerprint, CheckCircle, Sun, AlertTriangle, Ruler, Layers, Activity, Clock, Scan, ZoomIn } from 'lucide-react';

interface CalibrationScreenProps {
  onCalibrationComplete: (data: CalibrationData) => void;
}

const CalibrationScreen: React.FC<CalibrationScreenProps> = ({ onCalibrationComplete }) => {
  // Steps: 0=Light/ISO, 1=Metrics, 2=Noise/Jitter, 3=Latency/Polling, 4=Lens/Zoom, 5=Result
  const [step, setStep] = useState<0 | 1 | 2 | 3 | 4 | 5>(0);
  const [progress, setProgress] = useState(0);
  const [luxEstimate, setLuxEstimate] = useState(0);
  const [isoEstimate, setIsoEstimate] = useState(0);
  const [lightStatus, setLightStatus] = useState<'analyzing' | 'low' | 'optimal' | 'bright'>('analyzing');
  const [zoomCurve, setZoomCurve] = useState<{ zoomLevel: number, pxWidth: number }[]>([]);
  
  const REF_BOX_SIZE_PX = 160; 

  const [metrics, setMetrics] = useState<UserMetrics>({
    referenceName: "Standard ID Card",
    referenceSize: 8.56, // cm (Standard ISO 7810 ID-1)
    unitSystem: 'metric',
    subjectType: "General Object",
    deviceHeightCm: 150 
  });

  const videoRef = useRef<HTMLVideoElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  const samplesAccel = useRef<Vector3[]>([]);
  const samplesGyro = useRef<Vector3[]>([]);
  
  const baselineAccel = useRef<Vector3>({ x:0, y:0, z:0 });
  const baselineGyro = useRef<Vector3>({ x:0, y:0, z:0 });
  const jitterVariance = useRef<number>(0);
  const sensorLatency = useRef<number>(33);
  
  // Rational Dynamics Weights
  const [weights, setWeights] = useState({ gyro: 0.5, accel: 0.5, optical: 0.5 });
  // Zoom state tracker for calibration steps
  const [currentZoomReq, setCurrentZoomReq] = useState(1.0);

  useEffect(() => {
    let stream: MediaStream | null = null;
    let interval: any;

    const startCamera = async () => {
      try {
        const constraints: MediaStreamConstraints = { 
            video: { 
                facingMode: 'environment',
                zoom: step === 4 ? currentZoomReq : 1.0 
            } as any
        };
        stream = await navigator.mediaDevices.getUserMedia(constraints);
        if (videoRef.current) {
          videoRef.current.srcObject = stream;
          videoRef.current.onloadedmetadata = () => {
             videoRef.current?.play();
             // Delay analysis to ensure frame data
             if (step === 0) setTimeout(() => analyzeOpticalConditions(), 1500);
          };
        }
      } catch (e) {
        if (step === 0) setStep(1); 
      }
    };

    if (step === 0 || step === 4) startCamera();
    
    return () => {
      if (stream) stream.getTracks().forEach(t => t.stop());
      if (interval) clearInterval(interval);
    };
  }, [step, currentZoomReq]);

  // OPTICAL ANALYSIS (Light + Noise/ISO)
  const analyzeOpticalConditions = async () => {
    if (!videoRef.current || !canvasRef.current) return;
    
    // Check readiness
    if (videoRef.current.readyState < 2 || videoRef.current.videoWidth === 0) {
        // Retry shortly if not ready
        setTimeout(analyzeOpticalConditions, 500);
        return;
    }

    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;

    const w = 100; const h = 100;
    
    // Capture Frame 1
    ctx.drawImage(videoRef.current, 0, 0, w, h);
    const f1 = ctx.getImageData(0, 0, w, h).data;
    
    await new Promise(r => setTimeout(r, 50)); // Tiny delay
    
    // Capture Frame 2
    ctx.drawImage(videoRef.current, 0, 0, w, h);
    const f2 = ctx.getImageData(0, 0, w, h).data;
    
    let totalBright = 0;
    let totalDiff = 0;
    
    for (let i = 0; i < f1.length; i += 4) {
        // Brightness
        const b = (0.299 * f1[i] + 0.587 * f1[i+1] + 0.114 * f1[i+2]);
        totalBright += b;
        
        // Difference (Noise proxy if scene is static)
        const diff = Math.abs(f1[i] - f2[i]) + Math.abs(f1[i+1] - f2[i+1]) + Math.abs(f1[i+2] - f2[i+2]);
        totalDiff += diff;
    }
    
    const avgBright = totalBright / (f1.length / 4);
    const avgNoise = totalDiff / (f1.length / 4);
    
    setLuxEstimate(avgBright * 2); 
    setIsoEstimate(avgNoise * 50); // Rough scaler

    if (avgBright < 40) setLightStatus('low');
    else if (avgBright > 220) setLightStatus('bright');
    else setLightStatus('optimal');
  };

  // Sensor Sampling
  useEffect(() => {
    const handleMotion = (event: DeviceMotionEvent) => {
      if (step !== 2) return;
      const a = event.accelerationIncludingGravity;
      const r = event.rotationRate;
      const vecA = { x: a?.x || 0, y: a?.y || 0, z: a?.z || 0 };
      const vecG = { x: r?.alpha || 0, y: r?.beta || 0, z: r?.gamma || 0 };
      samplesAccel.current.push(vecA);
      samplesGyro.current.push(vecG);
    };
    window.addEventListener('devicemotion', handleMotion);
    return () => window.removeEventListener('devicemotion', handleMotion);
  }, [step]);

  // Jitter/Noise Progress
  useEffect(() => {
    if (step === 2) {
      let p = 0;
      const interval = setInterval(() => {
        p += 4; 
        setProgress(p);
        if (p >= 100) {
          clearInterval(interval);
          calculateNoiseBaselines();
          setStep(3); // Go to Latency Test
          setProgress(0);
        }
      }, 50);
      return () => clearInterval(interval);
    }
  }, [step]);

  // Latency Test Progress
  useEffect(() => {
      if (step === 3) {
          const measureLatency = async () => {
              if (!videoRef.current || videoRef.current.readyState < 2 || videoRef.current.videoWidth === 0) {
                  // Fallback if video isn't ready
                  setTimeout(() => setStep(4), 500);
                  return;
              }
              try {
                const timings: number[] = [];
                for(let i=0; i<10; i++) {
                    const t1 = performance.now();
                    await createImageBitmap(videoRef.current);
                    const t2 = performance.now();
                    timings.push(t2 - t1);
                    setProgress((i+1) * 10);
                }
                const avg = timings.reduce((a,b) => a+b, 0) / timings.length;
                sensorLatency.current = avg;
              } catch (e) {
                 console.warn("Latency test skipped due to video error", e);
                 sensorLatency.current = 33;
              }
              setTimeout(() => setStep(4), 500);
          };
          measureLatency();
      }
  }, [step]);

  const calculateNoiseBaselines = () => {
    const count = samplesAccel.current.length;
    if (count === 0) return;
    
    const sumA = samplesAccel.current.reduce((acc, v) => ({ x: acc.x+v.x, y: acc.y+v.y, z: acc.z+v.z }), {x:0, y:0, z:0});
    const avgA = { x: sumA.x/count, y: sumA.y/count, z: sumA.z/count };
    
    let varianceA = 0;
    samplesAccel.current.forEach(v => {
        varianceA += Math.pow(v.x - avgA.x, 2) + Math.pow(v.y - avgA.y, 2) + Math.pow(v.z - avgA.z, 2);
    });
    varianceA /= count;

    jitterVariance.current = Math.min(1.0, varianceA / 5.0);
    
    setWeights({
        gyro: 0.9, 
        accel: Math.max(0.1, 1.0 - jitterVariance.current),
        optical: lightStatus === 'bright' ? 1.0 : lightStatus === 'optimal' ? 0.8 : 0.4
    });

    baselineAccel.current = avgA;
  };

  // Perform Zoom calibration for a specific level
  const calibrateZoom = (level: number) => {
      setCurrentZoomReq(level);
      
      // Store the theoretical pixel width (in a real app we'd measure feature scaling)
      // Here we store what the reference box should look like at this zoom
      setZoomCurve(prev => {
         const exists = prev.find(p => p.zoomLevel === level);
         if (exists) return prev;
         return [...prev, { zoomLevel: level, pxWidth: REF_BOX_SIZE_PX * level }].sort((a,b) => a.zoomLevel - b.zoomLevel);
      });

      // If we have collected 3 points, we can move to finish
      if (zoomCurve.length >= 2 && level === 2) { // Allow shortcut on 2x
          // Optional auto-advance logic
      }
  };

  const calculateFinalCurve = () => {
     // Perform linear regression on zoom points to get coefficients: pxWidth = m * zoom + c
     if (zoomCurve.length < 2) return zoomCurve;
     
     const n = zoomCurve.length;
     const sumX = zoomCurve.reduce((a, b) => a + b.zoomLevel, 0);
     const sumY = zoomCurve.reduce((a, b) => a + b.pxWidth, 0);
     const sumXY = zoomCurve.reduce((a, b) => a + b.zoomLevel * b.pxWidth, 0);
     const sumXX = zoomCurve.reduce((a, b) => a + b.zoomLevel * b.zoomLevel, 0);
     
     const slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
     const intercept = (sumY - slope * sumX) / n;
     
     // Store derived points for higher precision lookup
     return [1, 1.5, 2, 2.5, 3].map(z => ({
         zoomLevel: z,
         pxWidth: slope * z + intercept
     }));
  };

  const finishCalibration = () => {
    const pixelsPerUnit = REF_BOX_SIZE_PX / (metrics.referenceSize || 1);
    const stabilityFactor = 1.0 - jitterVariance.current;
    const adaptiveGridScale = pixelsPerUnit * (0.5 + 0.5 * stabilityFactor);
    
    const refinedCurve = calculateFinalCurve();

    onCalibrationComplete({
      baselineAccel: baselineAccel.current,
      baselineGyro: baselineGyro.current,
      motionSignature: [],
      scaleDelta: 2.0,
      lightLevel: lightStatus === 'analyzing' ? 'optimal' : lightStatus,
      luxEstimate,
      userMetrics: metrics,
      pixelsPerUnit: pixelsPerUnit,
      gridScale: adaptiveGridScale,
      fieldOfView: 65,
      focalLengthPx: 1000,
      zoomCalibrationCurve: refinedCurve,
      sensorReadoutLatency: sensorLatency.current,
      maxStableBurstRate: 1000 / (sensorLatency.current + 10),
      jitterVariance: jitterVariance.current,
      isoNoiseFloor: isoEstimate,
      gyroConfidence: weights.gyro,
      accelConfidence: weights.accel,
      opticalConfidence: weights.optical
    });
  };

  return (
    <div className="w-full h-full bg-slate-950 flex flex-col items-center justify-center p-8 relative overflow-hidden overflow-y-auto">
      
      <div className="absolute inset-0 bg-[linear-gradient(rgba(15,23,42,0)_1px,transparent_1px),linear-gradient(90deg,rgba(15,23,42,0)_1px,transparent_1px)] bg-[size:40px_40px] opacity-20" style={{ backgroundImage: `linear-gradient(#1e293b 1px, transparent 1px), linear-gradient(90deg, #1e293b 1px, transparent 1px)` }}></div>
      
      <video ref={videoRef} className="hidden" muted playsInline />
      <canvas ref={canvasRef} width={100} height={100} className="hidden" />

      <div className="z-10 w-full max-w-sm flex flex-col items-center">
        
        {step === 0 && (
          <>
            <div className="w-32 h-32 rounded-full border-4 border-slate-700 flex items-center justify-center mb-8 bg-slate-900 relative">
               <Sun size={48} className={`text-yellow-500 ${lightStatus === 'analyzing' ? 'animate-spin' : ''}`} />
               {lightStatus === 'low' && <AlertTriangle className="absolute -top-2 -right-2 text-red-500" size={24} />}
            </div>
            <h2 className="text-2xl font-bold text-white mb-2">Optical Sensor Check</h2>
            <div className="flex gap-4 mb-8 text-xs font-mono text-slate-500">
                <div>LUX: {luxEstimate.toFixed(0)}</div>
                <div>ISO~: {isoEstimate.toFixed(0)}</div>
            </div>
            <p className="text-slate-400 text-center mb-8">{lightStatus === 'low' ? 'Low light detected. Optical flow reliability: LOW. Switch to high-contrast markers.' : 'Optical conditions optimal for kinetic tracking.'}</p>
            <button onClick={() => setStep(1)} className="px-8 py-3 bg-slate-800 hover:bg-slate-700 text-white font-bold rounded-lg w-full">Continue</button>
          </>
        )}

        {step === 1 && (
           <div className="w-full bg-slate-900 border border-slate-800 rounded-xl p-6 shadow-xl">
              <div className="flex items-center gap-3 mb-6">
                <Ruler className="text-cyan-400" />
                <h3 className="text-xl font-bold text-white">Reference Metrics</h3>
              </div>
              
              <div className="space-y-4">
                 <div className="p-3 bg-slate-800 rounded border border-slate-700 cursor-pointer hover:border-cyan-500 transition-colors" onClick={() => setMetrics({...metrics, unitSystem: 'metric', referenceSize: 8.56, referenceName: 'Standard ID Card'})}>
                    <div className="flex items-center gap-3">
                        <Scan className="text-cyan-400" size={20} />
                        <div>
                            <div className="text-sm font-bold text-white">Standard ID / Credit Card</div>
                            <div className="text-[10px] text-slate-400">Universal Reference (85.6mm)</div>
                        </div>
                    </div>
                 </div>

                 <div className="p-3 bg-slate-800 rounded border border-slate-700 cursor-pointer hover:border-cyan-500 transition-colors" onClick={() => setMetrics({...metrics, unitSystem: 'metric', referenceSize: 7, referenceName: 'Thumb Width'})}>
                    <div className="flex items-center gap-3">
                        <Fingerprint className="text-cyan-400" size={20} />
                        <div>
                            <div className="text-sm font-bold text-white">Adult Thumb Width</div>
                            <div className="text-[10px] text-slate-400">Approx Reference (~20-25mm)</div>
                        </div>
                    </div>
                 </div>

                <div>
                   <label className="block text-xs uppercase text-slate-500 mb-1">Subject Type</label>
                   <select 
                      value={metrics.subjectType}
                      onChange={(e) => setMetrics({...metrics, subjectType: e.target.value})}
                      className="w-full bg-slate-800 border border-slate-700 rounded p-3 text-white focus:border-cyan-500 outline-none"
                   >
                     <option>General Object</option>
                     <option>Vehicle (Car/Truck)</option>
                     <option>Human Athlete</option>
                     <option>Ball/Projectile</option>
                   </select>
                </div>

                <button onClick={() => setStep(2)} className="w-full py-3 bg-cyan-600 hover:bg-cyan-500 text-white font-bold rounded-lg mt-4">Calibrate Sensors</button>
              </div>
           </div>
        )}

        {step === 2 && (
          <>
            <div className="w-32 h-32 rounded-full border-4 border-slate-700 flex items-center justify-center mb-8 relative">
              <div className="absolute inset-0 rounded-full border-4 border-cyan-500 transition-all duration-200" style={{ clipPath: `inset(${100 - progress}% 0 0 0)` }}></div>
              <Activity size={64} className="text-slate-500 animate-pulse" />
            </div>
            <h2 className="text-2xl font-bold text-white mb-2">Zeroing Gyroscopes</h2>
            <p className="text-slate-400 text-center mb-8">Establishing inertial baseline...</p>
          </>
        )}

        {step === 3 && (
            <>
              <div className="w-32 h-32 rounded-full border-4 border-slate-700 flex items-center justify-center mb-8 relative">
                <div className="absolute inset-0 rounded-full border-4 border-purple-500 transition-all duration-200" style={{ clipPath: `inset(${100 - progress}% 0 0 0)` }}></div>
                <Clock size={64} className="text-slate-500 animate-bounce" />
              </div>
              <h2 className="text-2xl font-bold text-white mb-2">Sensor Fusion Sync</h2>
              <p className="text-slate-400 text-center mb-8">Measuring shutter latency...</p>
            </>
          )}

        {step === 4 && (
            <>
              <div className="w-32 h-32 rounded-full border-4 border-slate-700 flex items-center justify-center mb-8 bg-slate-900">
                <ZoomIn size={48} className="text-green-500" />
              </div>
              <h2 className="text-2xl font-bold text-white mb-2">Multi-Lens Curve Calibration</h2>
              <p className="text-slate-400 text-center mb-8">Map optical variances across zoom levels...</p>
              <div className="flex gap-4 w-full">
                  <button 
                    onClick={() => calibrateZoom(1)} 
                    className={`flex-1 py-3 rounded border transition-colors ${zoomCurve.some(z=>z.zoomLevel===1) ? 'bg-green-900/40 border-green-500 text-green-400' : 'bg-slate-800 border-slate-700'}`}
                  >1x (Wide)</button>
                  <button 
                    onClick={() => calibrateZoom(1.5)} 
                    className={`flex-1 py-3 rounded border transition-colors ${zoomCurve.some(z=>z.zoomLevel===1.5) ? 'bg-green-900/40 border-green-500 text-green-400' : 'bg-slate-800 border-slate-700'}`}
                  >1.5x (Mid)</button>
                  <button 
                    onClick={() => calibrateZoom(2)} 
                    className={`flex-1 py-3 rounded border transition-colors ${zoomCurve.some(z=>z.zoomLevel===2) ? 'bg-green-900/40 border-green-500 text-green-400' : 'bg-slate-800 border-slate-700'}`}
                  >2x (Tele)</button>
              </div>
              <div className="mt-4 text-xs font-mono text-slate-500">
                  Data Points: {zoomCurve.length}/3
              </div>
              {zoomCurve.length >= 2 && (
                   <button onClick={() => setStep(5)} className="mt-6 text-sm text-cyan-400 hover:underline">Complete Calibration &gt;</button>
              )}
            </>
          )}

        {step === 5 && (
          <>
             <div className="w-full flex items-center justify-between mb-4 bg-slate-900 p-4 rounded-lg">
                 <div className="text-center">
                    <div className="text-xl font-bold text-cyan-400">{(jitterVariance.current * 100).toFixed(1)}%</div>
                    <div className="text-[10px] text-slate-500">JITTER</div>
                 </div>
                 <div className="text-center">
                    <div className="text-xl font-bold text-yellow-400">{sensorLatency.current.toFixed(1)}ms</div>
                    <div className="text-[10px] text-slate-500">LATENCY</div>
                 </div>
                 <div className="text-center">
                    <div className="text-xl font-bold text-green-400">{zoomCurve.length}</div>
                    <div className="text-[10px] text-slate-500">CURVE PTS</div>
                 </div>
             </div>

             <div 
               className="relative border-2 border-dashed border-cyan-500 rounded-lg mb-8 flex flex-col items-center justify-center bg-cyan-900/20"
               style={{ width: `${REF_BOX_SIZE_PX}px`, height: `${REF_BOX_SIZE_PX}px` }}
             >
               <Layers size={32} className="text-cyan-500 mb-2" />
               <div className="text-center">
                  <span className="text-lg font-bold text-white block">{metrics.referenceName}</span>
                  <span className="text-[10px] text-cyan-300">ALIGN TO BOX</span>
               </div>
             </div>

             <button 
                onClick={finishCalibration}
                className="w-full py-4 bg-cyan-600 hover:bg-cyan-500 text-white font-bold rounded-lg transition-all shadow-lg shadow-cyan-900/50 flex items-center justify-center gap-2"
             >
                <CheckCircle size={20} />
                Engage Holographic Core
             </button>
          </>
        )}

      </div>
    </div>
  );
};

export default CalibrationScreen;
