import React, { useState } from 'react';
import { Play, Activity, ChevronRight, RotateCcw, Zap, Camera, CheckCircle2, ShieldAlert, Download, Layers, Compass, BookOpen, ToggleLeft, ToggleRight, Archive, Cpu } from 'lucide-react';
import CaptureScreen from './components/CaptureScreen';
import CalibrationScreen from './components/CalibrationScreen';
import SensorChart from './components/SensorChart';
import GhostView from './components/GhostView';
import { analyzeMotionSession } from './services/geminiService';
import { packageSessionForExport } from './services/packageService';
import { AppState, CapturePoint, AnalysisResult, CalibrationData, CaptureMode } from './types';

function App() {
  const [appState, setAppState] = useState<AppState>(AppState.IDLE);
  const [captures, setCaptures] = useState<CapturePoint[]>([]);
  const [calibration, setCalibration] = useState<CalibrationData | null>(null);
  const [analysis, setAnalysis] = useState<AnalysisResult | null>(null);
  const [currentMode, setCurrentMode] = useState<CaptureMode>('TRAJECTORY');
  const [errorMsg, setErrorMsg] = useState<string | null>(null);
  const [isEncoding, setIsEncoding] = useState(false);
  
  // NEW: Gemini Integration Toggle
  const [useGemini, setUseGemini] = useState(true);

  const requestMotionPermission = async () => {
    if (typeof (DeviceMotionEvent as any).requestPermission === 'function') {
      try {
        const response = await (DeviceMotionEvent as any).requestPermission();
        if (response === 'granted') {
          setAppState(AppState.INSTRUCTIONS);
        } else {
          setErrorMsg("Motion sensor permission denied. This app requires accelerometer access.");
        }
      } catch (e) {
        setAppState(AppState.INSTRUCTIONS);
      }
    } else {
      setAppState(AppState.INSTRUCTIONS);
    }
  };

  const handleCalibrationComplete = (data: CalibrationData) => {
    setCalibration(data);
    setAppState(AppState.CAPTURING);
  };

  const handleCaptureComplete = (capturedPoints: CapturePoint[], mode: CaptureMode) => {
    if (!capturedPoints || capturedPoints.length === 0) {
        setErrorMsg("Capture failed: No frames were recorded.");
        setAppState(AppState.ERROR);
        return;
    }
    setIsEncoding(false);
    setCaptures(capturedPoints);
    setCurrentMode(mode);
    setAppState(AppState.ANALYZING);
    runAnalysis(capturedPoints, mode);
  };

  const runAnalysis = async (points: CapturePoint[], mode: CaptureMode) => {
    // Safety check for points
    if (!points || points.length < 1) {
       setErrorMsg("Insufficient data points for analysis.");
       setAppState(AppState.ERROR);
       return;
    }

    const session = {
      sessionId: crypto.randomUUID(),
      mode: mode,
      captures: points,
      calibration: calibration,
      startTime: points[0].timestamp,
      endTime: points[points.length - 1].timestamp
    };

    try {
      if (useGemini) {
        // Online Mode
        const result = await analyzeMotionSession(session);
        setAnalysis(result);
        setAppState(AppState.RESULTS);
      } else {
        // Offline / Export Mode
        // Create a dummy result for the UI
        const dummyAnalysis: AnalysisResult = {
           estimatedSpeed: "PENDING EXTERNAL ANALYSIS",
           trajectoryDescription: "Data packaged for external LLM.",
           confidenceScore: 1.0,
           reasoning: "Session exported to ZIP. Please upload the generated package to ChatGPT or Claude to view reasoning.",
           objectDetected: "See Composite Frame",
           kinematics: {
             cameraMotionDescription: "See Telemetry Log",
             subjectMotionVector: "See Telemetry Log",
             isolationConfidence: 1,
             depthCues: "Parallax data available in composite",
             estimatedPhysicalDisplacement: "N/A",
             floorMapAnalysis: "N/A",
             holographicInterferenceAnalysis: "N/A",
             occlusionMatrixReading: "N/A",
             parallaxEpitaxyAnalysis: "N/A",
             geometricProof: "Available in exported instructions."
           }
        };
        setAnalysis(dummyAnalysis);
        setAppState(AppState.RESULTS);
        
        // Auto trigger download
        const blob = await packageSessionForExport(session);
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `kinetic_lens_export_${session.sessionId.substring(0,8)}.zip`;
        a.click();
        URL.revokeObjectURL(url);
      }
    } catch (e: any) {
      console.error(e);
      setErrorMsg(e.message || "Failed to analyze motion.");
      setAppState(AppState.ERROR);
    }
  };

  return (
    <div className="w-full min-h-screen bg-slate-950 text-slate-100 flex flex-col overflow-hidden relative">
      
      {/* Header */}
      <header className="h-16 border-b border-slate-800 flex items-center justify-between px-6 bg-slate-900/80 backdrop-blur-md z-10">
        <div className="flex items-center gap-2">
          <Activity className="text-cyan-400" size={24} />
          <h1 className="text-xl font-bold tracking-tight text-white">Kinetic<span className="text-cyan-400">Lens</span></h1>
        </div>
        <div className="flex items-center gap-4">
          <div 
            onClick={() => setUseGemini(!useGemini)}
            className="flex items-center gap-2 cursor-pointer bg-slate-800 px-3 py-1.5 rounded-full border border-slate-700 hover:border-slate-500 transition-colors"
          >
             <Cpu size={14} className={useGemini ? "text-cyan-400" : "text-slate-500"} />
             <span className="text-xs font-mono text-slate-300">{useGemini ? "GEMINI CLOUD" : "OFFLINE MODE"}</span>
             {useGemini ? <ToggleRight size={18} className="text-cyan-400"/> : <ToggleLeft size={18} className="text-slate-500"/>}
          </div>
        </div>
      </header>

      {/* Main Content Area */}
      <main className="flex-1 relative flex flex-col">
        
        {/* State: IDLE */}
        {appState === AppState.IDLE && (
          <div className="flex-1 flex flex-col items-center justify-center p-8 text-center bg-[url('https://images.unsplash.com/photo-1550751827-4bd374c3f58b?auto=format&fit=crop&q=80&w=2670&ixlib=rb-4.0.3')] bg-cover bg-center">
            <div className="absolute inset-0 bg-slate-950/80 backdrop-blur-sm"></div>
            <div className="relative z-10 max-w-lg">
              <h2 className="text-4xl font-bold mb-4 text-white">Adaptive Chrono-Spatial<br/>Motion Analysis</h2>
              <p className="text-slate-300 mb-8 text-lg">
                Utilizing adaptive polling loops and hyper-vernier scales to analyze trajectory and vibration structural integrity.
              </p>
              <button 
                onClick={requestMotionPermission}
                className="group relative inline-flex items-center gap-3 px-8 py-4 bg-cyan-600 hover:bg-cyan-500 text-white font-semibold rounded-lg transition-all shadow-lg shadow-cyan-900/50"
              >
                Start Session
                <ChevronRight className="group-hover:translate-x-1 transition-transform" />
              </button>
            </div>
          </div>
        )}

        {/* State: INSTRUCTIONS */}
        {appState === AppState.INSTRUCTIONS && (
          <div className="flex-1 flex flex-col items-center justify-center p-6 bg-slate-950">
             <div className="max-w-md w-full bg-slate-900 border border-slate-800 rounded-2xl p-8 shadow-2xl">
                <div className="w-12 h-12 bg-cyan-900/30 rounded-full flex items-center justify-center mb-6 text-cyan-400">
                   <Zap size={24} />
                </div>
                <h3 className="text-2xl font-bold text-white mb-4">Preparation Protocol</h3>
                <ul className="space-y-4 mb-8 text-slate-300">
                  <li className="flex gap-3">
                    <span className="flex-shrink-0 w-6 h-6 rounded-full bg-slate-800 border border-slate-700 flex items-center justify-center text-xs font-mono">1</span>
                    <span>Hold device still for Sensor Jitter calibration.</span>
                  </li>
                  <li className="flex gap-3">
                    <span className="flex-shrink-0 w-6 h-6 rounded-full bg-slate-800 border border-slate-700 flex items-center justify-center text-xs font-mono">2</span>
                    <span>Allow Latency Test to sync shutter speed.</span>
                  </li>
                  <li className="flex gap-3">
                    <span className="flex-shrink-0 w-6 h-6 rounded-full bg-slate-800 border border-slate-700 flex items-center justify-center text-xs font-mono">3</span>
                    <span>Select TRAJECTORY (standard) or VIBRATION (high-speed) mode.</span>
                  </li>
                </ul>
                <button 
                  onClick={() => setAppState(AppState.CALIBRATION)}
                  className="w-full py-3 bg-white text-slate-950 font-bold rounded-lg hover:bg-slate-200 transition-colors"
                >
                  Begin Calibration
                </button>
             </div>
          </div>
        )}

        {/* State: CALIBRATION */}
        {appState === AppState.CALIBRATION && (
          <CalibrationScreen onCalibrationComplete={handleCalibrationComplete} />
        )}

        {/* State: CAPTURING */}
        {appState === AppState.CAPTURING && (
          <CaptureScreen 
            calibration={calibration}
            onCaptureComplete={handleCaptureComplete}
            onCancel={() => setAppState(AppState.IDLE)}
            isEncoding={isEncoding}
          />
        )}

        {/* State: ANALYZING (Loading) */}
        {appState === AppState.ANALYZING && (
          <div className="flex-1 flex flex-col items-center justify-center bg-slate-950">
             <div className="relative w-24 h-24 mb-8">
               <div className="absolute inset-0 border-4 border-cyan-900 rounded-full"></div>
               <div className="absolute inset-0 border-4 border-cyan-400 border-t-transparent rounded-full animate-spin"></div>
               <Activity className="absolute inset-0 m-auto text-cyan-400 animate-pulse" />
             </div>
             <h3 className="text-xl font-mono text-cyan-400 mb-2">RATIONAL DYNAMICS ENGINE</h3>
             <p className="text-slate-500 text-sm">{useGemini ? "Decoding Hyper-Vernier Scales..." : "Packaging Data for Export..."}</p>
          </div>
        )}

        {/* State: RESULTS */}
        {appState === AppState.RESULTS && analysis && (
          <div className="flex-1 overflow-y-auto bg-slate-950 p-6">
            <div className="max-w-4xl mx-auto space-y-6">
              
              {!useGemini && (
                 <div className="bg-yellow-900/20 border border-yellow-700 rounded-xl p-4 flex items-center gap-4">
                    <Archive className="text-yellow-500" size={24} />
                    <div>
                       <h3 className="font-bold text-yellow-500">Offline Export Complete</h3>
                       <p className="text-sm text-yellow-200/70">A ZIP file containing the Composite Frame and Prompt Logic has been downloaded.</p>
                    </div>
                 </div>
              )}

              {/* Top Summary Card */}
              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                <div className="bg-slate-900 border border-slate-800 rounded-xl p-6">
                   <div className="flex items-center justify-between mb-4">
                      <h3 className="text-lg font-semibold text-slate-100">Analysis Output</h3>
                      <div className="px-3 py-1 rounded-full bg-cyan-900/30 text-cyan-400 text-xs font-mono border border-cyan-900/50">
                        {currentMode} MODE
                      </div>
                   </div>
                   <div className="space-y-4">
                      <div>
                        <label className="text-xs text-slate-500 uppercase tracking-wider">Estimated Speed</label>
                        <p className="text-3xl font-bold text-white tracking-tight">{analysis.estimatedSpeed}</p>
                      </div>
                      <div>
                        <label className="text-xs text-slate-500 uppercase tracking-wider">Object Detected</label>
                        <p className="text-lg text-slate-300">{analysis.objectDetected}</p>
                      </div>
                   </div>
                </div>

                {/* Trajectory / Reasoning */}
                <div className="bg-slate-900 border border-slate-800 rounded-xl p-6 flex flex-col">
                   <h3 className="text-sm font-semibold text-slate-400 mb-3 uppercase">AI Reasoning</h3>
                   <p className="text-sm text-slate-300 leading-relaxed flex-1">
                     {analysis.reasoning}
                   </p>
                </div>
              </div>

              {/* GEOMETRIC PROOF CARD (NEW) */}
              <div className="bg-slate-900 border border-slate-800 rounded-xl p-6 relative overflow-hidden">
                 <div className="absolute right-0 top-0 p-4 opacity-10">
                    <BookOpen size={100} />
                 </div>
                 <div className="flex items-center gap-2 mb-4">
                    <BookOpen size={20} className="text-yellow-400" />
                    <h3 className="text-lg font-semibold text-white">Geometric Kinetic Proof</h3>
                 </div>
                 <div className="font-mono text-sm text-yellow-100/80 bg-slate-950/50 p-4 rounded border border-yellow-900/30">
                    {analysis.kinematics.geometricProof || "Generating proof..."}
                 </div>
              </div>

              {/* KINEMATIC ISOLATION DASHBOARD */}
              <div className="bg-slate-900 border border-slate-800 rounded-xl p-6">
                <div className="flex items-center gap-2 mb-4 border-b border-slate-800 pb-3">
                   <Compass size={20} className="text-cyan-400" />
                   <h3 className="text-lg font-semibold text-white">Kinematic Isolation (VIO)</h3>
                </div>
                
                <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
                  {/* Camera Vector */}
                  <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                     <h4 className="text-xs text-slate-500 uppercase mb-2">Camera Ego-Motion</h4>
                     <p className="text-sm font-mono text-yellow-400">{analysis.kinematics.cameraMotionDescription}</p>
                     <div className="w-full bg-slate-800 h-1 mt-2 rounded">
                        <div className="bg-yellow-500 h-1 rounded" style={{width: '60%'}}></div>
                     </div>
                  </div>

                  {/* Subject Vector */}
                  <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                     <h4 className="text-xs text-slate-500 uppercase mb-2">Subject Motion Vector</h4>
                     <p className="text-sm font-mono text-cyan-400">{analysis.kinematics.subjectMotionVector}</p>
                     <div className="w-full bg-slate-800 h-1 mt-2 rounded">
                        <div className="bg-cyan-500 h-1 rounded" style={{width: `${analysis.kinematics.isolationConfidence}%`}}></div>
                     </div>
                  </div>

                  {/* Depth Cues */}
                  <div className="p-4 bg-slate-950/50 rounded-lg border border-slate-800">
                     <h4 className="text-xs text-slate-500 uppercase mb-2">Depth & Lighting Cues</h4>
                     <p className="text-sm text-slate-300">{analysis.kinematics.depthCues}</p>
                  </div>
                </div>
              </div>

              {/* Ghost View & Charts */}
              <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
                <div className="lg:col-span-2 space-y-4">
                  <GhostView captures={captures} boundingBox={analysis.boundingBox} />
                  
                  <div className="grid grid-cols-3 gap-2 opacity-50 hover:opacity-100 transition-opacity">
                    {captures.map((cap, idx) => (
                      <div key={cap.id} className="relative aspect-video bg-slate-800 rounded-lg overflow-hidden border border-slate-700">
                        <img src={cap.imageUri} alt={`Capture ${idx+1}`} className="w-full h-full object-cover" />
                        {cap.zoomLevel > 1.0 && <div className="absolute top-1 right-1 bg-green-500/80 px-1 rounded text-[8px] text-black font-bold">ZOOM</div>}
                      </div>
                    ))}
                  </div>
                </div>

                <div className="lg:col-span-1 space-y-6">
                   <SensorChart data={captures} />
                   
                   <div className="p-4 bg-slate-900 border border-slate-800 rounded-lg">
                      <h4 className="text-xs text-slate-500 mb-4 uppercase">Session Actions</h4>
                      <button 
                        onClick={() => setAppState(AppState.IDLE)}
                        className="w-full flex items-center justify-center gap-2 py-2 bg-cyan-900/30 hover:bg-cyan-900/50 text-cyan-400 text-sm rounded transition-colors border border-cyan-900"
                      >
                        <RotateCcw size={16} /> New Analysis
                      </button>
                   </div>
                </div>
              </div>

            </div>
          </div>
        )}

        {/* State: ERROR */}
        {appState === AppState.ERROR && (
          <div className="flex-1 flex flex-col items-center justify-center p-6 text-center">
            <ShieldAlert size={48} className="text-red-500 mb-4" />
            <h3 className="text-xl font-bold text-white mb-2">Analysis Failed</h3>
            <p className="text-slate-400 mb-8 max-w-md">{errorMsg}</p>
            <button 
              onClick={() => setAppState(AppState.IDLE)}
              className="px-6 py-3 bg-slate-800 text-white rounded-lg"
            >
              Return Home
            </button>
          </div>
        )}

      </main>
    </div>
  );
}

export default App;
