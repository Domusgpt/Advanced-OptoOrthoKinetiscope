import { GoogleGenAI, Type } from "@google/genai";
import { SessionData, AnalysisResult, Vector3 } from "../types";
import { generateCompositeFrame } from "./imageCompositor";

const formatVector = (v: Vector3) => `(X:${v.x.toFixed(3)}, Y:${v.y.toFixed(3)}, Z:${v.z.toFixed(3)})`;

export const generateAnalysisContext = async (session: SessionData) => {
  let compositeImageBase64 = "";
  try {
     if (session.captures.length >= 3) {
       compositeImageBase64 = await generateCompositeFrame(session.captures, session.calibration!);
       compositeImageBase64 = compositeImageBase64.split(',')[1];
     }
  } catch (e) {
    console.error("Compositing failed", e);
  }

  // Calculate Z-Shift for Volumetric Analysis
  const first = session.captures[0];
  const last = session.captures[session.captures.length - 1];
  const zShift = Math.abs(last.acceleration.z - first.acceleration.z).toFixed(3);

  // Calculate Average Efficiency
  const avgEff = session.captures.reduce((acc, c) => acc + ((c as any).vectorEfficiency || 1.0), 0) / session.captures.length;
  const isStable = avgEff > 0.6;

  const systemInstruction = `
    You are the "KineticLens Visual Interpretation Module".
    You are analyzing a "Triletic Matrix Composite" with OptoOrtho-Gimbal telemetry.
    
    --- DATA RELIABILITY CHECK ---
    **VECTOR EFFICIENCY:** ${(avgEff * 100).toFixed(1)}% (Printed in Legend Zone 4)
    ${isStable 
        ? "Condition: STABLE. The Gold Lattice is a RELIABLE inertial reference." 
        : "Condition: UNSTABLE. High orthogonal jitter detected. Do NOT trust the Gold Lattice alignment. Rely on the Cyan Epipolar Flow."}

    --- VISUAL SEMIOTICS LEGEND (TOKENIZABLE SYMBOLS) ---
    The image contains a "Holographic Overlay" that translates invisible sensor data into visible geometry. Use this key:

    [ ⌖ ] **PARALLAX RETICLES (Ghost View)**
          - **RED Ring:** Frame T0 (Start).
          - **BLUE Ring:** Frame T2 (End).
          - **Analysis Logic:**
            1. **Concentricity:** If Red/Blue rings are perfectly aligned, the camera position is static (XYZ locked).
            2. **Scale Delta:** If the Blue ring is *larger* than the Red ring, the camera physically moved closer (Z-Axis +).
            3. **Chromatic Fringing (CRITICAL):** If the reticle lines appear split into RGB colors (like a glitch), this indicates **High Moire Variance** (Structural Vibration). You must discount fine details in this region.

    [ ⬡ ] **GOLD LATTICE (Inertial Truth)**
          - Represents the gyroscope's pure rotational data projected onto the image plane.
          - **Visual Cue:** If the subject moves *with* the Gold Lattice, it is camera shake. If the subject moves *across* the Gold Lattice, it is true motion.

    [ ≋ ] **CYAN RAYS (Epipolar Flow)**
          - Represents the optical flow background. Objects cutting across these lines have independent velocity.

    --- KINETIC ISOLATION FORMULA (Step-by-Step Reasoning) ---
    You must demonstrate your work by solving this vector equation in your reasoning:

    1. **Define $\\vec{V}_{cam}$ (Camera Vector):** 
       - Look at the [ ⬡ ] Gold Lattice shift. 
       - Look at the [ ⌖ ] Parallax Reticle expansion.
       - Estimate the camera's ego-motion.

    2. **Define $\\vec{V}_{optical}$ (Optical Vector):**
       - Look at the pixel displacement of the Subject relative to the background [ ≋ ] Cyan Rays.

    3. **Compute $\\vec{V}_{subject}$ (True Motion):**
       - $\\vec{V}_{subject} = \\vec{V}_{optical} - \\vec{V}_{cam}$
       - If $\\vec{V}_{cam}$ is high (shaky footage) but matches $\\vec{V}_{optical}$, then $\\vec{V}_{subject}$ is ZERO (Static object).

    Output your analysis as structured JSON.
  `;

  let userPrompt = `TELEMETRY LOG (Analog Calculations):\n`;
  session.captures.forEach((c, i) => {
    userPrompt += `[Frame ${i}] T+${c.relativeTime}ms | ${c.lensGroup || 'STD'} | Acc${formatVector(c.acceleration)} | Rot${formatVector(c.rotationRate)} | Zoom: ${c.zoomLevel}x | Efficiency: ${((c as any).vectorEfficiency*100).toFixed(0)}%\n`;
  });

  return { systemInstruction, userPrompt, compositeImageBase64 };
};

export const analyzeMotionSession = async (session: SessionData): Promise<AnalysisResult> => {
  const apiKey = process.env.API_KEY;
  if (!apiKey) throw new Error("API Key not found");

  const { systemInstruction, userPrompt, compositeImageBase64 } = await generateAnalysisContext(session);

  if (!compositeImageBase64) throw new Error("Failed to generate composite image");

  const ai = new GoogleGenAI({ apiKey });
  const model = "gemini-3-pro-preview";

  const response = await ai.models.generateContent({
    model: model,
    contents: {
      parts: [
        { text: userPrompt },
        { inlineData: { mimeType: "image/jpeg", data: compositeImageBase64 } }
      ]
    },
    config: {
      systemInstruction: systemInstruction,
      responseMimeType: "application/json",
      responseSchema: {
        type: Type.OBJECT,
        properties: {
          estimatedSpeed: { type: Type.STRING },
          trajectoryDescription: { type: Type.STRING },
          confidenceScore: { type: Type.NUMBER },
          reasoning: { type: Type.STRING },
          objectDetected: { type: Type.STRING },
          boundingBox: {
            type: Type.OBJECT,
            properties: {
              ymin: { type: Type.NUMBER },
              xmin: { type: Type.NUMBER },
              ymax: { type: Type.NUMBER },
              xmax: { type: Type.NUMBER },
            },
            nullable: true,
          },
          trajectoryPoints: {
            type: Type.ARRAY,
            items: {
              type: Type.OBJECT,
              properties: {
                x: { type: Type.NUMBER },
                y: { type: Type.NUMBER }
              }
            }
          },
          kinematics: {
            type: Type.OBJECT,
            properties: {
              cameraMotionDescription: { type: Type.STRING },
              subjectMotionVector: { type: Type.STRING },
              isolationConfidence: { type: Type.NUMBER },
              depthCues: { type: Type.STRING },
              estimatedPhysicalDisplacement: { type: Type.STRING },
              floorMapAnalysis: { type: Type.STRING },
              holographicInterferenceAnalysis: { type: Type.STRING },
              occlusionMatrixReading: { type: Type.STRING },
              parallaxEpitaxyAnalysis: { type: Type.STRING },
              geometricProof: { type: Type.STRING }
            },
            required: ["cameraMotionDescription", "subjectMotionVector", "isolationConfidence", "depthCues", "estimatedPhysicalDisplacement", "floorMapAnalysis", "holographicInterferenceAnalysis", "occlusionMatrixReading", "parallaxEpitaxyAnalysis", "geometricProof"]
          }
        },
        required: ["estimatedSpeed", "trajectoryDescription", "confidenceScore", "reasoning", "objectDetected", "kinematics"]
      }
    }
  });

  if (response.text) return JSON.parse(response.text);
  throw new Error("No analysis generated");
};