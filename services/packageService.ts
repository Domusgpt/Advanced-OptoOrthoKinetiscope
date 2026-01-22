import JSZip from 'jszip';
import { SessionData } from '../types';
import { generateAnalysisContext } from './geminiService';

export const packageSessionForExport = async (session: SessionData): Promise<Blob> => {
  const zip = new JSZip();
  const folder = zip.folder(`kinetic_session_${session.sessionId.substring(0,8)}`);
  
  if (!folder) throw new Error("Failed to create zip folder");

  // 1. Generate the Analysis Context (Prompt + Composite)
  // This uses the exact same logic as the live API call
  const { systemInstruction, userPrompt, compositeImageBase64 } = await generateAnalysisContext(session);

  // 2. Add Composite Image
  if (compositeImageBase64) {
      folder.file("composite_analysis_frame.jpg", compositeImageBase64, { base64: true });
  }

  // 3. Add Instructions for External LLM
  const llmInstructions = `
=== KINETIC LENS EXTERNAL ANALYSIS PROTOCOL ===

You are acting as the external Reasoning Engine for KineticLens.
Attached is a "Composite Analysis Frame" containing visual motion data.

--- SYSTEM INSTRUCTION ---
${systemInstruction}

--- USER TELEMETRY DATA ---
${userPrompt}

--- INSTRUCTIONS FOR USER ---
1. Upload 'composite_analysis_frame.jpg' to ChatGPT, Claude, or Gemini.
2. Paste the text above (System Instruction + Telemetry).
3. The AI will output the Geometric Proof and Analysis.
`;
  
  folder.file("INSTRUCTIONS_AND_PROMPT.txt", llmInstructions);

  // 4. Add Raw Data
  folder.file("raw_session_data.json", JSON.stringify(session, null, 2));

  // 5. Add Individual Frames (Optional, for manual review)
  const framesFolder = folder.folder("raw_frames");
  if (framesFolder) {
      session.captures.forEach((c, i) => {
          const data = c.imageUri.split(',')[1];
          framesFolder.file(`frame_${i}_t${c.relativeTime}.jpg`, data, {base64: true});
      });
  }

  return await zip.generateAsync({ type: "blob" });
};
