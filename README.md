# KineticLens AI: Rational Dynamics Engine

**KineticLens** is a commercial-grade motion analysis application that utilizes "Tri-Phase Photography" and sensor fusion (Accelerometer/Gyroscope) to estimate object speed, trajectory, and structural vibration.

Unlike standard computer vision which relies solely on pixel data, KineticLens projects a **Holographic Metadata Layer** onto the image before sending it to Google's Gemini Vision API. This allows the LLM to perform "Visual Vector Math" to isolate camera movement from subject movement.

## 🚀 Key Features

### 1. Parallax Gyro-Epitaxy
A visualization technique that stacks three sequential frames (T0, T1, T2) to reveal depth changes.
- **Red Reticle (T0):** Start position.
- **Blue Reticle (T2):** End position.
- **Visual Logic:** If the blue reticle is larger than the red, the camera moved forward (Z-axis). If they are misaligned, the camera rotated.

### 2. Geometric Kinetic Protocol
A strict mathematical prompting system that forces the AI to solve the equation:
$$ V_{subject} = V_{observed} - (V_{camera\_trans} + V_{camera\_rot}) $$
The app renders a "Legend" on the image so the AI knows exactly which visual symbols represent these variables.

### 3. Hyper-Vernier Scales
On-screen micro-rulers that allow the AI to measure pixel displacement with sub-pixel precision by comparing the offset against a known grid pattern.

### 4. Sensor Fusion Calibration
- **Dark Frame Analysis:** Estimates ISO noise to determine confidence levels.
- **Jitter Variance:** Calculates user hand-shake stability to weight the reliability of gyroscope data vs. optical flow.

## 🛠️ Technical Stack

- **Frontend:** React 19, TypeScript, TailwindCSS.
- **AI/LLM:** Google Gemini 1.5 Pro (via `@google/genai` SDK).
- **Visualization:** HTML5 Canvas (2D Context) for real-time overlay compositing.
- **Sensors:** DeviceMotion and DeviceOrientation APIs.

## 📦 Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-org/kinetic-lens.git
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Set API Key:**
   Ensure `process.env.API_KEY` is available with a valid Gemini API key.

4. **Run Development Server:**
   ```bash
   npm start
   ```

## 📱 Usage Guide

1. **Calibration:** Follow the on-screen steps to measure light levels (ISO) and sensor latency. Align the reference box with a standard ID card for scale.
2. **Target Zone:** Keep the subject in the center. The box turns RED if you are moving too fast.
3. **Capture:** The system auto-fires a 3-frame burst when stability is high.
4. **Analysis:** The "Rational Dynamics Engine" constructs a composite image and generates a Geometric Proof of motion.

---
*Powered by Google Gemini Vision*
