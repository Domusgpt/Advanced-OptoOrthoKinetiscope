import { Vector3, Quaternion } from "../types";

// --- 4D GEOMETRY KERNEL V2.1 (Refined OptoOrtho-Gimbal) ---

export interface Vector4 {
    x: number; y: number; z: number; w: number;
}

/**
 * ORTHOGONAL REJECTION FILTER
 * Instead of averaging everything, we identify the "Principal Axis" of motion.
 * We preserve acceleration ALONG that axis (Signal).
 * We aggressively dampen acceleration PERPENDICULAR to that axis (Jitter).
 */
export class OrthogonalFilter {
    private principalAxis: Vector3 = { x: 0, y: 0, z: 0 };
    private gravity: Vector3 = { x: 0, y: 0, z: 0 };
    private alphaGravity: number = 0.92; // Tuned for slightly faster gravity adaptation
    private alphaTrend: number = 0.15;   // Faster trend lock-in for "Snap" motions
    private currentEfficiency: number = 1.0;

    // Dot product helper
    private dot(v1: Vector3, v2: Vector3): number {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    // Vector scaling helper
    private scale(v: Vector3, s: number): Vector3 {
        return { x: v.x * s, y: v.y * s, z: v.z * s };
    }

    // Vector addition helper
    private add(v1: Vector3, v2: Vector3): Vector3 {
        return { x: v1.x + v2.x, y: v1.y + v2.y, z: v1.z + v2.z };
    }

    update(rawAccel: Vector3): { clean: Vector3, orthogonalNoise: number } {
        // 1. Isolate Gravity (Low Pass)
        this.gravity.x = this.alphaGravity * this.gravity.x + (1 - this.alphaGravity) * rawAccel.x;
        this.gravity.y = this.alphaGravity * this.gravity.y + (1 - this.alphaGravity) * rawAccel.y;
        this.gravity.z = this.alphaGravity * this.gravity.z + (1 - this.alphaGravity) * rawAccel.z;

        // 2. Get Linear Acceleration (High Pass)
        const linear = {
            x: rawAccel.x - this.gravity.x,
            y: rawAccel.y - this.gravity.y,
            z: rawAccel.z - this.gravity.z
        };

        // 3. Update Principal Axis (Exponential Moving Average of Direction)
        // This represents the "Intended Trajectory"
        this.principalAxis.x = (1 - this.alphaTrend) * this.principalAxis.x + this.alphaTrend * linear.x;
        this.principalAxis.y = (1 - this.alphaTrend) * this.principalAxis.y + this.alphaTrend * linear.y;
        this.principalAxis.z = (1 - this.alphaTrend) * this.principalAxis.z + this.alphaTrend * linear.z;

        // Normalize Principal Axis
        const mag = Math.sqrt(this.dot(this.principalAxis, this.principalAxis));
        const axis = mag > 0.001 
            ? this.scale(this.principalAxis, 1/mag) 
            : { x: 1, y: 0, z: 0 }; // Default x-axis if static

        // 4. DECOMPOSITION
        // Project Linear acceleration onto the Principal Axis
        const scalarProjection = this.dot(linear, axis);
        const parallelComponent = this.scale(axis, scalarProjection);

        // Calculate Orthogonal Component (The Jitter/Noise)
        const orthogonalComponent = {
            x: linear.x - parallelComponent.x,
            y: linear.y - parallelComponent.y,
            z: linear.z - parallelComponent.z
        };

        const orthoMag = Math.sqrt(this.dot(orthogonalComponent, orthogonalComponent));
        const totalMag = Math.sqrt(this.dot(linear, linear));

        // Calculate Efficiency (Signal / Total Energy)
        // Avoid divide by zero
        this.currentEfficiency = totalMag > 0.1 ? 1.0 - (orthoMag / totalMag) : 1.0;
        // Clamp
        this.currentEfficiency = Math.max(0, Math.min(1, this.currentEfficiency));

        // 5. RECONSTRUCTION
        // We keep 100% of the Parallel (Signal)
        // We only keep 10% of the Orthogonal (Noise dampening)
        const cleanOrthogonal = this.scale(orthogonalComponent, 0.1); 
        
        const cleanVector = this.add(parallelComponent, cleanOrthogonal);

        return { 
            clean: cleanVector, 
            orthogonalNoise: orthoMag // Return magnitude of rejected noise for UI
        };
    }

    // Get the current mathematical "Rail" for HUD visualization
    getPrincipalAxis(): Vector3 {
        return { ...this.principalAxis };
    }

    // Get current efficiency score (0.0 - 1.0)
    getEfficiency(): number {
        return this.currentEfficiency;
    }
}

/**
 * THE 24-CELL (ICOSITETRACHORON)
 * A self-dual regular 4D polytope. 24 vertices.
 * Serves as the "Concept Anchor" for the CPE.
 */
export const get24CellVertices = (): Vector4[] => {
    const vertices: Vector4[] = [];
    
    // Set A: 8 vertices (Permutations of ±1,0,0,0)
    const axes = [
        {x:1,y:0,z:0,w:0}, {x:-1,y:0,z:0,w:0},
        {x:0,y:1,z:0,w:0}, {x:0,y:-1,z:0,w:0},
        {x:0,y:0,z:1,w:0}, {x:0,y:0,z:-1,w:0},
        {x:0,y:0,z:0,w:1}, {x:0,y:0,z:0,w:-1}
    ];
    vertices.push(...axes);

    // Set B: 16 vertices (±0.5, ±0.5, ±0.5, ±0.5)
    for(let x=-0.5; x<=0.5; x+=1) {
        for(let y=-0.5; y<=0.5; y+=1) {
            for(let z=-0.5; z<=0.5; z+=1) {
                for(let w=-0.5; w<=0.5; w+=1) {
                    vertices.push({x, y, z, w});
                }
            }
        }
    }
    return vertices;
};

/**
 * 6-DoF HYPER-ROTATION WITH PHYSICS CORRECTION
 */
export const rotate4D_Hyper = (v: Vector4, q: Quaternion, accel: Vector3): Vector4 => {
    // 1. STANDARD 3D ROTATION (The Spatial Manifold)
    let ix = q.w * v.x + q.y * v.z - q.z * v.y;
    let iy = q.w * v.y + q.z * v.x - q.x * v.z;
    let iz = q.w * v.z + q.x * v.y - q.y * v.x;
    let iw = -q.x * v.x - q.y * v.y - q.z * v.z;

    let rx = ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y;
    let ry = iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z;
    let rz = iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x;
    
    let rw = v.w; 

    // 2. DIMENSIONAL INTERFERENCE (The W-Plane Rotation)
    // Map acceleration to hyper-rotation
    const scale = 0.1;
    let tx = rx * Math.cos(accel.x * scale) - rw * Math.sin(accel.x * scale);
    let tw = rx * Math.sin(accel.x * scale) + rw * Math.cos(accel.x * scale);
    rx = tx; rw = tw;

    let ty = ry * Math.cos(accel.y * scale) - rw * Math.sin(accel.y * scale);
    tw = ry * Math.sin(accel.y * scale) + rw * Math.cos(accel.y * scale);
    ry = ty; rw = tw;

    let tz = rz * Math.cos(accel.z * scale) - rw * Math.sin(accel.z * scale);
    tw = rz * Math.sin(accel.z * scale) + rw * Math.cos(accel.z * scale);
    rz = tz; rw = tw;

    return { x: rx, y: ry, z: rz, w: rw };
};

/**
 * PERSPECTIVE 4D PROJECTION
 */
export const project4Dto2D = (
    v: Vector4, 
    width: number, 
    height: number, 
    scale: number,
    cameraDist: number
): { x: number, y: number, depth: number, wFactor: number } => {
    
    // 4D Perspective Divide
    const wFactor = 1 / (cameraDist - v.w); 
    const x3 = v.x * wFactor;
    const y3 = v.y * wFactor;
    const z3 = v.z * wFactor;

    // 3D Perspective Divide
    const fov = 1000;
    const zOffset = 2.5; 
    const zFinal = z3 + zOffset;
    
    if (zFinal <= 0.1) return {x:0, y:0, depth: -1, wFactor};

    const pScale = (fov / zFinal) * scale;
    
    return {
        x: (x3 * pScale) + (width / 2),
        y: (y3 * pScale) + (height / 2),
        depth: z3,
        wFactor: wFactor 
    };
};