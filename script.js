// ============================================================
// Yaw-Pitch-Roll Gimbal Visualization
// Intrinsic 3-2-1 Euler Angles (Aerospace NED-like convention)
//
// Coordinate System (Three.js with aerospace mapping):
//   - X: Forward (out the nose)
//   - Y: Up in Three.js, but body Z points DOWN (out the belly)
//   - Z: Right (out starboard wing)
//
// Rotation Sequence:
//   1. Yaw (ψ): Rotate about VERTICAL axis (Three.js Y)
//      → Whole system spins left/right like a turntable
//      → OUTER ring (blue) rotates
//
//   2. Pitch (θ): Rotate about LATERAL axis (Three.js Z, after yaw)
//      → Nose up/down motion
//      → MIDDLE ring (orange) rotates
//
//   3. Roll (φ): Rotate about FORWARD axis (Three.js X, after yaw+pitch)
//      → Wings tilt (barrel roll)
//      → INNER ring (green) rotates
//
// Body Axes Colors:
//   - X (Red): Forward - Roll axis
//   - Y (Blue): Right - Pitch axis
//   - Z (Green): Down - Yaw axis
//
// Scene Hierarchy:
//   scene → yawGroup → pitchGroup → rollGroup → airplane
// ============================================================

// Global state
let scene, camera, renderer, controls;
let yawAngle = 0, pitchAngle = 0, rollAngle = 0;
let displayPlane = true, displayGimbals = true, displayAxes = true;

// Scene graph hierarchy groups
// Structure: scene -> yawGroup -> pitchGroup -> rollGroup -> airplane
let yawGroup, pitchGroup, rollGroup;
let yawRing, pitchRing, rollRing;
let planeMesh;
let axesGroup;

// Matrix display elements - using Greek letter notation (ψ, θ, φ)
const cpsi1 = document.getElementById("cpsi1");
const cpsi2 = document.getElementById("cpsi2");
const spsipos = document.getElementById("spsipos");
const spsineg = document.getElementById("spsineg");

const ctheta1 = document.getElementById("ctheta1");
const ctheta2 = document.getElementById("ctheta2");
const sthetapos = document.getElementById("sthetapos");
const sthetaneg = document.getElementById("sthetaneg");

const cphi1 = document.getElementById("cphi1");
const cphi2 = document.getElementById("cphi2");
const sphipos = document.getElementById("sphipos");
const sphineg = document.getElementById("sphineg");

let rEntries = [];
for (let i = 0; i < 3; i++) {
    rEntries.push([]);
    for (let j = 0; j < 3; j++) {
        rEntries[i].push(document.getElementById("r" + i + "" + j));
    }
}

// ============================================================
// DCM (Direction Cosine Matrix) FUNCTIONS
// Standard aerospace convention: [BN] = T1(φ) * T2(θ) * T3(ψ)
// These are passive rotation matrices (coordinate transformations)
// ============================================================

// T1(φ) - DCM for rotation about X-axis (Roll)
// Transforms from intermediate frame to body frame
function T1(phi) {
    const s = Math.sin(phi);
    const c = Math.cos(phi);
    return new THREE.Matrix4().set(
        1, 0,  0, 0,
        0, c,  s, 0,
        0, -s, c, 0,
        0, 0,  0, 1
    );
}

// T2(θ) - DCM for rotation about Y-axis (Pitch)
// Transforms from intermediate frame after yaw
function T2(theta) {
    const s = Math.sin(theta);
    const c = Math.cos(theta);
    return new THREE.Matrix4().set(
        c, 0, -s, 0,
        0, 1,  0, 0,
        s, 0,  c, 0,
        0, 0,  0, 1
    );
}

// T3(ψ) - DCM for rotation about Z-axis (Yaw)
// Transforms from navigation/inertial frame
function T3(psi) {
    const s = Math.sin(psi);
    const c = Math.cos(psi);
    return new THREE.Matrix4().set(
        c,  s, 0, 0,
        -s, c, 0, 0,
        0,  0, 1, 0,
        0,  0, 0, 1
    );
}

// Compute DCM [BN] = T1(φ) * T2(θ) * T3(ψ)
// This is the standard 3-2-1 Euler angle DCM
function yawPitchRoll2Rot(yaw, pitch, roll) {
    const rotRoll = T1(roll);    // T1(φ)
    const rotPitch = T2(pitch);  // T2(θ)
    const rotYaw = T3(yaw);      // T3(ψ)
    
    // [BN] = T1 * T2 * T3 (multiply left to right)
    const R = new THREE.Matrix4();
    R.multiplyMatrices(rotRoll, rotPitch);  // T1 * T2
    R.multiply(rotYaw);                      // (T1 * T2) * T3
    
    return { rotYaw, rotPitch, rotRoll, R };
}

function rot2YawPitchRoll(R) {
    const elements = R.elements;
    const R11 = elements[0], R21 = elements[1], R31 = elements[2];
    const R12 = elements[4], R22 = elements[5], R32 = elements[6];
    const R13 = elements[8], R23 = elements[9], R33 = elements[10];
    
    let pitch = -Math.asin(Math.max(-1, Math.min(1, R31)));
    let yaw = Math.atan2(R21, R11);
    let roll = Math.atan2(R32, R33);
    
    if (yaw < 0) yaw += 2 * Math.PI;
    if (pitch < 0) pitch += 2 * Math.PI;
    if (roll < 0) roll += 2 * Math.PI;
    
    return { yaw, pitch, roll };
}

function rot2Quat(R) {
    const q = new THREE.Quaternion();
    q.setFromRotationMatrix(R);
    return q;
}

function quat2Rot(q) {
    const R = new THREE.Matrix4();
    R.makeRotationFromQuaternion(q);
    return R;
}

// ============================================================
// 3D SCENE SETUP - PROPER HIERARCHY FOR INTRINSIC 3-2-1 EULER ANGLES
// ============================================================

function init() {
    const canvas = document.getElementById('MainGLCanvas');
    
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);
    
    // Camera - positioned for clear 3D view of all gimbal rotations
    // Viewing from above and to the side to see yaw (turntable), pitch (nose up/down), roll (wing tilt)
    camera = new THREE.PerspectiveCamera(45, canvas.width / canvas.height, 0.1, 1000);
    camera.position.set(4, 4, 4);
    camera.lookAt(0, 0, 0);
    
    // Renderer
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setSize(canvas.width, canvas.height);
    
    // Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    
    // Lighting
    const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 5);
    scene.add(directionalLight);
    
    const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight2.position.set(-5, -5, -5);
    scene.add(directionalLight2);
    
    // ============================================================
    // CREATE NESTED HIERARCHY FOR INTRINSIC 3-2-1 (YAW-PITCH-ROLL)
    // 
    // In Three.js, Y is the vertical axis. Each rotation occurs on updated local axes:
    //   1. Yaw (ψ): Rotate about GLOBAL Y axis (vertical - turntable motion)
    //   2. Pitch (θ): Rotate about LOCAL Z axis (lateral - nose up/down)
    //   3. Roll (φ): Rotate about LOCAL X axis (forward - barrel roll)
    //
    // Scene Graph Structure:
    //   scene
    //     └── yawGroup (rotation.y = ψ, rotates around global Y - turntable)
    //           └── pitchGroup (rotation.z = θ, rotates around local Z - nose up/down)
    //                 └── rollGroup (rotation.x = φ, rotates around local X - barrel roll)
    //                       └── airplane + axes
    //
    // The gimbal rings are visual aids - each attached to its respective group
    // ============================================================
    
    // Create the nested groups
    yawGroup = new THREE.Group();
    pitchGroup = new THREE.Group();
    rollGroup = new THREE.Group();
    
    // Build hierarchy: scene -> yawGroup -> pitchGroup -> rollGroup
    scene.add(yawGroup);
    yawGroup.add(pitchGroup);
    pitchGroup.add(rollGroup);
    
    // Create gimbal rings
    createGimbals();
    
    // Create airplane (add to rollGroup so it rotates with all three)
    createAirplane();
    rollGroup.add(planeMesh);
    
    // Create axes (add to rollGroup so they rotate with the body)
    createAxes();
    rollGroup.add(axesGroup);
    
    // Start animation loop
    animate();
}

function createGimbals() {
    // ============================================================
    // RIBBON-STYLE GIMBAL RINGS WITH SUBTLE AXIS INDICATORS
    // Rings are the primary visual; axes are faint hints
    // ============================================================
    
    const ribbonWidth = 0.12;  // Radial width of ribbon
    const ribbonDepth = 0.015; // Thickness of ribbon
    
    // Subtle rod and pivot dimensions (thin and understated)
    const rodRadius = 0.012;   // Very thin rods
    const pivotRadius = 0.025; // Tiny pivot caps
    
    // Subtle semi-transparent material for axis rods
    const subtleRodMat = new THREE.MeshBasicMaterial({
        color: 0x666666,
        transparent: true,
        opacity: 0.35
    });
    
    // Tiny muted pivot material
    const subtlePivotMat = new THREE.MeshBasicMaterial({
        color: 0x888888,
        transparent: true,
        opacity: 0.5
    });
    
    // Helper function to create a ribbon ring
    function createRibbonRing(outerRadius, color) {
        const innerRadius = outerRadius - ribbonWidth;
        
        // Create annulus shape
        const ringShape = new THREE.Shape();
        ringShape.absarc(0, 0, outerRadius, 0, Math.PI * 2, false);
        
        const holePath = new THREE.Path();
        holePath.absarc(0, 0, innerRadius, 0, Math.PI * 2, true);
        ringShape.holes.push(holePath);
        
        // Extrude for thickness
        const extrudeSettings = { depth: ribbonDepth, bevelEnabled: false };
        const geometry = new THREE.ExtrudeGeometry(ringShape, extrudeSettings);
        geometry.center();
        
        const material = new THREE.MeshPhongMaterial({ 
            color: color,
            side: THREE.DoubleSide,
            shininess: 60
        });
        return new THREE.Mesh(geometry, material);
    }
    
    // Helper function to create a tiny pivot cap
    function createPivot() {
        const geom = new THREE.SphereGeometry(pivotRadius, 8, 8);
        return new THREE.Mesh(geom, subtlePivotMat);
    }
    
    // Helper function to create a subtle rod
    function createRod(length) {
        const geom = new THREE.CylinderGeometry(rodRadius, rodRadius, length, 8);
        return new THREE.Mesh(geom, subtleRodMat);
    }
    
    // ============================================================
    // YAW GIMBAL (OUTER) - Blue ribbon, HORIZONTAL in XZ plane
    // Rotation about vertical Y axis (turntable motion)
    // ============================================================
    const yawRadius = 2.0;
    yawRing = createRibbonRing(yawRadius, 0x2f7bae);
    // Rotate to lie in XZ plane (horizontal)
    yawRing.rotation.x = Math.PI / 2;
    yawGroup.add(yawRing);
    
    // Yaw marker
    const yawMarkerGeom = new THREE.BoxGeometry(0.15, 0.08, 0.04);
    const yawMarkerMat = new THREE.MeshPhongMaterial({ color: 0x2f7bae });
    const yawMarker = new THREE.Mesh(yawMarkerGeom, yawMarkerMat);
    yawMarker.position.set(yawRadius - ribbonWidth/2, 0, 0);
    yawRing.add(yawMarker);
    
    // YAW AXIS: Very faint vertical line through center
    const yawShaftLength = 1.8;
    const yawShaft = createRod(yawShaftLength);
    yawShaft.position.y = yawShaftLength / 2 - 0.2;
    scene.add(yawShaft);
    
    // ============================================================
    // PITCH GIMBAL (MIDDLE) - Orange ribbon, vertical in XY plane
    // Rotation about lateral Z axis (nose up/down)
    // ============================================================
    const pitchRadius = 1.7;
    pitchRing = createRibbonRing(pitchRadius, 0xdb8e22);
    // Already in XY plane by default
    pitchGroup.add(pitchRing);
    
    // Pitch marker
    const pitchMarkerGeom = new THREE.BoxGeometry(0.15, 0.08, 0.04);
    const pitchMarkerMat = new THREE.MeshPhongMaterial({ color: 0xdb8e22 });
    const pitchMarker = new THREE.Mesh(pitchMarkerGeom, pitchMarkerMat);
    pitchMarker.position.set(pitchRadius - ribbonWidth/2, 0, 0);
    pitchRing.add(pitchMarker);
    
    // PITCH AXIS: Subtle thin rod across the ring (left-right, Z direction)
    const pitchRodLength = yawRadius * 2 - 0.1;
    const pitchRod = createRod(pitchRodLength);
    pitchRod.rotation.x = Math.PI / 2;  // Align along Z axis
    pitchGroup.add(pitchRod);
    
    // Tiny end caps where pitch rod meets yaw ring
    const pitchCapLeft = createPivot();
    pitchCapLeft.position.set(0, 0, -yawRadius + ribbonWidth/2);
    pitchGroup.add(pitchCapLeft);
    
    const pitchCapRight = createPivot();
    pitchCapRight.position.set(0, 0, yawRadius - ribbonWidth/2);
    pitchGroup.add(pitchCapRight);
    
    // ============================================================
    // ROLL GIMBAL (INNER) - Green ribbon, vertical in YZ plane
    // Rotation about forward X axis (barrel roll)
    // ============================================================
    const rollRadius = 1.4;
    rollRing = createRibbonRing(rollRadius, 0x2c9f2c);
    // Rotate 90° around Y to lie in YZ plane
    rollRing.rotation.y = Math.PI / 2;
    rollGroup.add(rollRing);
    
    // Roll marker
    const rollMarkerGeom = new THREE.BoxGeometry(0.15, 0.08, 0.04);
    const rollMarkerMat = new THREE.MeshPhongMaterial({ color: 0x2c9f2c });
    const rollMarker = new THREE.Mesh(rollMarkerGeom, rollMarkerMat);
    rollMarker.position.set(rollRadius - ribbonWidth/2, 0, 0);
    rollRing.add(rollMarker);
    
    // ROLL AXIS: Subtle thin rod along airplane's forward direction (X axis)
    const rollRodLength = pitchRadius * 2 - 0.1;
    const rollRod = createRod(rollRodLength);
    rollRod.rotation.z = Math.PI / 2;  // Align along X axis
    rollGroup.add(rollRod);
    
    // Tiny end caps where roll rod meets pitch ring
    const rollCapFront = createPivot();
    rollCapFront.position.set(pitchRadius - ribbonWidth/2, 0, 0);
    rollGroup.add(rollCapFront);
    
    const rollCapBack = createPivot();
    rollCapBack.position.set(-pitchRadius + ribbonWidth/2, 0, 0);
    rollGroup.add(rollCapBack);
    
    // ============================================================
    // GREEN RAY: Semi-transparent connection from Z-axis to roll gimbal
    // Connects the green Z-axis arrow (pointing down at -Y) to the green roll ring
    // Updated for longer axes (1.6 length)
    // ============================================================
    const axisEndY = -1.7;  // Where Z-axis arrow tip is (axis length + arrow)
    const ringY = -rollRadius;  // Bottom of roll ring
    const rayLength = Math.abs(ringY - axisEndY);
    const rayGeom = new THREE.PlaneGeometry(0.1, rayLength);
    const rayMat = new THREE.MeshBasicMaterial({
        color: 0x00ff00,
        transparent: true,
        opacity: 0.25,
        side: THREE.DoubleSide,
        depthWrite: false
    });
    const greenRay = new THREE.Mesh(rayGeom, rayMat);
    // Position ray between Z-axis tip and roll ring bottom
    greenRay.position.set(0, (axisEndY + ringY) / 2, 0);
    rollGroup.add(greenRay);
}

function createAirplane() {
    planeMesh = new THREE.Group();
    
    // B-2 Stealth Bomber - dark matte stealth finish
    const stealthMat = new THREE.MeshPhongMaterial({ color: 0x1a1a2e, shininess: 10 });
    const accentMat = new THREE.MeshPhongMaterial({ color: 0x16213e, shininess: 5 });
    const canopyMat = new THREE.MeshPhongMaterial({ color: 0x2d4059, shininess: 40, transparent: true, opacity: 0.7 });
    const engineMat = new THREE.MeshPhongMaterial({ color: 0x0f0f0f, shininess: 5 });
    
    // B-2 Flying Wing shape - main body
    const wingShape = new THREE.Shape();
    wingShape.moveTo(1.2, 0);
    wingShape.lineTo(0.3, 0.4);
    wingShape.lineTo(-0.6, 1.4);
    wingShape.lineTo(-0.8, 1.35);
    wingShape.lineTo(-0.7, 1.25);
    wingShape.lineTo(-0.9, 1.2);
    wingShape.lineTo(-0.5, 0.5);
    wingShape.lineTo(-0.7, 0);
    wingShape.lineTo(-0.5, -0.5);
    wingShape.lineTo(-0.9, -1.2);
    wingShape.lineTo(-0.7, -1.25);
    wingShape.lineTo(-0.8, -1.35);
    wingShape.lineTo(-0.6, -1.4);
    wingShape.lineTo(0.3, -0.4);
    wingShape.closePath();
    
    const wingExtrudeSettings = { depth: 0.08, bevelEnabled: true, bevelThickness: 0.02, bevelSize: 0.02 };
    const wingGeom = new THREE.ExtrudeGeometry(wingShape, wingExtrudeSettings);
    const mainWing = new THREE.Mesh(wingGeom, stealthMat);
    mainWing.rotation.x = Math.PI / 2;
    mainWing.position.y = 0.04;
    planeMesh.add(mainWing);
    
    // Center body bump (cockpit area)
    const bodyShape = new THREE.Shape();
    bodyShape.moveTo(0.8, 0);
    bodyShape.quadraticCurveTo(0.4, 0.25, -0.2, 0.2);
    bodyShape.lineTo(-0.3, 0);
    bodyShape.lineTo(-0.2, -0.2);
    bodyShape.quadraticCurveTo(0.4, -0.25, 0.8, 0);
    
    const bodyExtrudeSettings = { depth: 0.15, bevelEnabled: true, bevelThickness: 0.03, bevelSize: 0.03 };
    const bodyGeom = new THREE.ExtrudeGeometry(bodyShape, bodyExtrudeSettings);
    const centerBody = new THREE.Mesh(bodyGeom, accentMat);
    centerBody.rotation.x = Math.PI / 2;
    centerBody.position.set(0, 0.08, 0);
    planeMesh.add(centerBody);
    
    // Cockpit windows
    const canopyGeom = new THREE.BoxGeometry(0.3, 0.04, 0.25);
    const canopy = new THREE.Mesh(canopyGeom, canopyMat);
    canopy.position.set(0.55, 0.2, 0);
    planeMesh.add(canopy);
    
    // Engine exhausts (4 internal engines)
    const exhaustGeom = new THREE.CylinderGeometry(0.04, 0.05, 0.15, 8);
    const exhaustPositions = [
        { x: -0.55, z: 0.35 },
        { x: -0.55, z: 0.12 },
        { x: -0.55, z: -0.12 },
        { x: -0.55, z: -0.35 }
    ];
    
    exhaustPositions.forEach(pos => {
        const exhaust = new THREE.Mesh(exhaustGeom, engineMat);
        exhaust.rotation.z = Math.PI / 2;
        exhaust.position.set(pos.x, 0.04, pos.z);
        planeMesh.add(exhaust);
    });
    
    // Panel lines
    const panelMat = new THREE.MeshPhongMaterial({ color: 0x0d0d1a, shininess: 2 });
    const edgeGeom = new THREE.BoxGeometry(0.8, 0.005, 0.02);
    
    const leftEdge = new THREE.Mesh(edgeGeom, panelMat);
    leftEdge.position.set(0, 0.13, 0.6);
    leftEdge.rotation.y = -0.5;
    planeMesh.add(leftEdge);
    
    const rightEdge = new THREE.Mesh(edgeGeom, panelMat);
    rightEdge.position.set(0, 0.13, -0.6);
    rightEdge.rotation.y = 0.5;
    planeMesh.add(rightEdge);
    
    // Orient airplane so:
    // - Nose points along +X (forward / roll axis)
    // - Wings extend along ±Z (lateral)
    // - Up is +Y
    // No rotation needed - airplane is built with nose along +X
}

function createAxes() {
    axesGroup = new THREE.Group();
    
    const axisLength = 1.6;    // Longer axes to extend past the plane
    const axisRadius = 0.05;   // Thicker for visibility
    
    // ============================================================
    // BODY-FIXED AXES for 3-2-1 Euler angles (aerospace NED-like)
    // These axes rotate with the airplane body
    // Made prominent with emissive glow for visibility
    //
    // Convention:
    //   X (Red): Forward (out the nose) - Roll axis
    //   Y (Blue): Right (out starboard wing) - Pitch axis  
    //   Z (Green): Down (out the belly) - Yaw axis
    // ============================================================
    
    // X-axis (Red) - Forward/longitudinal - Roll axis
    // Points along the nose direction (+X in Three.js)
    const xGeom = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 12);
    const xMat = new THREE.MeshPhongMaterial({ 
        color: 0xff0000, 
        emissive: 0xff0000, 
        emissiveIntensity: 0.4 
    });
    const xAxis = new THREE.Mesh(xGeom, xMat);
    xAxis.rotation.z = -Math.PI / 2;
    xAxis.position.x = axisLength / 2;
    axesGroup.add(xAxis);
    
    // X arrow head
    const xArrowGeom = new THREE.ConeGeometry(axisRadius * 2.5, axisRadius * 6, 12);
    const xArrow = new THREE.Mesh(xArrowGeom, xMat);
    xArrow.rotation.z = -Math.PI / 2;
    xArrow.position.x = axisLength + axisRadius * 2;
    axesGroup.add(xArrow);
    
    // Y-axis (Blue) - Lateral/right - Pitch axis
    // Points along the right wing direction (+Z in Three.js)
    const yGeom = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 12);
    const yMat = new THREE.MeshPhongMaterial({ 
        color: 0x0066ff, 
        emissive: 0x0066ff, 
        emissiveIntensity: 0.4 
    });
    const yAxis = new THREE.Mesh(yGeom, yMat);
    yAxis.rotation.x = Math.PI / 2;
    yAxis.position.z = axisLength / 2;
    axesGroup.add(yAxis);
    
    // Y arrow head (pointing along +Z = starboard wing)
    const yArrowGeom = new THREE.ConeGeometry(axisRadius * 2.5, axisRadius * 6, 12);
    const yArrow = new THREE.Mesh(yArrowGeom, yMat);
    yArrow.rotation.x = Math.PI / 2;
    yArrow.position.z = axisLength + axisRadius * 2;
    axesGroup.add(yArrow);
    
    // Z-axis (Green) - Down/vertical - Yaw axis
    // Points down from the body (-Y in Three.js)
    const zGeom = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 12);
    const zMat = new THREE.MeshPhongMaterial({ 
        color: 0x00ff00, 
        emissive: 0x00ff00, 
        emissiveIntensity: 0.4 
    });
    const zAxis = new THREE.Mesh(zGeom, zMat);
    zAxis.position.y = -axisLength / 2;
    axesGroup.add(zAxis);
    
    // Z arrow head (pointing DOWN)
    const zArrowGeom = new THREE.ConeGeometry(axisRadius * 2.5, axisRadius * 6, 12);
    const zArrow = new THREE.Mesh(zArrowGeom, zMat);
    zArrow.rotation.x = Math.PI;  // Flip to point down
    zArrow.position.y = -axisLength - axisRadius * 2;
    axesGroup.add(zArrow);
}

// ============================================================
// UPDATE SCENE - Apply rotations in intrinsic 3-2-1 order
// ============================================================

function updateScene() {
    // ============================================================
    // INTRINSIC 3-2-1 EULER ANGLES (Yaw-Pitch-Roll)
    // Using Three.js where Y is the vertical axis
    //
    // The nested hierarchy automatically handles intrinsic rotations:
    // - yawGroup.rotation.y rotates around GLOBAL Y (vertical axis) - TURNTABLE
    // - pitchGroup.rotation.z rotates around LOCAL Z (lateral) - NOSE UP/DOWN
    // - rollGroup.rotation.x rotates around LOCAL X (forward) - BARREL ROLL
    //
    // This creates the proper body-frame rotation sequence where
    // each successive rotation is about the newly-rotated axis.
    // ============================================================
    
    // Apply rotations - each group rotates about its local axis
    // The nesting ensures proper intrinsic rotation behavior
    yawGroup.rotation.y = -yawAngle;     // Yaw (ψ): Rotate about global Y (vertical) - turntable (negated for right-hand rule)
    pitchGroup.rotation.z = pitchAngle;  // Pitch (θ): Rotate about local Z (lateral) - nose up/down
    rollGroup.rotation.x = rollAngle;    // Roll (φ): Rotate about local X (forward) - barrel roll
    
    // Update visibility
    planeMesh.visible = displayPlane;
    yawRing.visible = displayGimbals;
    pitchRing.visible = displayGimbals;
    rollRing.visible = displayGimbals;
    axesGroup.visible = displayAxes;
    
    // Update matrix display
    updateMatrixDisplay();
}

function updateMatrixDisplay() {
    // T1(φ) - Roll DCM (rotation about X-axis)
    let c = Math.round(Math.cos(rollAngle) * 100) / 100;
    let s = Math.round(Math.sin(rollAngle) * 100) / 100;
    cphi1.innerHTML = c;
    cphi2.innerHTML = c;
    sphipos.innerHTML = s;   // +sin(φ) in position (1,2)
    sphineg.innerHTML = -s;  // -sin(φ) in position (2,1)
    
    // T2(θ) - Pitch DCM (rotation about Y-axis)
    c = Math.round(Math.cos(pitchAngle) * 100) / 100;
    s = Math.round(Math.sin(pitchAngle) * 100) / 100;
    ctheta1.innerHTML = c;
    ctheta2.innerHTML = c;
    sthetapos.innerHTML = s;   // +sin(θ) in position (2,0)
    sthetaneg.innerHTML = -s;  // -sin(θ) in position (0,2)
    
    // T3(ψ) - Yaw DCM (rotation about Z-axis)
    c = Math.round(Math.cos(yawAngle) * 100) / 100;
    s = Math.round(Math.sin(yawAngle) * 100) / 100;
    cpsi1.innerHTML = c;
    cpsi2.innerHTML = c;
    spsipos.innerHTML = s;   // +sin(ψ) in position (0,1)
    spsineg.innerHTML = -s;  // -sin(ψ) in position (1,0)
    
    // Combined DCM [BN] = T1(φ) * T2(θ) * T3(ψ)
    const { R } = yawPitchRoll2Rot(yawAngle, pitchAngle, rollAngle);
    const elements = R.elements;
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            rEntries[i][j].innerHTML = Math.round(100 * elements[j * 4 + i]) / 100;
        }
    }
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// ============================================================
// UI EVENT HANDLERS
// ============================================================

const yawSlider = document.getElementById('yawSlider');
const yawTxt = document.getElementById('yawTxt');
const pitchSlider = document.getElementById('pitchSlider');
const pitchTxt = document.getElementById('pitchTxt');
const rollSlider = document.getElementById('rollSlider');
const rollTxt = document.getElementById('rollTxt');

yawSlider.addEventListener('input', function() {
    yawAngle = 2 * Math.PI * yawSlider.value / 1000.0;
    yawTxt.value = (yawAngle * 180.0 / Math.PI).toFixed(1);
    updateScene();
});

pitchSlider.addEventListener('input', function() {
    pitchAngle = 2 * Math.PI * pitchSlider.value / 1000.0;
    pitchTxt.value = (pitchAngle * 180.0 / Math.PI).toFixed(1);
    updateScene();
});

rollSlider.addEventListener('input', function() {
    rollAngle = 2 * Math.PI * rollSlider.value / 1000.0;
    rollTxt.value = (rollAngle * 180.0 / Math.PI).toFixed(1);
    updateScene();
});

function callYawSet() {
    yawAngle = Math.PI * parseFloat(yawTxt.value) / 180.0;
    yawSlider.value = yawAngle * 1000 / (2 * Math.PI);
    updateScene();
}

function callPitchSet() {
    pitchAngle = Math.PI * parseFloat(pitchTxt.value) / 180.0;
    pitchSlider.value = pitchAngle * 1000 / (2 * Math.PI);
    updateScene();
}

function callRollSet() {
    rollAngle = Math.PI * parseFloat(rollTxt.value) / 180.0;
    rollSlider.value = rollAngle * 1000 / (2 * Math.PI);
    updateScene();
}

// Checkbox controls
document.getElementById('displayPlaneCheckbox').addEventListener('change', function() {
    displayPlane = this.checked;
    updateScene();
});

document.getElementById('displayGimbalsCheckbox').addEventListener('change', function() {
    displayGimbals = this.checked;
    updateScene();
});

document.getElementById('displayAxesCheckbox').addEventListener('change', function() {
    displayAxes = this.checked;
    updateScene();
});

// Animation controls
const yaw1 = document.getElementById('yaw1');
const pitch1 = document.getElementById('pitch1');
const roll1 = document.getElementById('roll1');
const yaw2 = document.getElementById('yaw2');
const pitch2 = document.getElementById('pitch2');
const roll2 = document.getElementById('roll2');
const lerpButton = document.getElementById('lerpButton');
const slerpButton = document.getElementById('slerpButton');

function callOrientation1SetCurrent() {
    yaw1.value = yawTxt.value;
    pitch1.value = pitchTxt.value;
    roll1.value = rollTxt.value;
}

function callOrientation2SetCurrent() {
    yaw2.value = yawTxt.value;
    pitch2.value = pitchTxt.value;
    roll2.value = rollTxt.value;
}

function updateSliders() {
    yawTxt.value = (yawAngle * 180.0 / Math.PI).toFixed(1);
    yawSlider.value = yawAngle * 1000.0 / (2 * Math.PI);
    pitchTxt.value = (pitchAngle * 180.0 / Math.PI).toFixed(1);
    pitchSlider.value = pitchAngle * 1000.0 / (2 * Math.PI);
    rollTxt.value = (rollAngle * 180.0 / Math.PI).toFixed(1);
    rollSlider.value = rollAngle * 1000.0 / (2 * Math.PI);
}

// Animation state
let animating = false;
let startTime = 0;
let totalTime = 0;
let y1 = 0, p1 = 0, r1 = 0, y2 = 0, p2 = 0, r2 = 0;
let q1 = null, q2 = null;

function doALERPAnimationStep() {
    const currTime = Date.now();
    let dT = (currTime - startTime) / 1000.0;
    
    if (dT > totalTime) {
        animating = false;
        dT = 1;
    } else {
        dT = dT / totalTime;
    }
    
    yawAngle = (1 - dT) * y1 + dT * y2;
    pitchAngle = (1 - dT) * p1 + dT * p2;
    rollAngle = (1 - dT) * r1 + dT * r2;
    
    updateSliders();
    updateScene();
    
    if (animating) {
        requestAnimationFrame(doALERPAnimationStep);
    }
}

function doSLERPAnimationStep() {
    const currTime = Date.now();
    let dT = (currTime - startTime) / 1000.0;
    
    if (dT > totalTime) {
        animating = false;
        dT = 1;
    } else {
        dT = dT / totalTime;
    }
    
    const q = new THREE.Quaternion();
    q.slerpQuaternions(q1, q2, dT);
    
    const R = quat2Rot(q);
    const angles = rot2YawPitchRoll(R);
    
    yawAngle = angles.yaw;
    pitchAngle = angles.pitch;
    rollAngle = angles.roll;
    
    updateSliders();
    updateScene();
    
    if (animating) {
        requestAnimationFrame(doSLERPAnimationStep);
    }
}

function doAnimation() {
    y1 = parseFloat(yaw1.value || 0) * Math.PI / 180;
    p1 = parseFloat(pitch1.value || 0) * Math.PI / 180;
    r1 = parseFloat(roll1.value || 0) * Math.PI / 180;
    y2 = parseFloat(yaw2.value || 0) * Math.PI / 180;
    p2 = parseFloat(pitch2.value || 0) * Math.PI / 180;
    r2 = parseFloat(roll2.value || 0) * Math.PI / 180;
    
    animating = true;
    startTime = Date.now();
    
    if (lerpButton.checked) {
        totalTime = (Math.abs(y1 - y2) + Math.abs(p1 - p2) + Math.abs(r1 - r2)) / Math.PI;
        totalTime = Math.max(totalTime, 0.5);
        requestAnimationFrame(doALERPAnimationStep);
    } else {
        const R1 = yawPitchRoll2Rot(y1, p1, r1).R;
        q1 = rot2Quat(R1);
        const R2 = yawPitchRoll2Rot(y2, p2, r2).R;
        q2 = rot2Quat(R2);
        
        totalTime = q1.angleTo(q2) / Math.PI;
        totalTime = Math.max(totalTime, 0.5);
        requestAnimationFrame(doSLERPAnimationStep);
    }
}

// ============================================================
// INITIALIZATION
// ============================================================

yawTxt.value = "0";
pitchTxt.value = "0";
rollTxt.value = "0";

init();
updateScene();
