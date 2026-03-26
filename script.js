// ============================================================
// MAE345: 3-2-1 Euler Angles Gimbal Visualization
// Rotation Convention: R_3(ψ) * R_2(θ) * R_1(φ)
//   - Yaw (ψ): rotation around Z-axis (axis 3, vertical) - OUTER gimbal (blue)
//   - Pitch (θ): rotation around Y-axis (axis 2, lateral) - MIDDLE gimbal (orange)
//   - Roll (φ): rotation around X-axis (axis 1, forward) - INNER gimbal (green)
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

// Matrix display elements
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
// ROTATION MATRIX FUNCTIONS (for display purposes)
// ============================================================

function fromXRotation(theta) {
    const s = Math.sin(theta);
    const c = Math.cos(theta);
    return new THREE.Matrix4().set(
        1, 0, 0, 0,
        0, c, -s, 0,
        0, s, c, 0,
        0, 0, 0, 1
    );
}

function fromYRotation(theta) {
    const s = Math.sin(theta);
    const c = Math.cos(theta);
    return new THREE.Matrix4().set(
        c, 0, s, 0,
        0, 1, 0, 0,
        -s, 0, c, 0,
        0, 0, 0, 1
    );
}

function fromZRotation(theta) {
    const s = Math.sin(theta);
    const c = Math.cos(theta);
    return new THREE.Matrix4().set(
        c, -s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    );
}

function yawPitchRoll2Rot(yaw, pitch, roll) {
    const rotYaw = fromZRotation(yaw);
    const rotPitch = fromYRotation(pitch);
    const rotRoll = fromXRotation(roll);
    
    const R = new THREE.Matrix4();
    R.multiplyMatrices(rotYaw, rotPitch);
    R.multiply(rotRoll);
    
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
// 3D SCENE SETUP - PROPER HIERARCHY
// ============================================================

function init() {
    const canvas = document.getElementById('MainGLCanvas');
    
    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);
    
    // Camera
    camera = new THREE.PerspectiveCamera(45, canvas.width / canvas.height, 0.1, 1000);
    camera.position.set(5, 3, 5);
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
    // CREATE NESTED HIERARCHY
    // 
    // Scene Graph Structure:
    //   scene
    //     └── yawGroup (rotates around Z-axis)
    //           ├── yawRing (blue outer ring, lies in XY plane)
    //           └── pitchGroup (rotates around Y-axis)
    //                 ├── pitchRing (orange middle ring, lies in XZ plane)
    //                 └── rollGroup (rotates around X-axis)
    //                       ├── rollRing (green inner ring, lies in YZ plane)
    //                       ├── airplane
    //                       └── axes
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
    
    // Create airplane (add to rollGroup so it rotates with roll)
    createAirplane();
    rollGroup.add(planeMesh);
    
    // Create axes (add to rollGroup so they rotate with the body)
    createAxes();
    rollGroup.add(axesGroup);
    
    // Start animation loop
    animate();
}

function createGimbals() {
    const tubeRadius = 0.04;
    
    // ============================================================
    // YAW GIMBAL (OUTER) - Blue ring in XY plane
    // Rotates around Z-axis (vertical)
    // This ring is a child of yawGroup, so it rotates when yaw changes
    // ============================================================
    const yawRadius = 2.0;
    const yawGeom = new THREE.TorusGeometry(yawRadius, tubeRadius, 16, 100);
    const yawMat = new THREE.MeshPhongMaterial({ color: 0x2f7bae });
    yawRing = new THREE.Mesh(yawGeom, yawMat);
    // Ring lies in XY plane by default (rotates around Z) - no rotation needed
    yawGroup.add(yawRing);
    
    // Add a small marker to show yaw direction
    const yawMarkerGeom = new THREE.SphereGeometry(tubeRadius * 2, 8, 8);
    const yawMarker = new THREE.Mesh(yawMarkerGeom, yawMat);
    yawMarker.position.set(yawRadius, 0, 0);
    yawRing.add(yawMarker);
    
    // ============================================================
    // PITCH GIMBAL (MIDDLE) - Orange ring in XZ plane
    // Rotates around Y-axis (lateral)
    // This ring is a child of pitchGroup, so it rotates when pitch changes
    // ============================================================
    const pitchRadius = 1.8;
    const pitchGeom = new THREE.TorusGeometry(pitchRadius, tubeRadius, 16, 100);
    const pitchMat = new THREE.MeshPhongMaterial({ color: 0xdb8e22 });
    pitchRing = new THREE.Mesh(pitchGeom, pitchMat);
    // Rotate ring to lie in XZ plane (so it visually shows rotation around Y)
    pitchRing.rotation.x = Math.PI / 2;
    pitchGroup.add(pitchRing);
    
    // Add marker for pitch direction
    const pitchMarkerGeom = new THREE.SphereGeometry(tubeRadius * 2, 8, 8);
    const pitchMarker = new THREE.Mesh(pitchMarkerGeom, pitchMat);
    pitchMarker.position.set(pitchRadius, 0, 0);
    pitchRing.add(pitchMarker);
    
    // ============================================================
    // ROLL GIMBAL (INNER) - Green ring in YZ plane
    // Rotates around X-axis (forward/longitudinal)
    // This ring is a child of rollGroup, so it rotates when roll changes
    // ============================================================
    const rollRadius = 1.6;
    const rollGeom = new THREE.TorusGeometry(rollRadius, tubeRadius, 16, 100);
    const rollMat = new THREE.MeshPhongMaterial({ color: 0x2c9f2c });
    rollRing = new THREE.Mesh(rollGeom, rollMat);
    // Rotate ring to lie in YZ plane (so it visually shows rotation around X)
    // Flipped 180° so natural convection has Z pointing down
    rollRing.rotation.y = Math.PI / 2;
    rollRing.rotation.x = Math.PI;
    rollGroup.add(rollRing);
    
    // Add marker for roll direction
    const rollMarkerGeom = new THREE.SphereGeometry(tubeRadius * 2, 8, 8);
    const rollMarker = new THREE.Mesh(rollMarkerGeom, rollMat);
    rollMarker.position.set(rollRadius, 0, 0);
    rollRing.add(rollMarker);
}

function createAirplane() {
    planeMesh = new THREE.Group();
    
    // Fuselage - along X-axis (nose pointing +X)
    const fuselageGeom = new THREE.CylinderGeometry(0.15, 0.1, 2, 16);
    const fuselageMat = new THREE.MeshPhongMaterial({ color: 0xcccccc });
    const fuselage = new THREE.Mesh(fuselageGeom, fuselageMat);
    fuselage.rotation.z = Math.PI / 2;  // Rotate to align with X-axis
    planeMesh.add(fuselage);
    
    // Nose cone
    const noseGeom = new THREE.ConeGeometry(0.15, 0.4, 16);
    const nose = new THREE.Mesh(noseGeom, fuselageMat);
    nose.rotation.z = -Math.PI / 2;
    nose.position.x = 1.2;
    planeMesh.add(nose);
    
    // Wings - span along Z-axis
    const wingGeom = new THREE.BoxGeometry(0.5, 0.02, 1.8);
    const wingMat = new THREE.MeshPhongMaterial({ color: 0x888888 });
    const wings = new THREE.Mesh(wingGeom, wingMat);
    wings.position.x = -0.1;
    planeMesh.add(wings);
    
    // Tail vertical stabilizer - up along Y
    const tailVGeom = new THREE.BoxGeometry(0.3, 0.4, 0.02);
    const tailV = new THREE.Mesh(tailVGeom, wingMat);
    tailV.position.set(-0.9, 0.2, 0);
    planeMesh.add(tailV);
    
    // Tail horizontal stabilizer
    const tailHGeom = new THREE.BoxGeometry(0.2, 0.02, 0.6);
    const tailH = new THREE.Mesh(tailHGeom, wingMat);
    tailH.position.set(-0.9, 0, 0);
    planeMesh.add(tailH);
}

function createAxes() {
    axesGroup = new THREE.Group();
    
    const axisLength = 0.6;
    const axisRadius = 0.03;
    
    // X-axis (Red) - Roll axis, points forward
    const xGeom = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 8);
    const xMat = new THREE.MeshPhongMaterial({ color: 0xff0000 });
    const xAxis = new THREE.Mesh(xGeom, xMat);
    xAxis.rotation.z = -Math.PI / 2;
    xAxis.position.x = axisLength / 2;
    axesGroup.add(xAxis);
    
    // X arrow head
    const xArrowGeom = new THREE.ConeGeometry(axisRadius * 2, axisRadius * 4, 8);
    const xArrow = new THREE.Mesh(xArrowGeom, xMat);
    xArrow.rotation.z = -Math.PI / 2;
    xArrow.position.x = axisLength;
    axesGroup.add(xArrow);
    
    // Y-axis (Green) - Pitch axis, points down (natural convection)
    const yGeom = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 8);
    const yMat = new THREE.MeshPhongMaterial({ color: 0x00ff00 });
    const yAxis = new THREE.Mesh(yGeom, yMat);
    yAxis.position.y = -axisLength / 2;
    axesGroup.add(yAxis);
    
    // Y arrow head
    const yArrowGeom = new THREE.ConeGeometry(axisRadius * 2, axisRadius * 4, 8);
    const yArrow = new THREE.Mesh(yArrowGeom, yMat);
    yArrow.rotation.x = Math.PI;  // Point downward
    yArrow.position.y = -axisLength;
    axesGroup.add(yArrow);
    
    // Z-axis (Blue) - Yaw axis, points right (starboard)
    const zGeom = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 8);
    const zMat = new THREE.MeshPhongMaterial({ color: 0x0000ff });
    const zAxis = new THREE.Mesh(zGeom, zMat);
    zAxis.rotation.x = Math.PI / 2;
    zAxis.position.z = axisLength / 2;
    axesGroup.add(zAxis);
    
    // Z arrow head
    const zArrowGeom = new THREE.ConeGeometry(axisRadius * 2, axisRadius * 4, 8);
    const zArrow = new THREE.Mesh(zArrowGeom, zMat);
    zArrow.rotation.x = Math.PI / 2;
    zArrow.position.z = axisLength;
    axesGroup.add(zArrow);
}

// ============================================================
// UPDATE SCENE - Simply set group rotations!
// ============================================================

function updateScene() {
    // ============================================================
    // KEY FIX: Apply rotations to the GROUP containers
    // Each group rotates around its respective axis.
    // Because of the parent-child hierarchy, rotations accumulate
    // automatically through the scene graph.
    //
    // yawGroup.rotation.z   -> rotates yawRing + everything inside
    // pitchGroup.rotation.y -> rotates pitchRing + everything inside  
    // rollGroup.rotation.x  -> rotates rollRing + airplane + axes
    // ============================================================
    
    yawGroup.rotation.z = yawAngle;      // Yaw: rotate around Z
    pitchGroup.rotation.y = pitchAngle;  // Pitch: rotate around Y
    rollGroup.rotation.x = rollAngle;    // Roll: rotate around X
    
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
    // R_3(ψ) - Yaw matrix (rotation around Z / axis 3)
    let c = Math.round(Math.cos(yawAngle) * 100) / 100;
    let s = Math.round(Math.sin(yawAngle) * 100) / 100;
    cpsi1.innerHTML = c;
    cpsi2.innerHTML = c;
    spsipos.innerHTML = s;
    spsineg.innerHTML = -s;
    
    // R_2(θ) - Pitch matrix (rotation around Y / axis 2)
    c = Math.round(Math.cos(pitchAngle) * 100) / 100;
    s = Math.round(Math.sin(pitchAngle) * 100) / 100;
    ctheta1.innerHTML = c;
    ctheta2.innerHTML = c;
    sthetapos.innerHTML = s;
    sthetaneg.innerHTML = -s;
    
    // R_1(φ) - Roll matrix (rotation around X / axis 1)
    c = Math.round(Math.cos(rollAngle) * 100) / 100;
    s = Math.round(Math.sin(rollAngle) * 100) / 100;
    cphi1.innerHTML = c;
    cphi2.innerHTML = c;
    sphipos.innerHTML = s;
    sphineg.innerHTML = -s;
    
    // Combined rotation matrix R = R_3(ψ) * R_2(θ) * R_1(φ)
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
