import "./style.css";

import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import URDFLoader from "urdf-loader";

const viewport = document.querySelector("#viewport");
const statusEl = document.querySelector("#status");
const tableBody = document.querySelector("#joint-table");
const connectionDetail = document.querySelector("#connection-detail");
const mappingPanel = document.querySelector("#mapping-panel");
const mappingTable = document.querySelector("#mapping-table");
const captureAnchorButton = document.querySelector("#capture-anchor");
const enableAllButton = document.querySelector("#enable-all");
const disableAllButton = document.querySelector("#disable-all");
const resetMapButton = document.querySelector("#reset-map");
const saveMappingButton = document.querySelector("#save-mapping");
const exportMappingButton = document.querySelector("#export-mapping");
const mappingYaml = document.querySelector("#mapping-yaml");
const alphaInput = document.querySelector("#map-alpha");
const maxStepInput = document.querySelector("#map-max-step");

const showVisual = document.querySelector("#show-visual");
const showCollision = document.querySelector("#show-collision");
const showAxes = document.querySelector("#show-axes");
const showGrid = document.querySelector("#show-grid");

const params = new URLSearchParams(window.location.search);
const mode = params.get("mode") || "slave_state";
const wsUrl = params.get("ws") || "ws://127.0.0.1:8767";
const urdfPath = params.get("urdf") || "/@fs/" + encodePath(`${projectRootPath()}/assets/pink_slave_urdf/urdf/kaka_arm_v7.urdf`);
const packageRoot = params.get("packageRoot") || "/@fs/" + encodePath(`${projectRootPath()}/assets/pink_slave_urdf`);

connectionDetail.textContent = `WebSocket: ${wsUrl}\nURDF: ${urdfPath}\npackage://kaka_arm_v7 -> ${packageRoot}`;

let robot = null;
let latestJoints = Object.fromEntries(Array.from({ length: 7 }, (_, index) => [`joint${index + 1}`, 0]));
let latestMasterJoints = Object.fromEntries(Array.from({ length: 7 }, (_, index) => [`joint${index + 1}`, 0]));
let latestSlaveCurrentJoints = Object.fromEntries(Array.from({ length: 7 }, (_, index) => [`joint${index + 1}`, null]));
let continuousMasterJoints = {};
let lastRawMasterJoints = {};
let lastPacketTime = 0;
let urdfJointToRobotJoint = new Map();
let virtualTeleopConfig = null;
let masterAnchor = {};
let filteredTargets = {};
let targetLimitStatus = {};
let mappingRowsBuilt = false;
let wsConnection = null;
const defaultMappingSigns = {
  joint1: -1,
  joint2: 1,
  joint3: 1,
  joint4: -1,
  joint5: -1,
  joint6: -1,
  joint7: -1,
};
const mappingStorageKey = "pink_master_virtual_slave_mapping_v2";

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111318);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 20);
camera.position.set(0.55, -0.85, 0.55);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.shadowMap.enabled = true;
viewport.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.0, 0.0, 0.24);
controls.enableDamping = true;

const hemi = new THREE.HemisphereLight(0xffffff, 0x303645, 2.0);
scene.add(hemi);

const key = new THREE.DirectionalLight(0xffffff, 2.3);
key.position.set(0.7, -1.0, 1.2);
key.castShadow = true;
scene.add(key);

const fill = new THREE.DirectionalLight(0x9fc5ff, 0.9);
fill.position.set(-0.8, 0.7, 0.8);
scene.add(fill);

// Three.js 的 GridHelper 默认就在 XZ 地面平面上，Y 为高度方向。
// 不要再旋转它，否则会变成竖直的“墙面网格”。
const grid = new THREE.GridHelper(1.2, 24, 0x415066, 0x293241);
scene.add(grid);

const axes = new THREE.AxesHelper(0.25);
scene.add(axes);

const ghostMaterial = new THREE.MeshStandardMaterial({
  color: 0x8fa7e8,
  metalness: 0.18,
  roughness: 0.48,
});
const gripperMaterial = new THREE.MeshStandardMaterial({
  color: 0xf1aaaa,
  metalness: 0.1,
  roughness: 0.55,
});

buildJointTable();
if (mode === "master_teleop" || mode === "real_slave_dry_run") {
  mappingPanel.classList.remove("hidden");
}
loadRobot();
connectSocket();
animate();

showVisual.addEventListener("change", updateMeshVisibility);
showCollision.addEventListener("change", updateMeshVisibility);
showAxes.addEventListener("change", () => { axes.visible = showAxes.checked; });
showGrid.addEventListener("change", () => { grid.visible = showGrid.checked; });
window.addEventListener("resize", resize);
captureAnchorButton.addEventListener("click", captureMasterAnchor);
enableAllButton.addEventListener("click", () => setAllMappingEnabled(true));
disableAllButton.addEventListener("click", () => setAllMappingEnabled(false));
resetMapButton.addEventListener("click", resetMapping);
saveMappingButton.addEventListener("click", saveMappingToServer);
exportMappingButton.addEventListener("click", exportMappingYaml);
resize();

function projectRootPath() {
  const marker = "/web_viewer/pink_slave_3d";
  const path = window.__PROJECT_ROOT__;
  if (path) return path;
  // Vite /@fs/ supports absolute Windows paths. This fallback is replaced by
  // the Python starter through query parameters when needed.
  return new URL("../../..", import.meta.url).pathname.replace(/^\/([A-Za-z]:)/, "$1").replace(/\/$/, "");
}

function encodePath(path) {
  return path.replace(/\\/g, "/").split("/").map((part, index) => {
    if (index === 0 && /^[A-Za-z]:$/.test(part)) return part;
    return encodeURIComponent(part);
  }).join("/");
}

function loadRobot() {
  const manager = new THREE.LoadingManager();
  const stlLoader = new STLLoader(manager);
  const loader = new URDFLoader(manager);
  loader.packages = {
    kaka_arm_v7: packageRoot.endsWith("/") ? packageRoot : `${packageRoot}/`,
  };
  loader.loadMeshCb = (path, managerForMesh, done) => {
    stlLoader.load(
      path,
      (geometry) => {
        geometry.computeVertexNormals();
        const material = /GL\.STL|GR\.STL|L7\.STL/i.test(path) ? gripperMaterial : ghostMaterial;
        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        done(mesh);
      },
      undefined,
      (error) => {
        console.error("STL load failed", path, error);
        done(null, error);
      },
    );
  };

  statusEl.textContent = "loading URDF/STL...";
  loader.load(
    urdfPath,
    (loadedRobot) => {
      robot = loadedRobot;
      robot.rotation.x = -Math.PI / 2;
      scene.add(robot);
      buildJointMap();
      placeGroundGridUnderRobot();
      updateMeshVisibility();
      applyJointState();
      statusEl.textContent = "URDF loaded, waiting for joint states...";
    },
    undefined,
    (error) => {
      console.error(error);
      statusEl.textContent = `URDF load failed: ${error.message || error}`;
    },
  );
}

function placeGroundGridUnderRobot() {
  if (!robot) return;
  robot.updateMatrixWorld(true);
  const box = new THREE.Box3().setFromObject(robot);
  if (box.isEmpty()) return;

  const center = new THREE.Vector3();
  const size = new THREE.Vector3();
  box.getCenter(center);
  box.getSize(size);

  // 把地面网格放到模型最低点稍下方，看起来像工作台/地面，而不是穿过机械臂。
  grid.position.set(center.x, box.min.y - 0.01, center.z);
  grid.scale.setScalar(Math.max(1.0, Math.max(size.x, size.z) * 1.8));

  controls.target.copy(center);
  camera.position.set(center.x + 0.65, center.y + 0.42, center.z + 0.75);
  controls.update();
}

function buildJointMap() {
  urdfJointToRobotJoint.clear();
  for (let index = 1; index <= 7; index += 1) {
    const urdfName = `J${index}`;
    const displayName = `joint${index}`;
    if (robot.joints[urdfName]) {
      urdfJointToRobotJoint.set(displayName, robot.joints[urdfName]);
    }
  }
}

function connectSocket() {
  const ws = new WebSocket(wsUrl);
  wsConnection = ws;
  ws.addEventListener("open", () => {
    statusEl.textContent = "WebSocket connected";
  });
  ws.addEventListener("close", () => {
    statusEl.textContent = "WebSocket disconnected, retrying...";
    setTimeout(connectSocket, 1000);
  });
  ws.addEventListener("error", () => {
    statusEl.textContent = "WebSocket error";
  });
  ws.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "urdf_info") return;
    if (message.type === "virtual_teleop_config") {
      handleVirtualTeleopConfig(message);
      return;
    }
    if (message.type === "mapping_saved") {
      statusEl.textContent = `mapping saved\n${message.path}`;
      return;
    }
    if (message.type === "error") {
      statusEl.textContent = `server error\n${message.message}`;
      return;
    }
    if (message.type === "master_state") {
      handleMasterState(message);
      return;
    }
    if (message.type === "slave_state") {
      handleSlaveState(message);
      return;
    }
    if (!message.joints) return;
    latestJoints = { ...latestJoints, ...message.joints };
    lastPacketTime = message.timestamp || Date.now() / 1000;
    applyJointState();
    updateJointTable();
  });
}

function handleVirtualTeleopConfig(message) {
  virtualTeleopConfig = loadLocalMapping(message);
  alphaInput.value = virtualTeleopConfig.runtime?.alpha ?? 0.35;
  maxStepInput.value = virtualTeleopConfig.runtime?.max_step_rad ?? 0.06;
  buildMappingTable();
  const label = virtualTeleopConfig.mode_label || "pink_master -> virtual pink_slave";
  const safety = mode === "real_slave_dry_run"
    ? "Dry Run: real slave read-only; no motion command"
    : "Virtual only: no real slave command";
  connectionDetail.textContent = `${connectionDetail.textContent}\nMode: ${label}\n${safety}`;
}

function handleMasterState(message) {
  if (mode !== "master_teleop" && mode !== "real_slave_dry_run") return;
  latestMasterJoints = { ...latestMasterJoints, ...message.joints };
  updateContinuousMasterJoints(message.joints || {});
  if (!virtualTeleopConfig) return;
  if (Object.keys(masterAnchor).length === 0) {
    captureMasterAnchor();
  }
  latestJoints = computeVirtualTargets();
  lastPacketTime = message.timestamp || Date.now() / 1000;
  applyJointState();
  updateJointTable();
  updateMappingTable();
}

function handleSlaveState(message) {
  latestSlaveCurrentJoints = { ...latestSlaveCurrentJoints, ...message.joints };
  updateMappingTable();
}

function applyJointState() {
  if (!robot) return;
  for (const [displayName, value] of Object.entries(latestJoints)) {
    const joint = urdfJointToRobotJoint.get(displayName);
    if (joint) {
      joint.setJointValue(value);
    }
  }
}

function updateMeshVisibility() {
  if (!robot) return;
  robot.traverse((child) => {
    if (!child.isMesh) return;
    const name = `${child.name || ""} ${child.parent?.name || ""}`;
    const looksCollision = /collision/i.test(name);
    child.visible = looksCollision ? showCollision.checked : showVisual.checked;
  });
}

function buildJointTable() {
  tableBody.innerHTML = "";
  for (let index = 1; index <= 7; index += 1) {
    const tr = document.createElement("tr");
    tr.dataset.joint = `joint${index}`;
    tr.innerHTML = `<td>joint${index}<br><small>J${index}</small></td><td class="rad">0.0000</td><td class="deg">0.0°</td>`;
    tableBody.appendChild(tr);
  }
}

function updateJointTable() {
  for (const [name, value] of Object.entries(latestJoints)) {
    const row = tableBody.querySelector(`tr[data-joint="${name}"]`);
    if (!row) continue;
    row.querySelector(".rad").textContent = Number(value).toFixed(4);
    row.querySelector(".deg").textContent = `${(Number(value) * 180 / Math.PI).toFixed(1)}°`;
  }
}

function loadLocalMapping(message) {
  if (message.locked_config) return message;
  const saved = window.localStorage.getItem(mappingStorageKey);
  if (!saved) return message;
  try {
    const parsed = JSON.parse(saved);
    const mergedMappings = { ...message.mappings };
    for (const [name, savedMap] of Object.entries(parsed.mappings || {})) {
      const serverMap = message.mappings[name] || {};
      mergedMappings[name] = {
        ...serverMap,
        enabled: savedMap.enabled ?? serverMap.enabled,
        scale: savedMap.scale ?? serverMap.scale,
        sign: savedMap.sign ?? serverMap.sign,
        offset: savedMap.offset ?? serverMap.offset,
      };
    }
    return {
      ...message,
      mappings: mergedMappings,
      runtime: { ...message.runtime, ...parsed.runtime },
    };
  } catch {
    return message;
  }
}

function saveLocalMapping() {
  if (!virtualTeleopConfig) return;
  if (virtualTeleopConfig.locked_config) return;
  window.localStorage.setItem(
    mappingStorageKey,
    JSON.stringify({
      mappings: virtualTeleopConfig.mappings,
      runtime: {
        alpha: Number(alphaInput.value),
        max_step_rad: Number(maxStepInput.value),
      },
    }),
  );
}

function currentMappingPayload() {
  if (!virtualTeleopConfig) return null;
  syncMappingFromTable();
  return {
    type: "save_virtual_mapping",
    master_arm: "pink_master",
    slave_arm: "pink_slave",
    mappings: virtualTeleopConfig.mappings,
    runtime: {
      alpha: Number(alphaInput.value),
      max_step_rad: Number(maxStepInput.value),
    },
  };
}

function saveMappingToServer() {
  const payload = currentMappingPayload();
  if (!payload) return;
  if (!wsConnection || wsConnection.readyState !== WebSocket.OPEN) {
    statusEl.textContent = "WebSocket not connected, cannot save mapping";
    return;
  }
  wsConnection.send(JSON.stringify(payload));
  statusEl.textContent = "saving mapping...";
}

function exportMappingYaml() {
  const payload = currentMappingPayload();
  if (!payload) return;
  mappingYaml.value = mappingPayloadToYaml(payload);
}

function mappingPayloadToYaml(payload) {
  const lines = [
    "# Pink master -> pink slave mapping verified in virtual teleop.",
    "# Copy into configs/teleop_mapping.yaml if needed.",
    "master_arm: pink_master",
    "slave_arm: pink_slave",
    "mode: master_to_slave_verified_virtual",
    "runtime:",
    `  alpha: ${payload.runtime.alpha}`,
    `  max_step_rad: ${payload.runtime.max_step_rad}`,
    "joints:",
  ];
  for (let index = 1; index <= 7; index += 1) {
    const name = `joint${index}`;
    const item = payload.mappings[name] || {};
    lines.push(`  - master_joint: ${name}`);
    lines.push(`    slave_joint: ${name}`);
    lines.push(`    enabled: ${Boolean(item.enabled)}`);
    lines.push(`    scale: ${Number(item.scale ?? 1)}`);
    lines.push(`    sign: ${Number(item.sign ?? defaultMappingSigns[name])}`);
    lines.push(`    offset_rad: ${Number(item.offset ?? 0)}`);
    if (item.mapping_mode === "zero_delta") {
      lines.push("    mapping_mode: zero_delta");
    }
    if (item.mapping_mode === "range") {
      lines.push("    mapping_mode: range");
      lines.push(`    master_min_rad: ${Number(item.master_min_rad)}`);
      lines.push(`    master_max_rad: ${Number(item.master_max_rad)}`);
      lines.push(`    slave_min_rad: ${Number(item.slave_min_rad)}`);
      lines.push(`    slave_max_rad: ${Number(item.slave_max_rad)}`);
    }
  }
  return `${lines.join("\n")}\n`;
}

function buildMappingTable() {
  if (!virtualTeleopConfig) return;
  mappingRowsBuilt = true;
  mappingTable.innerHTML = "";
  for (let index = 1; index <= 7; index += 1) {
    const name = `joint${index}`;
    const map = virtualTeleopConfig.mappings[name] || { enabled: true, scale: 1, sign: 1, offset: 0 };
    const tr = document.createElement("tr");
    tr.dataset.joint = name;
    tr.innerHTML = `
      <td>${name}</td>
      <td><input class="map-enabled" type="checkbox" ${map.enabled ? "checked" : ""}></td>
      <td class="master">0.0000</td>
      <td class="slave-current">--</td>
      <td class="target">0.0000</td>
      <td><input class="map-scale" type="number" step="0.05" value="${map.scale}"></td>
      <td>
        <select class="map-sign">
          <option value="1" ${map.sign >= 0 ? "selected" : ""}>+1</option>
          <option value="-1" ${map.sign < 0 ? "selected" : ""}>-1</option>
        </select>
      </td>
      <td><input class="map-offset" type="number" step="0.01" value="${map.offset || 0}"></td>
      <td class="limit">-</td>
    `;
    mappingTable.appendChild(tr);
  }
  mappingTable.addEventListener("input", syncMappingFromTable);
}

function syncMappingFromTable() {
  if (!virtualTeleopConfig) return;
  for (const row of mappingTable.querySelectorAll("tr[data-joint]")) {
    const name = row.dataset.joint;
    const previous = virtualTeleopConfig.mappings[name] || {};
    virtualTeleopConfig.mappings[name] = {
      enabled: row.querySelector(".map-enabled").checked,
      scale: Number(row.querySelector(".map-scale").value),
      sign: Number(row.querySelector(".map-sign").value),
      offset: Number(row.querySelector(".map-offset").value),
      mapping_mode: previous.mapping_mode || "anchor_delta",
      master_min_rad: previous.master_min_rad,
      master_max_rad: previous.master_max_rad,
      slave_min_rad: previous.slave_min_rad,
      slave_max_rad: previous.slave_max_rad,
    };
  }
  saveLocalMapping();
}

function captureMasterAnchor() {
  if (Object.keys(continuousMasterJoints).length === 0) {
    updateContinuousMasterJoints(latestMasterJoints);
  }
  masterAnchor = { ...continuousMasterJoints };
  if (mode === "real_slave_dry_run" && virtualTeleopConfig) {
    virtualTeleopConfig.slave_anchor = Object.fromEntries(
      Object.entries(latestSlaveCurrentJoints)
        .filter(([, value]) => Number.isFinite(Number(value)))
        .map(([name, value]) => [name, Number(value)]),
    );
  }
  filteredTargets = {};
  targetLimitStatus = {};
  if (virtualTeleopConfig) {
    for (const name of Object.keys(virtualTeleopConfig.mappings)) {
      filteredTargets[name] = slaveBaseForJoint(name);
      targetLimitStatus[name] = "OK";
    }
  }
}

function resetMapping() {
  if (!virtualTeleopConfig) return;
  for (let index = 1; index <= 7; index += 1) {
    const name = `joint${index}`;
    const previous = virtualTeleopConfig.mappings[name] || {};
    virtualTeleopConfig.mappings[name] = {
      enabled: true,
      scale: 1,
      sign: defaultMappingSigns[name],
      offset: 0,
      mapping_mode: previous.mapping_mode || "anchor_delta",
      master_min_rad: previous.master_min_rad,
      master_max_rad: previous.master_max_rad,
      slave_min_rad: previous.slave_min_rad,
      slave_max_rad: previous.slave_max_rad,
    };
  }
  window.localStorage.removeItem(mappingStorageKey);
  buildMappingTable();
  captureMasterAnchor();
}

function setAllMappingEnabled(enabled) {
  if (!virtualTeleopConfig) return;
  for (const map of Object.values(virtualTeleopConfig.mappings)) {
    map.enabled = enabled;
  }
  for (const input of mappingTable.querySelectorAll(".map-enabled")) {
    input.checked = enabled;
  }
  saveLocalMapping();
}

function computeVirtualTargets() {
  const targets = {};
  const statuses = {};
  const alpha = clamp(Number(alphaInput.value), 0, 1);
  const maxStep = Math.max(0, Number(maxStepInput.value));
  if (virtualTeleopConfig) {
    virtualTeleopConfig.runtime = { alpha, max_step_rad: maxStep };
  }
  for (let index = 1; index <= 7; index += 1) {
    const name = `joint${index}`;
    const map = virtualTeleopConfig.mappings[name] || { enabled: false, scale: 1, sign: 1, offset: 0 };
    const slaveBase = slaveBaseForJoint(name);
    if (!map.enabled) {
      targets[name] = filteredTargets[name] ?? slaveBase;
      statuses[name] = "DISABLED";
      continue;
    }
    const masterNow = continuousMasterJoints[name] ?? latestMasterJoints[name] ?? 0;
    const masterBase = masterAnchor[name] ?? masterNow;
    const rawDelta = masterNow - masterBase;
    let rawTarget = slaveBase + map.sign * map.scale * rawDelta + (map.offset || 0);
    if (map.mapping_mode === "zero_delta") {
      const masterZero = Number(virtualTeleopConfig?.master_zero?.[name]);
      const slaveZero = Number(virtualTeleopConfig?.slave_zero?.[name]);
      if (Number.isFinite(masterZero) && Number.isFinite(slaveZero)) {
        rawTarget = slaveZero + map.sign * map.scale * (masterNow - masterZero) + (map.offset || 0);
      }
    }
    if (map.mapping_mode === "range") {
      const masterMin = Number(map.master_min_rad);
      const masterMax = Number(map.master_max_rad);
      const slaveMin = Number(map.slave_min_rad);
      const slaveMax = Number(map.slave_max_rad);
      if (
        Number.isFinite(masterMin)
        && Number.isFinite(masterMax)
        && Number.isFinite(slaveMin)
        && Number.isFinite(slaveMax)
        && Math.abs(masterMax - masterMin) > 1e-6
      ) {
        let ratio = clamp((masterNow - masterMin) / (masterMax - masterMin), 0, 1);
        if (map.sign < 0) ratio = 1 - ratio;
        rawTarget = slaveMin + ratio * (slaveMax - slaveMin) + (map.offset || 0);
      }
    }
    const previous = filteredTargets[name] ?? slaveBase;
    const filtered = previous + alpha * (rawTarget - previous);
    const stepped = limitStep(previous, filtered, maxStep);
    const limited = clampToSlaveLimit(name, stepped);
    targets[name] = limited.target;
    filteredTargets[name] = limited.target;
    statuses[name] = limited.status;
  }
  targetLimitStatus = statuses;
  saveLocalMapping();
  return targets;
}

function updateMappingTable() {
  if (!mappingRowsBuilt || !virtualTeleopConfig) return;
  for (const row of mappingTable.querySelectorAll("tr[data-joint]")) {
    const name = row.dataset.joint;
    const target = latestJoints[name] || 0;
    const master = latestMasterJoints[name] || 0;
    const slaveCurrent = latestSlaveCurrentJoints[name];
    const status = targetLimitStatus[name] || "OK";
    const isOk = status === "OK" || status === "DISABLED";
    row.querySelector(".master").textContent = master.toFixed(4);
    row.querySelector(".slave-current").textContent = Number.isFinite(Number(slaveCurrent))
      ? Number(slaveCurrent).toFixed(4)
      : "--";
    row.querySelector(".target").textContent = target.toFixed(4);
    const limitCell = row.querySelector(".limit");
    limitCell.textContent = status;
    limitCell.className = `limit ${isOk ? "ok" : "warn"}`;
  }
}

function slaveBaseForJoint(name) {
  if (Number.isFinite(Number(virtualTeleopConfig?.slave_anchor?.[name]))) {
    return Number(virtualTeleopConfig.slave_anchor[name]);
  }
  if (Number.isFinite(Number(virtualTeleopConfig?.slave_zero?.[name]))) {
    return Number(virtualTeleopConfig.slave_zero[name]);
  }
  return 0;
}

function updateContinuousMasterJoints(rawJoints) {
  for (const [name, rawValue] of Object.entries(rawJoints)) {
    const raw = Number(rawValue);
    if (!Number.isFinite(raw)) continue;

    if (!(name in lastRawMasterJoints) || !(name in continuousMasterJoints)) {
      lastRawMasterJoints[name] = raw;
      continuousMasterJoints[name] = raw;
      continue;
    }

    let delta = raw - lastRawMasterJoints[name];
    while (delta > Math.PI) delta -= Math.PI * 2;
    while (delta < -Math.PI) delta += Math.PI * 2;
    continuousMasterJoints[name] += delta;
    lastRawMasterJoints[name] = raw;
  }
}

function limitStep(previous, target, maxStep) {
  if (maxStep <= 0) return target;
  const delta = target - previous;
  if (delta > maxStep) return previous + maxStep;
  if (delta < -maxStep) return previous - maxStep;
  return target;
}

function clampToSlaveLimit(name, value) {
  const limit = virtualTeleopConfig?.slave_limits?.[name];
  if (!limit) return { target: value, status: "NO_LIMIT" };
  if (value < limit.min) return { target: limit.min, status: "LIMIT_MIN" };
  if (value > limit.max) return { target: limit.max, status: "LIMIT_MAX" };
  return { target: value, status: "OK" };
}

function clamp(value, low, high) {
  return Math.max(low, Math.min(high, value));
}

function resize() {
  const { clientWidth, clientHeight } = viewport;
  renderer.setSize(clientWidth, clientHeight, false);
  camera.aspect = clientWidth / Math.max(1, clientHeight);
  camera.updateProjectionMatrix();
}

function animate() {
  controls.update();
  const age = lastPacketTime ? Date.now() / 1000 - lastPacketTime : 0;
  if (lastPacketTime) {
    statusEl.textContent = `WebSocket connected\npacket age: ${age.toFixed(2)}s`;
  }
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
