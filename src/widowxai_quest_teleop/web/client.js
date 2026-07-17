"use strict";

const statusText = document.getElementById("status");
const statusDot = document.getElementById("dot");
const enterButton = document.getElementById("enter");
const exitButton = document.getElementById("exit");
const calibrateButton = document.getElementById("calibrate");
const capturedText = document.getElementById("captured");
const sentText = document.getElementById("sent");
const coalescedText = document.getElementById("coalesced");
const calibrationText = document.getElementById("calibration");
const canvas = document.getElementById("xr-canvas");

let socket = null;
let session = null;
let referenceSpace = null;
let gl = null;
let latestPacket = null;
let sequence = 0;
let captured = 0;
let sent = 0;
let coalesced = 0;
let leftInputSource = null;
let wristOffset = JSON.parse(localStorage.getItem("widowxai.leftWristOffset") || "null");
let calibrationArmed = wristOffset === null;
let calibrationStartedAt = null;
let calibrationSamples = [];
calibrationText.textContent = wristOffset ? `${wristOffset.map((v) => v.toFixed(3)).join(", ")} m` : "Not calibrated";

function setStatus(text, ok = false) {
  statusText.textContent = text;
  statusDot.classList.toggle("ok", ok);
}

function connectRelay() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${location.host}/ws`);
  socket.onopen = async () => {
    setStatus("Relay connected", true);
    enterButton.disabled = !(navigator.xr && await navigator.xr.isSessionSupported("immersive-vr"));
    socket.send(JSON.stringify({
      type: "clock_ping",
      sequence: 0,
      client_send_monotonic_ms: performance.now(),
      client_send_epoch_ms: Date.now(),
    }));
  };
  socket.onclose = () => {
    setStatus("Relay disconnected; retrying…");
    enterButton.disabled = true;
    setTimeout(connectRelay, 750);
  };
  socket.onerror = () => setStatus("Relay connection error");
  socket.onmessage = (event) => {
    let message;
    try { message = JSON.parse(event.data); } catch (_) { return; }
    if (message.type === "haptic" && leftInputSource && leftInputSource.gamepad) {
      const actuator = leftInputSource.gamepad.hapticActuators?.[0];
      if (actuator) actuator.pulse(Math.max(0, Math.min(1, Number(message.intensity) || 0)), Number(message.duration_ms) || 40);
    }
  };
}

function buttonValue(gamepad, index) {
  const button = gamepad?.buttons?.[index];
  return button ? Number(button.value || (button.pressed ? 1 : 0)) : 0;
}

function rotationMatrix(transform) {
  const {x, y, z, w} = transform.orientation;
  return [
    1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w),
    2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w),
    2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y),
  ];
}

function shiftedPosition(transform, offset) {
  const r = rotationMatrix(transform);
  return [
    transform.position.x + r[0]*offset[0] + r[1]*offset[1] + r[2]*offset[2],
    transform.position.y + r[3]*offset[0] + r[4]*offset[1] + r[5]*offset[2],
    transform.position.z + r[6]*offset[0] + r[7]*offset[1] + r[8]*offset[2],
  ];
}

function transformObject(transform, offset = [0, 0, 0]) {
  return {
    position: shiftedPosition(transform, offset),
    orientation_xyzw: [transform.orientation.x, transform.orientation.y, transform.orientation.z, transform.orientation.w],
  };
}

function solve3x3(a, b) {
  const m = [
    [a[0], a[1], a[2], b[0]],
    [a[3], a[4], a[5], b[1]],
    [a[6], a[7], a[8], b[2]],
  ];
  for (let col = 0; col < 3; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < 3; row += 1) if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) pivot = row;
    if (Math.abs(m[pivot][col]) < 1e-9) throw new Error("insufficient wrist rotation");
    [m[col], m[pivot]] = [m[pivot], m[col]];
    const divisor = m[col][col];
    for (let j = col; j < 4; j += 1) m[col][j] /= divisor;
    for (let row = 0; row < 3; row += 1) {
      if (row === col) continue;
      const factor = m[row][col];
      for (let j = col; j < 4; j += 1) m[row][j] -= factor * m[col][j];
    }
  }
  return [m[0][3], m[1][3], m[2][3]];
}

function finishWristCalibration() {
  const n = calibrationSamples.length;
  if (n < 60) throw new Error("too few samples");
  const meanP = [0, 0, 0];
  const meanR = new Array(9).fill(0);
  for (const sample of calibrationSamples) {
    for (let i = 0; i < 3; i += 1) meanP[i] += sample.p[i] / n;
    for (let i = 0; i < 9; i += 1) meanR[i] += sample.r[i] / n;
  }
  const normal = new Array(9).fill(0);
  const rhs = [0, 0, 0];
  for (const sample of calibrationSamples) {
    const dp = sample.p.map((value, i) => value - meanP[i]);
    const dr = sample.r.map((value, i) => value - meanR[i]);
    for (let row = 0; row < 3; row += 1) {
      for (let col = 0; col < 3; col += 1) {
        let sum = 0;
        for (let k = 0; k < 3; k += 1) sum += dr[k*3 + row] * dr[k*3 + col];
        normal[row*3 + col] += sum;
      }
      for (let k = 0; k < 3; k += 1) rhs[row] += dr[k*3 + row] * dp[k];
    }
  }
  wristOffset = solve3x3(normal, rhs.map((value) => -value));
  const norm = Math.hypot(...wristOffset);
  if (!Number.isFinite(norm) || norm > 0.20) throw new Error("offset failed sanity check");
  localStorage.setItem("widowxai.leftWristOffset", JSON.stringify(wristOffset));
  calibrationText.textContent = `${wristOffset.map((v) => v.toFixed(3)).join(", ")} m`;
  calibrationArmed = false;
  calibrationStartedAt = null;
  calibrationSamples = [];
  setStatus("Wrist calibrated; streaming", true);
}

function onXRFrame(frameTime, frame) {
  session.requestAnimationFrame(onXRFrame);
  gl.bindFramebuffer(gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
  gl.clearColor(0.025, 0.055, 0.10, 1.0);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  leftInputSource = Array.from(session.inputSources).find((source) => source.handedness === "left" && source.gripSpace) || null;
  const rightInputSource = Array.from(session.inputSources).find((source) => source.handedness === "right" && source.gripSpace) || null;
  const controllerPose = leftInputSource ? frame.getPose(leftInputSource.gripSpace, referenceSpace) : null;
  if (!controllerPose) return;
  const bothGrips = buttonValue(leftInputSource.gamepad, 1) >= 0.7 && buttonValue(rightInputSource?.gamepad, 1) >= 0.7;
  if (calibrationArmed && calibrationStartedAt === null && bothGrips) {
    calibrationStartedAt = frameTime;
    calibrationSamples = [];
    setStatus("Calibrating: twist both wrists, keep wrist pivots still", true);
  }
  if (calibrationStartedAt !== null) {
    calibrationSamples.push({
      p: [controllerPose.transform.position.x, controllerPose.transform.position.y, controllerPose.transform.position.z],
      r: rotationMatrix(controllerPose.transform),
    });
    if (frameTime - calibrationStartedAt >= 5000) {
      try { finishWristCalibration(); }
      catch (error) {
        calibrationStartedAt = null;
        calibrationSamples = [];
        setStatus(`Wrist calibration failed: ${error.message}`);
      }
    }
    return;
  }
  const viewerPose = frame.getViewerPose(referenceSpace);
  const nextPacket = {
    type: "pose",
    schema_version: 1,
    sequence: sequence++,
    capture_monotonic_ms: frameTime,
    capture_epoch_ms: performance.timeOrigin + frameTime,
    enqueue_monotonic_ms: performance.now(),
    send_monotonic_ms: 0,
    left: {
      ...transformObject(controllerPose.transform, wristOffset || [0, 0, 0]),
      grip: buttonValue(leftInputSource.gamepad, 1),
      trigger: buttonValue(leftInputSource.gamepad, 0),
    },
    head: viewerPose ? transformObject(viewerPose.transform) : null,
  };
  if (latestPacket !== null) coalesced += 1;
  latestPacket = nextPacket;
  captured += 1;
  capturedText.textContent = captured;
  coalescedText.textContent = coalesced;
}

setInterval(() => {
  if (!latestPacket || !socket || socket.readyState !== WebSocket.OPEN) return;
  if (socket.bufferedAmount > 2048) return;
  const packet = latestPacket;
  latestPacket = null;
  packet.send_monotonic_ms = performance.now();
  socket.send(JSON.stringify(packet));
  sent += 1;
  sentText.textContent = sent;
}, 1000 / 120);

enterButton.addEventListener("click", async () => {
  try {
    session = await navigator.xr.requestSession("immersive-vr", { requiredFeatures: ["local-floor"] });
    gl = canvas.getContext("webgl", { xrCompatible: true, antialias: false });
    await gl.makeXRCompatible();
    session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });
    referenceSpace = await session.requestReferenceSpace("local-floor");
    session.addEventListener("end", () => {
      session = null;
      latestPacket = null;
      enterButton.disabled = false;
      exitButton.disabled = true;
      setStatus("Relay connected; VR stopped", true);
    });
    enterButton.disabled = true;
    exitButton.disabled = false;
    setStatus("Streaming left controller", true);
    session.requestAnimationFrame(onXRFrame);
  } catch (error) {
    setStatus(`Unable to enter VR: ${error.message}`);
  }
});

exitButton.addEventListener("click", () => session?.end());
calibrateButton.addEventListener("click", () => {
  calibrationArmed = true;
  calibrationStartedAt = null;
  calibrationSamples = [];
  calibrationText.textContent = "Armed: squeeze both grips in VR";
  setStatus("Enter VR, squeeze both grips, then twist wrists for 5 seconds", true);
});
connectRelay();
