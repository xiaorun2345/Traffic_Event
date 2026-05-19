const mainFields = [
  "CameraConfig", "RsuIp", "RsuPort", "CloudIp", "CloudPort", "CameraURI",
  "CameraShow", "SendTime", "LedWarningText", "LedIp", "LedPort",
  "LedSdkKey", "LedSdkSecret", "LedDeviceId", "RtspPushPort", "RtspTransport",
  "SourceTimeoutSec", "ReconnectTimes", "CameraLength", "CameraWidth",
  "location_angle", "location_longitude", "location_latitude", "IsGcj02"
];

const fieldNames = {
  CameraConfig: "视觉配置文件",
  RsuIp: "RSU IP",
  RsuPort: "RSU 端口",
  CloudIp: "云平台 IP",
  CloudPort: "云平台端口",
  CameraURI: "摄像头/视频地址",
  CameraShow: "显示模式",
  SendTime: "上报周期 ms",
  LedWarningText: "默认预警文字",
  LedIp: "LED IP",
  LedPort: "LED 端口",
  LedSdkKey: "LED SDK Key",
  LedSdkSecret: "LED SDK Secret",
  LedDeviceId: "LED 设备 ID",
  RtspPushPort: "带框 RTSP 端口",
  RtspTransport: "RTSP 协议",
  SourceTimeoutSec: "无帧超时秒数",
  ReconnectTimes: "重连次数",
  CameraLength: "图像高度",
  CameraWidth: "图像宽度",
  location_angle: "安装航向角",
  location_longitude: "安装经度",
  location_latitude: "安装纬度",
  IsGcj02: "是否 GCJ02",
  model_engine_file: "模型路径",
  class_names: "类别列表",
  network_mode: "量化模式",
  idx_npu: "NPU",
  nms_iou_threshold: "NMS 阈值",
  "pre-cluster-threshold": "检测置信度",
  low_thresh: "低分跟踪阈值",
  high_thresh: "高分跟踪阈值",
  high_thresh_person: "行人建轨阈值",
  high_thresh_motorbike: "非机动车建轨阈值",
  max_iou_thresh: "IOU 匹配阈值"
};

const gieFields = ["model_engine_file", "class_names", "network_mode", "idx_npu", "nms_iou_threshold", "pre-cluster-threshold"];
const trackerFields = ["low_thresh", "high_thresh", "high_thresh_person", "high_thresh_motorbike", "max_iou_thresh"];
let mainConfig = {};
let modelConfig = { gie: {}, tracker: {} };
let profiles = [];
let models = [];

document.querySelectorAll(".tabs button").forEach((button) => {
  button.addEventListener("click", () => {
    document.querySelectorAll(".tabs button").forEach((item) => item.classList.remove("active"));
    document.querySelectorAll(".panel").forEach((item) => item.classList.remove("active"));
    button.classList.add("active");
    document.getElementById(button.dataset.tab).classList.add("active");
  });
});

async function api(path, options = {}) {
  const init = { ...options };
  if (init.body && typeof init.body !== "string") {
    init.headers = { "Content-Type": "application/json", ...(init.headers || {}) };
    init.body = JSON.stringify(init.body);
  }
  const response = await fetch(path, init);
  const payload = await response.json();
  if (!payload.ok) throw new Error(payload.error || "请求失败");
  return payload.data;
}

function toast(message) {
  const box = document.getElementById("toast");
  box.textContent = message;
  box.classList.add("show");
  setTimeout(() => box.classList.remove("show"), 2600);
}

function secondsText(value) {
  if (!value && value !== 0) return "-";
  const h = Math.floor(value / 3600);
  const m = Math.floor((value % 3600) / 60);
  const s = value % 60;
  return `${h}h ${m}m ${s}s`;
}

function mb(kb) {
  if (!kb && kb !== 0) return "-";
  return `${(kb / 1024).toFixed(1)} MB`;
}

async function loadStatus() {
  const data = await api("/api/status");
  const alg = data.algorithm || {};
  const state = document.getElementById("runState");
  state.textContent = alg.running ? "运行中" : "已停止";
  state.className = `status-pill ${alg.running ? "ok" : "bad"}`;
  document.getElementById("metricState").textContent = alg.running ? "运行中" : "已停止";
  document.getElementById("metricPid").textContent = alg.pid || "-";
  document.getElementById("metricRss").textContent = mb(alg.rss_kb);
  document.getElementById("metricCpu").textContent = alg.cpu_seconds ? `${alg.cpu_seconds}s` : "-";
  document.getElementById("metricThreads").textContent = alg.threads || "-";
  document.getElementById("metricUptime").textContent = secondsText(alg.uptime_seconds);
  document.getElementById("statusDetail").textContent = JSON.stringify(data, null, 2);
  updateVideoInfo(data.video || {});
}

async function loadMainConfig() {
  mainConfig = await api("/api/main-config");
  renderForm("mainConfigForm", mainFields, mainConfig);
  document.getElementById("warningText").value = mainConfig.LedWarningText || "";
}

async function loadModelConfig() {
  modelConfig = await api("/api/model-config");
  const values = { ...modelConfig.gie, ...modelConfig.tracker };
  renderForm("modelConfigForm", [...gieFields, ...trackerFields], values);
}

async function loadModels() {
  const data = await api("/api/models");
  models = data.models || [];
  profiles = data.profiles || [];
  const modelSelect = document.getElementById("modelSelect");
  modelSelect.innerHTML = models.map((model) => `<option value="${escapeHtml(model.path)}">${escapeHtml(model.path)}</option>`).join("");
  const profileSelect = document.getElementById("profileSelect");
  profileSelect.innerHTML = profiles.map((item) => `<option value="${escapeHtml(item.name)}">${escapeHtml(item.name)}</option>`).join("");
}

async function loadWarnings() {
  const data = await api("/api/warnings");
  renderWarnings(data || []);
}

async function loadAll() {
  await Promise.all([loadStatus(), loadMainConfig(), loadModelConfig(), loadModels(), loadWarnings()]);
}

function renderForm(id, fields, values) {
  const form = document.getElementById(id);
  form.innerHTML = fields.map((key) => {
    const value = values[key] ?? "";
    const type = key.includes("threshold") || key.includes("thresh") ? "number" : "text";
    const step = type === "number" ? ' step="0.01" min="0" max="1"' : "";
    return `<label>${fieldNames[key] || key}<input name="${escapeHtml(key)}" value="${escapeHtml(value)}" type="${type}"${step}></label>`;
  }).join("");
}

function collectForm(id) {
  const data = {};
  const form = document.getElementById(id);
  [...form.elements].forEach((item) => {
    if (!item.name) return;
    data[item.name] = item.value;
  });
  return data;
}

async function saveMainConfig(restart) {
  const data = collectForm("mainConfigForm");
  await api("/api/main-config", { method: "POST", body: data });
  if (restart) await processAction("restart", false);
  await loadMainConfig();
  toast(restart ? "配置已保存并重启算法" : "配置已保存");
}

async function saveModelConfig(restart) {
  const values = collectForm("modelConfigForm");
  const body = { gie: {}, tracker: {} };
  gieFields.forEach((key) => body.gie[key] = values[key]);
  trackerFields.forEach((key) => body.tracker[key] = values[key]);
  await api("/api/model-config", { method: "POST", body });
  if (restart) await processAction("restart", false);
  await loadModelConfig();
  toast(restart ? "模型参数已保存并重启算法" : "模型参数已保存");
}

async function applyProfile() {
  const name = document.getElementById("profileSelect").value;
  await api("/api/model-profile/apply", { method: "POST", body: { name } });
  await loadModelConfig();
  toast("模型预设已应用");
}

function useSelectedModel() {
  const selected = document.getElementById("modelSelect").value;
  const form = document.getElementById("modelConfigForm");
  const input = form.elements["model_engine_file"];
  if (input) input.value = selected;
}

async function processAction(action, showToast = true) {
  const data = await api(`/api/process/${action}`, { method: "POST", body: {} });
  await loadStatus();
  if (showToast) toast(JSON.stringify(data));
}

async function videoAction(action) {
  const data = await api(`/api/video/${action}`, { method: "POST", body: {} });
  updateVideoInfo(data);
  if (action === "start") setTimeout(refreshVideo, 1500);
  toast(action === "start" ? "视频流已启动" : "视频流已停止");
}

function updateVideoInfo(video) {
  const playlist = video.playlist || "-";
  document.getElementById("sourcePath").textContent = video.source || "-";
  document.getElementById("playlistPath").textContent = playlist;
  if (video.playlist) {
    document.getElementById("videoHint").textContent = "如果浏览器不支持 HLS，可用 VLC 打开该地址";
  } else if (video.source) {
    document.getElementById("videoHint").textContent = "视频流从配置文件 CameraURI 读取";
  }
}

function refreshVideo() {
  const playlist = document.getElementById("playlistPath").textContent;
  if (!playlist || playlist === "-") return;
  const video = document.getElementById("videoPlayer");
  video.src = `${playlist}?t=${Date.now()}`;
  video.load();
}

function setWarningText(text) {
  document.getElementById("warningText").value = text;
}

async function publishWarning(restart) {
  const text = document.getElementById("warningText").value.trim();
  const data = await api("/api/warning/publish", { method: "POST", body: { text, restart } });
  renderWarnings(data.history || []);
  if (restart) await loadStatus();
  toast(data.result || "预警文字已保存");
}

function renderWarnings(rows) {
  const body = document.getElementById("warningHistory");
  body.innerHTML = rows.map((row) => `<tr><td>${escapeHtml(row.time)}</td><td>${escapeHtml(row.text)}</td><td>${escapeHtml(row.result)}</td></tr>`).join("");
}

function escapeHtml(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

loadAll().catch((error) => toast(error.message));
setInterval(() => loadStatus().catch(() => {}), 5000);
