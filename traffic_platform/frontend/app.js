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

function setText(id, value) {
  const node = document.getElementById(id);
  if (node) node.textContent = value;
}

function setValue(id, value) {
  const node = document.getElementById(id);
  if (node) node.value = value;
}

function getActiveTab() {
  return document.querySelector(".tabs button.active")?.dataset.tab || "video";
}

function getWarningText() {
  const activeId = getActiveTab() === "warning" ? "warningTextPanel" : "warningText";
  const activeText = document.getElementById(activeId)?.value || "";
  const fallbackId = activeId === "warningText" ? "warningTextPanel" : "warningText";
  const fallbackText = document.getElementById(fallbackId)?.value || "";
  return (activeText || fallbackText).trim();
}

function syncWarningTextInputs(value, sourceId) {
  ["warningText", "warningTextPanel"].forEach((id) => {
    if (id !== sourceId) setValue(id, value);
  });
}

function updateClock() {
  const now = new Date();
  const text = `${now.getFullYear()}-${String(now.getMonth() + 1).padStart(2, "0")}-${String(now.getDate()).padStart(2, "0")} ${String(now.getHours()).padStart(2, "0")}:${String(now.getMinutes()).padStart(2, "0")}:${String(now.getSeconds()).padStart(2, "0")}`;
  setText("clockText", text);
  setText("deviceIp", window.location.hostname || "127.0.0.1");
}

async function loadStatus() {
  const data = await api("/api/status");
  const alg = data.algorithm || {};
  const state = document.getElementById("runState");
  state.textContent = alg.running ? "运行中" : "已停止";
  state.className = `status-pill ${alg.running ? "ok" : "bad"}`;
  setText("metricState", alg.running ? "运行中" : "已停止");
  setText("metricPid", alg.pid || "-");
  setText("metricRss", mb(alg.rss_kb));
  setText("metricCpu", alg.cpu_seconds ? `${alg.cpu_seconds}s` : "-");
  setText("metricThreads", alg.threads || "-");
  setText("metricUptime", secondsText(alg.uptime_seconds));
  setText("videoPid", alg.pid || "-");
  setText("videoRss", mb(alg.rss_kb));
  setText("videoCpu", alg.cpu_seconds ? `${alg.cpu_seconds}s` : "-");
  setText("videoThreads", alg.threads || "-");
  setText("videoUptime", secondsText(alg.uptime_seconds));
  setText("statusDetail", JSON.stringify(data, null, 2));
  updateVideoInfo(data.video || {});
}

async function loadMainConfig() {
  mainConfig = await api("/api/main-config");
  renderForm("mainConfigForm", mainFields, mainConfig);
  setValue("warningText", mainConfig.LedWarningText || "");
  setValue("warningTextPanel", mainConfig.LedWarningText || "");
}

async function loadModelConfig() {
  modelConfig = await api("/api/model-config");
  const values = { ...modelConfig.gie, ...modelConfig.tracker };
  renderForm("modelConfigForm", [...gieFields, ...trackerFields], values);
  renderModelSummary(values);
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

function renderModelSummary(values) {
  setText("modelCurrent", values.model_engine_file || "-");
  setText("modelConfidence", values["pre-cluster-threshold"] ?? "-");
  setText("modelNms", values.nms_iou_threshold ?? "-");
  setText("modelTrack", values.high_thresh ?? values.low_thresh ?? "-");
}

async function processAction(action, showToast = true) {
  const data = await api(`/api/process/${action}`, { method: "POST", body: {} });
  await loadStatus();
  if (showToast) toast(JSON.stringify(data));
}

async function videoAction(action) {
  const data = await api(`/api/video/${action}`, { method: "POST", body: {} });
  if (action === "stop") {
    await loadStatus();
  } else {
    updateVideoInfo(data);
  }
  if (action === "start") {
    refreshVideo();
    toast("WebRTC 转发已启动");
  } else {
    const frame = document.getElementById("videoPlayer");
    frame.removeAttribute("src");
    const hint = document.getElementById("videoHint");
    hint.textContent = "已停止显示";
    hint.style.display = "grid";
    toast("WebRTC 转发已停止");
  }
}

function updateVideoInfo(video) {
  const stream = video.webrtc || "-";
  setText("sourcePath", video.source || "-");
  setText("streamPath", stream);
  const mediaState = video.installed
    ? `${video.running ? "MediaMTX运行中" : "MediaMTX未运行"}，${video.relay_running ? "转推运行中" : "转推未运行"}，WebRTC:${video.webrtc_port || "-"}，RTSP:${video.rtsp_port || "-"}`
    : "未找到 mediamtx";
  setText("mediaState", mediaState);
  setText("videoMedia", video.running ? "运行中" : "未运行");
  setText("videoRelay", video.relay_running ? "运行中" : "未运行");
  const hint = document.getElementById("videoHint");
  const frame = document.getElementById("videoPlayer");
  if (!hint || !frame) return;
  if (video.webrtc && video.running) {
    hint.textContent = "带框 RTSP 正在通过 MediaMTX WebRTC 显示";
    hint.style.display = "none";
    if (!frame.getAttribute("src")) frame.src = `${video.webrtc}?t=${Date.now()}`;
  } else if (video.source) {
    hint.textContent = "已读取带框 RTSP 源流，启动 WebRTC 转发后显示";
    hint.style.display = "grid";
    frame.removeAttribute("src");
  }
}

function refreshVideo() {
  const stream = document.getElementById("streamPath").textContent;
  if (!stream || stream === "-") return;
  const frame = document.getElementById("videoPlayer");
  frame.src = `${stream}?t=${Date.now()}`;
  document.getElementById("videoHint").style.display = "none";
}

function setWarningText(text) {
  setValue("warningText", text);
  setValue("warningTextPanel", text);
}

async function publishWarning(restart) {
  const text = getWarningText();
  const data = await api("/api/warning/publish", { method: "POST", body: { text, restart } });
  renderWarnings(data.history || []);
  if (restart) await loadStatus();
  toast(data.result || "预警文字已保存");
}

function renderWarnings(rows) {
  const body = document.getElementById("warningHistory");
  if (body) {
    body.innerHTML = rows.map((row) => `<tr><td>${escapeHtml(row.time)}</td><td>${escapeHtml(row.text)}</td><td>${escapeHtml(row.result)}</td></tr>`).join("");
  }
  renderEventFeed(rows || []);
}

function renderEventFeed(rows) {
  const feed = document.getElementById("eventFeed");
  if (!feed) return;
  const source = rows.length ? rows.slice(0, 5) : [
    { time: "--:--:--", text: "暂无预警事件", result: "等待发布" }
  ];
  feed.innerHTML = source.map((row, index) => {
    const level = index % 3 === 0 ? "high" : index % 3 === 1 ? "mid" : "low";
    const label = level === "high" ? "高" : level === "mid" ? "中" : "低";
    return `<div class="event-item">
      <div class="event-level ${level}">${label}</div>
      <div><strong>${escapeHtml(row.text || "-")}</strong><span>${escapeHtml(row.result || "-")}</span></div>
      <time>${escapeHtml(row.time || "-")}</time>
    </div>`;
  }).join("");
}

function escapeHtml(value) {
  return String(value ?? "")
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

updateClock();
document.querySelectorAll("#warningText, #warningTextPanel").forEach((node) => {
  node.addEventListener("input", () => syncWarningTextInputs(node.value, node.id));
});
loadAll().catch((error) => toast(error.message));
setInterval(updateClock, 1000);
setInterval(() => loadStatus().catch(() => {}), 5000);
