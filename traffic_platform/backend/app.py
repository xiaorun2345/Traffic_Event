#!/usr/bin/env python3
import argparse
import hashlib
import hmac
import json
import os
import re
import shutil
import signal
import subprocess
import time
from datetime import datetime
from email.utils import formatdate
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse
from urllib.request import Request, urlopen


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PLATFORM_ROOT = PROJECT_ROOT / "traffic_platform"
FRONTEND_ROOT = PLATFORM_ROOT / "frontend"
RUNTIME_ROOT = PLATFORM_ROOT / "runtime"
BACKUP_ROOT = RUNTIME_ROOT / "backups"
LOG_ROOT = RUNTIME_ROOT / "logs"
MEDIAMTX_ROOT = PLATFORM_ROOT / "mediamtx"
MEDIAMTX_RUNTIME_CONFIG = RUNTIME_ROOT / "mediamtx.yml"

MAIN_CONFIG = PROJECT_ROOT / "config" / "DealRCF.cfg"
CAMERA_CONFIG = PROJECT_ROOT / "config" / "camera_config_lane.cfg"
OUTPUT_DIR = PROJECT_ROOT / "output"
RUN_SCRIPT = OUTPUT_DIR / "run.sh"
ALGORITHM_BIN = OUTPUT_DIR / "cw_DealRCF_nebulalink"
MODEL_PROFILE_FILE = RUNTIME_ROOT / "model_profiles.json"
WARNING_HISTORY_FILE = RUNTIME_ROOT / "warnings.json"
MEDIAMTX_PID_FILE = RUNTIME_ROOT / "mediamtx.pid"
RTSP_RELAY_PID_FILE = RUNTIME_ROOT / "rtsp-relay.pid"
ALGORITHM_PID_FILE = RUNTIME_ROOT / "algorithm.pid"

ALGORITHM_NAME = "cw_DealRCF_nebulalink"
MEDIAMTX_PATH = os.environ.get("TRAFFIC_MEDIAMTX_PATH", "boxed")
MEDIAMTX_RTSP_PORT = int(os.environ.get("TRAFFIC_MEDIAMTX_RTSP_PORT", "8555"))
MEDIAMTX_WEBRTC_PORT = int(os.environ.get("TRAFFIC_MEDIAMTX_WEBRTC_PORT", "8889"))

MAIN_NODE_FIELDS = [
    "CameraConfig",
    "RsuIp",
    "RsuPort",
    "CloudIp",
    "CloudPort",
    "CameraURI",
    "CameraShow",
    "SendTime",
    "LedWarningText",
    "LedIp",
    "LedPort",
    "LedSdkKey",
    "LedSdkSecret",
    "LedDeviceId",
    "RtspPushPort",
    "RtspOutputWidth",
    "RtspOutputHeight",
    "RtspTransport",
    "SourceTimeoutSec",
    "ReconnectTimes",
    "CameraLength",
    "CameraWidth",
    "location_angle",
    "location_longitude",
    "location_latitude",
    "IsGcj02",
]

GIE_FIELDS = [
    "model_engine_file",
    "class_names",
    "network_mode",
    "idx_npu",
    "nms_iou_threshold",
    "pre-cluster-threshold",
]

TRACKER_FIELDS = [
    "low_thresh",
    "high_thresh",
    "high_thresh_person",
    "high_thresh_motorbike",
    "max_iou_thresh",
]

STRING_FIELDS = {
    "CameraConfig",
    "RsuIp",
    "CloudIp",
    "CameraURI",
    "LedWarningText",
    "LedIp",
    "LedSdkKey",
    "LedSdkSecret",
    "LedDeviceId",
    "RtspTransport",
    "model_engine_file",
    "class_names",
    "nms_iou_threshold",
    "pre-cluster-threshold",
    "low_thresh",
    "high_thresh",
    "high_thresh_person",
    "high_thresh_motorbike",
    "max_iou_thresh",
}

FLOAT_LIMIT_FIELDS = {
    "nms_iou_threshold",
    "pre-cluster-threshold",
    "low_thresh",
    "high_thresh",
    "high_thresh_person",
    "high_thresh_motorbike",
    "max_iou_thresh",
}


def ensure_runtime():
    for path in (BACKUP_ROOT, LOG_ROOT):
        path.mkdir(parents=True, exist_ok=True)
    if not MODEL_PROFILE_FILE.exists():
        write_json(MODEL_PROFILE_FILE, default_model_profiles())
    if not WARNING_HISTORY_FILE.exists():
        write_json(WARNING_HISTORY_FILE, [])


def now_text():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def read_text(path):
    return path.read_text(encoding="utf-8")


def write_text_atomic(path, content):
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(content, encoding="utf-8")
    tmp.replace(path)


def read_json(path, default):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return default


def write_json(path, data):
    write_text_atomic(path, json.dumps(data, ensure_ascii=False, indent=2))


def backup_file(path):
    BACKUP_ROOT.mkdir(parents=True, exist_ok=True)
    target = BACKUP_ROOT / f"{path.name}.{now_text()}.bak"
    shutil.copy2(path, target)
    return str(target.relative_to(PROJECT_ROOT))


def parse_config_value(raw):
    raw = raw.strip()
    if raw.startswith('"') and raw.endswith('"'):
        return raw[1:-1]
    if raw in ("true", "false"):
        return raw == "true"
    try:
        if "." in raw:
            return float(raw)
        return int(raw)
    except ValueError:
        return raw


def format_config_value(key, value):
    if key in STRING_FIELDS:
        escaped = str(value).replace("\\", "\\\\").replace('"', '\\"')
        return f'"{escaped}"'
    if isinstance(value, bool):
        return "1" if value else "0"
    return str(value)


def group_range(text, group_name):
    match = re.search(rf"(?m)^\s*{re.escape(group_name)}\s*=\s*\{{", text)
    if not match:
        raise ValueError(f"group not found: {group_name}")
    brace_pos = text.find("{", match.start(), match.end())
    depth = 0
    for idx in range(brace_pos, len(text)):
        ch = text[idx]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                end = idx + 1
                if end < len(text) and text[end] == ";":
                    end += 1
                return match.start(), end
    raise ValueError(f"group not closed: {group_name}")


def parse_group(path, group_name, fields):
    text = read_text(path)
    start, end = group_range(text, group_name)
    group = text[start:end]
    result = {}
    for key in fields:
        pattern = rf"(?m)^(?!\s*#)\s*{re.escape(key)}\s*=\s*(\"(?:\\.|[^\"])*\"|[^;]+);"
        match = re.search(pattern, group)
        if match:
            result[key] = parse_config_value(match.group(1))
    return result


def update_group_values(path, group_name, values):
    text = read_text(path)
    backup = backup_file(path)
    start, end = group_range(text, group_name)
    group = text[start:end]
    for key, value in values.items():
        formatted = format_config_value(key, value)
        pattern = rf"(?m)^(\s*){re.escape(key)}\s*=\s*(\"(?:\\.|[^\"])*\"|[^;]+);(.*)$"
        if re.search(pattern, group):
            group = re.sub(pattern, rf"\1{key} = {formatted};\3", group, count=1)
        else:
            insert_at = group.rfind("}")
            line = f"    {key} = {formatted};\n"
            group = group[:insert_at] + line + group[insert_at:]
    write_text_atomic(path, text[:start] + group + text[end:])
    return backup


def validate_thresholds(values):
    errors = []
    for key in FLOAT_LIMIT_FIELDS:
        if key not in values:
            continue
        try:
            number = float(values[key])
        except (TypeError, ValueError):
            errors.append(f"{key} must be a number")
            continue
        if number < 0.0 or number > 1.0:
            errors.append(f"{key} must be between 0 and 1")
    low = values.get("low_thresh")
    high = values.get("high_thresh")
    if low is not None and high is not None:
        try:
            if float(high) < float(low):
                errors.append("high_thresh must be greater than or equal to low_thresh")
        except (TypeError, ValueError):
            pass
    return errors


def default_model_profiles():
    return [
        {
            "name": "sb.rknn",
            "path": "./weights/sb.rknn",
            "classes": ["car", "bus", "truck", "person", "motorbike", "tricycle"],
            "network_mode": 0,
            "idx_npu": 4,
            "defaults": {
                "pre-cluster-threshold": 0.65,
                "nms_iou_threshold": 0.5,
                "low_thresh": 0.5,
                "high_thresh": 0.7,
                "high_thresh_person": 0.55,
                "high_thresh_motorbike": 0.65,
                "max_iou_thresh": 0.8,
            },
        },
        {
            "name": "yolov5s_best.rknn",
            "path": "./weights/yolov5s_best.rknn",
            "classes": ["car", "bus", "truck", "person", "motorbike", "tricycle", "cone"],
            "network_mode": 1,
            "idx_npu": 4,
            "defaults": {
                "pre-cluster-threshold": 0.35,
                "nms_iou_threshold": 0.5,
                "low_thresh": 0.5,
                "high_thresh": 0.7,
                "high_thresh_person": 0.7,
                "high_thresh_motorbike": 0.7,
                "max_iou_thresh": 0.8,
            },
        },
    ]


def scan_models():
    roots = [
        OUTPUT_DIR / "weights",
        OUTPUT_DIR,
        PROJECT_ROOT / "ffmedia_release" / "inference_examples" / "yolov5" / "model",
    ]
    models = []
    seen = set()
    for root in roots:
        if not root.exists():
            continue
        for path in root.rglob("*.rknn"):
            rel = path.relative_to(OUTPUT_DIR) if str(path).startswith(str(OUTPUT_DIR)) else path.relative_to(PROJECT_ROOT)
            if str(path) in seen:
                continue
            seen.add(str(path))
            models.append({
                "name": path.name,
                "path": "./" + str(rel) if path.is_relative_to(OUTPUT_DIR) else "../" + str(rel),
                "absolute_path": str(path),
                "size": path.stat().st_size,
            })
    return sorted(models, key=lambda item: (item["name"], item["path"]))


def read_main_config():
    return parse_group(MAIN_CONFIG, "node", MAIN_NODE_FIELDS)


def read_model_config():
    gie = parse_group(CAMERA_CONFIG, "gie", GIE_FIELDS)
    tracker = parse_group(CAMERA_CONFIG, "tracker", TRACKER_FIELDS)
    return {"gie": gie, "tracker": tracker}


def apply_model_profile(profile_name):
    profiles = read_json(MODEL_PROFILE_FILE, default_model_profiles())
    selected = None
    for profile in profiles:
        if profile.get("name") == profile_name:
            selected = profile
            break
    if not selected:
        raise ValueError("model profile not found")
    defaults = selected.get("defaults", {})
    gie_values = {
        "model_engine_file": selected.get("path"),
        "class_names": ";".join(selected.get("classes", [])),
        "network_mode": selected.get("network_mode", 0),
        "idx_npu": selected.get("idx_npu", 4),
        "nms_iou_threshold": str(defaults.get("nms_iou_threshold", 0.5)),
        "pre-cluster-threshold": str(defaults.get("pre-cluster-threshold", 0.65)),
    }
    tracker_values = {key: str(defaults[key]) for key in TRACKER_FIELDS if key in defaults}
    backups = [
        update_group_values(CAMERA_CONFIG, "gie", gie_values),
        update_group_values(CAMERA_CONFIG, "tracker", tracker_values),
    ]
    return backups


def find_algorithm_pid():
    binary_pid = find_process_by_token(ALGORITHM_NAME)
    if binary_pid:
        return binary_pid
    if ALGORITHM_PID_FILE.exists():
        try:
            pid = int(ALGORITHM_PID_FILE.read_text().strip())
            if process_exists(pid):
                return pid
        except Exception:
            pass
    return find_process_by_token(RUN_SCRIPT.name)


def find_process_by_token(token):
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        cmdline_path = proc / "cmdline"
        try:
            raw = cmdline_path.read_bytes()
        except Exception:
            continue
        parts = [item.decode("utf-8", "ignore") for item in raw.split(b"\x00") if item]
        if any(Path(part).name == token for part in parts):
            return int(proc.name)
    return None


def process_exists(pid):
    if not pid:
        return False
    proc = Path(f"/proc/{pid}")
    if not proc.exists():
        return False
    try:
        status = (proc / "status").read_text()
        return not re.search(r"^State:\s+Z\b", status, re.MULTILINE)
    except Exception:
        return True


def read_proc_stats(pid):
    if not process_exists(pid):
        return None
    proc = Path(f"/proc/{pid}")
    try:
        status = (proc / "status").read_text()
        stat = (proc / "stat").read_text().split()
        uptime = float(Path("/proc/uptime").read_text().split()[0])
        ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        page_size = os.sysconf("SC_PAGE_SIZE")
        vmrss = parse_status_kb(status, "VmRSS")
        vmsize = parse_status_kb(status, "VmSize")
        threads = parse_status_int(status, "Threads")
        utime = int(stat[13])
        stime = int(stat[14])
        start_ticks = int(stat[21])
        started_seconds_ago = max(0, uptime - (start_ticks / ticks))
        rss_pages = int(stat[23])
        return {
            "pid": pid,
            "running": True,
            "rss_kb": vmrss if vmrss is not None else int(rss_pages * page_size / 1024),
            "vms_kb": vmsize,
            "threads": threads,
            "cpu_seconds": round((utime + stime) / ticks, 2),
            "uptime_seconds": int(started_seconds_ago),
            "cmdline": (proc / "cmdline").read_bytes().replace(b"\x00", b" ").decode("utf-8", "ignore").strip(),
        }
    except Exception as exc:
        return {"pid": pid, "running": True, "error": str(exc)}


def parse_status_kb(status, key):
    match = re.search(rf"^{key}:\s+(\d+)\s+kB", status, re.MULTILINE)
    return int(match.group(1)) if match else None


def parse_status_int(status, key):
    match = re.search(rf"^{key}:\s+(\d+)", status, re.MULTILINE)
    return int(match.group(1)) if match else None


def start_algorithm():
    pid = find_algorithm_pid()
    if pid:
        return {"started": False, "pid": pid, "message": "algorithm already running"}
    if not RUN_SCRIPT.exists():
        raise ValueError(f"run script not found: {RUN_SCRIPT}")
    log_path = LOG_ROOT / "algorithm.log"
    log = open(log_path, "ab")
    proc = subprocess.Popen(["bash", str(RUN_SCRIPT)], cwd=str(PROJECT_ROOT), stdout=log, stderr=subprocess.STDOUT)
    ALGORITHM_PID_FILE.write_text(str(proc.pid), encoding="utf-8")
    return {"started": True, "pid": proc.pid, "log": str(log_path.relative_to(PROJECT_ROOT))}


def stop_algorithm():
    pid = find_algorithm_pid()
    if not pid:
        return {"stopped": False, "message": "algorithm is not running"}
    terminate_pid(pid)
    if ALGORITHM_PID_FILE.exists():
        ALGORITHM_PID_FILE.unlink()
    return {"stopped": True, "pid": pid}


def terminate_pid(pid, timeout=5):
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not process_exists(pid):
            return
        time.sleep(0.2)
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def mediamtx_pid():
    if not MEDIAMTX_PID_FILE.exists():
        return None
    try:
        pid = int(MEDIAMTX_PID_FILE.read_text().strip())
    except Exception:
        return None
    return pid if process_exists(pid) else None


def rtsp_relay_pid():
    if not RTSP_RELAY_PID_FILE.exists():
        return None
    try:
        pid = int(RTSP_RELAY_PID_FILE.read_text().strip())
    except Exception:
        return None
    return pid if process_exists(pid) else None


def push_rtsp_suffix(camera_uri):
    parsed = urlparse(camera_uri)
    if parsed.scheme == "rtsp" and parsed.hostname:
        name = parsed.hostname
    else:
        name = Path(camera_uri).stem or "camera"
    name = re.sub(r"[^A-Za-z0-9._-]+", "_", name)
    return f"{name}/camera"


def configured_video_source(config):
    source = str(config.get("CameraURI", "")).strip()
    try:
        camera_show = int(config.get("CameraShow", 0))
    except (TypeError, ValueError):
        camera_show = 0
    if camera_show == 4 and source:
        port = config.get("RtspPushPort", 8554) or 8554
        return f"rtsp://127.0.0.1:{port}/{push_rtsp_suffix(source)}"
    return source


def public_host_from_header(host_header):
    host = (host_header or "").split(",", 1)[0].strip()
    if not host:
        return "127.0.0.1"
    if host.startswith("[") and "]" in host:
        return host[1:host.index("]")]
    return host.split(":", 1)[0]


def webrtc_url(host_header):
    host = public_host_from_header(host_header)
    return f"http://{host}:{MEDIAMTX_WEBRTC_PORT}/{MEDIAMTX_PATH}/"


def mediamtx_binary():
    candidates = [
        os.environ.get("MEDIAMTX_BIN"),
        str(MEDIAMTX_ROOT / "mediamtx"),
        shutil.which("mediamtx"),
        shutil.which("rtsp-simple-server"),
    ]
    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate)
        if path.exists() and os.access(path, os.X_OK):
            return str(path)
    return None


def yaml_quote(value):
    return "'" + str(value).replace("'", "''") + "'"


def write_mediamtx_config():
    content = f"""# Generated by traffic_platform/backend/app.py
logLevel: info
logDestinations: [stdout]

rtsp: yes
rtspAddress: :{MEDIAMTX_RTSP_PORT}
rtmp: no
hls: no
webrtc: yes
webrtcAddress: :{MEDIAMTX_WEBRTC_PORT}
srt: no

paths:
  {MEDIAMTX_PATH}:
    source: publisher
"""
    write_text_atomic(MEDIAMTX_RUNTIME_CONFIG, content)
    return MEDIAMTX_RUNTIME_CONFIG


def video_status(host_header=None):
    pid = mediamtx_pid()
    relay_pid = rtsp_relay_pid()
    config = read_main_config()
    source = configured_video_source(config)
    binary = mediamtx_binary()
    return {
        "running": bool(pid),
        "pid": pid,
        "relay_running": bool(relay_pid),
        "relay_pid": relay_pid,
        "installed": bool(binary),
        "binary": binary,
        "source": source,
        "publish": f"rtsp://127.0.0.1:{MEDIAMTX_RTSP_PORT}/{MEDIAMTX_PATH}" if source else None,
        "webrtc": webrtc_url(host_header) if source else None,
        "path": MEDIAMTX_PATH,
        "rtsp_port": MEDIAMTX_RTSP_PORT,
        "webrtc_port": MEDIAMTX_WEBRTC_PORT,
        "config": str(MEDIAMTX_RUNTIME_CONFIG.relative_to(PROJECT_ROOT)) if MEDIAMTX_RUNTIME_CONFIG.exists() else None,
    }


def start_video(host_header=None):
    pid = mediamtx_pid()
    relay_pid = rtsp_relay_pid()
    if pid and relay_pid:
        return {"started": False, **video_status(host_header)}
    binary = mediamtx_binary()
    if not binary:
        raise FileNotFoundError("未找到 mediamtx，请运行 traffic_platform/scripts/install_mediamtx.sh，或将 Linux arm64 版二进制放到 traffic_platform/mediamtx/mediamtx")
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise FileNotFoundError("未找到 ffmpeg，无法将带框 RTSP 发布到 MediaMTX")
    config = read_main_config()
    source = configured_video_source(config)
    if not source:
        raise ValueError("CameraURI is empty")
    if not pid:
        config_path = write_mediamtx_config()
        log = open(LOG_ROOT / "mediamtx.log", "ab")
        proc = subprocess.Popen([binary, str(config_path)], cwd=str(PROJECT_ROOT), stdout=log, stderr=subprocess.STDOUT)
        MEDIAMTX_PID_FILE.write_text(str(proc.pid), encoding="utf-8")
        time.sleep(0.3)
        if proc.poll() is not None:
            if MEDIAMTX_PID_FILE.exists():
                MEDIAMTX_PID_FILE.unlink()
            raise RuntimeError(f"MediaMTX 启动失败，请查看 {str((LOG_ROOT / 'mediamtx.log').relative_to(PROJECT_ROOT))}")
    if not rtsp_relay_pid():
        transport = str(config.get("RtspTransport", "tcp") or "tcp").lower()
        if transport not in ("tcp", "udp", "multicast"):
            transport = "tcp"
        publish_url = f"rtsp://127.0.0.1:{MEDIAMTX_RTSP_PORT}/{MEDIAMTX_PATH}"
        relay_log = open(LOG_ROOT / "rtsp-relay.log", "ab")
        relay_cmd = [
            ffmpeg,
            "-hide_banner",
            "-loglevel", "warning",
            "-rtsp_transport", transport,
            "-i", source,
            "-an",
            "-c:v", "copy",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            publish_url,
        ]
        relay = subprocess.Popen(relay_cmd, cwd=str(OUTPUT_DIR), stdout=relay_log, stderr=subprocess.STDOUT)
        RTSP_RELAY_PID_FILE.write_text(str(relay.pid), encoding="utf-8")
        time.sleep(1.0)
        if relay.poll() is not None:
            if RTSP_RELAY_PID_FILE.exists():
                RTSP_RELAY_PID_FILE.unlink()
            raise RuntimeError(f"RTSP 转推启动失败，请查看 {str((LOG_ROOT / 'rtsp-relay.log').relative_to(PROJECT_ROOT))}")
    return {"started": True, **video_status(host_header)}


def stop_video():
    relay_pid = rtsp_relay_pid()
    if relay_pid:
        terminate_pid(relay_pid)
    if RTSP_RELAY_PID_FILE.exists():
        RTSP_RELAY_PID_FILE.unlink()
    pid = mediamtx_pid()
    if pid:
        terminate_pid(pid)
    if MEDIAMTX_PID_FILE.exists():
        MEDIAMTX_PID_FILE.unlink()
    return {"stopped": bool(pid or relay_pid), "pid": pid, "relay_pid": relay_pid}


def add_warning_record(text, result):
    history = read_json(WARNING_HISTORY_FILE, [])
    history.insert(0, {
        "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "text": text,
        "result": result,
    })
    write_json(WARNING_HISTORY_FILE, history[:100])
    return history[:100]


def build_led_program(text, device_id):
    areas = []
    colors = ["#ff0000", "#ffff00", "#0000ff"]
    for index, color in enumerate(colors):
        areas.append({
            "x": index * 32,
            "y": 0,
            "width": 32,
            "height": 96,
            "border": {"type": 0, "speed": 5, "effect": "rotate"},
            "item": [{
                "type": "text",
                "string": text,
                "multiLine": True,
                "PlayText": False,
                "font": {
                    "name": "宋体",
                    "size": 14,
                    "underline": False,
                    "bold": False,
                    "italic": False,
                    "color": color,
                },
                "effect": {"type": 0, "speed": 5, "hold": 5000},
            }],
        })
    return {
        "method": "replace",
        "id": device_id,
        "data": [{
            "name": "节目1",
            "type": "normal",
            "uuid": "A3",
            "area": areas,
        }],
    }


def publish_led_text(text, config):
    led_ip = str(config.get("LedIp", "")).strip()
    led_port = int(config.get("LedPort", 0) or 0)
    sdk_key = str(config.get("LedSdkKey", "") or "")
    sdk_secret = str(config.get("LedSdkSecret", "") or "")
    device_id = str(config.get("LedDeviceId", "") or "")
    if not led_ip or led_port <= 0:
        return {"ok": False, "result": "LED 配置缺少 IP 或端口"}
    if not sdk_key or not sdk_secret or not device_id:
        return {"ok": False, "result": "LED 配置缺少 SDK Key/Secret/DeviceId"}

    body = json.dumps(build_led_program(text, device_id), ensure_ascii=False, separators=(",", ":"))
    date = formatdate(timeval=None, localtime=False, usegmt=True)
    request_id = f"{int(time.time() * 1000):x}-{os.getpid():x}"
    sign = hmac.new(sdk_secret.encode("utf-8"), (body + sdk_key + date).encode("utf-8"), hashlib.md5).hexdigest()
    url = f"http://{led_ip}:{led_port}/api/program/"
    request = Request(url, data=body.encode("utf-8"), method="POST", headers={
        "requestId": request_id,
        "sdkKey": sdk_key,
        "date": date,
        "sign": sign,
        "Content-Type": "application/json",
    })
    try:
        with urlopen(request, timeout=5) as response:
            response_body = response.read().decode("utf-8", "ignore")
            status = response.getcode()
    except Exception as exc:
        return {"ok": False, "result": f"LED 下发失败: {exc}"}

    if status < 200 or status >= 300:
        return {"ok": False, "result": f"LED HTTP {status}: {response_body}"}
    try:
        payload = json.loads(response_body or "{}")
    except Exception:
        return {"ok": False, "result": f"LED 返回非 JSON: {response_body}"}
    if payload.get("message") == "ok":
        return {"ok": True, "result": "LED 已下发"}
    return {"ok": False, "result": f"LED 拒绝: {response_body}"}


def response_payload(data=None, ok=True, error=None, status=200):
    payload = {"ok": ok}
    if data is not None:
        payload["data"] = data
    if error:
        payload["error"] = error
    return status, payload


class PlatformHandler(SimpleHTTPRequestHandler):
    server_version = "TrafficPlatform/0.1"

    def do_GET(self):
        try:
            parsed = urlparse(self.path)
            path = parsed.path
            if path.startswith("/api/"):
                self.handle_api_get(path)
                return
            if path == "/":
                path = "/index.html"
            self.serve_file(FRONTEND_ROOT, path.lstrip("/"), None)
        except Exception as exc:
            self.send_json(*response_payload(ok=False, error=str(exc), status=500))

    def do_POST(self):
        try:
            parsed = urlparse(self.path)
            if not parsed.path.startswith("/api/"):
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            self.handle_api_post(parsed.path, self.read_json_body())
        except Exception as exc:
            self.send_json(*response_payload(ok=False, error=str(exc), status=500))

    def read_json_body(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw.strip() else {}

    def send_json(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def serve_file(self, root, relative, content_type):
        relative = unquote(relative).strip("/")
        target = (root / relative).resolve()
        if not str(target).startswith(str(root.resolve())) or not target.exists() or not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        if not content_type:
            content_type = self.guess_type(str(target))
        data = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def handle_api_get(self, path):
        if path == "/api/status":
            pid = find_algorithm_pid()
            self.send_json(*response_payload({
                "algorithm": read_proc_stats(pid) if pid else {"running": False},
                "video": video_status(self.headers.get("Host")),
                "config": {
                    "main": str(MAIN_CONFIG.relative_to(PROJECT_ROOT)),
                    "camera": str(CAMERA_CONFIG.relative_to(PROJECT_ROOT)),
                },
            }))
        elif path == "/api/main-config":
            self.send_json(*response_payload(read_main_config()))
        elif path == "/api/model-config":
            self.send_json(*response_payload(read_model_config()))
        elif path == "/api/models":
            self.send_json(*response_payload({"models": scan_models(), "profiles": read_json(MODEL_PROFILE_FILE, default_model_profiles())}))
        elif path == "/api/video/status":
            self.send_json(*response_payload(video_status(self.headers.get("Host"))))
        elif path == "/api/warnings":
            self.send_json(*response_payload(read_json(WARNING_HISTORY_FILE, [])))
        else:
            self.send_json(*response_payload(ok=False, error="not found", status=404))

    def handle_api_post(self, path, body):
        if path == "/api/main-config":
            backup = update_group_values(MAIN_CONFIG, "node", body)
            self.send_json(*response_payload({"backup": backup, "config": read_main_config()}))
        elif path == "/api/model-config":
            gie_values = body.get("gie", {})
            tracker_values = body.get("tracker", {})
            errors = validate_thresholds({**gie_values, **tracker_values})
            if errors:
                self.send_json(*response_payload(ok=False, error="; ".join(errors), status=400))
                return
            backups = []
            if gie_values:
                backups.append(update_group_values(CAMERA_CONFIG, "gie", gie_values))
            if tracker_values:
                backups.append(update_group_values(CAMERA_CONFIG, "tracker", tracker_values))
            self.send_json(*response_payload({"backups": backups, "config": read_model_config()}))
        elif path == "/api/model-profile/apply":
            backups = apply_model_profile(body.get("name", ""))
            self.send_json(*response_payload({"backups": backups, "config": read_model_config()}))
        elif path == "/api/process/start":
            self.send_json(*response_payload(start_algorithm()))
        elif path == "/api/process/stop":
            self.send_json(*response_payload(stop_algorithm()))
        elif path == "/api/process/restart":
            stopped = stop_algorithm()
            time.sleep(1)
            started = start_algorithm()
            self.send_json(*response_payload({"stop": stopped, "start": started}))
        elif path == "/api/video/start":
            self.send_json(*response_payload(start_video(self.headers.get("Host"))))
        elif path == "/api/video/stop":
            self.send_json(*response_payload(stop_video()))
        elif path == "/api/warning/publish":
            text = str(body.get("text", "")).strip()
            if not text:
                self.send_json(*response_payload(ok=False, error="warning text is empty", status=400))
                return
            backup = update_group_values(MAIN_CONFIG, "node", {"LedWarningText": text})
            config = read_main_config()
            led_result = publish_led_text(text, config)
            result = f"saved to config; {led_result['result']}"
            if body.get("restart"):
                stop_algorithm()
                time.sleep(1)
                start_algorithm()
                result += "; restarted algorithm"
            history = add_warning_record(text, result)
            self.send_json(*response_payload({"backup": backup, "result": result, "led": led_result, "history": history}))
        else:
            self.send_json(*response_payload(ok=False, error="not found", status=404))

    def log_message(self, fmt, *args):
        log_line = "%s %s\n" % (datetime.now().strftime("%Y-%m-%d %H:%M:%S"), fmt % args)
        with open(LOG_ROOT / "access.log", "a", encoding="utf-8") as file:
            file.write(log_line)


def main():
    parser = argparse.ArgumentParser(description="Traffic Event web platform")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    args = parser.parse_args()
    ensure_runtime()
    server = ThreadingHTTPServer((args.host, args.port), PlatformHandler)
    print(f"Traffic platform listening on http://{args.host}:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
