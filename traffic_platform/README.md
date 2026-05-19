# Traffic Event 管理平台

独立 Web 管理平台，源码全部位于 `traffic_platform/`。设备开机后可通过局域网访问页面，默认地址：

```bash
http://设备IP:8080
```

## 功能

- 修改 `config/DealRCF.cfg` 云平台、摄像头、LED、预警文字等配置。
- 修改 `config/camera_config_lane.cfg` 模型、类别、检测阈值、NMS、ByteTrack 跟踪阈值。
- 扫描 `output/weights/` 和 FFMedia 示例模型目录中的 `.rknn` 模型。
- 启动、停止、重启 `output/cw_DealRCF_nebulalink`。
- 监控算法进程 PID、RSS 内存、VMS、线程数、CPU 时间、运行时长。
- 当 `CameraShow = 4` 时，算法会在本机 `RtspPushPort` 端口推送硬件编码后的带框 RTSP 流。
- 视频监控页面通过 FFmpeg copy 模式将带框 RTSP 发布到 MediaMTX，再由 MediaMTX 转成 WebRTC 播放，不使用 HLS 或 MJPEG，也不重编码。
- 保存预警文字，可选择保存后重启算法。

## 启动

```bash
./traffic_platform/scripts/start_platform.sh
```

## 开机自启

```bash
./traffic_platform/scripts/install_service.sh
```

## 目录

```text
traffic_platform/
  backend/app.py                 后端服务
  frontend/                      页面资源
  mediamtx/                      MediaMTX 示例配置和可选二进制目录
  runtime/backups/               配置备份
  runtime/mediamtx.yml           平台按当前配置生成的 MediaMTX 运行配置
  runtime/logs/                  平台和 MediaMTX 日志
  runtime/model_profiles.json    模型预设
  scripts/                       启停和安装脚本
  systemd/                       开机自启服务
```

## 注意

- 带框视频流地址格式为 `rtsp://设备IP:RtspPushPort/摄像头IP/camera`，端口由 `config/DealRCF.cfg` 里的 `RtspPushPort` 配置，默认 `8554`。
- 浏览器不能直接播放 RTSP，平台会启动 MediaMTX，并用 FFmpeg 将本机带框 RTSP 原样发布到 MediaMTX，在 `http://设备IP:8889/boxed/` 输出 WebRTC 页面。
- MediaMTX 的 RTSP 服务端口默认 `8555`，避免和算法带框 RTSP 推流端口 `8554` 冲突。
- 转推地址为 `rtsp://127.0.0.1:8555/boxed`，页面播放地址为 `http://设备IP:8889/boxed/`。
- 如果系统没有 `mediamtx` 命令，可将 Linux arm64 版二进制放到 `traffic_platform/mediamtx/mediamtx` 并赋予执行权限。
- 也可以用脚本安装 MediaMTX，默认版本为 `v1.18.2`：

```bash
./traffic_platform/scripts/install_mediamtx.sh
```

- 启停 WebRTC 转发可以在页面“视频监控”中操作，也可以使用：

```bash
./traffic_platform/scripts/start_mediamtx.sh
./traffic_platform/scripts/stop_mediamtx.sh
```

- 如需用外部播放器，可用 VLC 或 GStreamer 打开带框 RTSP：

```text
rtsp://设备IP:RtspPushPort/摄像头IP/camera
```

- 平台会在修改配置前自动备份原文件到 `traffic_platform/runtime/backups/`。
- 真实 RTSP 密码、LED SDK 密钥需要在设备本地配置，不建议提交到 GitHub。
