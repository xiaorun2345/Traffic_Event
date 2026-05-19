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
- 通过 `ffmpeg` 将配置中的 RTSP/视频源转为 HLS，页面访问 `/hls/live.m3u8`。
- 当 `CameraShow = 4` 时，算法会在本机 `8554` 端口推送硬件编码后的带框 RTSP 流，平台会优先转这一路带框流。
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
  runtime/backups/               配置备份
  runtime/hls/                   视频流切片
  runtime/logs/                  平台和转码日志
  runtime/model_profiles.json    模型预设
  scripts/                       启停和安装脚本
  systemd/                       开机自启服务
```

## 注意

- 带框视频流地址格式为 `rtsp://设备IP:8554/摄像头IP/camera`。平台本机转 HLS 时使用 `rtsp://127.0.0.1:8554/摄像头IP/camera`。
- 浏览器不一定原生支持 HLS。如果页面不能直接播放，可用 VLC 打开：

```text
http://设备IP:8080/hls/live.m3u8
```

- 平台会在修改配置前自动备份原文件到 `traffic_platform/runtime/backups/`。
- 真实 RTSP 密码、LED SDK 密钥需要在设备本地配置，不建议提交到 GitHub。
