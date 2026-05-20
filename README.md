# Traffic_Event

基于 RK3588 的交通事件视觉感知程序，面向道路相机实时取流、目标检测、跟踪定位、UDP 上报和 LED 屏预警等场景。当前工程保留单视觉链路，包含 RTSP 拉流、MPP 硬解码、RGA 图像转换、RKNN 推理、ByteTrack 跟踪、坐标定位和预警消息发送。

## 功能特性

- RTSP 摄像机接入，支持 TCP/UDP/组播配置，默认 TCP 更稳定。
- 基于 RKNN 的目标检测，适配 RK3588 NPU。
- ByteTrack 多目标跟踪。
- 基于标定文件的目标定位和经纬度转换。
- UDP 感知结果上报。
- 检测到目标后调用 LED 屏接口显示预警文本。
- 取流无新帧超时自动重连，避免算法只处理一两帧后卡住。
- 支持硬件编码后的带框 RTSP 推流，供 VLC、平台页面或其它客户端查看。
- 提供局域网 Web 管理平台，支持配置修改、模型选择、阈值调整、ROI 画图配置、运行监控、视频预览和预警发布。

## 目录结构

```text
src/                 主进程、配置读取、取流调度、UDP、LED
camera/              检测、跟踪、定位和相机应用封装
decoder/             备用视频解码链路
common/              协议结构、protobuf、公共头文件
config/              配置模板、视觉配置、标定文件和字体
ffmedia_release/     FFMedia 头文件、示例和文档
3rdparty/            第三方依赖源码或头文件
build.sh             CMake 构建脚本
CMakeLists.txt       主工程构建配置
traffic_platform/    局域网 Web 管理平台
```

运行时动态库、模型文件、构建目录和日志文件不提交到仓库，需要在目标设备上按实际环境准备。

## 运行环境

- 硬件：RK3588 或兼容平台
- 系统：Linux aarch64
- 主要依赖：OpenCV、FFmpeg、GStreamer、RKNN Runtime、RGA、MPP、protobuf、libconfig、jsoncpp、curl、OpenSSL
- 运行时需要模型文件，例如 `output/weights/*.rknn`
- 运行时需要动态库，例如 `lib/*.so`

## 配置

仓库只提供脱敏模板：

```bash
cp config/DealRCF.example.cfg config/DealRCF.cfg
```

然后按现场环境修改：

```cfg
CameraURI = "rtsp://USER:PASSWORD@CAMERA_IP:554/Streaming/Channels/102";
CameraShow = 4;
RtspPushPort = 8554;
RsuIp = "192.168.0.100";
RsuPort = 30088;
CloudIp = "192.168.0.101";
CloudPort = 20000;
LedIp = "192.168.0.102";
LedSdkKey = "";
LedSdkSecret = "";
LedDeviceId = "";
```

不要把真实 RTSP 账号密码、LED SDK 密钥或公网地址提交到仓库。

视频显示相关配置：

```cfg
CameraShow = 1;       # 本机窗口显示
CameraShow = 4;       # 启用带框 RTSP 推流
RtspPushPort = 8554;  # 带框 RTSP 推流端口
RtspOutputWidth = 1280;
RtspOutputHeight = 720; # 平台预览推流分辨率，降低浏览器解码压力
```

当 `CameraShow = 4` 时，算法会在本机启动 RTSP Server，地址格式为：

```text
rtsp://设备IP:RtspPushPort/摄像头IP/camera
```

例如设备 IP 为 `192.168.88.30`、摄像头 IP 为 `192.168.88.33`、端口为 `8554` 时：

```text
rtsp://192.168.88.30:8554/192.168.88.33/camera
```

视觉配置文件：

```text
config/camera_config_lane.cfg
```

其中 `model_engine_file`、`class_names`、`pre-cluster-threshold`、`nms_iou_threshold`、`tracker` 阈值需要和实际模型匹配。当前 `sb.rknn` 对应 6 类：

```cfg
class_names = "car;bus;truck;person;motorbike;tricycle";
pre-cluster-threshold = "0.65";
```

## 编译

```bash
./build.sh
```

等价于：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

默认可执行文件输出到：

```text
output/cw_DealRCF_nebulalink
```

## 运行

建议从 `output/` 目录启动，保证配置里的相对路径有效：

```bash
cd output
export LD_LIBRARY_PATH=../lib:$LD_LIBRARY_PATH
./cw_DealRCF_nebulalink ../config/DealRCF.cfg
```

也可以通过平台页面启动、停止和重启算法。

## 局域网管理平台

平台源码位于 `traffic_platform/`，默认监听 `0.0.0.0:8080`。启动后局域网其它电脑可访问：

```text
http://设备IP:8080
```

前端界面采用深色科技感布局，默认进入“视频监控”页面，中心显示带框实时画面，右侧集中展示预警事件、模型阈值和预警发布，底部展示算法进程和 MediaMTX 转推状态。界面参考图：

![Traffic Event 平台前端设计图](traffic_platform_frontend_design.png)

启动平台：

```bash
./traffic_platform/scripts/start_platform.sh
```

安装开机自启：

```bash
./traffic_platform/scripts/install_service.sh
```

平台主要功能：

- 修改 `config/DealRCF.cfg` 云平台、摄像头、LED、预警文字和 `RtspPushPort` 等配置。
- 修改 `config/camera_config_lane.cfg` 模型、类别、检测阈值、NMS、ByteTrack 阈值。
- 启动、停止、重启算法进程。
- 查看 PID、内存、线程数、CPU 时间、运行时长。
- 当 `CameraShow = 4` 时，平台用 FFmpeg copy 模式把算法内部带框 RTSP 发布到 MediaMTX，再由 MediaMTX 转成 WebRTC 在页面视频监控中显示，不使用 HLS 或 MJPEG。
- 发布和保存 LED 预警文字。

## 主流程

```text
src/main.cpp
  -> DealRCF::LoadConfig()
  -> DealRCF::InitUDP()
  -> DealRCF::StartMatDealThread()
  -> DealRCF::StartCameraDealThread()
```

取流链路：

```text
MatDealFfmediaThread()
  -> FfmediaInit()
     -> ModuleRtspClient
     -> ModuleMppDec
     -> ModuleRga
     -> callback_external()
        -> DealRCF::mat_info_
```

视觉链路：

```text
CameraDealThread()
  -> CameraAPP::Init()
  -> CameraAPP::Process()
     -> RKNNDetector
     -> ByteTrack
     -> LocationApp
  -> CameraAPP::DrawResult()
  -> SerializationsObjectListSend()
  -> LedScreen::ShowText()
```

## 近期排查和修复

- 将 RTSP 默认取流协议调整为 TCP，并增加无新帧超时重连。
- 修复 NMS 类别索引错误，避免后处理耗时异常升高。
- 限制 NMS 候选数量，降低密集候选场景下的 CPU 开销。
- 修正 `sb.rknn` 模型类别数配置：模型输出为 6 类，配置不能写 7 类，否则会读错输出通道并产生大量误检。
- 将检测阈值调整为更适合现场误检抑制的配置。

## 注意事项

- 本仓库不包含私有运行库、模型权重、日志和真实现场配置。
- `config/DealRCF.cfg` 被 `.gitignore` 忽略，仅用于本地运行。
- 如果需要发布模型或运行库，建议使用 GitHub Release 或对象存储，不要直接提交到 git。
