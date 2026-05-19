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
