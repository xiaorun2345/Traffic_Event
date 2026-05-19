#include "DealRCF.h"
#include <ctime>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <vector>

#include "../camera/detector/MppEncoder.h"
// #include "../camera/detector/MppDecoder.h"
#include <mutex>
#include "../camera/RtspServer/xop/RtspServer.h"
#include "../camera/RtspServer/net/EventLoop.h"

// std::mutex g_frame_mutex;
// cv::Mat g_yuvImg;


const int g_width = 1920;
const int g_height = 1080;


/* @brief : 引用配置参数 */
// extern ParamReader pr;

s_DealInfo DealRCF::deal_info_;
// 各模块输出缓存队列
vector<perceptron::s_ObjectList> DealRCF::sensor_object_lists_;
cv::Mat DealRCF::mat_, DealRCF::mat_camera_;

s_MatInfo DealRCF::mat_info_;
unsigned char yushi_yuv_data[4096 * 2160 * 3 / 2 + 1];
unsigned long yushi_timestamp;
//在cuda中分配的内存指针
unsigned char *d_rgb_ptr = NULL; //rgb的数据地址
unsigned char *d_yuv_ptr = NULL; //yuv的数据地址

// 各个模块的互斥锁以及信号量
pthread_mutex_t DealRCF::sensor_object_lists_mutex_, DealRCF::mat_object_list_mutex_,
	DealRCF::yushi_mat_object_list_mutex_, DealRCF::yuv_data_mutex_, DealRCF::yuv_time_mutex_;
pthread_rwlock_t DealRCF::yushi_mat_rwlock_;
pthread_cond_t yuv_data_cond_;

// UDP 通信相关参数
struct sockaddr_in DealRCF::send_addr_, DealRCF::send_cloud_addr_, DealRCF::pad_listen_addr, DealRCF::pad_client_addr;
int DealRCF::send_cloud_fd_, DealRCF::send_fd_, DealRCF::pad_fd;
int DealRCF::width_;
int DealRCF::height_;
sem_t DealRCF::pad_debug_sem_;
// atomic_bool DealRCF::is_send_to_pad = false;
atomic_bool DealRCF::is_send_to_pad{false};
// atomic_ullong DealRCF::pad_timestamp1 = 0, DealRCF::pad_timestamp2 = DEBUG_PAD_CYCLE + 100;
atomic_ullong DealRCF::pad_timestamp1{0}, DealRCF::pad_timestamp2{DEBUG_PAD_CYCLE + 100};
atomic_ullong DealRCF::last_frame_time_ms_{0};
atomic_uint DealRCF::frame_counter_{0};

pthread_mutex_t source_lock;
struct timeval source_last_buffer_time;
bool is_source_timeout = false;
using namespace libconfig;

static RTSP_STREAM_TYPE ParseRtspTransport(const std::string& transport)
{
	std::string lower = transport;
	for (char& ch : lower)
	{
		if (ch >= 'A' && ch <= 'Z')
			ch = ch - 'A' + 'a';
	}

	if (lower == "udp")
		return RTSP_STREAM_TYPE_UDP;
	if (lower == "multicast")
		return RTSP_STREAM_TYPE_MULTICAST;

	return RTSP_STREAM_TYPE_TCP;
}

namespace
{
constexpr int kPushRtspPort = 8554;
constexpr int kPushRtspFps = 25;

std::string ExtractRtspStreamName(const std::string& uri)
{
	std::string name = uri;
	std::string::size_type start = 0;
	if (name.compare(0, 7, "rtsp://") == 0)
		start = 7;

	std::string::size_type at = name.find('@', start);
	if (at != std::string::npos)
		start = at + 1;

	name = name.substr(start);
	std::string::size_type slash = name.find('/');
	if (slash != std::string::npos)
		name = name.substr(0, slash);

	std::string::size_type colon = name.find(':');
	if (colon != std::string::npos)
		name = name.substr(0, colon);

	if (name.empty())
		name = "camera";

	for (char& ch : name)
	{
		if (!(std::isalnum(static_cast<unsigned char>(ch)) || ch == '.' || ch == '_' || ch == '-'))
			ch = '_';
	}
	return name;
}

class HardwareRtspStreamer
{
public:
	bool Start(const std::string& suffix, int port, int width, int height)
	{
		if (started_)
			return true;

		port_ = port > 0 ? port : kPushRtspPort;
		width_ = width;
		height_ = height;
		if (width_ <= 0 || height_ <= 0)
		{
			std::cout << "RTSP push invalid frame size: " << width_ << "x" << height_ << std::endl;
			return false;
		}

		event_loop_.reset(new xop::EventLoop());
		server_ = xop::RtspServer::Create(event_loop_.get());
		if (!server_->Start("0.0.0.0", port_))
		{
			std::cout << "RTSP push server listen on " << port_ << " failed." << std::endl;
			server_.reset();
			event_loop_.reset();
			return false;
		}

		xop::MediaSession* session = xop::MediaSession::CreateNew(suffix);
		session->AddSource(xop::channel_0, xop::H264Source::CreateNew(kPushRtspFps));
		session->AddNotifyConnectedCallback([](xop::MediaSessionId, std::string peer_ip, uint16_t peer_port) {
			std::cout << "RTSP push client connect, ip=" << peer_ip << ", port=" << peer_port << std::endl;
		});
		session->AddNotifyDisconnectedCallback([](xop::MediaSessionId, std::string peer_ip, uint16_t peer_port) {
			std::cout << "RTSP push client disconnect, ip=" << peer_ip << ", port=" << peer_port << std::endl;
		});

		session_id_ = server_->AddSession(session);
		if (session_id_ == 0)
		{
			std::cout << "RTSP push add session failed: " << suffix << std::endl;
			server_.reset();
			event_loop_.reset();
			return false;
		}

		encoder_.reset(new whale::vision::MppEncoder());
		encoder_->MppEncdoerInit(width_, height_, kPushRtspFps);
		encoded_.resize(static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3);
		yuv_size_ = width_ * height_ * 3 / 2;
		suffix_ = suffix;
		started_ = true;
		std::cout << "RTSP push stream ready: rtsp://127.0.0.1:" << port_
		          << "/" << suffix_ << std::endl;
		return true;
	}

	bool Push(const cv::Mat& bgr)
	{
		if (!started_ || bgr.empty())
			return false;

		if (server_->GetSessionClientCount(session_id_) == 0)
			return true;

		cv::Mat frame = bgr;
		if (frame.cols != width_ || frame.rows != height_)
			cv::resize(frame, frame, cv::Size(width_, height_));

		cv::Mat yuv;
		cv::cvtColor(frame, yuv, cv::COLOR_BGR2YUV_I420);
		if (!yuv.isContinuous())
			yuv = yuv.clone();

		int length = 0;
		if (encoder_->encode(yuv.data, yuv_size_, encoded_.data(), &length) != MPP_OK || length <= 0)
			return false;

		xop::AVFrame video_frame(length);
		video_frame.type = 0;
		video_frame.size = static_cast<uint32_t>(length);
		video_frame.timestamp = xop::H264Source::GetTimestamp();
		memcpy(video_frame.buffer.get(), encoded_.data(), static_cast<size_t>(length));
		return server_->PushFrame(session_id_, xop::channel_0, video_frame);
	}

private:
	bool started_ = false;
	int port_ = kPushRtspPort;
	int width_ = 0;
	int height_ = 0;
	int yuv_size_ = 0;
	std::string suffix_;
	std::unique_ptr<xop::EventLoop> event_loop_;
	std::shared_ptr<xop::RtspServer> server_;
	xop::MediaSessionId session_id_ = 0;
	std::unique_ptr<whale::vision::MppEncoder> encoder_;
	std::vector<char> encoded_;
};
}




DealRCF::DealRCF()
{
	sem_init(&pad_debug_sem_, 0, 0);
}

// 加载配置文件
bool DealRCF::LoadConfig(const char *filename)
{
	Config cfg;
	cout << "Load config file: " << filename << endl;
	try
	{
		// 打开并读配置文件
		cfg.readFile(filename);
	}
	catch (const FileIOException &fioex)
	{
		cerr << "I/O error while reading file. " << endl;
		return (EXIT_FAILURE);
	}
	catch (const ParseException &pex)
	{
		cerr << "Parse error at " << endl;
		return (EXIT_FAILURE);
	}

	try
	{
		// 得到名字里面的值
		string name = "";
		cfg.lookupValue("name",name);
		cout << "Store name = " << name << endl;
	}
	catch (const SettingNotFoundException &nfex)
	{
		cerr << "No 'name' setting in configuration file" << endl;
		return (EXIT_FAILURE);
	}

	// 得到配置文件的根节点
	const Setting &root = cfg.getRoot();

	try
	{
		const Setting &rsu = root["node"];
		string CameraConfig, RsuIp, CloudIp, CameraURI, DeviceId, LedWarningText;
		int RsuPort, CloudPort, CameraShow, SendSensorType, IsSaveData, SendTime;
		if (!(rsu.lookupValue("CameraConfig", CameraConfig) &&
			  rsu.lookupValue("SendSensorType", SendSensorType) &&
			  rsu.lookupValue("RsuIp", RsuIp) &&
			  rsu.lookupValue("CloudIp", CloudIp) &&
			  rsu.lookupValue("CameraURI", CameraURI) &&
			  rsu.lookupValue("CameraShow", CameraShow) &&
			  rsu.lookupValue("IsSaveData", IsSaveData) &&
			  rsu.lookupValue("CloudPort", CloudPort) &&
			  rsu.lookupValue("SendTime", SendTime) &&
			  rsu.lookupValue("ListenPadPort", DealRCF::deal_info_.ListenPadPort) &&
			  rsu.lookupValue("IsPadDebug", DealRCF::deal_info_.IsPadDebug) &&
			  rsu.lookupValue("CameraLength", DealRCF::deal_info_.CameraLength) &&
			  rsu.lookupValue("GetMatMode", DealRCF::deal_info_.GetMatMode) &&
			  rsu.lookupValue("CameraWidth", DealRCF::deal_info_.CameraWidth) &&
			  rsu.lookupValue("location_angle", DealRCF::deal_info_.location_angle) &&
			  rsu.lookupValue("location_longitude", DealRCF::deal_info_.location_longitude) &&
			  rsu.lookupValue("location_latitude", DealRCF::deal_info_.location_latitude) &&
			  rsu.lookupValue("IsGcj02", DealRCF::deal_info_.IsGcj02) &&
			  rsu.lookupValue("RsuPort", RsuPort)))
		{
			cerr << "waveconfig.cfg error " << endl;
			return ERROR_READ_CONFIG;
		}
		if(rsu.lookupValue("DeviceId",DeviceId)){
			DealRCF::deal_info_.DeviceID = DeviceId;
		}else{
			DealRCF::deal_info_.DeviceID = "device_id";
		}

		int sendModel = 0;
		if(rsu.lookupValue("SendModel",sendModel)){
			DealRCF::deal_info_.SendModel = sendModel;
		}else{
			DealRCF::deal_info_.SendModel = 0; 
		}
		// if(DealRCF::deal_info_.SendModel >= 3){
		// 	string PortName;
		// 	int BaudRate;
		// 	if(rsu.lookupValue("BaudRate",BaudRate) && rsu.lookupValue("PortName",PortName) ){
		// 		DealRCF::deal_info_.BaudRate = BaudRate;
		// 		DealRCF::deal_info_.PortName = PortName;
		// 		serialSend = new SerialPortCommunication(DealRCF::deal_info_.PortName, DealRCF::deal_info_.BaudRate);
		// 	}else{
		// 		cerr << "cfg BaudRate or PortName error " << endl;
		// 		return ERROR_READ_CONFIG;
		// 	}
		// }

		DealRCF::deal_info_.CameraConfig = CameraConfig;
		DealRCF::deal_info_.SendSensorType = SEND_SENSOR_CAMERA;
		RsuIp.copy(DealRCF::deal_info_.RsuIp, IP_NUM, 0);
		*(DealRCF::deal_info_.RsuIp + IP_NUM - 1) = '\0';
		CloudIp.copy(DealRCF::deal_info_.CloudIp, IP_NUM, 0);
		*(DealRCF::deal_info_.CloudIp + IP_NUM - 1) = '\0';
		DealRCF::deal_info_.RsuPort = RsuPort;
		DealRCF::deal_info_.CloudPort = CloudPort;
		DealRCF::deal_info_.CameraURI = CameraURI;
		DealRCF::deal_info_.CameraShow = CameraShow;
		DealRCF::deal_info_.SendTime = SendTime;
		if(!rsu.lookupValue("RtspPushPort", DealRCF::deal_info_.RtspPushPort) || DealRCF::deal_info_.RtspPushPort <= 0){
			DealRCF::deal_info_.RtspPushPort = kPushRtspPort;
		}
		if(!rsu.lookupValue("RtspTransport", DealRCF::deal_info_.RtspTransport) || DealRCF::deal_info_.RtspTransport.empty()){
			DealRCF::deal_info_.RtspTransport = "tcp";
		}
		if(!rsu.lookupValue("ReconnectTimes", DealRCF::deal_info_.ReconnectTimes)){
			DealRCF::deal_info_.ReconnectTimes = 0;
		}
		if(!rsu.lookupValue("SourceTimeoutSec", DealRCF::deal_info_.SourceTimeoutSec) || DealRCF::deal_info_.SourceTimeoutSec <= 0){
			DealRCF::deal_info_.SourceTimeoutSec = 10;
		}
		if(rsu.lookupValue("LedWarningText", LedWarningText) && !LedWarningText.empty()){
			DealRCF::deal_info_.LedWarningText = LedWarningText;
		}else{
			DealRCF::deal_info_.LedWarningText = "前方目标闯入";
		}

		if(!rsu.lookupValue("LedIp", DealRCF::deal_info_.LedIp) || DealRCF::deal_info_.LedIp.empty()){
			DealRCF::deal_info_.LedIp = "192.168.88.31";
		}
		if(!rsu.lookupValue("LedPort", DealRCF::deal_info_.LedPort)){
			DealRCF::deal_info_.LedPort = 30080;
		}
		if(!rsu.lookupValue("LedSdkKey", DealRCF::deal_info_.LedSdkKey)){
			DealRCF::deal_info_.LedSdkKey = "";
		}
		if(!rsu.lookupValue("LedSdkSecret", DealRCF::deal_info_.LedSdkSecret)){
			DealRCF::deal_info_.LedSdkSecret = "";
		}
		if(!rsu.lookupValue("LedDeviceId", DealRCF::deal_info_.LedDeviceId)){
			DealRCF::deal_info_.LedDeviceId = "";
		}

		LedScreen::Instance().Init(
			DealRCF::deal_info_.LedIp,
			DealRCF::deal_info_.LedPort,
			DealRCF::deal_info_.LedSdkKey,
			DealRCF::deal_info_.LedSdkSecret,
			DealRCF::deal_info_.LedDeviceId
		);
#if IS_SN_PROJECT
		if(DealRCF::deal_info_.CameraURI.find("@")!= std::string::npos)
		{
			string camera = DealRCF::deal_info_.CameraURI;
			camera.erase(0,camera.find_first_of("@")+1);
			DealRCF::deal_info_.DeviceID = camera;
		}else{
			DealRCF::deal_info_.DeviceID = "device_id";
		}
#endif
		switch(IsSaveData)
		{
			case 2:
				DealRCF::deal_info_.IsSaveData = SAVE_CAMERA_RESULT_VIDEO;
				break;
			default:
				DealRCF::deal_info_.IsSaveData = DONOT_SAVE;
		}		
	}

	catch (const SettingNotFoundException &nfex)
	{
		// Ignore
		cout << "Ignore" << endl;
	}

	cout << "---- Main DealRCF config ----  " << endl;
	cout << "CameraConfig: " << DealRCF::deal_info_.CameraConfig << endl;
	cout << "SendSensorType: " << DealRCF::deal_info_.SendSensorType << endl;
	cout << "CameraShow: " << DealRCF::deal_info_.CameraShow << endl;
	cout << "RsuIp: " << DealRCF::deal_info_.RsuIp << endl;
	cout << "RsuPort: " << DealRCF::deal_info_.RsuPort << endl;
	cout << "CloudIp: " << DealRCF::deal_info_.CloudIp << endl;
	cout << "CloudPort: " << DealRCF::deal_info_.CloudPort << endl;
	cout << "CameraURI: " << DealRCF::deal_info_.CameraURI << endl;
	cout << "LedWarningText: " << DealRCF::deal_info_.LedWarningText << endl;
	cout << "LedIp: " << DealRCF::deal_info_.LedIp << endl;
	cout << "LedPort: " << DealRCF::deal_info_.LedPort << endl;
	cout << "LedDeviceId: " << DealRCF::deal_info_.LedDeviceId << endl;
	cout << "CameraShow: " << DealRCF::deal_info_.CameraShow << endl;
	cout << "IsSaveData: " << DealRCF::deal_info_.IsSaveData << endl;
	cout << "SendTime: " << DealRCF::deal_info_.SendTime << endl;
	cout << "GetMatMode: " << DealRCF::deal_info_.GetMatMode << endl;
	cout << "RtspPushPort: " << DealRCF::deal_info_.RtspPushPort << endl;
	cout << "RtspTransport: " << DealRCF::deal_info_.RtspTransport << endl;
	cout << "ReconnectTimes: " << DealRCF::deal_info_.ReconnectTimes << endl;
	cout << "SourceTimeoutSec: " << DealRCF::deal_info_.SourceTimeoutSec << endl;
	printf("location_angle: %lf \n",DealRCF::deal_info_.location_angle);
	printf("location_longitude: %lf \n",DealRCF::deal_info_.location_longitude);
	printf("location_latitude: %lf \n",DealRCF::deal_info_.location_latitude);

	return (EXIT_SUCCESS);
}

/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/***********************************************各SDK模块开启与调用***************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/

// 开启视觉线程
int DealRCF::StartCameraDealThread(void *arg)
{
	pthread_t pid;
	int ret;
	typedef void *(*FUNC)(void *);
	FUNC camera_callback = (FUNC)&DealRCF::CameraDealThread;
	if ((ret = pthread_create(&pid, NULL, camera_callback, this)) != 0)
	{
		cerr << "StartCameraDealThread error ... " << endl;
		return RETURN_NEGATIVE;
	}

	return ret;
}

// 视觉线程执行体
void *DealRCF::CameraDealThread(void *arg)
{
	camera::CameraAPP camera_app;
	char config_path[CONFIG_PATH_LEN];
	memset(config_path, 0, CONFIG_PATH_LEN);
	DealRCF::deal_info_.CameraConfig.copy(config_path, DealRCF::deal_info_.CameraConfig.length(), 0);
	*(config_path + DealRCF::deal_info_.CameraConfig.length()) = '\0';

	cv::Mat img;
	unsigned long long time1, time2, time3 = GetMSecondTimeFrom1970(), timestamp, old_timestamp = 0;
	unsigned long long last_led_warn_time = 0;

	bool ret = camera_app.Init(config_path, DealRCF::deal_info_.CameraLength, DealRCF::deal_info_.CameraWidth, DealRCF::deal_info_.CameraURI,
					DealRCF::deal_info_.location_angle,DealRCF::deal_info_.location_longitude,DealRCF::deal_info_.location_latitude);

	if(!ret)
	{
		cout << "ERROR:  camera_app init error\n";
		exit(0);
	}

	//是否显示视频框: 单视觉模式，显示等于1
	bool is_show = (SEND_SENSOR_CAMERA == DealRCF::deal_info_.SendSensorType) && (1==DealRCF::deal_info_.CameraShow);

    //保存结果视频或推带框 RTSP 流
	cv::VideoWriter writer;
    bool is_push_rtsp = (4 == DealRCF::deal_info_.CameraShow) && (SEND_SENSOR_CAMERA == DealRCF::deal_info_.SendSensorType);
	bool is_save = (SAVE_CAMERA_RESULT_VIDEO == DealRCF::deal_info_.IsSaveData);
	HardwareRtspStreamer rtsp_streamer;
	bool rtsp_streamer_started = false;
	std::string rtsp_suffix;
	// std::cout << "********is_push_rtsp: " << is_push_rtsp << std::endl;
	if(is_save)
	{
        std::string file_name, pipeline;
		std::string::size_type pos;
        //rtsp流，获取摄像机的ip
        if(DealRCF::deal_info_.CameraURI.find("rtsp") != std::string::npos)
        {
            pos = DealRCF::deal_info_.CameraURI.find("@");
            file_name = DealRCF::deal_info_.CameraURI.substr(pos+1, DealRCF::deal_info_.CameraURI.length());

            pos = file_name.find(":");
            if(pos!=std::string::npos)
                file_name = file_name.substr(0, pos);

            std::cout << "camera_ip: " << file_name << std::endl;
        } 
        else  //视频获取名称
        {
            pos = DealRCF::deal_info_.CameraURI.rfind("/");
            file_name = DealRCF::deal_info_.CameraURI.substr(pos+1, DealRCF::deal_info_.CameraURI.length());

            pos = file_name.rfind(".");
            if(pos!=std::string::npos)
                file_name = file_name.substr(0, pos);

            std::cout << "video_name: " << file_name << std::endl;
        }

		std::time_t currentTime = std::time(nullptr);
		// 将时间转换为本地时间
		std::tm* localTime = std::localtime(&currentTime);
		// 格式化时间为字符串
		char formattedTime[100];
		std::strftime(formattedTime, sizeof(formattedTime), "%Y%m%d%H%M%S", localTime);
        std::string temp = "result_camera_" + file_name + "_" + std::string(formattedTime) + ".mkv";
        // pipeline="appsrc ! video/x-raw, format=BGR ! timeoverlay ！videoconvert ! queue ! nvvidconv ! nvv4l2h265enc ! h265parse ! matroskamux ! filesink location="+temp;
        // pipeline="appsrc ! video/x-raw, format=BGR ! videoconvert ! queue ! nvvidconv ! nvv4l2h265enc ! h265parse ! mp4mux ! filesink location=camera_result.mp4";
		pipeline = "appsrc is-live=true ! video/x-raw,format=BGR ! timeoverlay ! videoconvert ! queue ! nvvidconv ! nvv4l2h265enc ! h265parse ! matroskamux ! filesink location="+temp;

		writer.open(pipeline, cv::CAP_GSTREAMER, cv::VideoWriter::fourcc('h', '2', '6', '5'), 25, cv::Size(DealRCF::deal_info_.CameraLength, DealRCF::deal_info_.CameraWidth), true);
 		if(!writer.isOpened())
		{
			std::cout << "writer video open error in camera thread...\n";
			is_save = false;
		}
	}
	if(is_push_rtsp)
	{
		rtsp_suffix = ExtractRtspStreamName(DealRCF::deal_info_.CameraURI) + "/camera";
		std::cout << "push live stream uri: rtsp://127.0.0.1:" << DealRCF::deal_info_.RtspPushPort
		          << "/" << rtsp_suffix << std::endl;
	}

	while (true)
	{
		// time1 = GetMSecondTimeFrom1970();
		pthread_mutex_lock(&DealRCF::mat_object_list_mutex_);
		
		pthread_rwlock_rdlock(&DealRCF::yushi_mat_rwlock_);

		// img = DealRCF::mat_info_.mat.clone();
		DealRCF::mat_info_.mat.copyTo(img);
	
		timestamp = DealRCF::mat_info_.timestamp;

		pthread_rwlock_unlock(&DealRCF::yushi_mat_rwlock_);

		pthread_mutex_unlock(&DealRCF::mat_object_list_mutex_);
		if (timestamp == old_timestamp)
		{
			usleep(CAMERA_DEAL_TIME);
			continue;
		}
		
		old_timestamp = timestamp;
		if (img.empty())
		{
			// std::cout << "in camera thread: img is empty..\n";
			usleep(MAT_GET_TIME);
			continue;
		}
		perceptron::s_ObjectList objects;
		time1 = GetMSecondTimeFrom1970();
		camera_app.Process(img, (uint64_t)timestamp, &objects);
		objects.time_stamp = timestamp;
		time2 = GetMSecondTimeFrom1970();
		// printf("[CAMERA] camera deal time: %ld\n", time2-time1);

		camera_app.DrawResult(img, &objects);
		if(is_push_rtsp)
		{
			if(!rtsp_streamer_started)
				rtsp_streamer_started = rtsp_streamer.Start(rtsp_suffix, DealRCF::deal_info_.RtspPushPort, img.cols, img.rows);
			if(rtsp_streamer_started)
				rtsp_streamer.Push(img);
		}
		if(is_save)
			writer << img;
		
		// 显示
		if (is_show)
		{
			static bool is_init_win = false;
			if (!is_init_win)
			{
				cv::namedWindow("camera", cv::WINDOW_NORMAL);
				// cv::resizeWindow("camera", 1280, 720);
				is_init_win = true;
			}
			// cv::Mat resized;
			// cv::resize(img, resized, cv::Size(DealRCF::deal_info_.CameraLength * MAT_RESIZE, DealRCF::deal_info_.CameraWidth * MAT_RESIZE));
			cv::imshow("camera", img);
			if (cv::waitKey(C_WAITKEY_TIME) == 'q')
				break;
		}
		objects.sensor_type = SensorType_CAMERA;
		if (objects.object_list.empty())
		{
			continue;
		}
#if 0
		// 复值
		AveCopyObjectList(objects);
		// 检查是否超过阈值
		AveJudgeSensorObjectListsSize();
#endif
		if (SEND_SENSOR_CAMERA == DealRCF::deal_info_.SendSensorType)
		{
			// SerializationsObjectListSend(objects, 3);
			// PrintObjectList(objects, "CAMERA");
			time2 = GetMSecondTimeFrom1970();			
			if (DealRCF::deal_info_.SendTime <= (time2 - time3))
			{
				SerializationsObjectListSend(objects, 3);
				// AveSerializationsObjectListSend(ave_object_list_, 3);
				// PrintObjectList(objects, "CAMERA");
				// printf("send camera------%ld\n", time2 - time3);
				time3 = GetMSecondTimeFrom1970();
			}
			if((!objects.object_list.empty()) && (time2 - last_led_warn_time >= LED_WARN_INTERVAL_MS))
			{
				std::cout << "target detected, send led warning: " << DealRCF::deal_info_.LedWarningText << std::endl;
				LedScreen::Instance().ShowText(DealRCF::deal_info_.LedWarningText);
				last_led_warn_time = time2;
			}
			objects.object_list.clear();

			continue;
		}

		PrintObjectList(objects, "CAMERA");
		pthread_mutex_lock(&DealRCF::sensor_object_lists_mutex_);
		JudgeSensorObjectListsSize(&DealRCF::sensor_object_lists_);
		DealRCF::sensor_object_lists_.push_back(objects);
		// objects.object_list.clear();
		// cout << "DealRCF::sensor_object_lists_ push_back end size() =" << DealRCF::sensor_object_lists_.size() << endl;
		pthread_mutex_unlock(&DealRCF::sensor_object_lists_mutex_);
		time2 = GetMSecondTimeFrom1970();
		// printf("camera------%ld\n", time2 - time1);
	}

	if(is_show)
		cv::destroyAllWindows();

	if(is_save)
		writer.release();
}

// 开启获取图像线程（拉流）
int DealRCF::StartMatDealThread(void *arg)
{

	pthread_t pid;
	int ret;
	typedef void *(*FUNC)(void *);
	FUNC mat_callback; // = (FUNC)&DealRCF::MatDealThread;

	mat_callback = (FUNC)&DealRCF::MatDealFfmediaThread;

	if ((ret = pthread_create(&pid, NULL, mat_callback, this)) != 0)
	{
		cerr << "StartCameraDealThread error ... " << endl;
		return RETURN_NEGATIVE;
	}

	return ret;
}

void GetFrameRawData(cv::Mat &img,uint64_t& timestamp)
{
	pthread_rwlock_wrlock(&DealRCF::yushi_mat_rwlock_);
	// DealRCF::mat_info_.timestamp = GetMSecondTimeFrom1970();
	// printf("%s(%d)-<%s>: \n",__FILE__, __LINE__, __FUNCTION__);
	DealRCF::mat_info_.timestamp = timestamp;
	img.copyTo(DealRCF::mat_info_.mat);
	// printf("GetImg timestamp_ = %ld\n ",timestamp);
	pthread_rwlock_unlock(&DealRCF::yushi_mat_rwlock_);
}


void *DealRCF::MatDealFfmediaThread(void *arg)
{
	FfmediaInit(DealRCF::deal_info_.CameraURI);
	return NULL;
}
// 获取图像线程执行体
void *DealRCF::MatDealFFmpegThread(void *arg)
{
	std::cout << "mat deal by ffmpeg thread start!\n" << std::endl;
	unsigned long long time1, time2;
	cv::Mat img;
        img.create(DealRCF::deal_info_.CameraWidth, DealRCF::deal_info_.CameraLength, CV_8UC3);

	cv::VideoCapture cap;

	cap.open(DealRCF::deal_info_.CameraURI, cv::CAP_FFMPEG);

	int width = (int)cap.get(3); 
	int height = (int)cap.get(4);
	int fps = (int)cap.get(5);
	cap.release();
	std::cout << "cap width = " << width << "  height=" << height << "  fps=" << fps << std::endl;

	if(DealRCF::deal_info_.CameraLength<width || DealRCF::deal_info_.CameraWidth<height)
	{
		DealRCF::deal_info_.CameraLength = width;
		DealRCF::deal_info_.CameraWidth = height;
	}

	camera::VideoDecoder dec;
	dec.Init(DealRCF::deal_info_.CameraURI, DealRCF::deal_info_.CameraLength, DealRCF::deal_info_.CameraWidth, DealRCF::deal_info_.GetMatMode); 
	uint64_t timestamp_;
	dec.Run(GetFrameRawData);

	// while (true)
	// {
	// 	// printf("%s(%d)-<%s>: ",__FILE__, __LINE__, __FUNCTION__);
	// 	dec.GetImg(img, timestamp_);
	// 	// printf("%s(%d)-<%s>: ",__FILE__, __LINE__, __FUNCTION__);
	// 	//cap >> img;
	// 	if (img.empty())
	// 	{
	// 		cout << "camera disconect, then reconect camera ... \n";
	// 		sleep(5);
	// 	}
	// 	printf("GetImg timestamp_ = %ld\n ",timestamp_);
	// 	// printf("%s(%d)-<%s>: ",__FILE__, __LINE__, __FUNCTION__);
	// 	pthread_rwlock_wrlock(&DealRCF::yushi_mat_rwlock_);
	// 	// DealRCF::mat_info_.timestamp = GetMSecondTimeFrom1970();
	// 	DealRCF::mat_info_.timestamp = timestamp_;
	// 	img.copyTo(DealRCF::mat_info_.mat);
	// 	pthread_rwlock_unlock(&DealRCF::yushi_mat_rwlock_);

	// }
	
	printf("MatDealFFmpegThread thread exit....\n");

	return NULL;
}

void callback_external(void* _ctx, shared_ptr<MediaBuffer> buffer)
{
    External_ctx* ctx = static_cast<External_ctx*>(_ctx);
    shared_ptr<ModuleMedia> module = ctx->module;

    if (buffer == NULL || buffer->getMediaBufferType() != BUFFER_TYPE_VIDEO)
        return;
    shared_ptr<VideoBuffer> buf = static_pointer_cast<VideoBuffer>(buffer);

    void* ptr = buf->getActiveData();
    size_t size = buf->getActiveSize();
    uint32_t width = buf->getImagePara().hstride;
    uint32_t height = buf->getImagePara().vstride;
    // flush dma buf to cpu
    buf->invalidateDrmBuf();

    UNUSED(size);
    //=================================================

	unsigned long long now_ms = DealRCF::GetMSecondTimeFrom1970();
	pthread_rwlock_wrlock(&DealRCF::yushi_mat_rwlock_);
	DealRCF::mat_info_.timestamp = now_ms;
	cv::Mat img(cv::Size(width, height), CV_8UC3, ptr);
	img.copyTo(DealRCF::mat_info_.mat);
	pthread_rwlock_unlock(&DealRCF::yushi_mat_rwlock_);
	DealRCF::last_frame_time_ms_.store(now_ms, std::memory_order_relaxed);
	DealRCF::frame_counter_.fetch_add(1, std::memory_order_relaxed);
	#if 0
    cout << "dds_debug: width = " << width << ", height = " << height << endl;
    cv::Mat mat(cv::Size(width, height), CV_8UC3, ptr);
    cv::imshow(module->getName(), mat);
    cv::waitKey(1);
    //=================================================
	#endif
}
int DealRCF::FfmediaInit(string rtsp)
{
    int reconnect_count = 0;
    RTSP_STREAM_TYPE stream_type = ParseRtspTransport(DealRCF::deal_info_.RtspTransport);

    while (DealRCF::deal_info_.ReconnectTimes <= 0 ||
           reconnect_count <= DealRCF::deal_info_.ReconnectTimes)
    {
        int ret;
        bool started = false;
        shared_ptr<ModuleRtspClient> rtsp_c = NULL;
        shared_ptr<ModuleMppDec> dec = NULL;
        shared_ptr<ModuleRga> rga = NULL;
        ImagePara input_para;
        ImagePara output_para;
        External_ctx* ctx1 = NULL;

        std::cout << "ffmedia rtsp start, transport=" << DealRCF::deal_info_.RtspTransport
                  << ", reconnect_count=" << reconnect_count << std::endl;

        rtsp_c = make_shared<ModuleRtspClient>(rtsp, stream_type);
        rtsp_c->setTimeOutSec(2, 0);
        ret = rtsp_c->init();
        if (ret < 0) {
            ff_error("rtsp client init failed\n");
            goto RETRY;
        }

        input_para = rtsp_c->getOutputImagePara();
        dec = make_shared<ModuleMppDec>(input_para);
        dec->setProductor(rtsp_c);
        ret = dec->init();
        if (ret < 0) {
            ff_error("Dec init failed\n");
            goto RETRY;
        }

        input_para = dec->getOutputImagePara();
        output_para = input_para;
        output_para.width = input_para.width;
        output_para.height = input_para.height;
        output_para.hstride = output_para.width;
        output_para.vstride = output_para.height;
        output_para.v4l2Fmt = V4L2_PIX_FMT_BGR24;
        rga = make_shared<ModuleRga>(input_para, output_para, RGA_ROTATE_NONE);
        rga->setProductor(dec);
        ret = rga->init();
        if (ret < 0) {
            ff_error("rga init failed\n");
            goto RETRY;
        }

        ctx1 = new External_ctx();
        ctx1->module = rga;
        rga->setOutputDataCallback(ctx1, callback_external);
        DealRCF::last_frame_time_ms_.store(DealRCF::GetMSecondTimeFrom1970(), std::memory_order_relaxed);
        rtsp_c->start();
        started = true;

        while (true)
        {
            sleep(1);
            unsigned long long now_ms = DealRCF::GetMSecondTimeFrom1970();
            unsigned long long last_ms = DealRCF::last_frame_time_ms_.load(std::memory_order_relaxed);
            if (now_ms > last_ms &&
                now_ms - last_ms > (unsigned long long)DealRCF::deal_info_.SourceTimeoutSec * 1000)
            {
                ff_error("rtsp source no frame timeout, reconnecting\n");
                break;
            }
        }

RETRY:
        if (started)
            rtsp_c->stop();
        if (ctx1)
            delete ctx1;

        reconnect_count++;
        if (DealRCF::deal_info_.ReconnectTimes > 0 &&
            reconnect_count > DealRCF::deal_info_.ReconnectTimes)
        {
            break;
        }
        sleep(RTSP_RECONNECT_INTERVAL_SEC);
    }

    ff_error("ffmedia rtsp stopped after reconnect limit\n");
    return RETURN_NEGATIVE;
}

/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/*************************************调试工具接口，如pad/图像显示，图像保存等*******************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/


// 开启与pad交互线程
int DealRCF::StartDealPadThread(void *arg)
{
	pthread_t pid;
	int ret;
	typedef void *(*FUNC)(void *);
	FUNC deal_pad_callback = (FUNC)&DealRCF::PadDealThread;
	if ((ret = pthread_create(&pid, NULL, deal_pad_callback, this)) != 0)
	{
		cerr << "StartDealPadThread error ... " << endl;
		return RETURN_NEGATIVE;
	}
	sleep(1);
	FUNC send_pad_callback = (FUNC)&DealRCF::SendDealThread;
	if ((ret = pthread_create(&pid, NULL, send_pad_callback, this)) != 0)
	{
		cerr << "StartSendDealThread ... " << endl;
		return RETURN_NEGATIVE;
	}

	return ret;
}

// 发给pad心跳信息
void *DealRCF::SendDealThread(void *arg)
{
	string send_str = "mec process is online ...\n";
	char recv_buf[gMaxPacketSize];
	int sendto_len = 0;

	while (true)
	{

		if ((true == DealRCF::is_send_to_pad) && (DEBUG_PAD_CYCLE > (DealRCF::pad_timestamp2 - DealRCF::pad_timestamp1)))
		{
			// recv_buf = send_str.c_str();
			strncpy(recv_buf, send_str.c_str(), strlen(send_str.c_str()));
			// recv_buf[sizeof(send_str) + 1] = '\0';
			if (RETURN_ZERO > (sendto_len = sendto(DealRCF::pad_fd, recv_buf, strlen(send_str.c_str()), MSG_DONTWAIT,
												   (struct sockaddr *)&DealRCF::pad_client_addr, sizeof(DealRCF::pad_client_addr))))
			{
				cerr << "sendto heart message to pad error .. ..." << endl;
				return ERROR_SEND_TO;
			}
			else
			{
				printf("send %d data successfully, pad ip = %s, port = %d\n", sendto_len, inet_ntoa(DealRCF::pad_client_addr.sin_addr), ntohs(DealRCF::pad_client_addr.sin_port));
			}
		}
		else
		{
			DealRCF::is_send_to_pad = false;
		}
		sleep(1);
		DealRCF::pad_timestamp2 = GetMSecondTimeFrom1970();
	}
}

// 与pad交互线程获取图像线程执行体
void *DealRCF::PadDealThread(void *arg)
{
	// 创建套接字
	int num = 0;
	char recv_buf[gMaxPacketSize];
	socklen_t addrlen = sizeof(DealRCF::pad_client_addr);
	// DealRCF::pad_timestamp1 = DealRCF::pad_timestamp2 = GetMSecondTimeFrom1970();

	while ((num = recvfrom(DealRCF::pad_fd, recv_buf, gMaxPacketSize, 0, (struct sockaddr *)&DealRCF::pad_client_addr, &addrlen)) > 0)
	{
		if (0x00 == recv_buf[0] && 0x00 == recv_buf[1] && 0x00 == recv_buf[2] && 0x01 == recv_buf[3])
		{
			DealRCF::pad_timestamp1 = GetMSecondTimeFrom1970();
			DealRCF::is_send_to_pad = true;
		}
	}
}

// 开启录制离线数据线程
int DealRCF::StartWriteDataThread(void *arg)
{
}

// 录制离线数据线程执行体
void *DealRCF::WriteDataThread(void *arg)
{
}



/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/***********************************************UDP初始化及数据发送接口************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/

// 初始化UDP
int DealRCF::InitUDP(void *pargma)
{
	// 创建套接字
	int fd_send, fd_cloud_send, fd_pad;
	if ((fd_send = socket(AF_INET, SOCK_DGRAM, 0)) == RETURN_NEGATIVE)
	{
		cerr << "Creating socket fd failed." << endl;
		return RETURN_NEGATIVE;
	}

	if ((fd_cloud_send = socket(AF_INET, SOCK_DGRAM, 0)) == RETURN_NEGATIVE)
	{
		cerr << "Creating socket cloud fd failed." << endl;
		return RETURN_NEGATIVE;
	}

	if ((fd_pad = socket(AF_INET, SOCK_DGRAM, 0)) == RETURN_NEGATIVE)
	{
		cerr << "Creating socket pad fd failed." << endl;
		return RETURN_NEGATIVE;
	}

	// 填充网络信息结构体
	struct sockaddr_in server_add, server_cloud_add, pad_listen_add;

	socklen_t addrlen = sizeof(server_add);
	bzero(&server_add, sizeof(server_add));
	server_add.sin_family = AF_INET;
	server_add.sin_addr.s_addr = inet_addr(DealRCF::deal_info_.RsuIp);
	server_add.sin_port = htons(DealRCF::deal_info_.RsuPort);
	DealRCF::send_addr_ = server_add;
	DealRCF::send_fd_ = fd_send;

	bzero(&server_add, sizeof(server_cloud_add));
	server_cloud_add.sin_family = AF_INET;
	server_cloud_add.sin_addr.s_addr = inet_addr(DealRCF::deal_info_.CloudIp);
	server_cloud_add.sin_port = htons(DealRCF::deal_info_.CloudPort);
	DealRCF::send_cloud_addr_ = server_cloud_add;
	DealRCF::send_cloud_fd_ = fd_cloud_send;
	// 开启与pad端联调线程
	if (1 == DealRCF::deal_info_.IsPadDebug)
	{
	bzero(&pad_listen_add, sizeof(pad_listen_add));
	pad_listen_add.sin_family = AF_INET;
	pad_listen_add.sin_addr.s_addr = htonl(INADDR_ANY);
	pad_listen_add.sin_port = htons(DealRCF::deal_info_.ListenPadPort);
	// 将套接字和网络信息结构体绑定
	if (bind(fd_pad, (struct sockaddr *)&pad_listen_add, sizeof(pad_listen_add)) < 0)

	{
		cerr << "bind listen pad port error ..." << endl;
		exit(0);
	}
	DealRCF::pad_listen_addr = pad_listen_add;
	DealRCF::pad_fd = fd_pad;
	}

	return RETURN_ZERO;
}

// 打包成protobuf发出
int DealRCF::SerializationsObjectListSend(perceptron::s_ObjectList object_list_, int type)
{
	//将目标加入事件线程
	// tracker_cw.EventAppProcess(object_list_);
#if IS_SN_PROJECT
	if(DealRCF::deal_info_.SendTime == 0)
		return 0;
#endif 
	nebulalink::perceptron::PerceptronSet perceptrons = nebulalink::perceptron::PerceptronSet();
	unsigned char send2fusion[gMaxPacketSize] = {0};
	// nebulalink::perceptron::Perceptron *perceptron = new nebulalink::perceptron::Perceptron();

	unsigned long long m_time = GetMSecondTime();
	for (unsigned i = 0; i < object_list_.object_list.size(); ++i)
	{
		if(object_list_.object_list[i].object_class_type > 4)
			continue;

		nebulalink::perceptron::Perceptron *perceptron = perceptrons.add_perceptron();

		nebulalink::perceptron::PointGPS *point_gps = perceptron->mutable_point_gps();
		nebulalink::perceptron::TargetSize *target_size = perceptron->mutable_target_size();
		nebulalink::perceptron::Point4 *point4f = perceptron->mutable_point4f();
		nebulalink::perceptron::Acc4Way *acc4_way = perceptron->mutable_accel_4way();

		point_gps->set_object_elevation(object_list_.object_list[i].point_gps.object_elevation);
		if (1 == deal_info_.IsGcj02)
		{
			Wgs84toGcj02(object_list_.object_list[i].point_gps.object_longitude, object_list_.object_list[i].point_gps.object_latitude);
		}
		point_gps->set_object_latitude(object_list_.object_list[i].point_gps.object_latitude);
		point_gps->set_object_longitude(object_list_.object_list[i].point_gps.object_longitude);
		target_size->set_object_height(object_list_.object_list[i].target_size.object_height);
		target_size->set_object_length(object_list_.object_list[i].target_size.object_length);
		target_size->set_object_width(object_list_.object_list[i].target_size.object_width);
		point4f->set_camera_h(object_list_.object_list[i].point4f.camera_h);
		point4f->set_camera_w(object_list_.object_list[i].point4f.camera_w);
		point4f->set_camera_x(object_list_.object_list[i].point4f.camera_x);
		point4f->set_camera_y(object_list_.object_list[i].point4f.camera_y);

		nebulalink::perceptron::Point3 *point3f = perceptron->mutable_point3f();
		point3f->set_x(object_list_.object_list[i].point3f.x);
		point3f->set_y(object_list_.object_list[i].point3f.y);
		point3f->set_z(0);
		//perceptron->set_allocated_point3f(point3f);

		perceptron->set_object_id((object_list_.object_list[i].object_id) % 65534 + 1);
		// perceptron->set_allocated_target_size(target_size);
		// perceptron->set_allocated_point_gps(point_gps);
		// perceptron->set_allocated_point4f(point4f);

		perceptron->set_object_ns(object_list_.object_list[i].object_NS);
		perceptron->set_object_we(object_list_.object_list[i].object_WE);
		perceptron->set_object_direction(object_list_.object_list[i].object_direction);
		perceptron->set_object_heading(object_list_.object_list[i].object_heading);
		perceptron->set_object_speed(object_list_.object_list[i].object_speed);
		perceptron->set_object_class_type(object_list_.object_list[i].object_class_type);
		perceptron->set_ptc_veh_type(object_list_.object_list[i].ptc_veh_type);
		perceptron->set_ptc_veh_color(object_list_.object_list[i].ptc_veh_color);
		perceptron->set_object_acceleration(object_list_.object_list[i].object_acceleration);
		acc4_way->set_acc4waylat(object_list_.object_list[i].object_acceleration * cos(object_list_.object_list[i].object_heading));
		acc4_way->set_acc4waylon(object_list_.object_list[i].object_acceleration * sin(object_list_.object_list[i].object_heading));
		// perceptron->set_allocated_accel_4way(acc4_way);

		// 车牌号
		perceptron->set_plate_num(object_list_.object_list[i].plate_num);
		// 车牌颜色
		perceptron->set_ptc_veh_plate_color(object_list_.object_list[i].ptc_veh_plate_color);
	
		perceptron->set_is_tracker(true);
		perceptron->set_lane_id(object_list_.object_list[i].lane_id);
		perceptron->set_obj_time_stamp(m_time);
		perceptron->set_ptc_sourcetype(type);
	}

	//障碍物列表
	for(int i=0; i<object_list_.obstacle.size(); i++)
	{
		nebulalink::perceptron::Obstacles *obs = perceptrons.add_obstacle();
		//类型
		obs->set_obstype(object_list_.obstacle[i].obstype);
		//置信度
		obs->set_obstype_cfd(object_list_.obstacle[i].obstype_cfd);
		//id
		obs->set_obsid(object_list_.obstacle[i].obsId);
		//来源
		obs->set_obs_source(object_list_.obstacle[i].obs_source);
		//时间戳
		nebulalink::perceptron::TimeBase *time_base = obs->mutable_obs_time_stamp();
		time_base->set_year(object_list_.obstacle[i].obs_time_stamp.year);
		time_base->set_month(object_list_.obstacle[i].obs_time_stamp.month);
		time_base->set_day(object_list_.obstacle[i].obs_time_stamp.day);
		time_base->set_hour(object_list_.obstacle[i].obs_time_stamp.hour);
		time_base->set_min(object_list_.obstacle[i].obs_time_stamp.minute);
		time_base->set_second(object_list_.obstacle[i].obs_time_stamp.second);
		time_base->set_miilsecond(object_list_.obstacle[i].obs_time_stamp.millisecond);
		// obs->set_allocated_obs_time_stamp(time_base);
		//位置
		nebulalink::perceptron::PointDesc *pdesc = obs->mutable_obs_gps();
		pdesc->set_p_longitude(object_list_.obstacle[i].obs_gps.p_longitude);
		pdesc->set_p_latitude(object_list_.obstacle[i].obs_gps.p_latitude);
		pdesc->set_p_altitude(object_list_.obstacle[i].obs_gps.p_altitude);
		// obs->set_allocated_obs_gps(pdesc);
		//速度
		obs->set_obs_speed(object_list_.obstacle[i].obs_speed);
		//航向角
		obs->set_obs_heading(object_list_.obstacle[i].obs_heading);
		//尺寸
		nebulalink::perceptron::TargetSize *size = obs->mutable_obs_size();
		size->set_object_height(object_list_.obstacle[i].obs_size.object_height);
		size->set_object_width(object_list_.obstacle[i].obs_size.object_width);
		size->set_object_length(object_list_.obstacle[i].obs_size.object_length);
		// obs->set_allocated_obs_size(size);
	}
	if(object_list_.object_list.empty())
	{
		for(auto obj:object_list_.link_jam_sense_params)
		{
			nebulalink::perceptron::LinkJamSenseParams *link=perceptrons.add_link_jam_sense_params();
			link->set_link_id(obj.link_id);
			link->set_link_len(obj.link_len);
			link->set_link_avgspeed(obj.link_avgspeed);
			link->set_link_veh_num(obj.link_veh_num);
			link->set_link_space_occupancy(obj.link_space_occupancy);
			link->set_link_time_occupancy(obj.link_time_occupancy);
			link->set_link_motor_volume(obj.link_motor_volume);
			link->set_link_pcu(obj.link_pcu);
			link->set_link_headway(obj.link_headway);
			link->set_link_avstop(obj.link_avstop);
			link->set_link_gap(obj.link_gap);
			link->set_link_queuelength(obj.link_queueLength);
			for(auto lane:obj.road_lanelist)
			{
				nebulalink::perceptron::LaneJamSenseParams *lane_=link->add_road_lanelist();
				lane_->set_lane_id(lane.lane_id);
				lane_->set_lane_sense_len(lane.lane_sense_len);
				lane_->set_lane_avg_speed(lane.lane_avg_speed);
				lane_->set_lane_veh_num(lane.lane_veh_num);
				lane_->set_lane_ave_distance(lane.lane_ave_distance);
				lane_->set_lane_headway(lane.lane_headway);
				lane_->set_lane_avstop(lane.lane_avstop);
				lane_->set_lane_cur_distance(lane.lane_cur_distance);
				lane_->set_lane_peron_volume(lane.lane_peron_volume);
				lane_->set_lane_no_motor_volume(lane.lane_no_motor_volume);
				lane_->set_lane_minmotor_volume(lane.lane_minmotor_volume);
				lane_->set_lane_medmotor_volume(lane.lane_medmotor_volume);
				lane_->set_lane_maxmotor_volume(lane.lane_maxmotor_volume);
				lane_->set_lane_pcu(lane.lane_pcu);
				lane_->set_lane_queuelength(lane.lane_queueLength);

				std::cout<<"lane.linkid = "<<lane.lane_id<<std::endl;
				std::cout<<"lane.lane_avg_speed = "<<lane.lane_avg_speed<<std::endl;
				std::cout<<"lane.lane_veh_num = "<<lane.lane_veh_num<<std::endl;
				std::cout<<"lane.lane_space_occupancy = "<<lane.lane_space_occupancy<<std::endl;
				std::cout<<"lane.lane_minmotor_volume = "<<lane.lane_minmotor_volume<<std::endl;
				std::cout<<"lane.lane_pcu = "<<lane.lane_pcu<<std::endl;
				std::cout<<"lane.lane_headway = "<<lane.lane_headway<<std::endl;
				std::cout<<"lane.lane_avstop = "<<lane.lane_avstop<<std::endl;
				std::cout<<"lane.lane_queueLength = "<<lane.lane_queueLength<<std::endl;
				std::cout<<"lane.lane_gap = "<<lane.lane_gap<<std::endl;
				std::cout<<"lane.lane_time_occupancy = "<<lane.lane_time_occupancy<<std::endl;
				std::cout<<"lane.lane_ave_distance = "<<lane.lane_ave_distance<<std::endl;
				std::cout<<"lane.lane_no_motor_volume = "<<lane.lane_no_motor_volume<<std::endl;
				std::cout<<"lane.lane_minmotor_volume = "<<lane.lane_minmotor_volume<<std::endl;
				std::cout<<"lane.lane_medmotor_volume = "<<lane.lane_medmotor_volume<<std::endl;
				std::cout<<"lane.lane_maxmotor_volume = "<<lane.lane_maxmotor_volume<<std::endl;
			}
		}
	}
	
	if("device_id" == DealRCF::deal_info_.DeviceID)	
		perceptrons.set_devide_id(object_list_.devide_id);
	else
		perceptrons.set_devide_id(DealRCF::deal_info_.DeviceID);
	perceptrons.set_time_stamp(object_list_.time_stamp);
	perceptrons.set_number_frame(object_list_.number_frame);
	nebulalink::perceptron::PointGPS *point_gps = perceptrons.mutable_perception_gps();
	point_gps->set_object_elevation(object_list_.perception_gps.object_elevation);
	point_gps->set_object_latitude(object_list_.perception_gps.object_latitude);
	point_gps->set_object_longitude(object_list_.perception_gps.object_longitude);
	// perceptrons.set_allocated_perception_gps(point_gps);

	// perceptrons.SerializeToArray(send2fusion + 9, perceptrons.ByteSize());
	perceptrons.SerializeToArray(send2fusion + 8, perceptrons.ByteSize());
	send2fusion[0] = 0xDA;
	send2fusion[1] = 0xDB;
	send2fusion[2] = 0xDC;
	send2fusion[3] = 0xDD;
	send2fusion[4] = 0x01;
	// send2fusion[5] = 0x01;
	// 0 1 2 3(摄像头) 4（微波雷达） 5 6 7（融合）
	switch (type)
	{
	case 1:
		send2fusion[5] = 0x01;
		break;
	case 2:
		send2fusion[5] = 0x02;
		break;
	case 3:
		send2fusion[5] = 0x03;
		break;
	case 4:
		send2fusion[5] = 0x04;
		break;
	case 5:
		send2fusion[5] = 0x05;
		break;
	case 6:
		send2fusion[5] = 0x06;
		break;
	case 7:
		send2fusion[5] = 0x07;
		break;
	default:
		send2fusion[5] = 0x00;
		break;
	}

	//*((unsigned short *)(send2fusion + 7)) = htons(perceptrons.ByteSize());
	*((unsigned short *)(send2fusion + 6)) = htons(perceptrons.ByteSize());

	int sendto_len = 0;

	// if (0 != perceptrons.perceptron_size()) //仅有障碍物也会发送
	{
		if (RETURN_ZERO > (sendto_len = sendto(send_fd_, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
											   (struct sockaddr *)&send_addr_, sizeof(send_addr_))))
		{
			cerr << "sendto fusion message to rsu error .. ..." << endl;
			//return ERROR_SEND_TO;
		}
		// else
		// 	cout << "sendto rsu successfully ... send length ---- " << perceptrons.ByteSize() << endl;

		if (RETURN_ZERO > (sendto_len = sendto(send_cloud_fd_, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
											   (struct sockaddr *)&send_cloud_addr_, sizeof(send_cloud_addr_))))
		{
			cerr << "sendto fusion message to cloud error .. ..." << endl;
			//return ERROR_SEND_TO;
		}
		else
			// cout << "sendto other successfully ... send length ---- " << perceptrons.ByteSize() << endl;

		// if ((true == DealRCF::is_send_to_pad) && (DEBUG_PAD_CYCLE > (DealRCF::pad_timestamp2 - DealRCF::pad_timestamp1)))
		if (true == DealRCF::is_send_to_pad)
		{
			if (RETURN_ZERO > (sendto_len = sendto(DealRCF::pad_fd, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
												   (struct sockaddr *)&DealRCF::pad_client_addr, sizeof(DealRCF::pad_client_addr))))
			{
				cerr << "sendto fusion message to pad error .. ..." << endl;
				return ERROR_SEND_TO;
			}
		}
	}
	// PrintObjectList(object_list_, "Send to info");
	return sendto_len;
}


/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/******************************************************工具型接口****************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/
/********************************************************************************************************************/



// 打印s_ObjectList
void DealRCF::PrintObjectList(perceptron::s_ObjectList object_list, char *sign)
{
	int size = object_list.object_list.size();
	printf("%s: object_list.object_list.size() = %d\n", sign, size);
	printf("object_list.time_stamp = %ld\n", object_list.time_stamp);
	printf("object_list.devide_id = %s\n", object_list.devide_id.c_str());
#if 0
	for (int i = 0; i < size; ++i)
	{
		printf("id =  %d\n", object_list.object_list[i].object_id);
		printf("class =  %d\n", object_list.object_list[i].object_class_type);
		printf("target_size.object_height = %f\n", object_list.object_list[i].target_size.object_height);
		printf("target_size.object_length = %f\n", object_list.object_list[i].target_size.object_length);
		printf("target_size.object_width = %f\n", object_list.object_list[i].target_size.object_width);
		printf("object_heading = %f\n", object_list.object_list[i].object_heading);
		printf("object_speed = %f\n", object_list.object_list[i].object_speed);
		printf("point3f.x = %f\n", object_list.object_list[i].point3f.x);
		printf("point3f.y = %f\n", object_list.object_list[i].point3f.y);
		//printf("point3f.z = %f\n", object_list.object_list[i].point3f.z);
		printf("speed3f.speed_x = %f\n", object_list.object_list[i].speed3f.speed_x);
		printf("speed3f.speed_y = %f\n", object_list.object_list[i].speed3f.speed_y);
		//printf("speed3f.speed_z = %f\n", object_list.object_list[i].speed3f.speed_z);
		printf("object_latitude =  %lf\n", object_list.object_list[i].point_gps.object_latitude);
		printf("object_longitude =  %lf\n", object_list.object_list[i].point_gps.object_longitude);
	}
#endif
	return;
}

// 结构体变量之间赋值
void DealRCF::CopyObjectList(perceptron::s_ObjectList *object_list1, perceptron::s_ObjectList *object_list2)
{
	// memset(object_list1, 0, sizeof(perceptron::s_ObjectList));
	// perceptron::s_Object object;
	object_list1->object_list.clear();
	object_list1->object_list.shrink_to_fit();
	for (int i = 0; i < object_list2->object_list.size(); ++i)
	{
		object_list1->object_list.push_back(object_list2->object_list[i]);
	}
	object_list1->time_stamp = object_list2->time_stamp;
	object_list1->devide_is_true = object_list2->devide_is_true;
	object_list1->number_frame = object_list2->number_frame;
	object_list1->devide_id = object_list2->devide_id;
	object_list1->sensor_type = object_list2->sensor_type;

	return;
}


// 判断缓存区的大小是否超过最大阈值
bool DealRCF::JudgeSensorObjectListsSize(vector<perceptron::s_ObjectList> *sensor_object_lists)
{
	if (SENSOR_OBJECT_LISTS_MAX <= sensor_object_lists->size())
	{
		vector<perceptron::s_ObjectList>::iterator k = sensor_object_lists->begin();
		sensor_object_lists->erase(k);
		// 避免内存泄漏
		vector<perceptron::s_ObjectList>(*sensor_object_lists).swap(*sensor_object_lists);
		return true;
	}
	return false;
}

// 目标的时间是否超时
bool DealRCF::JudgeSensorObjectOverTime(perceptron::s_ObjectList sensor_objects)
{
	if (SENSOR_OBJECT_OVER_TIME < (GetMSecondTimeFrom1970() - sensor_objects.time_stamp))
	{
		return false;
	}
	else
	{
		return true;
	}
}

// 获取从1970.1.1截至当前时间的毫秒数
unsigned long long DealRCF::GetMSecondTimeFrom1970()
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	unsigned long long nsecond = ts.tv_nsec;
	unsigned long long second = ts.tv_sec * 1000 + (nsecond / 1000000);

	return second;
}

unsigned long long DealRCF::GetMSecondTime()
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	unsigned long long nsecond = ts.tv_nsec;
	unsigned long long second = ts.tv_sec * 1000 + (nsecond / 1000000);
	unsigned long long minute = second - (ts.tv_sec / 60 * 60 * 1000);

	// printf("ms:%lld, t_ms:%lld\n", second, minute);
	return minute;
}

// 将数据按照一帧目标一行写入文件


// 打印主进程相关信息
bool DealRCF::PrintMainProcessInfo()
{
	cout << "###############################################################\n";
	cout << "############# Proc: " << PROCESS_NAME << " #####################" << endl;
	cout << "############# Version: " << PROCESS_VERSION << " ####################################" << endl;
	cout << "###############################################################\n";
	cout << endl;
}




// 结构体变量之间赋值
void DealRCF::AveCopyObjectList(perceptron::s_ObjectList object_list)
{
	std::vector<s_AveObject>::iterator it_track;
	bool is_track_exit = false;

	for (int i = 0; i < object_list.object_list.size(); ++i)
	{
		object_list.object_list[i].is_tracker = false;
	}
	for (it_track = ave_object_list_.object_list.begin(); it_track != ave_object_list_.object_list.end();)
	{
		is_track_exit = false;
		for (int i = 0; i < object_list.object_list.size(); ++i)
		{
			if (it_track->object_id == object_list.object_list[i].object_id)
			{
				it_track->is_head_tail = object_list.object_list[i].is_head_tail;
				object_list.object_list[i].is_tracker = true;
				it_track->is_tracker = object_list.object_list[i].is_tracker;
				it_track->lane_id = object_list.object_list[i].lane_id;
				it_track->lane_type = object_list.object_list[i].lane_type;
				it_track->object_acceleration = object_list.object_list[i].object_acceleration;
				it_track->object_class_type = object_list.object_list[i].object_class_type;
				it_track->object_confidence = object_list.object_list[i].object_confidence;
				it_track->object_direction = object_list.object_list[i].object_direction;
				it_track->object_heading = object_list.object_list[i].object_heading;
				it_track->object_id = object_list.object_list[i].object_id;
				it_track->object_NS = object_list.object_list[i].object_NS;
				it_track->object_speed = object_list.object_list[i].object_speed;
				it_track->object_WE = object_list.object_list[i].object_WE;
				it_track->point3f = object_list.object_list[i].point3f;
				it_track->point4f = object_list.object_list[i].point4f;
				it_track->point_gps_list.push_back(object_list.object_list[i].point_gps);
				it_track->speed3f = object_list.object_list[i].speed3f;
				it_track->target_size = object_list.object_list[i].target_size;
				is_track_exit = true;
				break;
			}
		}
		if (false == is_track_exit)
		{
			it_track = ave_object_list_.object_list.erase(it_track);
			// ave_object_list_.object_list.shrink_to_fit();
		}
		else
		{
			it_track++;
		}


	}
	for (int i = 0; i < object_list.object_list.size(); ++i)
	{
		s_AveObject ave_object;
		if (false == object_list.object_list[i].is_tracker)
		{
			ave_object.is_head_tail = object_list.object_list[i].is_head_tail;
			object_list.object_list[i].is_tracker = true;
			ave_object.is_tracker = object_list.object_list[i].is_tracker;
			ave_object.lane_id = object_list.object_list[i].lane_id;
			ave_object.lane_type = object_list.object_list[i].lane_type;
			ave_object.object_acceleration = object_list.object_list[i].object_acceleration;
			ave_object.object_class_type = object_list.object_list[i].object_class_type;
			ave_object.object_confidence = object_list.object_list[i].object_confidence;
			ave_object.object_direction = object_list.object_list[i].object_direction;
			ave_object.object_heading = object_list.object_list[i].object_heading;
			ave_object.object_id = object_list.object_list[i].object_id;
			ave_object.object_NS = object_list.object_list[i].object_NS;
			ave_object.object_speed = object_list.object_list[i].object_speed;
			ave_object.object_WE = object_list.object_list[i].object_WE;
			ave_object.point3f = object_list.object_list[i].point3f;
			ave_object.point4f = object_list.object_list[i].point4f;
			ave_object.point_gps_list.push_back(object_list.object_list[i].point_gps);
			ave_object.speed3f = object_list.object_list[i].speed3f;
			ave_object.target_size = object_list.object_list[i].target_size;
			ave_object_list_.object_list.push_back(ave_object);
		}
	}

	ave_object_list_.time_stamp = object_list.time_stamp;
	ave_object_list_.devide_is_true = object_list.devide_is_true;
	ave_object_list_.number_frame = object_list.number_frame;
	ave_object_list_.devide_id = object_list.devide_id;

	return;
}

// 判断GPS缓存区的大小是否超过最大阈�?
bool DealRCF::AveJudgeSensorObjectListsSize()
{
	std::vector<perceptron::s_PointGPS>::iterator it;
	for (int i = 0; i < ave_object_list_.object_list.size(); ++i)
	{
		if (AVE_MAX_HISTORY < ave_object_list_.object_list[i].point_gps_list.size())
		{
			it = ave_object_list_.object_list[i].point_gps_list.begin();
			it = ave_object_list_.object_list[i].point_gps_list.erase(it);
			ave_object_list_.object_list[i].point_gps_list.shrink_to_fit();
		}
	}
	return true;
}

// 打包成protobuf发出
int DealRCF::AveSerializationsObjectListSend(s_AveObjectList object_list_, int type)
{
	nebulalink::perceptron::PerceptronSet perceptrons = nebulalink::perceptron::PerceptronSet();
	unsigned char send2fusion[gMaxPacketSize] = {0};
	nebulalink::perceptron::Perceptron *perceptron = new nebulalink::perceptron::Perceptron();
	for (unsigned i = 0; i < object_list_.object_list.size(); ++i)
	{
		perceptron = perceptrons.add_perceptron();

		nebulalink::perceptron::PointGPS *point_gps = new nebulalink::perceptron::PointGPS();
		nebulalink::perceptron::TargetSize *target_size = new nebulalink::perceptron::TargetSize();
		nebulalink::perceptron::Point4 *point4f = new nebulalink::perceptron::Point4();
		nebulalink::perceptron::Acc4Way *acc4_way = new nebulalink::perceptron::Acc4Way();

		double sum_lat = 0.0, sum_lon = 0.0, sum_alt = 0.0;
		int sum_size = object_list_.object_list[i].point_gps_list.size();
		for (int j = 0; j < sum_size; ++j)
		{
			sum_lat += object_list_.object_list[i].point_gps_list[j].object_latitude;
			sum_lon += object_list_.object_list[i].point_gps_list[j].object_longitude;
			sum_alt += object_list_.object_list[i].point_gps_list[j].object_elevation;
		}

		point_gps->set_object_elevation(sum_alt / sum_size);
		point_gps->set_object_latitude(sum_lat / sum_size);
		point_gps->set_object_longitude(sum_lon / sum_size);

		target_size->set_object_height(object_list_.object_list[i].target_size.object_height);
		target_size->set_object_length(object_list_.object_list[i].target_size.object_length);
		target_size->set_object_width(object_list_.object_list[i].target_size.object_width);
		// point4f->set_camera_h(object_list_.object_list[i].point4f.camera_h);
		// point4f->set_camera_w(object_list_.object_list[i].point4f.camera_w);
		// point4f->set_camera_x(object_list_.object_list[i].point4f.camera_x);
		// point4f->set_camera_y(object_list_.object_list[i].point4f.camera_y);

		perceptron->set_object_id((object_list_.object_list[i].object_id) % 65534 + 1);
		perceptron->set_allocated_target_size(target_size);
		perceptron->set_allocated_point_gps(point_gps);
		perceptron->set_allocated_point4f(point4f);

		perceptron->set_object_ns(object_list_.object_list[i].object_NS);
		perceptron->set_object_we(object_list_.object_list[i].object_WE);
		perceptron->set_object_direction(object_list_.object_list[i].object_direction);
		perceptron->set_object_heading(object_list_.object_list[i].object_heading);
		perceptron->set_object_speed(object_list_.object_list[i].object_speed);
		perceptron->set_object_class_type(object_list_.object_list[i].object_class_type);
		//perceptron->set_ptc_veh_type(object_list_.object_list[i].ptc_veh_type);
		//perceptron->set_ptc_veh_color(object_list_.object_list[i].ptc_veh_color);
		// perceptron->set_object_acceleration(object_list_.object_list[i].object_acceleration);
		acc4_way->set_acc4waylat(object_list_.object_list[i].object_acceleration * cos(object_list_.object_list[i].object_heading));
		acc4_way->set_acc4waylon(object_list_.object_list[i].object_acceleration * sin(object_list_.object_list[i].object_heading));
		perceptron->set_allocated_accel_4way(acc4_way);
	}

	perceptrons.set_devide_id(DealRCF::deal_info_.DeviceID);
	perceptrons.set_time_stamp(object_list_.time_stamp);
	perceptrons.set_number_frame(object_list_.number_frame);

	// perceptrons.SerializeToArray(send2fusion + 9, perceptrons.ByteSize());
	perceptrons.SerializeToArray(send2fusion + 8, perceptrons.ByteSize());
	send2fusion[0] = 0xDA;
	send2fusion[1] = 0xDB;
	send2fusion[2] = 0xDC;
	send2fusion[3] = 0xDD;
	send2fusion[4] = 0x01;
	// send2fusion[5] = 0x01;
	// 0 1 2 3(摄像�? 4（微波雷达） 5 6 7（融合）
	switch (type)
	{
	case 1:
		send2fusion[5] = 0x01;
		break;
	case 2:
		send2fusion[5] = 0x02;
		break;
	case 3:
		send2fusion[5] = 0x03;
		break;
	case 4:
		send2fusion[5] = 0x04;
		break;
	case 5:
		send2fusion[5] = 0x05;
		break;
	case 6:
		send2fusion[5] = 0x06;
		break;
	case 7:
		send2fusion[5] = 0x07;
		break;
	default:
		send2fusion[5] = 0x00;
		break;
	}

	//*((unsigned short *)(send2fusion + 7)) = htons(perceptrons.ByteSize());
	*((unsigned short *)(send2fusion + 6)) = htons(perceptrons.ByteSize());

	int sendto_len = 0;

	if (0 != perceptrons.perceptron_size())
	{
		if (RETURN_ZERO > (sendto_len = sendto(send_fd_, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
											   (struct sockaddr *)&send_addr_, sizeof(send_addr_))))
		{
			cerr << "sendto fusion message to rsu error .. ..." << endl;
			return ERROR_SEND_TO;
		}
		// else
		// cout << "sendto rsu successfully ... send length ---- " << perceptrons.ByteSize() << endl;

		if (RETURN_ZERO > (sendto_len = sendto(send_cloud_fd_, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
											   (struct sockaddr *)&send_cloud_addr_, sizeof(send_cloud_addr_))))
		{
			cerr << "sendto fusion message to cloud error .. ..." << endl;
			return ERROR_SEND_TO;
		}
		// else
		// 	cout << "sendto other successfully ... send length ---- " << perceptrons.ByteSize() << endl;

		// if ((true == DealRCF::is_send_to_pad) && (DEBUG_PAD_CYCLE > (DealRCF::pad_timestamp2 - DealRCF::pad_timestamp1)))
		if (true == DealRCF::is_send_to_pad)
		{
			if (RETURN_ZERO > (sendto_len = sendto(DealRCF::pad_fd, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
												   (struct sockaddr *)&DealRCF::pad_client_addr, sizeof(DealRCF::pad_client_addr))))
			{
				cerr << "sendto fusion message to pad error .. ..." << endl;
				return ERROR_SEND_TO;
			}
		}
	}
	// PrintObjectList(object_list_, "Send to info");
	return sendto_len;
}



// wgs to gcj20
double DealRCF::TransFormLat(double lng, double lat)
{
	double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * sqrt(abs(lng));  
	ret += (20.0 * sin(6.0 * lng * M_PI) + 20.0 * sin(2.0 * lng * M_PI)) * 2.0 / 3.0;
	ret += (20.0 * sin(lat * M_PI) + 40.0 * sin(lat / 3.0 * M_PI)) * 2.0 / 3.0;
	ret += (160.0 * sin(lat / 12.0 * M_PI) + 320 * sin(lat * M_PI / 30.0)) * 2.0 / 3.0;
	return ret;
}

double DealRCF::TransFormLng(double lng, double lat)
{
	double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * sqrt(abs(lng));
	ret += (20.0 * sin(6.0 * lng * M_PI) + 20.0 * sin(2.0 * lng * M_PI)) * 2.0 / 3.0;
	ret += (20.0 * sin(lng * M_PI) + 40.0 * sin(lng / 3.0 * M_PI)) * 2.0 / 3.0;
	ret += (150.0 * sin(lng / 12.0 * M_PI) + 300.0 * sin(lng / 30.0 * M_PI)) * 2.0 / 3.0;
	return ret;
}

/**
 * 判断是否在国内，不在国内则不做偏移
 * @param lng
 * @param lat
 * @returns {boolean}
 */
double DealRCF::OutOfChina(double &lng, double &lat)
{
	return (lng < 72.004 || lng > 137.8347) || ((lat < 0.8293 || lat > 55.8271) || false);
}

/**
 * WGS84转GCj02
 * @param lng
 * @param lat
 * @returns {*[]}
 */
bool DealRCF::Wgs84toGcj02(double &lng, double &lat)
{
	lng -= LNG_OFFSET;
	lat += LAT_OFFSET;
	const double a = 6378245.0;
	const double ee = 0.00669342162296594323;
	if (DealRCF::OutOfChina(lng, lat))
	{
		return false;
	}
	else
	{
		double dlat = DealRCF::TransFormLat(lng - 105.0, lat - 35.0);
		double dlng = DealRCF::TransFormLng(lng - 105.0, lat - 35.0);
		double radlat = lat / 180.0 * M_PI;
		double magic = sin(radlat);
		magic = 1 - ee * magic * magic;
		double sqrtmagic = sqrt(magic);
		dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * M_PI);
		dlng = (dlng * 180.0) / (a / sqrtmagic * cos(radlat) * M_PI);
		double mglat = lat + dlat;
		double mglng = lng + dlng;
		lng = mglng;
		lat = mglat;
		return true;
	}
}

// 获取指定线程的内存占用大小
size_t GetThreadMemoryUsage(pid_t tid)
{
    std::stringstream ss;
    ss << "/proc/" << tid << "/status";
    std::ifstream ifs(ss.str().c_str());
    if (!ifs)
    {
        return 0;
    }

    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.find("VmSize:") != std::string::npos)
        {
            std::istringstream iss(line);
            std::string name;
            size_t memoryUsage;
            iss >> name >> memoryUsage;
            return memoryUsage * 1024; // 转换为字节
        }
    }

    return 0;
}
