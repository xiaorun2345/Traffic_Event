/*
* camera_app_test.cpp
* Created on: 20210609
* Author: yueyingying
* Description: 视觉测试程序
*/

#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include "../app/camera_app.h"
#include "../common/utils.h"
#include <NebulalinkObjectStruct3.h>
#include <nebulalink.perceptron3.0.5.pb.h>
#include <atomic> //C++11原子操作库
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>
#include <time.h>
#include <iostream>


#include <vector>
#include <queue>
#include <thread>
#include <stdio.h>
#include "../camera/detector/MppEncoder.h"
#include "../camera/detector/MppDecoder.h"
#include <mutex>
#include "../RtspServer/xop/RtspServer.h"
#include "../RtspServer/net/Timer.h"

//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"


using namespace std;

//using std::queue;
//using std::vecctor;

#define CAMERA_WIN "camera"
#define DEBUG_PAD_CYCLE 30000 //毫秒

std::mutex g_frame_mutex;
cv::Mat g_frame;
cv::Mat g_yuvImg;
std::mutex g_img_mutex;
cv::Mat g_img;


const int g_width = 1920;
const int g_height = 1080;
const int g_fps = 25;

atomic_bool is_send_to_pad{false};
atomic_ullong pad_timestamp1, pad_timestamp2;
const int gMaxPacketSize = 10000;
int pad_fd = -1, send_fd;
struct sockaddr_in pad_listen_addr, pad_client_addr;
struct sockaddr_in client_addr;
int send_port = 10086;
std::string send_ip;

// 获取从1970.1.1截至当前时间的毫秒数
static unsigned long long GetMSecondTimeFrom1970()
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	unsigned long long nsecond = ts.tv_nsec;
	unsigned long long second = ts.tv_sec * 1000 + (nsecond / 1000000);

	return second;
}

// 与pad交互线程获取图像线程执行体
void *PadDealThread(void *arg)
{
	// 接收pad发送的请求
	int num = 0;
	char recv_buf[gMaxPacketSize];
	socklen_t addrlen = sizeof(pad_client_addr);

	while ((num = recvfrom(pad_fd, recv_buf, gMaxPacketSize, 0, (struct sockaddr *)&pad_client_addr, &addrlen)) > 0)
	{
		if (0x00 == recv_buf[0] && 0x00 == recv_buf[1] && 0x00 == recv_buf[2] && 0x01 == recv_buf[3])
		{
			pad_timestamp1 = GetMSecondTimeFrom1970();
			is_send_to_pad = true;
		}
	}
}

void *SendDealThread(void *arg)
{
	string send_str = "mec process is online ...\n";
	char recv_buf[gMaxPacketSize];
	int sendto_len = 0;

	while (true)
	{
		if ((true == is_send_to_pad) && (DEBUG_PAD_CYCLE > (pad_timestamp2 - pad_timestamp1)))
		{
			// recv_buf = send_str.c_str();
			strncpy(recv_buf, send_str.c_str(), strlen(send_str.c_str()));
			// recv_buf[sizeof(send_str) + 1] = '\0';
			if (0 > (sendto_len = sendto(pad_fd, recv_buf, strlen(send_str.c_str()), MSG_DONTWAIT,
												   (struct sockaddr *)&pad_client_addr, sizeof(pad_client_addr))))
			{
				cerr << "sendto heart message to pad error .. ..." << endl;
				return nullptr;
			}
			else
			{
				printf("send %d data successfully, pad ip = %s, port = %d\n", sendto_len, inet_ntoa(pad_client_addr.sin_addr), ntohs(pad_client_addr.sin_port));
			}
		}
		else
		{
			is_send_to_pad = false;
		}
		sleep(1);
		pad_timestamp2 = GetMSecondTimeFrom1970();
	}
}

//与pad通信
bool StartPadThread(int port)
{
	printf("start send thread.....\n");
    // 创建套接字
    if ((pad_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		cerr << "Creating socket pad fd failed." << endl;
        is_send_to_pad = false;
		return false;
	}

	// 填充网络信息结构体
    bzero(&pad_listen_addr, sizeof(pad_listen_addr));
	pad_listen_addr.sin_family = AF_INET;
	pad_listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	pad_listen_addr.sin_port = htons(port);

	// 将套接字和网络信息结构体绑定
	if (bind(pad_fd, (struct sockaddr *)&pad_listen_addr, sizeof(pad_listen_addr)) < 0)
	{
		cerr << "bind listen pad port error ..." << endl;
        is_send_to_pad = false;
        return false;
	}


	//发送给本地
	if ((send_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		cerr << "Creating socket send fd failed." << endl;
		return false;
	}
	memset(&client_addr, 0, sizeof(struct sockaddr_in));
	client_addr.sin_family = AF_INET;
	client_addr.sin_addr.s_addr = inet_addr(send_ip.c_str());
	client_addr.sin_port = htons(8008);

    //开启心跳和监听线程
	pthread_t pid;
	int ret;
	if ((ret = pthread_create(&pid, NULL, PadDealThread, NULL)) != 0)
	{
		cerr << "StartDealPadThread error ... " << endl;
        // return false;
	}

    sleep(1);
	if ((ret = pthread_create(&pid, NULL, SendDealThread, NULL)) != 0)
	{
		cerr << "StartSendDealThread ... " << endl;
		return false;
	}

    return true;
}
int SerializationsObjectListSend(perceptron::s_ObjectList object_list_, bool IsGcj02 = true)
{
	nebulalink::perceptron::PerceptronSet perceptrons = nebulalink::perceptron::PerceptronSet();
	unsigned char send2fusion[gMaxPacketSize] = {0};

	for (unsigned i = 0; i < object_list_.object_list.size(); ++i)
	{
		if(object_list_.object_list[i].object_class_type > 4)
			continue;
		nebulalink::perceptron::Perceptron *perceptron = perceptrons.add_perceptron();

		nebulalink::perceptron::PointGPS *point_gps = new nebulalink::perceptron::PointGPS();
		nebulalink::perceptron::TargetSize *target_size = new nebulalink::perceptron::TargetSize();
		nebulalink::perceptron::Point4 *point4f = new nebulalink::perceptron::Point4();
		nebulalink::perceptron::Acc4Way *acc4_way = new nebulalink::perceptron::Acc4Way();

		point_gps->set_object_elevation(object_list_.object_list[i].point_gps.object_elevation);
		if (IsGcj02)
		{
			camera::WGS84ToGCJ02(object_list_.object_list[i].point_gps.object_longitude, object_list_.object_list[i].point_gps.object_latitude);
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

		nebulalink::perceptron::Point3 *point3f = new nebulalink::perceptron::Point3();
		point3f->set_x(object_list_.object_list[i].point3f.x);
		point3f->set_y(object_list_.object_list[i].point3f.y);
		point3f->set_z(0);
		perceptron->set_allocated_point3f(point3f);

		perceptron->set_object_id(object_list_.object_list[i].object_id);
		perceptron->set_allocated_target_size(target_size);
		perceptron->set_allocated_point_gps(point_gps);
		perceptron->set_allocated_point4f(point4f);

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
		perceptron->set_allocated_accel_4way(acc4_way);
		// 车牌号
		perceptron->set_plate_num(object_list_.object_list[i].plate_num);
		perceptron->set_is_tracker(true);
		printf(" id =  %d\n",object_list_.object_list[i].ptc_nodeid);
		perceptron->set_ptc_nodeid(object_list_.object_list[i].ptc_nodeid);

		perceptron->set_lane_id(object_list_.object_list[i].lane_id);
		perceptron->set_obj_time_stamp(object_list_.object_list[i].obj_time_stamp);
		perceptron->set_ptc_sourcetype(3);
	}

	perceptrons.set_devide_id(object_list_.devide_id);
	perceptrons.set_time_stamp(object_list_.time_stamp);
	perceptrons.set_number_frame(object_list_.number_frame);
	nebulalink::perceptron::PointGPS *point_gps = new nebulalink::perceptron::PointGPS();
	point_gps->set_object_elevation(object_list_.perception_gps.object_elevation);
	point_gps->set_object_latitude(object_list_.perception_gps.object_latitude);
	point_gps->set_object_longitude(object_list_.perception_gps.object_longitude);
	perceptrons.set_allocated_perception_gps(point_gps);

	// perceptrons.SerializeToArray(send2fusion + 9, perceptrons.ByteSize());
	perceptrons.SerializeToArray(send2fusion + 8, perceptrons.ByteSize());
	send2fusion[0] = 0xDA;
	send2fusion[1] = 0xDB;
	send2fusion[2] = 0xDC;
	send2fusion[3] = 0xDD;
	send2fusion[4] = 0x01;
	send2fusion[5] = 0x03;

	//*((unsigned short *)(send2fusion + 7)) = htons(perceptrons.ByteSize());
	*((unsigned short *)(send2fusion + 6)) = htons(perceptrons.ByteSize());

	int sendto_len = 0;

	// if (0 != perceptrons.perceptron_size())
	{
		//发送本地
		if (0 > (sendto_len = sendto(send_fd, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
											   (struct sockaddr *)&client_addr, sizeof(struct sockaddr_in))))
		{
			cerr << "sendto fusion message to local error .. ..." << endl;
			return -1;
		}
		printf("****************send num : %d *************\n", sendto_len);

		// if ((true == DealRCF::is_send_to_pad) && (DEBUG_PAD_CYCLE > (DealRCF::pad_timestamp2 - DealRCF::pad_timestamp1)))
		if (true == is_send_to_pad)
		{
			if (0 > (sendto_len = sendto(pad_fd, send2fusion, perceptrons.ByteSize() + 8, MSG_DONTWAIT,
												   (struct sockaddr *)&pad_client_addr, sizeof(pad_client_addr))))
			{
				cerr << "sendto fusion message to pad error .. ..." << endl;
				return -1;
			}
		}
	}

	return sendto_len;
}


//rtsp流原始数据为NV12格式
void GetFrameRawData(unsigned char *data, int h_stride, int v_stride, int width, int height, uint64_t timestamp)
{
    int i;
	unsigned char *base_y = data;
	unsigned char *base_c = data + h_stride * v_stride;

	//转为Mat
	int idx = 0;
	for (i = 0; i < height; i++, base_y += h_stride) 
    {
        memcpy(g_yuvImg.data + idx, base_y, width);
		idx += width;
	}
	for (i = 0; i < height / 2; i++, base_c += h_stride) 
    {
		memcpy(g_yuvImg.data + idx, base_c, width);
		idx += width;
	}

#if 1
    cv::Mat rgbImg;
    cv::cvtColor(g_yuvImg, rgbImg, cv::COLOR_YUV2BGR_NV12);

    g_frame_mutex.lock();
    rgbImg.copyTo(g_frame);
    g_frame_mutex.unlock();
#else
    g_frame_mutex.lock();
    g_yuvImg.copyTo(g_frame);
    g_frame_mutex.unlock();
#endif

    static uint64_t last=0;
    struct timespec ts;
    //clock_gettime(CLOCK_REALTIME, &ts);
   // uint64_t timestamp = ts.tv_sec * 1000 + (ts.tv_nsec / 1000000); //ms
    //uint64_t now = timestamp;
   // printf("captime: %d\n", now -last);
    //last = now;

    // cv::imwrite("./result/"+std::to_string(timestamp)+".jpg", rgbImg);
}

void CaptureRawDataThread(std::string video_name, bool display)
{
    if(g_yuvImg.empty())
    {
        g_yuvImg.create(g_height*3/2, g_width, CV_8UC1);
    }

    MppDecoder *decoder = new MppDecoder();
    CODEC_TYPE type = H264;
    int ret = decoder->Init(video_name, g_width, g_height, type);
    if(ret < 0)
    {
        printf("decoder init failure...\n");
        exit(-1);
    }
    decoder->Decoding(GetFrameRawData);

    printf("CaptureRawDataThread exit...");
    delete decoder;
    decoder = nullptr;
}


void inferencethread(std::string camera_cfg_file, int send_time, int port)
{

      if(send_time >= 0)
	  StartPadThread(port);
      std::cout << "star infer thread" << std::endl;
      
    int width = 1920;
    int height = 1080;
    camera::CameraAPP * capp = new camera::CameraAPP();
    capp->Init(camera_cfg_file, width, height); 
    unsigned int frame_count = 0;
    cv::Mat img;
    unsigned long long time1=0, time2=0;
    cv::namedWindow("Camera FPS",cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera FPS", 1920 / 2, 1080 /2 );
    
    while (1){
      
	      cv::Mat rgbImg;

	      g_frame_mutex.lock();
	      g_frame.copyTo(rgbImg);
	      g_frame_mutex.unlock();
	      
	      std::cout<< "-=================" << rgbImg.rows << rgbImg.cols << std::endl;
	      if(rgbImg.empty())
            {
            std::cout << "img for infer is empty..\n";
            	// break;
            //cap.open(video_name);
            continue;
            //break;
            }
	    
		struct timespec time_stamp;
	      clock_gettime(CLOCK_REALTIME, &time_stamp);
	      uint64_t timestamp = time_stamp.tv_sec * 1e3 + time_stamp.tv_nsec/1e6;  //ms

	      auto start = std::chrono::system_clock::now();
	      perceptron::s_ObjectList objects;
	       
		capp->Process(rgbImg, timestamp, &objects);
		
		auto end = std::chrono::system_clock::now();
		std::cout << "***********frame_"<< frame_count  <<"   total runtime: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
		capp->DrawResult(rgbImg, &objects);   // draw ROI
		cv::imshow("Camera FPS", rgbImg);
	      if (cv::waitKey(1) == 'q')
                break;
		
		SerializationsObjectListSend(objects, true);
	      time1 = GetMSecondTimeFrom1970();

		g_img_mutex.lock();
		rgbImg.copyTo(g_img);
		g_img_mutex.unlock();
	}
	
	if(capp)
    	{
        delete capp;
        capp = nullptr;
    	}
    	cv::destroyAllWindows();   

	
}

void loaddataprocess(std::string video_name, std::string camera_cfg_file, int send_time, int port)
{

    if(send_time >= 0)
        StartPadThread(port);
    std::cout << "star" << std::endl;
    
    cv::VideoCapture cap;

    cap.open(video_name, cv::CAP_FFMPEG);
    //if(!cap.isOpened())
   // {
        //std::cout << "open video error...\n";
        //exit();
    //}

    int width = (int)cap.get(3); 
    int height = (int)cap.get(4);
    int fps = (int)cap.get(5);
    
    //cv::VideoWriter outputVideo("out_video.avi",cv::VideoWriter::fourcc('M','J','P','G'), 25, cv::Size(width, height));

    printf("video = %s, width = %d, height = %d\n", video_name.c_str(), width, height);

    cv::namedWindow("Camera FPS",cv::WINDOW_NORMAL);
    cv::resizeWindow("Camera FPS", 1920, 1080);
    
    camera::CameraAPP * capp = new camera::CameraAPP();
    capp->Init(camera_cfg_file, width, height);  

    unsigned int frame_count = 0;
    cv::Mat img;
    unsigned long long time1=0, time2=0;
    

    while(cap.isOpened())
    {
      cap >> img;
      
      if(img.empty())
            {
            std::cout << "img is empty..\n";
            	// break;
            //cap.open(video_name);
            //continue;
            break;
            }

       
       struct timespec time_stamp;
       clock_gettime(CLOCK_REALTIME, &time_stamp);
       uint64_t timestamp = time_stamp.tv_sec * 1e3 + time_stamp.tv_nsec/1e6;  //ms

       auto start = std::chrono::system_clock::now();

       perceptron::s_ObjectList objects;
       
       capp->Process(img, timestamp, &objects);
       
       auto end = std::chrono::system_clock::now();
       std::cout << "***********frame_"<< frame_count  <<"   total runtime: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

       //if(display)
       capp->DrawResult(img, &objects);   // draw ROI
       
       //capp-> Countnum()
		
	 //cv::imshow("Camera FPS", img);
	 
	 cv::imshow("Camera FPS", img);
	 //outputVideo.write(img);
	 if (cv::waitKey(1) == 'q')
           break;
	  		        
        // SerializationsObjectListSend(objects, true);
	  time1 = GetMSecondTimeFrom1970();
	  if (send_time <= (time1 - time2))
		{
			SerializationsObjectListSend(objects, true);
			time2 = GetMSecondTimeFrom1970();
		}
        g_frame_mutex.lock();
        img.copyTo(g_frame);
        g_frame_mutex.unlock();
		
	  }
	if(capp)
    	{
        delete capp;
        capp = nullptr;
    	}
    	//outputVideo.release();
    	cv::destroyAllWindows();   
    	cap.release();
    
}



void EncodingSendFrameThread(xop::RtspServer* rtsp_server, xop::MediaSessionId session_id)
{
    
    int yuv_size = g_width*g_height*3/2;
    int length = 0;
    //编码器
    whale::vision::MppEncoder mppenc;
    mppenc.MppEncdoerInit(g_width, g_height, g_fps);

    char dst[1024*1024*4];
    char *pdst = dst;  
   
    
    cv::Mat rgbImg, yuvImg;
    while (1)
    {
        g_img_mutex.lock();
        g_img.copyTo(rgbImg);
        g_img.release();
        g_img_mutex.unlock();

        if(rgbImg.empty())
        {
            printf("img is empty, wait 10ms....\n");
            usleep(10000);
            continue;
        }


    //cv::imwrite("./result/out.jpg", rgbImg);
    //break;
        
    cv::cvtColor(rgbImg, yuvImg, cv::COLOR_BGR2YUV_I420);
    
     
    //yuvImg = rgbImg;

    auto start = std::chrono::system_clock::now();
    mppenc.encode((void*)yuvImg.data, yuv_size, pdst, &length);
    std::cout << "\nlength=" << length << std::endl;
        // fwrite(dst, length, 1,  fp);s
    auto end = std::chrono::system_clock::now();
    std::cout << "encode: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;


    start = std::chrono::system_clock::now();
    bool end_of_frame = false;
    int frame_size = length;
    if(frame_size > 0) 
        {
		xop::AVFrame videoFrame = {0};
		videoFrame.type = 0; 
		videoFrame.size = frame_size;
		videoFrame.timestamp = xop::H264Source::GetTimestamp();
		videoFrame.buffer.reset(new uint8_t[videoFrame.size]);    
		memcpy(videoFrame.buffer.get(), dst, videoFrame.size);
		rtsp_server->PushFrame(session_id, xop::channel_0, videoFrame);
	}

        end = std::chrono::system_clock::now();
        std::cout << "push: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    }
}




int main(int argc, char** argv) 
{
    std::string camera_cfg_file;
    std::string video_name;
    bool display = false;
    int send_time = -1;
    cv::VideoCapture cap;
    int port = 10020;
    int multithreading = 8555;

    if(argc == 7)
    {
        video_name = std::string(argv[1]);
        camera_cfg_file = std::string(argv[2]);
        display = atoi(argv[3]);
        send_time = atoi(argv[4]);
		send_ip = std::string(argv[5]);
        port = atoi(argv[6]);
    }else if(argc == 8)
	{
		video_name = std::string(argv[1]);
        camera_cfg_file = std::string(argv[2]);
        display = atoi(argv[3]);
        send_time = atoi(argv[4]);
		send_ip = std::string(argv[5]);
        port = atoi(argv[6]);
		multithreading = atoi(argv[7]);
	}
    else //默认路径
    {
        camera_cfg_file = "./config/camera_config.cfg";
        video_name = "./test.mp4";
        display = false;
		send_ip = "127.0.0.1";
		send_time = 0;
		port = 10020;
    }
    std::cout << "video name: " << video_name << "\n" 
     		<< "camera config file: " << camera_cfg_file << "\n"
			<< "display: " << display << "\n"
			<< "ip: " << send_ip << "\n"
			<< "port " << port << "\n"
			<< std::endl;


        
    //std::thread t1(loaddataprocess, video_name, camera_cfg_file, send_time, port);
    
    std::thread t1(CaptureRawDataThread, video_name, display);
    std::thread t2(inferencethread, camera_cfg_file, send_time, port);
    
    //t2.detach(); 
    
    std::string suffix = "mystream";
    std::string ip = "127.0.0.1";
    std::string port1 = to_string(multithreading);
    //std::string rtsp_url = "rtsp://" + ip + ":" + port + "/" + suffix;
	
    std::shared_ptr<xop::EventLoop> event_loop(new xop::EventLoop());
    std::shared_ptr<xop::RtspServer> server = xop::RtspServer::Create(event_loop.get());

    if (!server->Start("0.0.0.0", atoi(port1.c_str()))) 
    {
		printf("RTSP Server listen on %s failed.\n", port1.c_str());
		// return 0;
	}

    printf("RTSP Server listen on %s sucess.\n", port1.c_str());

    xop::MediaSession *session = xop::MediaSession::CreateNew("mystream"); 
    session->AddSource(xop::channel_0, xop::H264Source::CreateNew()); 
	//session->StartMulticast(); 
    session->AddNotifyConnectedCallback([] (xop::MediaSessionId sessionId, std::string peer_ip, uint16_t peer_port){
		printf("RTSP client connect, ip=%s, port=%hu \n", peer_ip.c_str(), peer_port);
	});
   
    session->AddNotifyDisconnectedCallback([](xop::MediaSessionId sessionId, std::string peer_ip, uint16_t peer_port) {
		printf("RTSP client disconnect, ip=%s, port=%hu \n", peer_ip.c_str(), peer_port);
	});

    xop::MediaSessionId session_id = server->AddSession(session);  
    std::cout<< "-----------session_id is-------" << session_id << std::endl;
    //std::thread t2(poss_EncodingSendFrameThread, server.get(), session_id, camera_cfg_file, send_time, port);
    //std::thread t3(EncodingSendFrameThread, server.get(), session_id);
    //t3.detach(); 
    
    t1.join();
    t2.join();
    //t3.join();
    	

    return 0;
}

