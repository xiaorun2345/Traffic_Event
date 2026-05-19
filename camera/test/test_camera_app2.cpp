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

using namespace std;

#define CAMERA_WIN "camera"
#define DEBUG_PAD_CYCLE 30000 //毫秒

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
	client_addr.sin_port = htons(send_port);

    //开启心跳和监听线程
	pthread_t pid;
	int ret;
	if ((ret = pthread_create(&pid, NULL, PadDealThread, NULL)) != 0)
	{
		cerr << "StartDealPadThread error ... " << endl;
        return false;
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

	if (0 != perceptrons.perceptron_size())
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

int main(int argc, char** argv) 
{
    std::string camera_cfg_file;
    std::string video_name;
    bool display = false;
    int send_time = -1;
    cv::VideoCapture cap;
    int port = 10020;
    int fps = 10;

    if(argc == 7)
    {
        video_name = std::string(argv[1]);
        camera_cfg_file = std::string(argv[2]);
        display = atoi(argv[3]);
        send_time = atoi(argv[4]);
		send_ip = std::string(argv[5]);
        port = atoi(argv[6]);
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

    //开启pad交互线程
    if(send_time >= 0)
        StartPadThread(port);

    cap.open(video_name, cv::CAP_FFMPEG);
    if(!cap.isOpened())
    {
        std::cout << "open video error...\n";
        return -1;
    }

    int width = (int)cap.get(3); 
    int height = (int)cap.get(4);
    //double fps = cap.get(cv::CAP_PROP_FPS);
    //int fourcc = static_cast<int>(cap.get(cv::CAP_PROP_FOURCC));
    //cv::VideoWriter outputvideo("output_video.avi", fourcc,fps,cv::Size(width, height));
    printf("video = %s, width = %d, height = %d\n", video_name.c_str(), width, height);

    // if(display)
    // {
    //     cv::namedWindow(CAMERA_WIN, cv::WINDOW_NORMAL);
    //     cv::resizeWindow(CAMERA_WIN, 1600, 900);
    // }

    // 视频应用程序初始化
    camera::CameraAPP * capp = new camera::CameraAPP();
    capp->Init(camera_cfg_file, width, height);  
    
    std::string pipeline = "appsrc ! videoconvert  ! x264enc ! h264parse ! flvmux ! rtmpsink location=rtmp://127.0.0.1:1935/mystream";  
    
    int width1 = (int)width ; 
    int height1 = (int)height ;
    cv::VideoWriter out;
    out.open(pipeline, cv::CAP_GSTREAMER, cv::VideoWriter::fourcc('h', '2', '6', '4'), fps, cv::Size(width1, height1), true);   
    

    unsigned int frame_count = 0;
    cv::Mat img;
	unsigned long long time1=0, time2=0;
    while(1)
    {
        cap >> img;
 		
        // frame_count++;
		// if(frame_count > 700)
		// {
		// 	capp->Release();
		// 	frame_count = 0;
		// 	cap.open(video_name);
		// 	continue;
		// }

        if(img.empty())
        {
            std::cout << "img is empty..\n";
            // break;
            cap.open(video_name);
            continue;
        }

        struct timespec time_stamp;
        clock_gettime(CLOCK_REALTIME, &time_stamp);
        uint64_t timestamp = time_stamp.tv_sec * 1e3 + time_stamp.tv_nsec/1e6;  //ms

        auto start = std::chrono::system_clock::now();

        perceptron::s_ObjectList objects;
        capp->Process(img, timestamp, &objects);
        //outputvideo.write(img);

        if(display)
        	capp->DrawResult(img, &objects);   // draw ROI
		
	  //cv::imwrite("result.jpg", img);
        
        auto end = std::chrono::system_clock::now();
        std::cout << "***********frame_"<< frame_count  <<"   total runtime: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

        auto start1 = std::chrono::system_clock::now();
        out.write(img);
        auto end1 = std::chrono::system_clock::now();
        std::cout << "write_ runtime: " << std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count() << "ms" << std::endl;
		time1 = GetMSecondTimeFrom1970();
		if (send_time <= (time1 - time2))
		{
			SerializationsObjectListSend(objects, true);
			time2 = GetMSecondTimeFrom1970();
		}
        

        // 显示
        // if(display)
        // {
        //     cv::imshow(CAMERA_WIN, img);
        //     if (cv::waitKey(10) == 'q')
        //         break;
        // }

      
    }

    // if(display)
    //     cv::destroyAllWindows();
    cap.release();
    out.release();
    //outputvideo.release();
    
    if(capp)
    {
        delete capp;
        capp = nullptr;
    }

    return 0;
}

