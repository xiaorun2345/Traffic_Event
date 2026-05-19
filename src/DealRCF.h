/*
 *  DealRCF.h
 *  Created on: 2021.7.6
 *  Author: DouDianSong
 *  Description: Deal radar camera fusion
 *
 */
#ifndef DealRCF_H
#define DealRCF_H

#include <iostream>
#include <stdlib.h>
#include <chrono>
#include <camera_app.h>
#include <nebulalink.perceptron3.0.5.pb.h>
#include <LibConfig.h++>
#include <LibConfig.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <strings.h>
#include <unistd.h>
#include <atomic> //C++11原子操作库
// #include <SerialPortCommunication.h>
#include <video_decoder.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <iostream>

#include "module/vi/module_rtspClient.hpp"
#include "module/vp/module_mppdec.hpp"
#include "module/vp/module_rga.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "LedScreen.h"
#define UNUSED(x) [&x] {}()
using namespace std;

struct External_ctx {
    shared_ptr<ModuleMedia> module;
    uint16_t test;
};

// #include <gst/gst.h>
// #include <gst/app/gstappsink.h>
// #include "../camera/detector/preprocess/preprocess.h"


#define LNG_OFFSET 0.0000034647
#define LAT_OFFSET 0.0000012072
#define CONFIG_PATH_LEN 100
#define MAIN_WHILE_TIME 10 // 秒
#define MAT_GET_TIME 30000 // 微秒
#define CAMERA_DEAL_TIME 10000
#define C_SHOW_LONG 1920
#define C_SHOE_WIDTH 1080
#define MAT_RESIZE 0.4
#define TOP_MAT_RESIZE 0.6
#define C_WAITKEY_TIME 1 //毫秒
#define RETURN_ZERO 0
#define RETURN_NEGATIVE -1
#define ERROR_ARG_NUM 2
#define ERROR_READ_CONFIG -9
#define ERROR_SEND_TO NULL
#define IP_NUM 16
#define gMaxPacketSize 20000
#define PRINT_DELAY_TIME 2 //秒
#define SEND_SENSOR_CAMERA 2
#define IS_SN_PROJECT  0    //是否是算能平台项目
#define SENSOR_OBJECT_LISTS_MAX 10
#define SENSOR_OBJECT_OVER_TIME 500 // 毫秒
#define PROCESS_VERSION "3.5.7"
#define RK3588_PROCESS_VERSION "1.3.1"
#define PROCESS_NAME "cw_DealRCF_nebulalink"
#define AVE_MAX_HISTORY 10
#define DEBUG_PAD_CYCLE 30000 //毫秒
#define LED_WARN_INTERVAL_MS 3000

#define RTSP_RECONNECT_INTERVAL_SEC 3  //rtsp超时重连时间，单位秒

enum e_SaveDataType
{
    DONOT_SAVE=0,               //默认不保存
    SAVE_CAMERA_RESULT_VIDEO=2, //保存视觉的结果视频
};

#define _DEBUG_TRACE_CMH_ 3
    #if 0 != _DEBUG_TRACE_CMH_
        #include <cstdio>
    #endif
        
    #if 1==_DEBUG_TRACE_CMH_    //普通打印
        #define TRACE_CMH printf
    #elif 2==_DEBUG_TRACE_CMH_    //打印文件名、行号
        #define TRACE_CMH(fmt,...) \
            //不支持新特性无法使用    printf("%s(%d): "##fmt, __FILE__, __LINE__, ##__VA_ARGS__)  
    #elif 3==_DEBUG_TRACE_CMH_    //打印文件名、行号、函数名
        #define TRACE_CMH (printf("%s(%d)-<%s>: ",__FILE__, __LINE__, __FUNCTION__), printf)
    #else
        #define TRACE_CMH
    #endif //_TRACE_CMH_DEBUG_

using namespace std;
using namespace perceptron;
//  一个字节对齐
#pragma pack(push, 4)

#if 1
// 取平均值
typedef struct AveObject
{
    bool is_tracker;                               // 物体是否被跟踪成功，如果没有，那么说明跟踪信息是不可靠的，一般都要求目标被追踪，字段暂且为1.
    float object_confidence;                       // 目标置信度，如果为int的话，最后给*0.01转换成float给出，并给出参考区间.
    string lane_id;                                // 目标所在车道id.
    perceptron::e_ObjectType object_class_type;    // 目标种类.
    int object_id;                                 // 目标Id.
    perceptron::s_Point3 point3f;                  // 目标的几何中心相对传感器坐标位置，单位为米.
    perceptron::s_Point4 point4f;                  // 图像坐标位置，1单位像素.
    float object_speed;                            // 目标速度，单位 m/s.
    perceptron::s_Speed3 speed3f;                  // 目标分轴速度，单位 m/s.
    float object_acceleration;                     // 目标加速度, 单位 m/s2.
    perceptron::s_TargetSize target_size;          // 目标尺寸.
    vector<perceptron::s_PointGPS> point_gps_list; // 目标经纬度坐标.
    perceptron::e_NS object_NS;                    // 南北纬.
    perceptron::e_EW object_WE;                    // 东西经.
    float object_direction;                        // 车辆车头朝向角度，与正北偏角，度为单位.
    float object_heading;                          // 行驶方向朝向航向角，以度为单位.
    int is_head_tail;                              // 1来向，0去向.
    perceptron::e_LaneType lane_type;              // 目标所在车道类型.
} s_AveObject;

typedef struct AveObjectList
{
    string devide_id;     // 设备号.
    bool devide_is_true;  // 判断设备数据的有效性.
    long long time_stamp; // 时间戳，到1970年1月1日0:00的毫秒数，sint最多10位，只能用int64表示.
    int number_frame;     // 帧的序列号.
    vector<s_AveObject> object_list;
    vector<perceptron::s_LaneJamSenseParams> lane_jam_sense_params;
    vector<perceptron::s_LinkJamSenseParams> link_jam_sense_params;

} s_AveObjectList;

#endif

typedef struct DealInfo
{
    string CameraConfig;
    string CameraURI;
    string LedWarningText;
    string LedIp;
    string LedSdkKey;
    string LedSdkSecret;
    string LedDeviceId;
    string DeviceID;
    int LedPort;
    int SendModel;
    int SendSensorType;
    char RsuIp[IP_NUM];
    int RsuPort;
    char CloudIp[IP_NUM];
    int CloudPort;
    string PortName;
    int BaudRate;
    int CameraShow;
    e_SaveDataType IsSaveData; //是否保存数据
    int SendTime;
    int IsPadDebug;
    int ListenPadPort;
    int CameraLength;
    int CameraWidth;
    float location_angle;
    double location_longitude;
    double location_latitude;
    int IsGcj02;
    int GetMatMode; // 0 普通的videocapture拉流 1 基于Xavier的gstreamer插件 H264编码 2 基于Xavier的gstreamer插件 H265编码 3 基于Origin的gstreamer插件 H264/H265编码(origin缺少omxh64dec插件)
    string RtspTransport;
    int ReconnectTimes;
    int SourceTimeoutSec;
} s_DealInfo;

typedef struct MatInfo
{
    unsigned long long timestamp;
    cv::Mat mat;
} s_MatInfo;
size_t GetThreadMemoryUsage(pid_t tid);
class DealRCF
{
public:
    DealRCF();

    static s_DealInfo deal_info_;

    // Trackers tracker_cw;
    // SerialPortCommunication *serialSend = nullptr;

    // 各模块输出缓存队列
    static vector<perceptron::s_ObjectList> sensor_object_lists_;
    static cv::Mat mat_, mat_camera_;
    static s_MatInfo mat_info_;

    // 各个模块的互斥锁以及信号量
    static pthread_mutex_t sensor_object_lists_mutex_, mat_object_list_mutex_, yushi_mat_object_list_mutex_, yuv_data_mutex_, yuv_time_mutex_;
    static pthread_rwlock_t yushi_mat_rwlock_;

    // UDP 通信相关参数
    static struct sockaddr_in send_addr_, send_cloud_addr_, pad_listen_addr, pad_client_addr;
    static int send_fd_, send_cloud_fd_, pad_fd;

    static int width_, height_;

    // pad数据同步信号量
    static sem_t pad_debug_sem_;

    // 判断是否给pad发数据
    static atomic_bool is_send_to_pad;
    static atomic_ullong pad_timestamp1, pad_timestamp2;
    static atomic_ullong last_frame_time_ms_;
    static atomic_uint frame_counter_;

public:
    // 加载配置文件    
    bool LoadConfig(const char *filename);
    
    //void GetFrameRawData(unsigned char *data, int h_stride, int v_stride, int width, int height, uint64_t timestamp);
    void *myMatDealThread();

    // 开启获取图像线程
    int StartMatDealThread(void *arg);

    // 获取图像线程执行体
    void *MatDealThread(void *arg);

    void *MatDealFFmpegThread(void *arg);
    void *MatDealGstreamerThread(void *arg);
    void *MatDealFfmediaThread(void *arg);

    // 开启与pad交互线程
    int StartDealPadThread(void *arg);

    // 与pad交互线程获取图像线程执行体
    void *PadDealThread(void *arg);

    // 发给pad心跳信息
    void *SendDealThread(void *arg);

    // 获取图像线程执行体, 宇视SDK
    void *YuShiMatDealThread(void *arg);

    // 获取图像线程执行体, 大华SDK
    void *DaHuaMatDealThread(void *arg);

    // 开启视觉线程
    int StartCameraDealThread(void *arg);

    // 视觉线程执行体
    void *CameraDealThread(void *arg);

    // 初始化UDP
    int InitUDP(void *pargma);

    // 打包成protobuf发出
    int SerializationsObjectListSend(perceptron::s_ObjectList object_list_, int type);

    // 打印s_ObjectList
    void PrintObjectList(perceptron::s_ObjectList object_list, char *sign);

    // 结构体变量之间赋值
    void CopyObjectList(perceptron::s_ObjectList *object_list1, perceptron::s_ObjectList *object_list2);

    // 判断缓存区的大小是否超过最大阈值
    bool JudgeSensorObjectListsSize(vector<perceptron::s_ObjectList> *sensor_object_lists);

    // 目标的时间是否超时
    bool JudgeSensorObjectOverTime(perceptron::s_ObjectList sensor_objects);

    // 获取从1970.1.1截至当前时间的毫秒数
    static unsigned long long GetMSecondTimeFrom1970();

    // 获取当前时间的毫秒数
    unsigned long long GetMSecondTime();

    // 打印主进程相关信息
    static bool PrintMainProcessInfo();

    // 开启录制离线数据线程
    int StartWriteDataThread(void *arg);

    // 录制离线数据线程执行体
    void *WriteDataThread(void *arg);

    // 平均
    s_AveObjectList ave_object_list_;
    // 结构体变量之间赋值
    void AveCopyObjectList(perceptron::s_ObjectList object_list);
    // 判断GPS缓存区的大小是否超过最大阈值
    bool AveJudgeSensorObjectListsSize();
    // 打包成protobuf发出
    int AveSerializationsObjectListSend(s_AveObjectList object_list_, int type);

    // 坐标系转换
    static bool Wgs84toGcj02(double &lng, double &lat);
    static double OutOfChina(double &lng, double &lat);
    static double TransFormLng(double lng, double lat);
    static double TransFormLat(double lng, double lat);

    int FfmediaInit(string rtsp);
};

#pragma pack(pop)

#endif // DealRCF_H
