/*
* camera_app.h
* Created on: 20210622
* Author: yueyingying
* Description: 视频应用程序对外接口函数定义
*
*/
#ifndef __CAMERA_APP__
#define __CAMERA_APP__

#include <string>
#include <opencv2/opencv.hpp>
#include "camera_config_parser.h"
#include "../tracker/ByteTrack/byteTrack.h"
#include "NebulalinkObjectStruct3.h"

//#include "../common/CvxText.h"
#include <pthread.h>
#include <mutex>


namespace camera 
{

class Detector;
class ByteTracker;
class LocationApp;

class CameraAPP
{
public:
    CameraAPP() = default;
    ~CameraAPP();

    // 初始化
    bool Init(std::string config_file, int width, int height, std::string camera_uri="", float angle=0, double lon=0, double lat=0);

    // 检测+跟踪+定位
    void Process(cv::Mat img, uint64_t timestamp, perceptron::s_ObjectList* objects);

    //画目标框
    void DrawResult(cv::Mat& img, perceptron::s_ObjectList* objects);

    // start plate thread 
    // bool StartPlateThread();

     void Release();

private:
    // 打印版本信息
    void PrintVersion();
    // plate rec fun
    // void* ProcessPlateRecognitionThread();

private:
    AppConfig config_;
    Detector *detector_;
    ByteTracker *tracker_;
    LocationApp *location_;

    //CvxText zh_text;

    TrackList tracks_;
    // std::map<int, std::shared_ptr<TrackNode>> tracks_map_;
    
}; //class CameraAPP

} // namespace camera
#endif
