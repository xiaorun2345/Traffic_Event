/*
* class_detector.h
* Created on: 20210530
* Author: yueyingying
* Description: yolo目标检测对外接口定义
*
*/
#ifndef __CLASS_DETECTOR__
#define __CLASS_DETECTOR__

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "../app/camera_config_parser.h"
//#include "../config.h"
#include "rknn_detect.h"
#include "class_utils.h"

#pragma once

namespace camera 
{

class Detector
{
public:
    Detector() = default;
    ~Detector();
    void Init(AppConfig* config, int width, int height);
    void Process(cv::Mat img, std::vector<Yolo::s_Detection>& res);

    // void PrepImage(cv::Mat& img, void* data, void *buf);
private:
    std::unique_ptr<RKNNDetector> p_net_ = nullptr;

};

} // namespace camera

#endif