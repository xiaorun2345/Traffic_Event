/*
* class_utils.h
* Created on: 20220830
* Author: yueyingying
* Description: yolo共用结构体及宏定义
*
*/
#ifndef __TRT_UTILS_H__
#define __TRT_UTILS_H__

#include <string>

namespace camera 
{

#define YOLOV3_V5_VERSION 

namespace Yolo
{
    static constexpr int CHECK_COUNT = 3;
    static constexpr float IGNORE_THRESH = 0.1f;
    static constexpr int MAX_OUTPUT_BBOX_COUNT = 1500;
    static constexpr int CLASS_NUM = 7;
    static constexpr int INPUT_H = 640;
    static constexpr int INPUT_W = 640;
    static constexpr int CHANNEL = 3;

    struct s_YoloKernel
    {
        int width;
        int height;
        float anchors[CHECK_COUNT*2];
    };

    static constexpr int LOCATIONS = 4;
    struct alignas(float) s_Detection
    {
        //xmin ymin xmax ymax
        float bbox[LOCATIONS];
        float conf;  // bbox_conf * cls_conf
        float class_id;
    };
} // namespace Yolo


} // namespace camera

#endif
