/*
* class_detector.cpp
* Created on: 20210530
* Author: yueyingying
* Description: yolo目标检测对外接口实现
*
*/
#include <chrono>
#include <fstream>
#include "class_detector.h"
#include "utils.h"
#include "rknn_detect.h"

namespace camera 
{

Detector::~Detector()
{
    std::cout << "detector deconstruct\n";
}

void Detector::Init(AppConfig* config, int width, int height)
{
    // p_net_ = std::unique_ptr<YoloV3> {new YoloV3(info, param, width, height)};
    // p_net_ = std::unique_ptr<YoloV5> {new YoloV5(info, param, width, height)};
    std::string classnames;
    for (uint i=0; i <config->gie_config.class_names.size(); i++)
    {
        if (!classnames.empty()) classnames += ";";
        classnames += config->gie_config.class_names[i];
    }
    
    p_net_ = std::unique_ptr<camera::RKNNDetector> {new RKNNDetector(config->gie_config.engine_file.c_str(), config->gie_config.class_names, config->gie_config.conf_thresh, config->gie_config.nms_thresh, 1, config->gie_config.npu)};    
}

void Detector::Process(cv::Mat img, std::vector<Yolo::s_Detection>& res)
{
    assert(!img.empty());

#ifdef PRINT_TIME
    auto start = std::chrono::system_clock::now();
#endif 

    //推理
    p_net_->Process(img);

#ifdef PRINT_TIME
    auto end = std::chrono::system_clock::now();
    std::cout << "rknn detection time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

    start = std::chrono::system_clock::now();
#endif

    // postprocess
    int x_offset, y_offset;
    int img_width  = img.cols;
    int img_height = img.rows;
    if(img_width > img_height)
    {
        x_offset = 0;
        y_offset = (img_width - img_height) /2 ;
    }
    else
    {
        x_offset = (img_height - img_width) /2;
        y_offset = 0;
    }

    for (int i = 0; i < p_net_->detect_result_group.count; i++)
    {
        Yolo::s_Detection output;
        detect_result_t *det_result = &(p_net_->detect_result_group.results[i]);
        output.conf = det_result->prop;
        output.class_id = det_result->class_idx;
        output.bbox[0] = det_result->box.left - x_offset;
        output.bbox[1] = det_result->box.top - y_offset;
        output.bbox[2] = det_result->box.right - x_offset;
        output.bbox[3] = det_result->box.bottom - y_offset;
        
        res.push_back(output);
    }

}

}
