#include "rga.h"
#include "im2d.h"
#include "RgaUtils.h"
#include "rknn_api.h"
#include "postprocess.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "utils.h"

using namespace std;
#define PRINT_TIME
#pragma once

namespace camera 
{

class RKNNDetector
{
public:
    RKNNDetector(const char *model_name, const std::vector<std::string> class_names, float conf_thresh, float nms_thresh, bool nosigmoid, int npu_idx);
    ~RKNNDetector();

    int Process(cv::Mat& ori_img, bool isdraw = false);

protected:
    void PrepImage(const cv::Mat& ori_img);
    void PrepImageLetterbox(const cv::Mat& ori_img);
    void PrepImageLetterboxRga(const cv::Mat& ori_img);
    int myPrepImageLetterbox(const cv::Mat& ori_img);
    void PrepImage_resize_Letterbox(const cv::Mat& ori_img);


public:
    detect_result_group_t detect_result_group;

private:
    rknn_context ctx;
    unsigned char *model_data;
    rknn_sdk_version version;
    rknn_input_output_num io_num;
    rknn_tensor_attr *input_attrs;
    rknn_tensor_attr *output_attrs;
    rknn_input inputs[1];
    int img_width;
    int img_height;
    int ret;
    int channel = 3;
    int net_width = 640;  //模型输入的width和height
    int net_height = 640; 
    //int net_width = 960;  //模型输入的width和height
    //int net_height = 960; 
    int object_class_num; //目标类别总数

    float conf_thresh;
    float nms_thresh;

    vector<string> labels;
    void* net_input_buffer = nullptr;
    int net_input_size;

    std::vector<float> out_scales;
    std::vector<int32_t> out_zps;

    bool nosigmoid;  //后处理是否需要sigmoid操作， 1-->官网 0-->rk提供
    int x_offset = 0;
    int y_offset = 0;
    float scale_w;
    float scale_h;
    float scale;
    static void dump_tensor_attr(rknn_tensor_attr* attr){
      printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
      "zp=%d, scale=%f\n",
       attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
       attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
       get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}
    
};

}
