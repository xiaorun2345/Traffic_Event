#ifndef _RKNN_ZERO_COPY_DEMO_POSTPROCESS_H_
#define _RKNN_ZERO_COPY_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "utils.h"

// #define OBJ_CLASS_NUM     7
// #define PROP_BOX_SIZE     (5+OBJ_CLASS_NUM)
// #define YOLO_MODEL 1  //yolo类型，0-->官网 1-->rk提供

int post_process(int8_t *input0, int8_t *input1, int8_t *input2, int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold, float scale_w, float scale_h, int obj_num,
                 std::vector<int32_t> &qnt_zps, std::vector<float> &qnt_scales, 
                 detect_result_group_t *group);

int post_process_nosigmoid(int8_t *input0, int8_t *input1, int8_t *input2, int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold, float scale_w, float scale_h, int obj_num,
                 std::vector<int32_t> &qnt_zps, std::vector<float> &qnt_scales,
                 detect_result_group_t *group);

int post_process_nosigmoid_addlayer(int8_t *input0, int8_t *input1, int8_t *input2, int8_t* input3, int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold, float scale_w, float scale_h, int obj_num,
                 std::vector<int32_t> &qnt_zps, std::vector<float> &qnt_scales,
                 detect_result_group_t *group);

#endif //_RKNN_ZERO_COPY_DEMO_POSTPROCESS_H_
