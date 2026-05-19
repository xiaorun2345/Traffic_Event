#include "postprocess.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <set>
#include <vector>
#include <iostream>

const int anchor0[6] = {10, 13, 16, 30, 33, 23};
const int anchor1[6] = {30, 61, 62, 45, 59, 119};
const int anchor2[6] = {116, 90, 156, 198, 373, 326};

const int anchor3[6] = {4, 5, 8, 10, 22, 18};
//const int anchor1[6] = {10, 13, 16, 30, 33, 23};
//const int anchor2[6] = {30, 61, 62, 45, 59, 119};
//const int anchor3[6] = {116, 90, 156, 198, 373, 326};
const int MAX_NMS_CANDIDATES = 1000;

inline static int clamp(float val, int min, int max) { return val > min ? (val < max ? val : max) : min; }

static float CalculateDIoU(float xmin0, float ymin0, float xmax0, float ymax0,
                           float xmin1, float ymin1, float xmax1, float ymax1)
{
    // 计算交集的宽度和高度
    float inter_w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float inter_h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);

    // 计算交集面积
    float inter_area = inter_w * inter_h;

    // 计算并集的面积
    float union_area = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) +
                       (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - inter_area;

    // 计算两个边界框的中心点
    float center0_x = (xmin0 + xmax0) / 2.0;
    float center0_y = (ymin0 + ymax0) / 2.0;
    float center1_x = (xmin1 + xmax1) / 2.0;
    float center1_y = (ymin1 + ymax1) / 2.0;

    // 计算中心点之间的距离的平方
    float dist_sq = (center0_x - center1_x) * (center0_x - center1_x) +
                    (center0_y - center1_y) * (center0_y - center1_y);

    // 计算对角线距离的平方
    float diag_dist_sq = (xmax0 - xmin0) * (xmax0 - xmin0) + (ymax0 - ymin0) * (ymax0 - ymin0) +
                         (xmax1 - xmin1) * (xmax1 - xmin1) + (ymax1 - ymin1) * (ymax1 - ymin1);

    // 计算 DIoU
    float diou = (union_area <= 0.f) ? 0.f : (inter_area / union_area - dist_sq / diag_dist_sq);

    return diou;
}


static float CalculateCIoU(float xmin0, float ymin0, float xmax0, float ymax0,
                           float xmin1, float ymin1, float xmax1, float ymax1)
{
    // 计算交集的宽度和高度
    float inter_w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
    float inter_h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
    float aspect_ratio0 = (xmax0 - xmin0) / (ymax0 - ymin0);
    float aspect_ratio1 = (xmax1 - xmin1) / (ymax1 - ymin1);

    // 计算宽高比的arctan值
    float aspect_arctan0 = atanf(aspect_ratio0);
    float aspect_arctan1 = atanf(aspect_ratio1);
    float denominator = powf((aspect_arctan0 - aspect_arctan1), 2);
    // 计算交集面积
    float inter_area = inter_w * inter_h;
    float union_area = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) +
                       (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - inter_area;

    float numerator = 4.0f;

    // 计算表达式结果
    float iou = inter_area / union_area;
    float v = (numerator * denominator) / M_PI;
    float alpha = v / (1 - iou + v);

    // 计算并集的面积


    // 计算中心点之间的距离的平方
    float center0_x = (xmin0 + xmax0) / 2.0;
    float center0_y = (ymin0 + ymax0) / 2.0;
    float center1_x = (xmin1 + xmax1) / 2.0;
    float center1_y = (ymin1 + ymax1) / 2.0;

    float dist_sq = (center0_x - center1_x) * (center0_x - center1_x) +
                    (center0_y - center1_y) * (center0_y - center1_y);

    // 计算对角线距离的平方
    float diag_dist_sq = (xmax0 - xmin0) * (xmax0 - xmin0) + (ymax0 - ymin0) * (ymax0 - ymin0) +
                         (xmax1 - xmin1) * (xmax1 - xmin1) + (ymax1 - ymin1) * (ymax1 - ymin1);

    // 计算 CIoU
    float cioU = (union_area <= 0.f) ? 0.f : (iou - dist_sq / diag_dist_sq + alpha * v);
    return cioU;
}

static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1,
                              float ymax1)
{
	float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
	float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
	float i = w * h;
	float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
	return u <= 0.f ? 0.f : (i / u);
}

static int nms(int validCount, std::vector<float>& outputLocations, std::vector<int> classIds, std::vector<int>& order,
	               int filterId, float threshold)
{
	//printf("val is: %d\n", validCount);
	for (int i = 0; i < validCount; ++i) {
		int n = order[i];
		if (n == -1 || classIds[n] != filterId) {
			continue;
		}
		for (int j = i + 1; j < validCount; ++j) {
			int m = order[j];
			if (m == -1 || classIds[m] != filterId) {
				continue;
			}
			float xmin0 = outputLocations[n * 4 + 0];
			float ymin0 = outputLocations[n * 4 + 1];
			float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
			float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];

			float xmin1 = outputLocations[m * 4 + 0];
			float ymin1 = outputLocations[m * 4 + 1];
			float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
			float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

			float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);

			if (iou > threshold) {
				order[j] = -1;
			}
		}
	}
	return 0;
}

static int quick_sort_indice_inverse(std::vector<float>& input, int left, int right, std::vector<int>& indices)
{
	float key;
	int   key_index;
	int   low  = left;
	int   high = right;
	if (left < right) {
		key_index = indices[left];
		key       = input[left];
		while (low < high) {
			while (low < high && input[high] <= key) {
				high--;
			}
			input[low]   = input[high];
			indices[low] = indices[high];
			while (low < high && input[low] >= key) {
				low++;
			}
			input[high]   = input[low];
			indices[high] = indices[low];
		}
		input[low]   = key;
		indices[low] = key_index;
		quick_sort_indice_inverse(input, left, low - 1, indices);
		quick_sort_indice_inverse(input, low + 1, right, indices);
	}
	return low;
}


static float sigmoid(float x) 
{ 
	return 1.0 / (1.0 + expf(-x)); 
	//return x;
}


static float unsigmoid(float y)
{ 
	return -1.0 / logf((1.0 / y) - 1.0); 
	//return y;
}

inline static int32_t __clip(float val, float min, float max)
{
	float f = val <= min ? min : (val >= max ? max : val);
	return f;
}

static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale)
{
	float  dst_val = (f32 / scale) + zp;
	int8_t res     = (int8_t)__clip(dst_val, -128, 127);
	return res;
}

static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) { return ((float)qnt - (float)zp) * scale; }

static int process(int8_t* input, int* anchor, int grid_h, int grid_w, int height, int width, int stride,
                   std::vector<float>& boxes, std::vector<float>& objProbs, std::vector<int>& classId, 
				   int obj_num, float threshold, int32_t zp, float scale)
{


      //std::cout << "input: " << static_cast<void*>(input) << std::endl;
    	//std::cout << "anchor: " << static_cast<void*>(anchor) << std::endl;

      //std::cout << "Input values:" << std::endl;
  	int numValuesToPrint = 5;  // ָ  Ҫ     ֵ      

  	//    ǰ    input     ֵ
  	//for (int i = 0; i < numValuesToPrint; ++i) {
      	//std::cout << "input[" << i << "]: " << static_cast<int>(input[i]) << std::endl;
	  //}
  	//std::cout << "Anchor values:" << std::endl;

  	//    ǰ    anchor     ֵ
  	//for (int i = 0; i < numValuesToPrint; ++i) {
      	//std::cout << "anchor[" << i << "]: " << anchor[i] << std::endl;
  	//}

    	//std::cout << "grid_h: " << grid_h << std::endl;
    	//std::cout << "grid_w: " << grid_w << std::endl;
    	//std::cout << "height: " << height << std::endl;
    	//std::cout << "width: " << width << std::endl;
    	//std::cout << "stride: " << stride << std::endl;
    	//std::cout << "obj_num: " << obj_num << std::endl;
    	//std::cout << "threshold: " << threshold << std::endl;
    	//std::cout << "zp: " << zp << std::endl;
    	//std::cout << "scale: " << scale << std::endl;
	//printf(" use pross_sig ");
	int    validCount = 0;
	int    grid_len   = grid_h * grid_w;
	static int prop_box_size = 5 + obj_num;
	float  thres      = unsigmoid(threshold);
	int8_t thres_i8   = qnt_f32_to_affine(thres, zp, scale);
	printf("******thres: %f   thres_i8: %d\n", thres, thres_i8);
	int flag = 3;
	for (int a = 0; a < 3; a++) {
		for (int i = 0; i < grid_h; i++) {
			for (int j = 0; j < grid_w; j++) {
				int8_t box_confidence = input[(prop_box_size * a + 4) * grid_len + i * grid_w + j];
			
				if (box_confidence >= thres_i8)
				//if (1)
				 {
					int     offset = (prop_box_size * a) * grid_len + i * grid_w + j;
					int8_t* in_ptr = input + offset;
					float   box_x  = sigmoid(deqnt_affine_to_f32(*in_ptr, zp, scale)) * 2.0 - 0.5;
					float   box_y  = sigmoid(deqnt_affine_to_f32(in_ptr[grid_len], zp, scale)) * 2.0 - 0.5;
					float   box_w  = sigmoid(deqnt_affine_to_f32(in_ptr[2 * grid_len], zp, scale)) * 2.0;
					float   box_h  = sigmoid(deqnt_affine_to_f32(in_ptr[3 * grid_len], zp, scale)) * 2.0;
					box_x          = (box_x + j) * (float)stride;
					box_y          = (box_y + i) * (float)stride;
					box_w          = box_w * box_w * (float)anchor[a * 2];
					box_h          = box_h * box_h * (float)anchor[a * 2 + 1];
					box_x -= (box_w / 2.0);
					box_y -= (box_h / 2.0);

					int8_t maxClassProbs = in_ptr[5 * grid_len];
					int    maxClassId    = 0;
					for (int k = 1; k < obj_num; ++k) {
						int8_t prob = in_ptr[(5 + k) * grid_len];
						if (prob > maxClassProbs) {
							maxClassId    = k;
							maxClassProbs = prob;
						}
					}
					//if (maxClassProbs>thres_i8)
					if (1)
					{
						objProbs.push_back(sigmoid(deqnt_affine_to_f32(maxClassProbs, zp, scale))* sigmoid(deqnt_affine_to_f32(box_confidence, zp, scale)));
						classId.push_back(maxClassId);
						validCount++;
						boxes.push_back(box_x);
						boxes.push_back(box_y);
						boxes.push_back(box_w);
						boxes.push_back(box_h);
					}
				}
			}
		}
  	}
  	return validCount;
}

static int process_nosigmoid(int8_t* input, int* anchor, int grid_h, int grid_w, int height, int width, int stride,
                   std::vector<float>& boxes, std::vector<float>& objProbs, std::vector<int>& classId, 
				   int obj_num, float threshold, int32_t zp, float scale)
{

	//std::cout << "input: " << static_cast<void*>(input) << std::endl;
    	//std::cout << "anchor: " << static_cast<void*>(anchor) << std::endl;

      //std::cout << "Input values:" << std::endl;
  	int numValuesToPrint = 10;  // ָ  Ҫ     ֵ      

  	//    ǰ    input     ֵ
  	//for (int i = 0; i < numValuesToPrint; ++i) {
      //	std::cout << "input[" << i << "]: " << static_cast<int>(input[i]) << std::endl;
	//  }
  	//std::cout << "Anchor values:" << std::endl;

  	//    ǰ    anchor     ֵ
  	//for (int i = 0; i < numValuesToPrint; ++i) {
      //	std::cout << "anchor[" << i << "]: " << anchor[i] << std::endl;
  	//}

    	//std::cout << "grid_h: " << grid_h << std::endl;
    	//std::cout << "grid_w: " << grid_w << std::endl;
    	//std::cout << "height: " << height << std::endl;
    	//std::cout << "width: " << width << std::endl;
    	//std::cout << "stride: " << stride << std::endl;
    	//std::cout << "obj_num: " << obj_num << std::endl;
    	//std::cout << "threshold: " << threshold << std::endl;
    	//std::cout << "zp: " << zp << std::endl;
    	//std::cout << "scale: " << scale << std::endl;
	//printf(" use pross_nosig ");
	int    validCount = 0;
	int    grid_len   = grid_h * grid_w;
	float  thres      = threshold;
	//static int prop_box_size = 5 + obj_num;
	int prop_box_size = (5 + obj_num);
  	int8_t thres_i8   = qnt_f32_to_affine(thres, zp, scale);
  	//std::cout << "thres_i8: " << static_cast<int>(thres_i8) << std::endl;
  	for (int a = 0; a < 3; a++) {
    	for (int i = 0; i < grid_h; i++) {
      		for (int j = 0; j < grid_w; j++) {
				int8_t box_confidence = input[(prop_box_size * a + 4) * grid_len + i * grid_w + j];
				//std::cout << "box_confidence: " << static_cast<int>(box_confidence) << std::endl;
				
				//if (1)
				if (box_confidence >= thres_i8) 
				{
					//std::cout << "box_confidence: " << static_cast<int>(box_confidence) << std::endl;
					int     offset = (prop_box_size * a) * grid_len + i * grid_w + j;
					int8_t* in_ptr = input + offset;
					float   box_x  = deqnt_affine_to_f32(*in_ptr, zp, scale) * 2.0 - 0.5;
					float   box_y  = deqnt_affine_to_f32(in_ptr[grid_len], zp, scale) * 2.0 - 0.5;
					float   box_w  = deqnt_affine_to_f32(in_ptr[2 * grid_len], zp, scale) * 2.0;
					float   box_h  = deqnt_affine_to_f32(in_ptr[3 * grid_len], zp, scale) * 2.0;
					box_x          = (box_x + j) * (float)stride;
					box_y          = (box_y + i) * (float)stride;
					box_w          = box_w * box_w * (float)anchor[a * 2];
					box_h          = box_h * box_h * (float)anchor[a * 2 + 1];
					box_x -= (box_w / 2.0);
					box_y -= (box_h / 2.0);

					int8_t maxClassProbs = in_ptr[5 * grid_len];
					int    maxClassId    = 0;
					//std::cout << "maxClassProbs: " << static_cast<int>(maxClassProbs) << std::endl;
					
					for (int k = 1; k < obj_num; ++k) {
						int8_t prob = in_ptr[(5 + k) * grid_len];
						if (prob > maxClassProbs) {
						maxClassId    = k;
						maxClassProbs = prob;
						}
					}
					
					//if (maxClassProbs>thres_i8){
					if (deqnt_affine_to_f32(maxClassProbs, zp, scale)* deqnt_affine_to_f32(box_confidence, zp, scale) > threshold){
						objProbs.push_back(deqnt_affine_to_f32(maxClassProbs, zp, scale)* deqnt_affine_to_f32(box_confidence, zp, scale));
						classId.push_back(maxClassId);
						validCount++;
						boxes.push_back(box_x);
						boxes.push_back(box_y);
						boxes.push_back(box_w);
						boxes.push_back(box_h);
					}
        		}
      		}
    	}
  	}
  	return validCount;
}


int post_process(int8_t* input0, int8_t* input1, int8_t* input2, int model_in_h, int model_in_w, float conf_threshold,
                 float nms_threshold, float scale_w, float scale_h, int obj_num, std::vector<int32_t>& qnt_zps, 
                 std::vector<float>& qnt_scales, detect_result_group_t* group)
{
	memset(group, 0, sizeof(detect_result_group_t));

	std::vector<float> filterBoxes;
	std::vector<float> objProbs;
	std::vector<int>   classId;

	// stride 8
	int stride0     = 8;
	int grid_h0     = model_in_h / stride0;
	int grid_w0     = model_in_w / stride0;
	int validCount0 = 0;

	validCount0 = process(input0, (int*)anchor0, grid_h0, grid_w0, model_in_h, model_in_w, stride0, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[0], qnt_scales[0]);

	// stride 16
	int stride1     = 16;
	int grid_h1     = model_in_h / stride1;
	int grid_w1     = model_in_w / stride1;
	int validCount1 = 0;

	validCount1 = process(input1, (int*)anchor1, grid_h1, grid_w1, model_in_h, model_in_w, stride1, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[1], qnt_scales[1]);

	// stride 32
	int stride2     = 32;
	int grid_h2     = model_in_h / stride2;
	int grid_w2     = model_in_w / stride2;
	int validCount2 = 0;

	validCount2 = process(input2, (int*)anchor2, grid_h2, grid_w2, model_in_h, model_in_w, stride2, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[2], qnt_scales[2]);


	int validCount = validCount0 + validCount1 + validCount2;
	// no object detect
	if (validCount <= 0) {
		printf("no object detect");
		return 0;
	}

	std::vector<int> indexArray;
	for (int i = 0; i < validCount; ++i) {
		indexArray.push_back(i);
	}

	quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);
	if (validCount > MAX_NMS_CANDIDATES) {
		validCount = MAX_NMS_CANDIDATES;
	}

	std::set<int> class_set(std::begin(classId), std::end(classId));

	for (auto c : class_set) {
		nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
	}

	int last_count = 0;
	group->count   = 0;
	/* box valid detect target */
	for (int i = 0; i < validCount; ++i) {
		if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE) {
			continue;
		}
		int n = indexArray[i];

		float x1       = filterBoxes[n * 4 + 0];
		float y1       = filterBoxes[n * 4 + 1];
		float x2       = x1 + filterBoxes[n * 4 + 2];
		float y2       = y1 + filterBoxes[n * 4 + 3];
		int   id       = classId[n];
		float obj_conf = objProbs[i];

		group->results[last_count].box.left   = (int)(clamp(x1, 0, model_in_w) / scale_w);
		group->results[last_count].box.top    = (int)(clamp(y1, 0, model_in_h) / scale_h);
		group->results[last_count].box.right  = (int)(clamp(x2, 0, model_in_w) / scale_w);
		group->results[last_count].box.bottom = (int)(clamp(y2, 0, model_in_h) / scale_h);
		group->results[last_count].prop       = obj_conf;
		group->results[last_count].class_idx = id;

		last_count++;
  	}
  	group->count = last_count;

  	return 0;
}

int post_process_nosigmoid_addlayer(int8_t* input0, int8_t* input1, int8_t* input2, int8_t* input3, int model_in_h, int model_in_w, float conf_threshold,
                 float nms_threshold, float scale_w, float scale_h, int obj_num, std::vector<int32_t>& qnt_zps, 
                 std::vector<float>& qnt_scales, detect_result_group_t* group)
{
  
	memset(group, 0, sizeof(detect_result_group_t));

	std::vector<float> filterBoxes;
	std::vector<float> objProbs;
	std::vector<int>   classId;

	// stride 8
	int stride0     = 4;
	int grid_h0     = model_in_h / stride0;
	int grid_w0     = model_in_w / stride0;
	int validCount0 = 0;

	validCount0 = process_nosigmoid(input0, (int*)anchor0, grid_h0, grid_w0, model_in_h, model_in_w, stride0, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[0], qnt_scales[0]);	

	// stride 16
	int stride1     = 8;
	int grid_h1     = model_in_h / stride1;
	int grid_w1     = model_in_w / stride1;
	int validCount1 = 0;

	validCount1 = process_nosigmoid(input1, (int*)anchor1, grid_h1, grid_w1, model_in_h, model_in_w, stride1, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[1], qnt_scales[1]);

	// stride 32
	int stride2     = 16;
	int grid_h2     = model_in_h / stride2;
	int grid_w2     = model_in_w / stride2;
	int validCount2 = 0;

	validCount2 = process_nosigmoid(input2, (int*)anchor2, grid_h2, grid_w2, model_in_h, model_in_w, stride2, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[2], qnt_scales[2]);
							
							
        // stride 32
	int stride3     = 32;
	int grid_h3     = model_in_h / stride3;
	int grid_w3     = model_in_w / stride3;
	int validCount3 = 0;

	validCount3 = process_nosigmoid(input3, (int*)anchor3, grid_h3, grid_w3, model_in_h, model_in_w, stride3, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[3], qnt_scales[3]);			
				

	int validCount =  validCount0 + validCount1 + validCount2 + validCount3 ;
	//int validCount = validCount1 + validCount2 ;
	
	std::cout<<"0==" << validCount0 << " 1==" <<  validCount1 <<" 2=="<<  validCount2 << " 3=="<<  validCount3 <<std::endl;;
	// no object detect
	//printf("-----detect----");
	if (validCount <= 0) {
		printf("no object detect");
		return 0;
	}

	printf("detect sth");
	printf("val is: %d\n", validCount);

	std::vector<int> indexArray;
	for (int i = 0; i < validCount; ++i) {
		indexArray.push_back(i);
	}

	quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);
	if (validCount > MAX_NMS_CANDIDATES) {
		validCount = MAX_NMS_CANDIDATES;
	}

	std::set<int> class_set(std::begin(classId), std::end(classId));

	for (auto c : class_set) {
		nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
	}

	int last_count = 0;
	group->count   = 0;
	/* box valid detect target */

	for (int i = 0; i < validCount; ++i) {
		if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE) {
			continue;
		}
		int n = indexArray[i];

		float x1       = filterBoxes[n * 4 + 0];
		float y1       = filterBoxes[n * 4 + 1];
		float x2       = x1 + filterBoxes[n * 4 + 2];
		float y2       = y1 + filterBoxes[n * 4 + 3];
		int   id       = classId[n];
		float obj_conf = objProbs[i];

		group->results[last_count].box.left   = (int)(clamp(x1, 0, model_in_w) / scale_w);
		group->results[last_count].box.top    = (int)(clamp(y1, 0, model_in_h) / scale_h);
		group->results[last_count].box.right  = (int)(clamp(x2, 0, model_in_w) / scale_w);
		group->results[last_count].box.bottom = (int)(clamp(y2, 0, model_in_h) / scale_h);
		group->results[last_count].prop       = obj_conf;
		group->results[last_count].class_idx = id;

		last_count++;
  	}
  	group->count = last_count;

  	return 0;
}

int post_process_nosigmoid(int8_t* input0, int8_t* input1, int8_t* input2, int model_in_h, int model_in_w, float conf_threshold,
                 float nms_threshold, float scale_w, float scale_h, int obj_num, std::vector<int32_t>& qnt_zps, 
                 std::vector<float>& qnt_scales, detect_result_group_t* group)
{
  
	memset(group, 0, sizeof(detect_result_group_t));

	std::vector<float> filterBoxes;
	std::vector<float> objProbs;
	std::vector<int>   classId;

	// stride 8
	int stride0     = 8;
	int grid_h0     = model_in_h / stride0;
	int grid_w0     = model_in_w / stride0;
	int validCount0 = 0;

	validCount0 = process_nosigmoid(input0, (int*)anchor0, grid_h0, grid_w0, model_in_h, model_in_w, stride0, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[0], qnt_scales[0]);	

	// stride 16
	int stride1     = 16;
	int grid_h1     = model_in_h / stride1;
	int grid_w1     = model_in_w / stride1;
	int validCount1 = 0;

	validCount1 = process_nosigmoid(input1, (int*)anchor1, grid_h1, grid_w1, model_in_h, model_in_w, stride1, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[1], qnt_scales[1]);

	// stride 32
	int stride2     = 32;
	int grid_h2     = model_in_h / stride2;
	int grid_w2     = model_in_w / stride2;
	int validCount2 = 0;

	validCount2 = process_nosigmoid(input2, (int*)anchor2, grid_h2, grid_w2, model_in_h, model_in_w, stride2, filterBoxes, objProbs,
							classId, obj_num, conf_threshold, qnt_zps[2], qnt_scales[2]);														
				
	int validCount =  validCount0 + validCount1 + validCount2  ;
	
	std::cout<<"0==" << validCount0 << " 1==" <<  validCount1 <<" 2=="<<  validCount2 << std::endl;;
	// no object detect
	//printf("-----detect----");
	if (validCount <= 0) {
		printf("no object detect");
		return 0;
	}

	printf("detect sth");
	printf("val is: %d\n", validCount);

	std::vector<int> indexArray;
	for (int i = 0; i < validCount; ++i) {
		indexArray.push_back(i);
	}

	quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);
	if (validCount > MAX_NMS_CANDIDATES) {
		validCount = MAX_NMS_CANDIDATES;
	}

	std::set<int> class_set(std::begin(classId), std::end(classId));

	for (auto c : class_set) {
		nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
	}

	int last_count = 0;
	group->count   = 0;
	/* box valid detect target */

	for (int i = 0; i < validCount; ++i) {
		if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE) {
			continue;
		}
		int n = indexArray[i];

		float x1       = filterBoxes[n * 4 + 0];
		float y1       = filterBoxes[n * 4 + 1];
		float x2       = x1 + filterBoxes[n * 4 + 2];
		float y2       = y1 + filterBoxes[n * 4 + 3];
		int   id       = classId[n];
		float obj_conf = objProbs[i];

		group->results[last_count].box.left   = (int)(clamp(x1, 0, model_in_w) / scale_w);
		group->results[last_count].box.top    = (int)(clamp(y1, 0, model_in_h) / scale_h);
		group->results[last_count].box.right  = (int)(clamp(x2, 0, model_in_w) / scale_w);
		group->results[last_count].box.bottom = (int)(clamp(y2, 0, model_in_h) / scale_h);
		group->results[last_count].prop       = obj_conf;
		group->results[last_count].class_idx = id;

		last_count++;
  	}
  	group->count = last_count;

  	return 0;
}


