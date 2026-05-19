/*
* utils.h
* Created on: 20210622
* Author: yueyingying
* Description: yolo网络层输入相关参数,预处理与后处理相关函数定义
*
*/
#ifndef __TRT_UTILS_H_
#define __TRT_UTILS_H_

#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "../app/camera_config_parser.h"

namespace camera
{

#define INT_INF_MAX             1e6
#define FLOAT_INF_MAX           1e5
#define LNG_OFFSET              0.0000034647
#define LAT_OFFSET              0.0000012072

// 判断文件是否存在, 不存在返回false, 存在返回true
bool FindFile(const std::string fileName, bool verbose=true);

// 判断点是否在ROI区域内
bool IsPointInPolygon(cv::Point pt, Polygon roi);

int ReadFilesInDir(const char *p_dir_name, std::vector<std::string> &file_names);

std::vector<std::string> LoadListFromTextFile(const std::string filename);

std::vector<std::string> LoadImageList(const std::string filename, const std::string prefix);

bool WGS84ToGCJ02(double& lng, double& lat);

std::string TrimSpace(std::string s);

// 转换车牌在opencv中显示
//int ToWchar(const char* src, wchar_t* dest, const char *locale = "zh_CN.utf8");

int ClampValue(int value, int min, int max); 

}

#endif