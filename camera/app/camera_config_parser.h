/*
* camera_config_parser.h
* Created on: 20210530
* Author: yueyingying
* Description: 配置文件解析相关函数及结构体定义头文件
*
*/
#ifndef __DETECT_APP_H__
#define __DETECT_APP_H__

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <vector>

namespace camera 
{

typedef std::vector<cv::Point> Polygon;

const std::string network_modes[] = {"INT8", "FP16", "FP32"};

typedef struct _lane
{
    std::string lane_id;
    int lane_type;
    int vehicle_count;
    float lane_length;
    double vehicle_speed;
    Polygon lane_points;   // lane roi points
}Lane;

typedef struct _link
{
    std::string link_id;
    float heading;
    int direction;
    int lane_count;
    double vehicle_speed;
    int vehicle_count;
    std::vector<Lane> lanes;  // lanes 
    Polygon link_points;
}Link;

typedef struct _nl_config
{
    std::string  camera_sensor_id;  // camera设备SN号
    std::string  font_file;         // 视频标定文件
    // double h[9];                 // 经纬度转图像坐标的转换矩阵
    // double h_inv[9];             // 图像坐标转经纬度的转换矩阵

    std::vector<Polygon> roi_points; // roi区域顶点
    Polygon plate_roi_points;       // plate_roi区域顶点
    std::vector<Link> links;        // 该摄像对应路口的link数组
}NLConfig;

typedef struct _gie_config
{
    std::string engine_file;
    std::string model_file;
    std::string label_file;
    std::string network_mode;   //INT8 FP16 FP32
    std::vector<std::string> class_names; 
    int class_num;
    int batch_size;
    int npu;
    float nms_thresh;
    float conf_thresh;
}GieConfig;

typedef struct _tracker_config
{
    float low_thresh;
    float high_thresh;
    float high_thresh_person;
    float high_thresh_motorbike;
    float max_iou_thresh;   
}TrackerConfig;

typedef struct _roi_heading
{
    Polygon roi;
    float heading;
}HeadingConfig;

typedef struct _location_config
{
    bool enable;
    std::string  calibrator_file;   // 视频标定文件
    double std_meas_x;          //测量噪声 x, y
    double std_meas_y;   
    double std_state_ax;        //状态噪声x, y
    double std_state_ay;
    float lock_velo;            // 速度阈值,用于锁死航向角
    float velo_converged_var;   // 判断速度是否收敛的阈值

    float  heading;                 // ROI区域内的初始航向角
    float located_offset_angle;     // 摄像头安装位置相对正北方向的夹角
    double located_longitude;       // 安装位置的经度
    double located_latitude;        // 安装位置的纬度
    double located_elevation;       // 安装位置的海拔

    // int dirving_direction;          // 行驶方向， 0--> 来向  1--> 去向  -1-->未设置
    std::vector<HeadingConfig> headingcfg;   // 道路初始方向航向角
    // Polygon roi_right;            // 上下左右的ROI区域
    // Polygon roi_left;
    // Polygon roi_up;
    // Polygon roi_down;

}LocationConfig;

typedef struct _app_config
{
    NLConfig nl_config;
    GieConfig gie_config;
    TrackerConfig tracker_config;
    LocationConfig loc_config;
}AppConfig;


// 解析配置文件
bool ParseCameraConfigFile(AppConfig* config, const char* cfg_file_path);
// 打印配置文件信息
void PrintCameraConfigMessage(AppConfig* config);

} // namespace camera

#endif
