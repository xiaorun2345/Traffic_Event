/*
* location_app.h
* Created on: 20230423
* Author: yueyingying
* Description: 定位模块主接口，用于计算目标的经纬度、速度、航向角等
*
*/

#pragma once

#include "ekf_location.h"
#include "../app/camera_config_parser.h"
#include <map>

namespace camera {

// 赤道周长的一半
#define EQUATORIAL_LEN 20037508.3427892
#define PI2 6.2831853071796
#define DISTANCEPSEC 30.83

class LocationApp 
{
public:
    // 构造函数
    LocationApp() = default;
    LocationApp(const LocationApp &) = delete;
    LocationApp &operator=(const LocationApp &) = delete;
    // 析构函数
    ~LocationApp();

    // 初始化函数, 计算映射矩阵
    bool Init(const LocationConfig& config);

    // 定位函数主程序
    void Process(const LocationConfig& config, u_int64_t timestamp, TrackList &tracks);

    void Release();

protected:
    //根据下边中心点计算经纬度
    bool CalculateGPS(TrackNode& obj);
    bool CalculateGPS(double x, double y, TrackNode& obj);

    //gps转相对于安装位置的xy
    void TransformGPSToXY(TrackNode& obj);

    //xy转GPS
    void TransformXYToGPS(TrackNode& obj);

private:
    // 目标列表， <目标ID， 定位算法对象> 
    // std::map<int, std::shared_ptr<EKFLocation>> objects_map_;
    std::map<int, EKFLocation*> objects_map_;

    // 映射矩阵
    double h_inv_[9];  
    double origin_longitude_;    // 摄像机安装位置的经度
    double origin_latitude_;     // 摄像机安装位置的纬度
    float origin_offset_angle_;  // 摄像机安装位置与正北方向的夹角
};

} //namespace camera