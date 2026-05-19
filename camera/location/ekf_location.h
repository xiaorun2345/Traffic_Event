/*
* ekf_location.h
* Created on: 20230423
* Author: yueyingying
* Description: 基于卡尔曼滤波方法进行定位
*
*/

#pragma once

#include "../app/camera_config_parser.h"
#include "../common/dataType.h"
#include "N_Kalman_filter.h"
#include <memory>

namespace camera {

#define MAX_SPEED_DIFF_THRESH 1.0   // 与上一阵速度相差最大值, m/s
#define MAX_HEADING_DIFF_THRESH  90  //与上一帧相差航向角最大值
#define MIN_HEADING_UPDATE_TIMES 40
#define MIN_H_THRESH  25  // 目标框的高的最小阈值, 像素
#define MAX_TRACE_NUM 10
#define MIN_INTERVAL_TIME 0.005
#define MAX_INTERVAL_TIME 0.100

#define MAX_LOST_FRAME 70

// #define USE_FILTER_HEADING_CONFIRMED
#define USE_UV_FILTER

class EKFLocation
{
public:
    EKFLocation(const LocationConfig& config, const TrackNode& obj, u_int64_t timestamp);

    ~EKFLocation();

    void Predict(u_int64_t timestamp);
    void Update(double x, double y);
    void Update(double x, double y, float u, float v);

    inline void SetLastTimestamp(u_int64_t timestamp){last_timestamp_ = timestamp;}
    inline void AddLostTimes(){lost_times_++;}
    inline int GetLostTimes(){return lost_times_;}
    inline void SetLostTimes(int times){lost_times_ = times;}
    inline void MarkUpdated(){is_updated_ = true;}
    inline bool GetUpdated(){return is_updated_;}
    float GetHeading(){return last_heading_;}

    void GetFilterUV(double &u, double& v);

    void CalculateSpeedHeading(TrackNode& obj, float lock_velo, float velo_converged_thresh);
    void CalculateSpeedHeading(TrackList &tracks, int idx, float lock_velo, float velo_converged_thresh);

private:
    float state_ax_;
    float state_ay_;
    float std_meas_x_;
    float std_meas_y_;

    //上次预测的时间戳
    u_int64_t last_timestamp_;      //上一帧的时间
    u_int64_t last_obj_timestamp_;   //目标的上次时间

    bool is_in_link_;

    //当前帧是否被更新，如果未被更新，丢失次数++
    bool is_updated_;

    bool filter_heading_confirmed_;
    float last_posx_;
    float last_posy_;

    // 丢失的次数
    int lost_times_;

    float last_heading_;
    float last_speed_;
    float heading_init_;

    unsigned int update_times_;
    unsigned int age_;

    // std::vector<TrackNode> trajectory_;

    //计算航向角及速度的滤波器
    // std::shared_ptr<N_KalmanFilter> ekf_;
    // std::shared_ptr<N_KalmanFilter> uv_ekf_;

    N_KalmanFilter *ekf_;
    N_KalmanFilter *uv_ekf_;
}; 

}//namespace camera