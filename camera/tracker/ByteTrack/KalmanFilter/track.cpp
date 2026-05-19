/*
* track.cpp
* Created on: 20210609
* Author: yueyingying
* Description: track类的实现, 单个跟踪目标
*/
#include "track.h"
#include <sys/time.h>

namespace camera
{


Track::~Track()
{
    std::vector<TrackNode>().swap(track_list_);
    classes_map_.clear();
}

Track::Track(KMean& mean, KCovar& covariance, int track_id_, int n_init, int max_age, DetectionRow detection)
{
    this->mean_ = mean;
    this->covariance_ = covariance;
    this->track_id_ = track_id_;
    this->hits_ = 1;
    this->age_ = 1;
    this->time_since_update_ = 0;
    this->state_ = TrackState::Tentative;

    this->latest_detection_ = detection;
    this->lastest_bbox_tlwh = detection.tlwh;

    this->n_init_ = n_init;
    this->max_age_ = max_age;

    this->locked_class_type_ = -1;

#if defined(WITHOUT_CLASS_TYPE_LOCKED)
    this->is_locked_class_ = true;
#else
    this->is_locked_class_ = false;
#endif
}

void Track::Predict(KAL::KalmanFilter *kf)
{
    /*Propagate the state distribution to the current time step using a
        Kalman filter prediction step.

        Parameters
        ----------
        kf : kalman_filter.KalmanFilter
            The Kalman filter.
    */

    kf->Predict(this->mean_, this->covariance_);

    this->age_ += 1;
    this->time_since_update_ += 1;

    // 滤波后的bbox小于0, 设置状态为delete
    Bbox b = ToCxymax();
    if(b(0) <= 0 || b(1) <= 0 || b(2) <= 0 || b(3) <= 0)
        this->state_ = TrackState::Deleted;
}

//匹配上了，用检测的结果更新跟踪的目标
void Track::Update(KAL::KalmanFilter * const kf, const DetectionRow& detection)
{
    //根据目标框的轨迹计算检测的目标是否可信，匹配上的目标
#if 0
    bool confirmed = true;
    double avg_x = 0, avg_y=0;
    auto size = track_list_.size();
    // printf("size: %ld\n", size);
    if(size > MAX_TRACK_LIST_SIZE-5)
    {
        for(int i=0; i<size; i++)
        {
            avg_x += track_list_[i].cywh[0];
            avg_y += track_list_[i].cywh[1];
        }

        //平均位置
        avg_x /= size;
        avg_y /= size;
 
        //轨迹中最早的位置
        float x_old = track_list_[0].cywh[0];
        float y_old = track_list_[0].cywh[1];

        float x_diff = avg_x - x_old;
        float y_diff = avg_y - y_old;

        float x_last = track_list_[size-1].cywh[0];
        float y_last = track_list_[size-1].cywh[1];
        // Eigen::Vector2f v1(x_last-x_old, y_last-y_old);

        Bbox box =  detection.ToCxymax();
        float x_new = box[0];
        float y_new = box[1];

        float x_diff1 = x_new - x_last;
        float y_diff1 = y_new - y_last;

        if(std::fabs(x_diff1) < std::fabs(y_diff1))
        {//沿着y方向移动
            if(std::fabs(y_diff1)>5 && y_diff1*y_diff < 0)
            {
                this->angle_ = 1;
                confirmed = false;
                printf("@@@@@@@@@@@@ id:%d angle:%f %f %f %f %f\n", this->track_id_, angle_, x_diff, y_diff, x_diff1, y_diff1);
            }
        }
        else
        {
            this->angle_=0;
            confirmed=true;
        }
            
        // Eigen::Vector2f v2(x_new-x_last, y_new-y_last);
        // 计算向量夹角
        // if(v1.norm()>0 && v2.norm()>0)
        // {
        //     float sita = v1.dot(v2) / (v1.norm()*v2.norm()); //角度
        //     float angle = acos(sita) * 180 / M_PI; //弧度
        //     this->angle_ = angle;
        // }
        // else
        //     this->angle_ =0;

    }
    else
    {
        this->angle_=0;
        confirmed=true;        
    }
#endif

#if USE_XYAH
    KPair pa = kf->Update(this->mean_, this->covariance_, detection.ToCxcyah());
#else
    KPair pa = kf->Update(this->mean_, this->covariance_, detection.ToCxcywh());
#endif


    this->mean_ = pa.first;
    this->covariance_ = pa.second;

    this->hits_ += 1;
 
    this->time_since_update_ = 0;


    this->latest_detection_ = detection;

#if 0
    if(!confirmed)
    {
        this->latest_detection_.tlwh[0] = track_list_[size-1].cywh[0]-track_list_[size-1].cywh[2]/2;
        this->latest_detection_.tlwh[1] = track_list_[size-1].cywh[1]-track_list_[size-1].cywh[3];
        this->latest_detection_.tlwh[2] = track_list_[size-1].cywh[2];
        this->latest_detection_.tlwh[3] = track_list_[size-1].cywh[3];
    }
#endif

    if(this->state_ == TrackState::Tentative && this->hits_ >= this->n_init_) 
    {
        this->state_ = TrackState::Confirmed;
    }

#ifndef WITHOUT_CLASS_TYPE_LOCKED
    //这个地方统计目标类型个数
    if(!is_locked_class_) //目标类型没锁死
    {
        int type = detection.type;
        auto it = classes_map_.find(type);
        if(it != classes_map_.end()) //存在
        {
            if(it->second.first < detection.confidence)
                it->second.first = detection.confidence;
            it->second.second ++;
        }
        else //不存在，添加到map中
        {
            std::pair<float, int> conf_count = std::pair<float, int>(detection.confidence, 1);
            classes_map_.insert(std::pair<int, std::pair<float, int> >(type, conf_count));
        }
    }
#endif
}

void Track::MarkMissed()
{
#if USE_AGE
    if(this->state_ == TrackState::Tentative)
    {
        this->state_ = TrackState::Deleted;
    } 
    else if(this->time_since_update_ >= this->max_age_) 
    {
        this->state_ = TrackState::Deleted;
    }
#else
    this->state_ = TrackState::Deleted;
#endif
}

bool Track::IsConfirmed()
{
    return this->state_ == TrackState::Confirmed;
}

bool Track::IsDeleted()
{
    return this->state_ == TrackState::Deleted;
}

bool Track::IsTentatived()
{
    return this->state_ == TrackState::Tentative;
}

void Track::DeleteTrack()
{
    this->state_ = TrackState::Deleted;
}

Bbox Track::ToTlwh()
{
    Bbox ret = mean_.leftCols(4);
#if USE_XYAH
    ret(2) *= ret(3);
    ret.leftCols(2) -= (ret.rightCols(2)/2);
#else
    ret.leftCols(2) -= (ret.rightCols(2)/2);  //xmin = cx - w/2
#endif 
    return ret;
}

Bbox Track::ToCxymax()
{
    Bbox ret = mean_.leftCols(4);
#if USE_XYAH
    // cx cy w/h h
    ret(2) *= ret(3);  // w = w/h * h
    ret(1) += ret(3)/2;  // ymax = cy + h/2
#else
    // cx cy w h 
    ret(1) = ret(1) + ret(3)/2;  // ymax = cy + h/2
    // ret(1) += ret(3)/2;  
#endif
    return ret;
}

void Track::InitTrackNode(u_int64_t timestamp, TrackNode& obj)
{
    obj.id = this->track_id_;
    obj.confidence = this->latest_detection_.confidence;
    obj.timestamp = timestamp;

    // obj.filter_var = this->angle_;

#if defined(WITHOUT_CLASS_TYPE_LOCKED)
    obj.type = this->latest_detection_.type;
    is_locked_class_ = true;
#else
    if(!is_locked_class_ && age_ > CLASS_TYPE_LOCK_MIN_COUNT) //目标类型没锁死且目标存在CLASS_TYPE_LOCK_MIN_COUNT帧后，进行目标锁死
    {
        //选取置信度大的类别or选取次数最多???
        float max_conf = 0;
        int max_count = 0;
        int max_conf_cls = -1;
        int max_count_cls = -1;
        for(auto it = classes_map_.begin(); it!=classes_map_.end(); it++)
        {
            if(it->second.second > max_count) //出现次数最多的类别
            {
                max_count = it->second.second;
                max_count_cls = it->first;
            }

            if(it->second.first > max_conf) //置信度最大的类别
            {
                max_conf = it->second.first;
                max_conf_cls = it->first;
            }
        }
        if(max_conf_cls == max_count_cls) //如果相当，直接选择
        {
            obj.type = max_count_cls;
        }
        else //不等选择出现次数多的类别
        {
            obj.type = max_count_cls;
            // obj.type = max_conf_cls;
        }

        // //选取次数大于锁死阈值的，直接锁死
        // if(max_count >= CLASS_TYPE_LOCK_MIN_COUNT)
        // {
        //     locked_class_type = max_count_cls;
        //     is_locked_class_ = true;
        // } 
        locked_class_type_ = max_count_cls;
        is_locked_class_ = true;

    }
    else
    {
        obj.type = locked_class_type_;
    }
#endif

    // obj.cywh = ToCxymax();
    // this->lastest_bbox_tlwh = ToTlwh();

    // kalman预测到的坐标点, 下边中心点
    Bbox pb = ToCxymax();
    // 最近检测出来的目标框
    Bbox db = this->latest_detection_.ToCxymax();
    if(this->time_since_update_ >= TIME_SINCE_UPDATE)
    {
        obj.cywh = pb;  // 未被更新, 相信预测结果
        this->lastest_bbox_tlwh = ToTlwh();
    }
    else
    {
        obj.cywh = db;  // 相信检测结果
        this->lastest_bbox_tlwh = this->latest_detection_.tlwh;
    }

#if 0
    this->track_list_.push_back(obj);
    if(this->track_list_.size() > MAX_TRACK_LIST_SIZE)
    {
        this->track_list_.erase(this->track_list_.begin());
        this->track_list_.swap(this->track_list_);
    }
#endif
}

}
