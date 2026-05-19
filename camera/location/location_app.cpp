#include "location_app.h"
#include "camera_calibrator.h"
#include "../common/utils.h"

#include <math.h>

namespace camera {

LocationApp::~LocationApp()
{
    Release();
}

bool LocationApp::Init(const LocationConfig& config)
{
    this->origin_latitude_ = config.located_latitude;
    this->origin_longitude_ = config.located_longitude;
    this->origin_offset_angle_ = config.located_offset_angle;

    // 计算映射矩阵
    CameraCalibrator camcal;
    bool is_file_exist = FindFile(config.calibrator_file, true);
    if(!is_file_exist)
        return false;

    camcal.LoadJsonFile(config.calibrator_file.c_str());
    camcal.CalculateHomoMatrix();
    cv::Mat h_inv = camcal.GetHomoMatrixInv();
    // cv::Mat h = camcal.GetHomoMatrix();
    // double hh[9];

    for (int k=0; k< h_inv.rows; k++)
    {
	    for(int j=0; j<h_inv.cols; j++)
        {
            this->h_inv_[k*h_inv.rows + j] = h_inv.at<double>(k,j);
        }
    }
    printf("homography inv matrix:\n");
    for (int k=0; k<9; k++)
    {
      printf("%.10f; ", this->h_inv_[k]);
    }
    printf("\n");

    // printf("homography matrix:\n");
    // for (int k=0; k<9; k++)
    // {
    //   printf("%.10f; ", hh[k]);
    // }
    // printf("\n");

    return true;
}

void LocationApp::Release()
{
    for(auto it=objects_map_.begin();it!=objects_map_.end();)
    {
        if(it->second)
        {
            delete it->second;
            it->second = nullptr;
        }
        it = objects_map_.erase(it);
    }
    objects_map_.clear();
}

void LocationApp::Process(const LocationConfig& config, u_int64_t timestamp, TrackList &tracks)
{
    bool ret;
    // predict
    for(auto& it : objects_map_)
    {
        it.second->Predict(timestamp);
    }

    // // 为tracks赋值上一帧的航向角
    // for(size_t i = 0; i < tracks.size(); i++)
    // {
    //     auto it = objects_map_.find(tracks[i].id);
    //     if(it != objects_map_.end())
    //         tracks[i].last_heading = it->second->GetHeading();
    // }

    // update && create location instance
    for(size_t i = 0; i < tracks.size(); i++)
    {
        // if(!tracks[i].in_roi)
        //     continue;

        // 查找id
        auto it = objects_map_.find(tracks[i].id);
        // 找到更新滤波器
        if(it != objects_map_.end())
        {

#if defined(USE_UV_FILTER)
            //获得滤波后的uv
            double u=0, v=0;
            it->second->GetFilterUV(u,v);
            
            ret = CalculateGPS(u, v, tracks[i]);
#else
            ret = CalculateGPS(traks[i]);
#endif
            if(!ret)
                it->second->SetLostTimes(MAX_LOST_FRAME);

            TransformGPSToXY(tracks[i]);

#if defined(USE_UV_FILTER)
            it->second->Update(tracks[i].x, tracks[i].y, tracks[i].cywh[0], tracks[i].cywh[1]);
#else
            it->second->Update(tracks[i].x, tracks[i].y);
#endif
            it->second->MarkUpdated();

            //计算速度，航向角等
            // it->second->CalculateSpeedHeading(tracks[i], config.lock_velo, config.velo_converged_var);
            it->second->CalculateSpeedHeading(tracks, i, config.lock_velo, config.velo_converged_var);

            //计算滤波后的经纬度
            TransformXYToGPS(tracks[i]);

            // tracks[i].heading = 13;
        }
        else //未找到，创建对象
        {
            // 初始对象的经纬度
            ret = CalculateGPS(tracks[i]);
            if(!ret)
                it->second->SetLostTimes(MAX_LOST_FRAME);

            // 转xy
            TransformGPSToXY(tracks[i]);
            // std::shared_ptr<EKFLocation> ekf_ptr(new EKFLocation(config, tracks[i], timestamp));
            // objects_map_.insert(std::pair<int, std::shared_ptr<EKFLocation>>(tracks[i].id, ekf_ptr));

            objects_map_[tracks[i].id] = new EKFLocation(config, tracks[i], timestamp);

            // tracks[i].heading = 13;
        }
    }
    
    // 删除丢失大于阈值的目标
    // std::map<int, std::shared_ptr<EKFLocation>>::iterator it;
    std::map<int, EKFLocation*>::iterator it;
    // printf("objects_map_.size()=%d \n", objects_map_.size());
    for(it = objects_map_.begin(); it!=objects_map_.end();)
    {
        it->second->SetLastTimestamp(timestamp);

        bool updated = it->second->GetUpdated();
        if(!updated)
        {
            it->second->AddLostTimes();
        }

        int lost_times = it->second->GetLostTimes();
        if(lost_times >= MAX_LOST_FRAME)
        {
            // printf("erase id: %d\n", it->first);
            if(it->second)
            {
                delete it->second;
                it->second = nullptr;
            }
            it = objects_map_.erase(it);
        }
        else
            it++;
    }
    // printf("after objects_map_.size()=%d \n", objects_map_.size());
}

bool LocationApp::CalculateGPS(TrackNode& obj)
{
    Bbox box = obj.cywh;
    double x0 = h_inv_[0]*box[0] + h_inv_[1]*box[1] + h_inv_[2];
    double y0 = h_inv_[3]*box[0] + h_inv_[4]*box[1] + h_inv_[5];
    double z0 = h_inv_[6]*box[0] + h_inv_[7]*box[1] + h_inv_[8];
    obj.longitude = x0 / z0;
    obj.latitude = y0 / z0;

    if(obj.longitude > 180 || obj.longitude < 0)
    {
        printf("bbox: %f %f %f %f\n", box[0], box[1], box[2], box[3]);
        printf("longtitude is out of range...\n");
        return false;
    }

    if(obj.latitude < 0 || obj.latitude > 90)
    {
        printf("bbox: %f %f %f %f\n", box[0], box[1], box[2], box[3]);
        printf("latitude is out of range...\n");
        return false;
    }

    return true;
}

bool LocationApp::CalculateGPS(double x, double y, TrackNode& obj)
{
    double x0 = h_inv_[0]*x + h_inv_[1]*y + h_inv_[2];
    double y0 = h_inv_[3]*x + h_inv_[4]*y + h_inv_[5];
    double z0 = h_inv_[6]*x + h_inv_[7]*y + h_inv_[8];
    double lon = x0 / z0;
    double lat = y0 / z0;

    if(lon > 180 || lon < 0)
    {
        printf("x y: %f %f\n", x, y);
        printf("longtitude is out of range...\n");
        return false;
    }

    if(lat < 0 || lat > 90)
    {
        printf("x y: %f %f\n\n", x, y);
        printf("latitude is out of range...\n");
        return false;
    }
    
    obj.longitude = lon;
    obj.latitude = lat;
    return true;
}

void LocationApp::TransformGPSToXY(TrackNode& obj)
{
    double lon = obj.longitude;
    double lat = obj.latitude;

	double del_lat = lat - origin_latitude_;
	float del_NS = del_lat * 3600 * DISTANCEPSEC;
	double del_lon = lon - origin_longitude_;
	float del_WE = del_lon * 3600 * DISTANCEPSEC * cos(lat * (M_PI / 180.0));

    obj.x = del_WE;
    obj.y = del_NS;
}

void LocationApp::TransformXYToGPS(TrackNode& obj)
{
    double x = obj.x;
    double y = obj.y;

    // 计算目标与正北夹角
	double beta = atan2(x, y);
	if (beta < 0.0)
	{
		beta = PI2 + beta;
	}
	if (beta > (PI2))
	{
		beta -= PI2;
	}

	double radius = sqrt(x * x + y * y);

	double object_latitude = radius * cos(beta) / DISTANCEPSEC;
	obj.latitude = object_latitude / 3600 + origin_latitude_;

	double object_longitude = radius * sin(beta) / (DISTANCEPSEC * cos((obj.latitude) * M_PI / 180));
	obj.longitude = object_longitude / 3600 + origin_longitude_;
}


} //namespace camera