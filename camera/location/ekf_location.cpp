#include "ekf_location.h"
#include "../common/utils.h"
#include <math.h>
#include <fstream>

namespace camera {

#define SAVE_LOG_TXT 0
#define COUT_LOG 0
#define PRINT_LOG SAVE_LOG_TXT||COUT_LOG

//cx ymax w h --> xmin,ymin,xmax,ymax
static float CalculateIOU(Bbox lbox, Bbox rbox) 
{
    float interBox[] = {
        std::max(lbox[0]-lbox[2]/2, rbox[0]-rbox[2]/2), //xmin = cx - w/2
        std::min(lbox[0]+lbox[2]/2, rbox[0]+rbox[2]/2),   //xmax = cx + w/2
        std::max(lbox[1]-lbox[3], rbox[1]-rbox[3]), //ymin  = ymax - h
        std::min(lbox[1], rbox[1])  //ymax = ymax
    };

    float lboxArea = lbox[2] * lbox[3];
    float rboxArea = rbox[2] * rbox[3];
    float interW = std::max(0.f, (interBox[1]-interBox[0]));
    float interH = std::max(0.f, (interBox[3]-interBox[2]));
    float interBoxS = interW * interH;

    float iou = interBoxS/(lboxArea + rboxArea - interBoxS);

    float xmin = std::min(lbox[0]-lbox[2]/2, rbox[0]-rbox[2]/2);
    float ymin = std::min(lbox[1]-lbox[3], rbox[1]-rbox[3]);
    float xmax = std::max(lbox[0]+lbox[2]/2, rbox[0]+rbox[2]/2);
    float ymax = std::max(lbox[1], rbox[1]);
    
    float lcx = lbox[0];
    float rcx = rbox[0];
    float lcy = lbox[1] - lbox[3]/2;
    float rcy = rbox[1] - rbox[3]/2;

    float c_distance = std::pow(xmax - xmin, 2) + std::pow(ymax - ymin, 2);
    float d_distance = std::pow(lcx - rcx, 2) + std::pow(lcy - rcy, 2);

    float diou = iou - d_distance/c_distance;

    return diou;
}

EKFLocation::EKFLocation(const LocationConfig& config, const TrackNode& obj, u_int64_t timestamp):
    last_timestamp_(timestamp),
    last_obj_timestamp_(timestamp),
    is_updated_(false),
    lost_times_(0),
    last_speed_(0),
    age_(0),
    update_times_(0),
    filter_heading_confirmed_(true)
{
    state_ax_ = config.std_state_ax;
    state_ay_ = config.std_state_ay;
    std_meas_x_ = config.std_meas_x;
    std_meas_y_ = config.std_meas_y;
    
    if(obj.link_heading > 0) //初始航向角为link的航向角
    {
        heading_init_ = obj.link_heading;
    }
    else //初始航向角为ROI的航向角
    {
        heading_init_ = config.heading;


        //道路配置了方向
        if(config.headingcfg.size() > 0)
        {
            //底边中心点
            cv::Point pt;
            pt.x = obj.cywh[0];
            pt.y = obj.cywh[1];
            
            for (auto &hcfg : config.headingcfg)
            {
                if(IsPointInPolygon(pt, hcfg.roi))
                {
                    heading_init_ = hcfg.heading;
                    // printf("!!!!id=%d, heading=%f\n", obj.id, hcfg.heading);
                    break;
                }
            }
        }

#if 0
        //配置了方向
        if(config.dirving_direction >= 0)
        {
            float right_heading, left_heading, up_heading, down_heading;
            if(0 == config.dirving_direction) //来向
            {
                right_heading = 90 + config.heading;
                if(right_heading < 0.0f)       right_heading += 360.f;
                if(right_heading >= 360.f)     right_heading -= 360.f;

                left_heading = config.heading - 90;
                if(left_heading < 0.0f)       left_heading += 360.f;
                if(left_heading >= 360.f)     left_heading -= 360.f;

                up_heading = config.heading;

                down_heading = 180 - config.heading;
                if(down_heading < 0.0f)       down_heading += 360.f;
                if(down_heading >= 360.f)     down_heading -= 360.f;
            }
            else //去向
            {
                right_heading = config.heading - 90;
                if(right_heading < 0.0f)       right_heading += 360.f;
                if(right_heading >= 360.f)     right_heading -= 360.f;

                left_heading = 90 + config.heading;
                if(left_heading < 0.0f)       left_heading += 360.f;
                if(left_heading >= 360.f)     left_heading -= 360.f;

                up_heading = 180 - config.heading;
                if(up_heading < 0.0f)       up_heading += 360.f;
                if(up_heading >= 360.f)     up_heading -= 360.f;

                down_heading = config.heading;
            }
            // printf("***************up: %f down: %f left: %f right: %f\n", up_heading, down_heading, left_heading, right_heading);
            cv::Point pt;
            pt.x = obj.cywh[0];
            pt.y = obj.cywh[1] - std::min(10, (int)obj.cywh[3]/2);

            bool flag = false;
            if(config.roi_right.size()>0 && IsPointInPolygon(pt, config.roi_right))
            {
                heading_init_ = right_heading;
                flag = true;
            }
            if(config.roi_left.size()>0 && IsPointInPolygon(pt, config.roi_left))
            {
                heading_init_ = left_heading;
                flag = true;
            }
            if(config.roi_up.size()>0 && IsPointInPolygon(pt, config.roi_up))
            {
                heading_init_ = up_heading;
                flag = true;
            }
            if(config.roi_down.size()>0 && IsPointInPolygon(pt, config.roi_down))
            {
                heading_init_ = down_heading;
                flag = true;
            }

            if(!flag)
                heading_init_ = config.heading;
        }
        else
        {
            heading_init_ = config.heading;
        }
 
#endif 
    }


    last_heading_ = heading_init_;

    // ekf_ = std::unique_ptr<N_KalmanFilter> {new N_KalmanFilter()};
    ekf_ = new N_KalmanFilter();
    ekf_->x_ = Eigen::VectorXd(4);
    ekf_->x_ <<  obj.x, obj.y, 0, 0; 

    last_posx_ = obj.x;
    last_posy_ = obj.y;

#if defined(USE_UV_FILTER)
    // uv_ekf_ = std::unique_ptr<N_KalmanFilter> {new N_KalmanFilter()};
    uv_ekf_ = new N_KalmanFilter();
    uv_ekf_->x_ = Eigen::VectorXd(4);
    uv_ekf_->x_ <<  obj.cywh[0], obj.cywh[1], 0, 0;  
#endif
}

EKFLocation::~EKFLocation()
{
    if(ekf_)
    {
        delete ekf_;
        ekf_ = nullptr;
    }
    if(uv_ekf_)
    {
        delete uv_ekf_;
        uv_ekf_ = nullptr;
    }
}

void EKFLocation::Predict(u_int64_t timestamp)
{
    float dt;
    dt = (timestamp - this->last_timestamp_)/1000.f;    //ms to s
    
    // if(dt < MIN_INTERVAL_TIME || dt > MAX_INTERVAL_TIME)
    //     dt = 0.04; //

    // printf("!!!!dt= %f\n", dt);

    ekf_->SetFMatrix(dt);
    ekf_->SetProcessCovarianceNoiseSquare(dt, state_ax_, state_ay_);
    ekf_->Predict();

#if defined(USE_UV_FILTER)
    uv_ekf_->SetFMatrix(dt);
    uv_ekf_->SetProcessCovarianceNoiseSquare(dt, state_ax_, state_ay_);
    uv_ekf_->Predict();
#endif

    this->is_updated_ = false;
    this->last_timestamp_ = timestamp;
    this->age_++;
}

void EKFLocation::Update(double x, double y)
{
    ekf_->SetMeasurementNoiseSquare(std_meas_x_, std_meas_y_);

    Eigen::VectorXd m(2);
    m << x, y;
    ekf_->Update(m);

    //被更新后，丢失次数置0
    this->lost_times_ = 0;

    // this->update_times_++;
}

void EKFLocation::Update(double x, double y, float u, float v)
{
    ekf_->SetMeasurementNoiseSquare(std_meas_x_, std_meas_y_);

    Eigen::VectorXd m(2);
    m << x, y;
    ekf_->Update(m);

    uv_ekf_->SetMeasurementNoiseSquare(std_meas_x_, std_meas_y_);
    Eigen::VectorXd m1(2);
    m1 << u, v;
    uv_ekf_->Update(m1);

    //被更新后，丢失次数置0
    this->lost_times_ = 0;
}

void EKFLocation::GetFilterUV(double &u, double &v)
{
    u = uv_ekf_->x_(0);
    v = uv_ekf_->x_(1);

    if(u <= 0 || v <= 0)
    {
        // printf("!!!!!!!!!!u=%f v=%f\n", u, v);
        // exit(-1);

        this->lost_times_ = MAX_LOST_FRAME + 1;
    }
}

//单个目标
void EKFLocation::CalculateSpeedHeading(TrackNode& obj, float lock_velo, float velo_converged_thresh)
{
    float heading;
    bool velo_confirmed = false, h_confirmed = false, heading_confirmed = true;
    float posx = ekf_->x_(0);
    float posy = ekf_->x_(1);
    float velx = ekf_->x_(2);
    float vely = ekf_->x_(3);

    float px_uncertainty = ekf_->P_(0,0);
    float py_uncertainty = ekf_->P_(1,1);
    float vx_uncertainty = ekf_->P_(2,2);
    float vy_uncertainty = ekf_->P_(3,3);

    std::cout << cv::format("!!!!!!!!!!!%d-->%f, %f, %f, %f, %f, %f, %f, %f, ", obj.id, obj.x, obj.y, posx, posy, velx, vely, vx_uncertainty, vy_uncertainty);

    double velo = sqrt(pow(velx,2) + pow(vely,2));
    // double velo_var = (px_uncertainty + py_uncertainty + vx_uncertainty + vy_uncertainty)/4;
    double velo_var = (vx_uncertainty + vy_uncertainty) / 2;


    //滤波角向测量角转换, 正北方向的夹角, 弧度转角度
    double filter_heading = atan2(velx, vely)*180/M_PI;
    // std::cout << cv::format("%f, %f, %f, ", velo, velo_var, filter_heading);

    while(filter_heading < 0.0f)       filter_heading += 360.f;
    while(filter_heading >= 360.f)     filter_heading -= 360.f;


    // 判断滤波速度收敛, 方差小于阈值, 认为可信
    if(velo_var < velo_converged_thresh)
    {
        velo_confirmed = true;
    }
    
    // 滤波的航向角可信状态下，上一帧与当前滤波的航向角相差大于MAX_HEADING_DIFF_THRESH，则不可信
    // if(filter_heading_confirmed_)
    // {
    //     float h_diff = std::abs(this->last_heading_ - filter_heading);
    //     h_diff = h_diff > 180 ? 360-h_diff : h_diff;
    //     if(h_diff > MAX_HEADING_DIFF_THRESH)
    //         heading_confirmed = false;
    // }


    // 目标框的高小于一定阈值, 认为该目标不可信
    if(obj.cywh[3] >= MIN_H_THRESH)
    {
        h_confirmed = true;
    }

#if 0
    //轨迹满， 计算位置方差
    double avg_velo = 0;
    double avg_x = 0, avg_y=0, x_var, y_var;
    if(trajectory_.size() > MAX_TRACE_NUM-2)
    {
        int size = trajectory_.size();
        // printf("[%s:%d]-->size: %d\n", __FUNC__, __LINE__, size);
        for(int i=0; i<size; i++)
        {
            // avg_velo += trajectory_[i].speed;
            avg_x += trajectory_[i].cywh[0];
            avg_y += trajectory_[i].cywh[1];
        }
        // avg_velo /= size;

        //平均位置
        avg_x /= size;
        avg_y /= size;
        double sum_x = 0, sum_y = 0;
        for(int i=0; i<size; i++)
        {
            sum_x += std::pow(trajectory_[i].cywh[0]-avg_x, 2);
            sum_y += std::pow(trajectory_[i].cywh[1]-avg_y, 2);
        }
        x_var = sum_x / size;
        y_var = sum_y / size;
        double pos_var = (x_var + y_var)/2;
        obj.filter_var = pos_var;
        if(pos_var < 2)
        {
            obj.state = 0;
        }
        else 
            obj.state = 1;
    }
#endif

    obj.confirmed = velo_confirmed;

    bool confirmed = velo_confirmed && h_confirmed && heading_confirmed;

    // printf("!!!!!!!!! filter_heading = %d - %d - %d - %d - %d - ", obj.id, velo_confirmed, h_confirmed, heading_confirmed, this->update_times_);
    std::cout << cv::format("%d, %d, %d, ", velo_confirmed, h_confirmed, heading_confirmed);
    //速度低于阈值，锁死航向角
    if(confirmed && velo > lock_velo)
    // if(confirmed && avg_velo > lock_velo)
    {
        heading = filter_heading;
    }
    else
    {
        // heading = this->last_heading_;
        if(1 < velo)
            heading = this->last_heading_;
        else //静止目标等于配置文件的航向角
        {
            // heading = this->heading_init_;
            if(obj.link_heading > 0)
            {
                heading = obj.link_heading; //link的heading
            }
            else
                heading = this->heading_init_;
        }
    }

    // printf("%f - %f - %f - %f - %f\n", heading, this->last_heading_, filter_heading, avg_velo, velo);
    std::cout << cv::format("%f, %f, %f\n",  heading, this->last_heading_, filter_heading);

    // 计算加速度
    u_int64_t dt = obj.timestamp - this->last_obj_timestamp_;
    if(obj.confirmed && dt > 0)
    {
        float dt1 = dt/1000.f;  //ms to s
        obj.acceleration = (velo - this->last_speed_) / dt1;
    }
    else
    {
        obj.acceleration = 0;
    }

    obj.heading = heading;
    obj.speed = velo;
    // 滤波后的x,y
    obj.x = posx;
    obj.y = posy;

    // printf("cx=%f ymax=%f\n", posx, posy);

    this->last_heading_ = heading;
    this->last_speed_ = velo;

    this->last_obj_timestamp_ = obj.timestamp;

    // trajectory_.push_back(obj);
    // if(trajectory_.size() > MAX_TRACE_NUM)
    // {
    //     trajectory_.erase(trajectory_.begin());
    //     trajectory_.swap(trajectory_);
    // }
}

//考虑临近目标
void EKFLocation::CalculateSpeedHeading(TrackList &tracks, int idx, float lock_velo, float velo_converged_thresh)
{
#if SAVE_LOG_TXT
    std::string file_name =  + "./Logs/" + std::to_string(tracks[idx].id) + ".txt";
    std::ofstream fout(file_name, std::ios::out | std::ios::app);
    std::streambuf *oldcout = std::cout.rdbuf(fout.rdbuf());
#endif

    float heading;
    bool velo_confirmed = false, h_confirmed = false, heading_confirmed = true;
    float posx = ekf_->x_(0);
    float posy = ekf_->x_(1);
    float velx = ekf_->x_(2);
    float vely = ekf_->x_(3);

    float px_uncertainty = ekf_->P_(0,0);
    float py_uncertainty = ekf_->P_(1,1);
    float vx_uncertainty = ekf_->P_(2,2);
    float vy_uncertainty = ekf_->P_(3,3);

    double velo = sqrt(pow(velx,2) + pow(vely,2));
    // double velo_var = (px_uncertainty + py_uncertainty + vx_uncertainty + vy_uncertainty)/4;
    double velo_var = (vx_uncertainty + vy_uncertainty) / 2;

    //滤波角向测量角转换, 正北方向的夹角, 弧度转角度
    double filter_heading = atan2(velx, vely)*180/M_PI;

    while(filter_heading < 0.0f)       filter_heading += 360.f;
    while(filter_heading >= 360.f)     filter_heading -= 360.f;


    // bool filter_heading_confirmed = true;

    // 判断滤波速度收敛, 方差小于阈值, 认为可信
    if(velo_var < velo_converged_thresh)
    {
        velo_confirmed = true;

#if defined(USE_FILTER_HEADING_CONFIRMED)
        //判断滤波的航向角是否可信： 滤波的航向角与道路航向角之差小于阈值
        float heading_init;
        if(tracks[idx].link_heading > 0) //使用link的航向角
            heading_init = tracks[idx].link_heading;
        else //使用初始化的航向角
            heading_init = heading_init_;
        
        float h_diff1 = std::abs(heading_init - filter_heading);
        h_diff1 = h_diff1 > 180 ? 360-h_diff1 : h_diff1;  //范围[0, 180]
        
        //逆行，与初始初始航向角相差180(+30,-30)
        if(h_diff1 >= 150) 
        {
            // filter_heading_confirmed_ = false;
            //如果航向角异常，查找与其相近的目标航向角
            TrackNode t = tracks[idx];
            float max_iou = -0.2;
            int max_iou_idx = -1;
            for(int i=0; i<tracks.size(); i++)
            {
                // if(i != idx && tracks[i].heading > 0 && tracks[i].type<3)//仅看车辆类型 car truck bus
                if(i != idx && tracks[i].type<3)//仅看车辆类型 car truck bus
                {
                    float iou = CalculateIOU(t.cywh, tracks[i].cywh);
                    if(iou > max_iou)
                    {
                        max_iou = iou;
                        max_iou_idx = i;
                    }
                }
            }
#if PRINT_LOG
            std::cout << cv::format("*******nearest:%d %f ", t.id, filter_heading);
#endif
            if(max_iou_idx > 0) //判断航向角是否一样
            {
                // printf("*******nearest:%d %d %f %f\n", t.id, tracks[max_iou_idx].id, max_iou, tracks[max_iou_idx].heading);
                float h_diff = std::abs(tracks[max_iou_idx].heading - filter_heading);
                h_diff = h_diff > 180 ? 360-h_diff : h_diff;  //范围[0, 180]
#if PRINT_LOG
                std::cout << cv::format("%d %f %f %f", tracks[max_iou_idx].id, max_iou, tracks[max_iou_idx].heading, h_diff);
#endif
                if(h_diff > 30) //最近邻与滤波的航向角类似，则认为滤波的航向角可信，否则不可信
                {
                    filter_heading_confirmed_ = false;
                }
            }
#if PRINT_LOG            
            std::cout << "\n";
#endif       
        }
        else
            filter_heading_confirmed_ = true;
        
        // if(!filter_heading_confirmed_ && h_diff1 <= 30)
        // {
        //     filter_heading_confirmed_ = true;
        // }
#endif
    }
    
    // 滤波的航向角可信状态下，上一帧与当前滤波的航向角相差大于MAX_HEADING_DIFF_THRESH，则不可信
    // if(filter_heading_confirmed_)
    // {
    //     float h_diff = std::abs(this->last_heading_ - filter_heading);
    //     h_diff = h_diff > 180 ? 360-h_diff : h_diff;
    //     if(h_diff > MAX_HEADING_DIFF_THRESH)
    //         heading_confirmed = false;
    // }


    // 目标框的高小于一定阈值, 认为该目标不可信
    if(tracks[idx].cywh[3] >= MIN_H_THRESH)
    {
        h_confirmed = true;
    }

#if 0
    //轨迹满， 计算位置方差
    double avg_velo = 0;
    double avg_x = 0, avg_y=0, x_var, y_var;
    if(trajectory_.size() > MAX_TRACE_NUM-2)
    {
        int size = trajectory_.size();
        // printf("[%s:%d]-->size: %d\n", __FUNC__, __LINE__, size);
        for(int i=0; i<size; i++)
        {
            // avg_velo += trajectory_[i].speed;
            avg_x += trajectory_[i].cywh[0];
            avg_y += trajectory_[i].cywh[1];
        }
        // avg_velo /= size;

        //平均位置
        avg_x /= size;
        avg_y /= size;
        double sum_x = 0, sum_y = 0;
        for(int i=0; i<size; i++)
        {
            sum_x += std::pow(trajectory_[i].cywh[0]-avg_x, 2);
            sum_y += std::pow(trajectory_[i].cywh[1]-avg_y, 2);
        }
        x_var = sum_x / size;
        y_var = sum_y / size;
        double pos_var = (x_var + y_var)/2;
        obj.filter_var = pos_var;
        if(pos_var < 2)
        {
            obj.state = 0;
        }
        else 
            obj.state = 1;
    }
#endif

    tracks[idx].confirmed = velo_confirmed;

    bool confirmed = velo_confirmed && h_confirmed && heading_confirmed && filter_heading_confirmed_;

    //速度低于阈值，锁死航向角
    if(confirmed && velo > lock_velo)
    // if(confirmed && avg_velo > lock_velo)
    {
        heading = filter_heading;
        update_times_++;
    }
    else
    {
        // heading = this->last_heading_;
        if(1 < velo)
            heading = this->last_heading_;
        else //静止目标等于配置文件的航向角
        {
            // heading = this->heading_init_;
            if(tracks[idx].link_heading > 0)
            {
                heading = tracks[idx].link_heading; //link的heading
            }
            else
                heading = this->heading_init_;
        }
    }

#if PRINT_LOG
    std::cout << cv::format("%d-->%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,", 
        tracks[idx].id, tracks[idx].x, tracks[idx].y, posx, posy, 
        velx, vely, velo, px_uncertainty, py_uncertainty, vx_uncertainty, vy_uncertainty);
    std::cout << cv::format("%d, %d, %d, %d, %d, ", velo_confirmed, h_confirmed, velo > lock_velo, filter_heading_confirmed_, update_times_);
    std::cout << cv::format("%f, %f, %f\n",  heading, this->last_heading_, filter_heading);
#endif

    // 计算加速度
    u_int64_t dt = tracks[idx].timestamp - this->last_obj_timestamp_;
    if(tracks[idx].confirmed && dt > 0)
    {
        float dt1 = dt/1000.f;  //ms to s
        tracks[idx].acceleration = (velo - this->last_speed_) / dt1;
    }
    else
    {
        tracks[idx].acceleration = 0;
    }

    tracks[idx].heading = heading;
    tracks[idx].speed = velo;
    // if(filter_heading_confirmed_)
    if(true)
    {
        // 滤波后的x,y
        tracks[idx].x = posx;
        tracks[idx].y = posy;
        last_posx_ = posx;
        last_posy_ = posy;
    }
    else
    {
        tracks[idx].x = last_posx_;
        tracks[idx].y = last_posy_;
    }

    this->last_heading_ = heading;
    this->last_speed_ = velo;

    this->last_obj_timestamp_ = tracks[idx].timestamp;

#if SAVE_LOG_TXT
    std::cout.rdbuf(oldcout);
    fout.close();
#endif
}


} //namespace camera