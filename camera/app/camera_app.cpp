/*
* camera_app.cpp
* Created on: 20210622
* Author: yueyingying
* Description: 视频应用程序对外接口函数实现
*
*/
#include "camera_app.h"
#include "../detector/class_detector.h"
#include "../location/camera_calibrator.h"
#include "../common/utils.h"
#include "version.h"
#include "../tracker/ByteTrack/byteTrack.h"
#include "../location/location_app.h"
#include <vector>

namespace camera 
{

static unsigned int frame_count = 0;

CameraAPP::~CameraAPP()
{
    if(detector_)
    {
        delete detector_;
        detector_ = nullptr;
    }

    if(tracker_)
    {
        delete tracker_;
        tracker_ = nullptr;
    }
    if(location_)
    {
        delete location_;
        location_ = nullptr;
    }
    
}

void CameraAPP::PrintVersion()
{
    std::cout << "********************************" << std::endl;
    std::cout << "***" << CAMERA_APP_NAME << "***" << std::endl;
    std::cout << "*****" << CAMERA_APP_VERSION << "*****" << std::endl;

    std::cout << "use nebulalink.perceptron3.0.5.proto\n";
    std::cout << "parse configure file by libconfig\n";

    std::cout << "*******************************" << std::endl;
}

bool CameraAPP::Init(std::string config_file, int width, int height, std::string camera_uri,float localtion_angle,double location_longitude,double location_latitude)
{
    PrintVersion();

    std::cout <<"config file : " << config_file << std::endl;
    
    // 解析配置文件
    bool ret = ParseCameraConfigFile(&config_, config_file.c_str());
    if(!ret)
    {
        std::cout << "parse config file error\n";
        return false;
    }

    if (localtion_angle > 0)
        config_.loc_config.heading = localtion_angle;
    if (location_longitude > 0)
        config_.loc_config.located_longitude = location_longitude;
    if (location_latitude > 0)
        config_.loc_config.located_latitude = location_latitude;

    PrintCameraConfigMessage(&config_);

    // 初始化跟踪对象
    std::cout << "****track byte ****\n";
    // tracker_ = new ByteTracker(config_.tracker_config.low_thresh, config_.tracker_config.high_thresh, config_.tracker_config.max_iou_thresh);
    tracker_ = new ByteTracker(config_.tracker_config.low_thresh, config_.tracker_config.high_thresh, 
            config_.tracker_config.max_iou_thresh, config_.tracker_config.high_thresh_person, 
            config_.tracker_config.high_thresh_motorbike);

    // 初始化检测对象
    detector_ = new Detector();
    detector_->Init(&config_, width, height);

    // 初始化定位模块
    if(config_.loc_config.enable)
    {
        location_ = new LocationApp();
        location_->Init(config_.loc_config);

        // //判断min max y
        // if(config_.loc_config.min_y > 0) //配置了中间区域的范围
        // {
        //     config_.loc_config.max_y = std::min(height-30, config_.loc_config.max_y);
        // }
    }


    printf("lpr is disabled.\n");

    // // 中文字体
    // bool is_file_exist = FindFile(config_.nl_config.font_file, true);
    // if(!is_file_exist)
    //     return false;
    // zh_text.loadFont(config_.nl_config.font_file.c_str()); //指定字体
    // cv::Scalar size1{ 40, 0.5, 0.1, 0 }; // (字体大小, 无效的, 字符间距, 无效的 }
    // zh_text.setFont(nullptr, &size1, nullptr, 0);

    printf("\nsuccess init camera application.......\n");

    return true;
}

static void CovertObjectsV3(TrackList tracks, uint64_t timestamp, AppConfig* config, int width, int height, perceptron::s_ObjectList* objects)
{
    objects->devide_id = config->nl_config.camera_sensor_id;
    objects->time_stamp = timestamp;
    objects->number_frame = frame_count;
    objects->devide_is_true = true;
    objects->sensor_type = perceptron::e_SensorType::SensorType_CAMERA;
    objects->perception_gps.object_longitude = config->loc_config.located_longitude;
    objects->perception_gps.object_latitude = config->loc_config.located_latitude;
    objects->perception_gps.object_elevation = config->loc_config.located_elevation;

    // init lane & link for current frame
    for(uint i=0; i<config->nl_config.links.size(); i++)
    {
        config->nl_config.links[i].vehicle_count = 0;
        config->nl_config.links[i].vehicle_speed = 0;
        for(int j=0; j<config->nl_config.links[i].lane_count; j++)
        {
            config->nl_config.links[i].lanes[j].vehicle_count = 0;
            config->nl_config.links[i].lanes[j].vehicle_speed = 0;
        }
    }

    // int not_in_roi_count=0, not_confirmed=0;
    for(uint i=0; i < tracks.size(); i++)
    {  

        //the object' bbox     
        int cx = (int)tracks[i].cywh[0];
        int ymax = (int)tracks[i].cywh[1];
        int w = (int)tracks[i].cywh[2];
        int h = (int)tracks[i].cywh[3];

        // cx = ClampValue(cx, 1, width-1);
        // ymax = ClampValue(ymax, 1 ,height-1);
        // w = ClampValue(w, 1, width-1);
        // h = ClampValue(h, 1, height-1);
#if 0
        // select obj in the roi
        bool is_in_roi = false;
        for(int k=0;k<config->nl_config.roi_points.size();k++)
        {
            if(IsPointInPolygon(cv::Point(cx, ymax), config->nl_config.roi_points[k]))
            {
                is_in_roi = true;
                break;
            }
        }

        if(!is_in_roi) 
        {
            not_in_roi_count++;
            continue; 
        } 
#else
        if(!tracks[i].in_roi)
            continue;
#endif

        // 定位滤波不收敛的目标不发送
        // if(config->loc_config.enable && !tracks[i].confirmed)
        // {
        //     // not_confirmed++;
        //     continue;
        // }
            
        perceptron::s_Object obj;
         // 是否为确认状态, 主要看是否速度滤波是否收敛
        obj.is_tracker = tracks[i].confirmed;

        obj.object_confidence = tracks[i].confidence;
        obj.lane_id = "";
        obj.object_class_type = perceptron::e_ObjectType::OBJECT_TYPE_UNKNOWN;
        obj.object_id = tracks[i].id;

        obj.point3f.x = tracks[i].x;
        obj.point3f.y = tracks[i].y;
        // printf("obj.point3f.y=%f %f\n", obj.point3f.x, obj.point3f.y);
        obj.point3f.z = 0;

        obj.point4f.camera_x = std::max(0, cx - w/2);  //left
        obj.point4f.camera_y = std::max(0, ymax - h);  //top
        obj.point4f.camera_w = w;
        obj.point4f.camera_h = h;

        obj.object_speed = tracks[i].speed;
        double speed_x = tracks[i].speed * sin(tracks[i].heading);
        double speed_y = tracks[i].speed * cos(tracks[i].heading);
        obj.speed3f.speed_x = speed_x;
        obj.speed3f.speed_y = speed_y;
        obj.speed3f.speed_z = 0;
        obj.object_acceleration = tracks[i].acceleration;

        obj.point_gps.object_longitude = tracks[i].longitude;
        obj.point_gps.object_latitude = tracks[i].latitude;
        obj.point_gps.object_elevation = config->loc_config.located_elevation;
        
        obj.object_NS = perceptron::e_NS::N;
        obj.object_WE = perceptron::e_EW::E;
        obj.object_direction = 0;
        obj.object_heading = tracks[i].heading;
        obj.is_head_tail = 0;
        obj.lane_type = perceptron::e_LaneType::MOTOR_LANE;
        //plate
        obj.plate_num = tracks[i].plate;

        obj.obj_time_stamp = timestamp;
        //0未知 1RSU自身 3摄像机 4毫米波 6激光雷达 7融合
        obj.ptc_sourcetype = 3;
        // vehicle color
        switch (tracks[i].veh_color)
        {
            case 0:
                obj.ptc_veh_color = perceptron::e_VehicleColor::VEHICLE_COLOR_UNKNOWN;break;
            case 1:
                obj.ptc_veh_color = perceptron::e_VehicleColor::WRITE;break;
            case 2:
                obj.ptc_veh_color = perceptron::e_VehicleColor::GREY;break;
            case 3:
                obj.ptc_veh_color = perceptron::e_VehicleColor::YELLOW;break;
            case 4:
                obj.ptc_veh_color = perceptron::e_VehicleColor::CYAN;break;
            case 5:
                obj.ptc_veh_color = perceptron::e_VehicleColor::RED;break;
            case 6:
                obj.ptc_veh_color = perceptron::e_VehicleColor::VIOLET;break;
            case 7:
                obj.ptc_veh_color = perceptron::e_VehicleColor::GREEN;break;
            case 8:
                obj.ptc_veh_color = perceptron::e_VehicleColor::BLUE;break;
            case 9:
                obj.ptc_veh_color = perceptron::e_VehicleColor::BROWN;break;
            case 10:
                obj.ptc_veh_color = perceptron::e_VehicleColor::BLACK;break;
            case 11:
                obj.ptc_veh_color = perceptron::e_VehicleColor::ORANGE;break;
            case 12:
                obj.ptc_veh_color = perceptron::e_VehicleColor::PINK;break;
            case 13:
                obj.ptc_veh_color = perceptron::e_VehicleColor::OTHER;break;
	        default:
	            obj.ptc_veh_color = perceptron::e_VehicleColor::VEHICLE_COLOR_UNKNOWN;break;	
        }

        if(config->gie_config.class_names[tracks[i].type] == "car")
        {
            obj.object_class_type = perceptron::e_ObjectType::MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::MOTOR_CAR;
            obj.target_size.object_height = 1.6;
            obj.target_size.object_length = 4.2;
            obj.target_size.object_width = 1.5;
        }
        else if(config->gie_config.class_names[tracks[i].type] == "bus")
        {
            obj.object_class_type = perceptron::e_ObjectType::MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::MOTOR_BUS;
            obj.target_size.object_height = 2.5;
            obj.target_size.object_length = 11.5;
            obj.target_size.object_width = 3.0;
        }
        else if(config->gie_config.class_names[tracks[i].type] == "truck")
        {
            obj.object_class_type = perceptron::e_ObjectType::MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::MOTOR_Truck;
            obj.target_size.object_height = 1.9;
            obj.target_size.object_length = 4.2;
            obj.target_size.object_width = 1.8;

        }
        else if(config->gie_config.class_names[tracks[i].type] == "person")
        {
            obj.object_class_type = perceptron::e_ObjectType::PEDESTRIAN;
            obj.ptc_veh_type = perceptron::e_VehicleType::VEHICLE_TYPE_UNKNOWN;
            obj.target_size.object_height = 0.5;
            obj.target_size.object_length = 0.2;
            obj.target_size.object_width = 1.7;
        }
        else if(config->gie_config.class_names[tracks[i].type] == "motorbike")
        {
            obj.object_class_type = perceptron::e_ObjectType::NON_MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::VEHICLE_TYPE_UNKNOWN;
            obj.target_size.object_height = 0.7;
            obj.target_size.object_length = 2.0;
            obj.target_size.object_width = 1.5;
        }
        else if(config->gie_config.class_names[tracks[i].type] == "tricycle")
        {
            obj.object_class_type = perceptron::e_ObjectType::NON_MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::MOTOR_MOTOBIKE;
            obj.target_size.object_height = 1.5;
            obj.target_size.object_length = 2.5;
            obj.target_size.object_width = 1.5;
        }   
        else if(config->gie_config.class_names[tracks[i].type] == "cone")
        {
            obj.object_class_type = perceptron::e_ObjectType::CONE;
            obj.ptc_veh_type = perceptron::e_VehicleType::VEHICLE_TYPE_UNKNOWN;
            obj.target_size.object_height = 0.5;
            obj.target_size.object_length = 0.5;
            obj.target_size.object_width = 0.5;
        } 
        else if(config->gie_config.class_names[tracks[i].type] == "throws")
        {
            obj.object_class_type = perceptron::e_ObjectType::THROWS;
            obj.ptc_veh_type = perceptron::e_VehicleType::VEHICLE_TYPE_UNKNOWN;
            obj.target_size.object_height = 1.5;
            obj.target_size.object_length = 2.5;
            obj.target_size.object_width = 1.5;
        } 

        //计算link和lane相关信息
        if(tracks[i].lane_idx != -1)  //配置到lane级,lane和link都存在
        {
            int link_idx = tracks[i].link_idx;
            int lane_idx = tracks[i].lane_idx;

            obj.lane_id = config->nl_config.links[link_idx].lanes[lane_idx].lane_id;
            obj.is_head_tail = config->nl_config.links[link_idx].direction;
            obj.object_direction = config->nl_config.links[link_idx].direction;
            obj.lane_type = (perceptron::e_LaneType)config->nl_config.links[link_idx].lanes[lane_idx].lane_type;

            if (config->gie_config.class_names[tracks[i].type] == "car" || 
                    config->gie_config.class_names[tracks[i].type] == "bus" || 
                    config->gie_config.class_names[tracks[i].type] == "truck")
            {
                //link级
                config->nl_config.links[link_idx].vehicle_speed += tracks[i].speed;
                config->nl_config.links[link_idx].vehicle_count ++;
                //lane级
                config->nl_config.links[link_idx].lanes[lane_idx].vehicle_speed += tracks[i].speed;
                config->nl_config.links[link_idx].lanes[lane_idx].vehicle_count ++;
            }
        }
        else if(tracks[i].link_idx != -1)  //配置到link级,仅link存在
        {
            int link_idx = tracks[i].link_idx;
            obj.lane_id = config->nl_config.links[link_idx].link_id;
            obj.is_head_tail = config->nl_config.links[link_idx].direction;
            obj.object_direction = config->nl_config.links[link_idx].direction;
            if (config->gie_config.class_names[tracks[i].type] == "car" || 
                    config->gie_config.class_names[tracks[i].type] == "bus" || 
                    config->gie_config.class_names[tracks[i].type] == "truck")
            {
                config->nl_config.links[link_idx].vehicle_speed += tracks[i].speed;
                config->nl_config.links[link_idx].vehicle_count ++;
            }
        }

        objects->object_list.push_back(obj);
    }

    // printf("not in roi:%d  %d\n", not_in_roi_count, not_confirmed);

    if(objects->object_list.size() > 0)
    {
        // link信息
        for(uint i=0; i<config->nl_config.links.size(); i++)
        {
            perceptron::s_LinkJamSenseParams link;
            link.link_id = config->nl_config.links[i].link_id;

            if(config->nl_config.links[i].vehicle_count > 0)
            {
                link.link_veh_num = config->nl_config.links[i].vehicle_count;
                link.link_avgspeed = config->nl_config.links[i].vehicle_speed/config->nl_config.links[i].vehicle_count;
            }
            else
            {
                link.link_avgspeed = FLOAT_INF_MAX;
            }
            link.link_direction = config->nl_config.links[i].direction;

            objects->link_jam_sense_params.push_back(link);

            // lane信息
            for(int j=0; j<config->nl_config.links[i].lane_count; j++)
            {
                perceptron::s_LaneJamSenseParams lane;
                lane.lane_id = config->nl_config.links[i].lanes[j].lane_id;
                lane.lane_sense_len = config->nl_config.links[i].lanes[j].lane_length;
                lane.lane_types = (perceptron::e_LaneType)config->nl_config.links[i].lanes[j].lane_type;
                lane.lane_direction = config->nl_config.links[i].direction;

                if(config->nl_config.links[i].lanes[j].vehicle_count > 0)
                {
                    lane.lane_veh_num = config->nl_config.links[i].lanes[j].vehicle_count;
                    lane.lane_avg_speed = config->nl_config.links[i].lanes[j].vehicle_speed/config->nl_config.links[i].lanes[j].vehicle_count;
                }
                else
                {
                    lane.lane_avg_speed = FLOAT_INF_MAX;
                }

                objects->lane_jam_sense_params.push_back(lane);
            }
        }
    }    
}

static void ConvertDetections(Detections detections, uint64_t timestamp, AppConfig* config, perceptron::s_ObjectList* objects)
{
    objects->devide_id = config->nl_config.camera_sensor_id;
    objects->time_stamp = timestamp;
    objects->number_frame = frame_count;
    objects->devide_is_true = true;
    objects->sensor_type = perceptron::e_SensorType::SensorType_CAMERA;
    objects->perception_gps.object_longitude = config->loc_config.located_longitude;
    objects->perception_gps.object_latitude = config->loc_config.located_latitude;

    for(uint i=0; i < detections.size(); i++)
    {  
        //the object' bbox   
        Bbox b = detections[i].ToCxymax();  
        int cx = (int)b(0);
        int ymax = (int)b(1);

        // select obj in the roi
        bool is_in_roi = false;
        for(int k=0;k<config->nl_config.roi_points.size();k++)
        {
            if(IsPointInPolygon(cv::Point(cx, ymax), config->nl_config.roi_points[k]))
            {
                is_in_roi = true;
                break;
            }
        }

        if(!is_in_roi)  
            continue;

        perceptron::s_Object obj;
        obj.object_confidence = detections[i].confidence;

        obj.point4f.camera_x = detections[i].tlwh(0);  //left
        obj.point4f.camera_y = detections[i].tlwh(1);  //top
        obj.point4f.camera_w = detections[i].tlwh(2);
        obj.point4f.camera_h = detections[i].tlwh(3);

        if(config->gie_config.class_names[detections[i].type] == "car")
        {
            obj.object_class_type = perceptron::e_ObjectType::MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::MOTOR_CAR;
            obj.target_size.object_height = 1.6;
            obj.target_size.object_length = 4.2;
            obj.target_size.object_width = 1.5;
        }
        else if(config->gie_config.class_names[detections[i].type] == "bus")
        {
            obj.object_class_type = perceptron::e_ObjectType::MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::MOTOR_BUS;
            obj.target_size.object_height = 2.5;
            obj.target_size.object_length = 11.5;
            obj.target_size.object_width = 3.0;
        }
        else if(config->gie_config.class_names[detections[i].type] == "truck")
        {
            obj.object_class_type = perceptron::e_ObjectType::MOTOR;
            obj.ptc_veh_type = perceptron::e_VehicleType::MOTOR_Truck;
            obj.target_size.object_height = 1.9;
            obj.target_size.object_length = 4.2;
            obj.target_size.object_width = 1.8;

        }
        else if(config->gie_config.class_names[detections[i].type] == "person")
        {
            obj.object_class_type = perceptron::e_ObjectType::PEDESTRIAN;
            obj.target_size.object_height = 0.5;
            obj.target_size.object_length = 0.2;
            obj.target_size.object_width = 1.7;
        }
        else if(config->gie_config.class_names[detections[i].type] == "motorbike")
        {
            obj.object_class_type = perceptron::e_ObjectType::NON_MOTOR;
            obj.target_size.object_height = 0.7;
            obj.target_size.object_length = 2.0;
            obj.target_size.object_width = 1.5;
        }
        else if(config->gie_config.class_names[detections[i].type] == "tricycle")
        {
            obj.object_class_type = perceptron::e_ObjectType::NON_MOTOR;
            obj.target_size.object_height = 1.5;
            obj.target_size.object_length = 2.5;
            obj.target_size.object_width = 1.5;
        }   
        else if(config->gie_config.class_names[detections[i].type] == "cone")
        {
            obj.object_class_type = perceptron::e_ObjectType::CONE;
            obj.ptc_veh_type = perceptron::e_VehicleType::VEHICLE_TYPE_UNKNOWN;
            obj.target_size.object_height = 0.5;
            obj.target_size.object_length = 0.5;
            obj.target_size.object_width = 0.5;
        } 
        else if(config->gie_config.class_names[detections[i].type] == "throws")
        {
            obj.object_class_type = perceptron::e_ObjectType::THROWS;
            obj.ptc_veh_type = perceptron::e_VehicleType::VEHICLE_TYPE_UNKNOWN;
            obj.target_size.object_height = 1.5;
            obj.target_size.object_length = 2.5;
            obj.target_size.object_width = 1.5;
        } 
        objects->object_list.push_back(obj);
    }
}

// 判断目标所在lane/link的位置
static void CalculateLinkLane(const std::vector<Link>& links, TrackList& objects)
{
    cv::Point pt;
    for(size_t k = 0; k < objects.size(); k++)
    { 
        bool flag = false;
        pt.x = objects[k].cywh[0];
        pt.y = objects[k].cywh[1];

        for(size_t i=0; i < links.size(); i++)
        {
            // 配置到link级, 判断点是否在links roi中
            if(links[i].lanes.size() < 0)
            {
                if(IsPointInPolygon(pt, links[i].link_points))
                {
                    objects[k].link_idx = i;
                    objects[k].link_heading = links[i].heading;  //目标初始航向角
                    objects[k].lane_idx = -1;
                    flag = true;
                    break;
                }
            }
            else
            {
                //配置到lane级, 判断是都在lane roi中
                for(size_t j=0; j<links[i].lanes.size(); j++)
                {
                    if(IsPointInPolygon(pt, links[i].lanes[j].lane_points))
                    {
                        objects[k].link_idx = i;
                        objects[k].lane_idx = j;
                        objects[k].link_heading = links[i].heading;  //目标初始航向角
                        flag = true;
                        break;
                    }
                }  
                if(flag) 
                    break;             
            }
        }
        if(!flag)
        {
            objects[k].lane_idx = -1;
            objects[k].link_idx = -1;
        }
    }
}


void CameraAPP::Process(cv::Mat img, uint64_t timestamp, perceptron::s_ObjectList* objects)
{
    // 检测
    std::vector<Yolo::s_Detection> output;
    detector_->Process(img, output);

    // cv::Mat img1;
    // img.copyTo(img1);
    // static std::vector<std::string> classnames = {"car","bus","truck","person","motorbike","tricycle"};

    // 跟踪
    Detections detections;
    for (uint j = 0; j < output.size(); j++) 
    {
        // xyxy to tlwh
        cv::Rect r;
        //ScaleCoords(output[j].bbox, img.cols, img.rows, r);
        DetectionRow tmpRow;
        tmpRow.tlwh = Bbox(output[j].bbox[0], output[j].bbox[1], output[j].bbox[2] - output[j].bbox[0], output[j].bbox[3] - output[j].bbox[1]);  
        tmpRow.type = (int)output[j].class_id;             //class id
        tmpRow.confidence = output[j].conf; 
        detections.push_back(tmpRow);

        // cv::rectangle(img1, r, cv::Scalar(0,255,0),1);
        // std::string text = cv::format("%s:%.1f", classnames[output[j].class_id].c_str(), output[j].conf);
        // cv::putText(img1, text, cv::Point(r.x, r.y), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }

    // cv::imshow("det", img1);

    frame_count++;
    // ConvertDetections(detections, timestamp, &config_, objects);

    // printf("det size: %d  tracks_.size: %d\n", detections.size(), tracks_.size());
    tracker_->Process(img, detections, timestamp, tracks_);
    // printf("after tracks_.size: %d\n", tracks_.size());

    //删掉DELETE状态的目标
    vector<TrackNode>::iterator it;
    for(it = tracks_.begin(); it != tracks_.end();) 
    {
        if((*it).state == e_State::DELETE) 
            it = tracks_.erase(it);
        else ++it;
    }
    tracks_.swap(tracks_);


#if 1
    for(auto& track : tracks_)
    {
        int cx = (int)track.cywh[0];
        int ymax = (int)track.cywh[1];

        // cx = ClampValue(cx, 1, img.cols-1);
        // ymax = ClampValue(ymax, 1, img.rows-1);

        cv::Rect rr(cx-track.cywh[2]/2, ymax-track.cywh[3], track.cywh[2], track.cywh[3]);
        cv::rectangle(img, rr, cv::Scalar(255,0,0),1);

        // select obj in the roi
        bool is_in_roi = false;
        for(int k=0;k<config_.nl_config.roi_points.size();k++)
        {
            if(IsPointInPolygon(cv::Point(cx, ymax), config_.nl_config.roi_points[k]))
            {
                is_in_roi = true;
                break;
            }
        }

        if(!is_in_roi) 
        {
            track.in_roi = false;
        }
        else
        {
            track.in_roi = true;
        } 
    }
#endif

    // 计算link或者lane
    if(config_.nl_config.links.size() > 0)
        CalculateLinkLane(config_.nl_config.links, tracks_);

    // // 定位
    if(config_.loc_config.enable)
        location_->Process(config_.loc_config, timestamp, tracks_);

    // 转协议
    CovertObjectsV3(tracks_, timestamp, &config_, img.cols, img.rows, objects);  
}

void CameraAPP::Release()
{
    tracker_->Release();
    location_->Release();
}

void CameraAPP::DrawResult(cv::Mat& img, perceptron::s_ObjectList* objects)
{
    // 画ROI
    cv::polylines(img, config_.nl_config.roi_points, true, cv::Scalar(255,0,255), 2);
    
    // draw link lane
    for(int i=0; i<config_.nl_config.links.size(); i++)
    {
        if(config_.nl_config.links[i].link_points.size()>0)
        {
            cv::polylines(img, config_.nl_config.links[i].link_points, true, cv::Scalar(0,255,0), 2);
        }
        else
        {
            for(int j=0; j<config_.nl_config.links[i].lane_count; j++)
            {
                cv::polylines(img, config_.nl_config.links[i].lanes[j].lane_points, true, cv::Scalar(0,255,0), 2);
            }
        }
    } 

#if 0
    for(auto &hcfg : config_.loc_config.headingcfg)
    {
        cv::polylines(img, hcfg.roi, true, cv::Scalar(0,255,0), 1);
    }
    // if(config_.loc_config.roi_down.size() > 0)
    //     cv::polylines(img, config_.loc_config.roi_down, true, cv::Scalar(0,255,255), 2);
    // if(config_.loc_config.roi_up.size() > 0)
    //     cv::polylines(img, config_.loc_config.roi_up, true, cv::Scalar(0,255,255), 2);
    // if(config_.loc_config.roi_right.size() > 0)
    //     cv::polylines(img, config_.loc_config.roi_right, true, cv::Scalar(0,255,255), 2);
    // if(config_.loc_config.roi_left.size() > 0)
    //     cv::polylines(img, config_.loc_config.roi_left, true, cv::Scalar(0,255,255), 2);
#endif

    for(uint i=0; i<objects->object_list.size(); i++)
    {
        int x = objects->object_list[i].point4f.camera_x;
        int y = objects->object_list[i].point4f.camera_y;
        int w = objects->object_list[i].point4f.camera_w;
        int h = objects->object_list[i].point4f.camera_h;
        perceptron::e_ObjectType cls = objects->object_list[i].object_class_type;
        float conf = objects->object_list[i].object_confidence;
        int object_id = objects->object_list[i].object_id;
        int is_tracker = objects->object_list[i].is_tracker;
        float heading = objects->object_list[i].object_heading;
        float speed = objects->object_list[i].object_speed;
        std::string plate = objects->object_list[i].plate_num; //plate

        std::string lane = objects->object_list[i].lane_id;


        std::string cls_name;
        switch(cls)
        {
            case perceptron::e_ObjectType::MOTOR:
            {
                perceptron::e_VehicleType veh_type = objects->object_list[i].ptc_veh_type;
                if(veh_type == perceptron::e_VehicleType::MOTOR_CAR)
                    cls_name = "car";
                else if(veh_type == perceptron::e_VehicleType::MOTOR_BUS)
                    cls_name = "bus";
                else if(veh_type == perceptron::e_VehicleType::MOTOR_Truck)
                    cls_name = "truck";
                else
                    cls_name = "unknown_motor";

                break;
            }
            case perceptron::e_ObjectType::PEDESTRIAN:
                cls_name = "person";break;
            case perceptron::e_ObjectType::NON_MOTOR:
                cls_name = "motorbike";break;
            case perceptron::e_ObjectType::CONE:
                cls_name = "cone";break;
            case perceptron::e_ObjectType::THROWS:
                cls_name = "throws";break;
            default:
                cls_name = "unknown";
        }
    
        // 画目标框信息
        cv::Rect rect(x, y, w, h);
        cv::rectangle(img, rect, cv::Scalar(0,0,255), 3);

        // 跟踪id
        std::string text = std::to_string(object_id) + ":" + cls_name;
        // std::string text = cls_name;
        // std::string text = std::to_string(object_id);
        //std::string text = cv::format("%d: %s-%.1f-%.2f", object_id, cls_name.c_str(), heading, speed);
        //std::string text = cv::format("%d: %.1f-%.2f", object_id, heading, speed);
        // std::string text = cv::format("%d: %s", object_id, lane.c_str());

        cv::putText(img, text, cv::Point(x,y), cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 255, 0), 1);
        // if (plate != "0")
        // {
        //     text = text + ":" + plate;  
        // }
        // wchar_t w_str[100];
        // ToWchar(text.c_str(), w_str);

        // zh_text.putText(img, w_str, cv::Point(x,y), cv::Scalar(0, 255, 0));
        
#if 0         
        // 画航向角箭头 optimize
        double arrow_length = speed*20;
        // 画箭头的方向 偏转摄像机安装位置与正北方向的夹角
        double sita = heading + config_.loc_config.located_offset_angle;
        while(sita < 0.0f)       sita += 360.f;
        while(sita >= 360.f)     sita -= 360.f;
        // double radian = A_ANGLE2RADIAN(sita);
        double radian = sita * M_PI / 180;

        int cx = x + w/2;
        int ymax = y + h;
        double arrow_start_pos_x = cx; //箭头起点空间坐标
        double arrow_start_pos_y = ymax;
        double arrow_end_pos_x = cx + arrow_length * sin(radian); //箭头终点空间坐标
        double arrow_end_pos_y = ymax + arrow_length * (-cos(radian));

        std::shared_ptr<cv::Point2d> arrow_start_front = std::make_shared<cv::Point2d>(arrow_start_pos_x, arrow_start_pos_y);
        std::shared_ptr<cv::Point2d> arrow_end_front = std::make_shared<cv::Point2d>(arrow_end_pos_x, arrow_end_pos_y);
        arrowedLine(img, *arrow_start_front, *arrow_end_front, cv::Scalar(255,0,0), 2, 8, 0, 0.1);
#endif

    }
    
}

}
