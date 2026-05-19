/*
* camera_config_parser.cpp
* Created on: 20210530
* Author: yueyingying
* Description: 配置文件解析
*
*/
#include "camera_config_parser.h"
#include <unistd.h>
#include <libconfig.h++>

namespace camera 
{

//NL_config configuration
#define NL_CONFIG_GROUP "NL_config"
#define CAM_SENSOR_ID "camera_device_id"
#define ROI_GROUP "roi"
#define ROI_POINTS "roi_points"
#define ROI_PLATE_POINTS "roi_plate_points"
#define JSONNAME "calibration_file"
#define FONTNAME "font_file"
#define HEADING "heading"
#define LINK_COUNT "link_count"
#define LINK_CONFIG_FILE "link_config_file"
#define COORDINATE_SYS "coordinate_system"
#define LOCATED_LONG "camera_location_longitude"
#define LOCATED_LAT "camera_location_latitude"
#define LOCATED_ANGLE "camera_location_angle"
#define LOCATED_ELEVATION "camera_location_elevation"
#define DRIVING_DIRECTION "driving_direction"
// #define ROI_RIGHT "roi_right_points"
// #define ROI_LEFT "roi_left_points"
// #define ROI_UP "roi_up_points"
// #define ROI_DOWN "roi_down_points"
#define ROI_HEADING_GROUPS "roi"
#define ROI_HEADING "heading"
#define ROI_HEADING_POINTS "roi_points"

#define CONFIG_GROUP_LINK "link"
#define LANE_GROUP "lane"
#define LINK_ID "link_id"
#define LINK_POINT "link_points"
#define LINK_DIRECTION "direction"
#define LANE_COUNT "lane_count"
#define LANE_ID "lane_id"
#define LANE_LENGTH "lane_length"
#define LANE_TYPE "lane_type"
#define LANE_POINTS "lane_points"

#define CONFIG_GROUP_GIE "gie"
#define MODEL_ENGINE_FILE "model_engine_file"
#define MODEL_FILE "model_file"
//#define LABELFILE_PATH "label_file"
#define LABEL_NAMES "class_names"
#define NETWORK_MODE "network_mode"
#define CLASS_NUM   "num_detected_classes"
#define NPU_IDX "idx_npu"
#define BATCH_SIZE  "batch_size"
#define NMS_THRESH  "nms_iou_threshold"
#define CLUSTER_THRESH   "pre-cluster-threshold"

#define CONFIG_GROUP_TRACKER "tracker"
#define ONNX_FILE "onnx_file"
#define LOW_THRESH "low_thresh"
#define HIGH_THRESH "high_thresh"
#define MAX_IOU_THRESH "max_iou_thresh"
#define STD_MEAS_X "std_measurement_x"
#define STD_MEAS_Y "std_measurement_y"
#define STD_STATE_X "std_state_ax"
#define STD_STATE_Y "std_state_ay"
#define LOCK_VELO "speed_thresh"
#define CONVERGED_THRESH "speed_converged_thresh"
#define HIGH_THRESH_PERSON "high_thresh_person"
#define HIGH_THRESH_MOTORBIKE "high_thresh_motorbike"

#define LOCATION_GROUP "location"

#define ENABLE "enable"

//去字符串两边的空格
static void TrimString(std::string &s) 
{
    if (s.empty()) 
    {
        return;
    }
    s.erase(0, s.find_first_not_of(" "));
    s.erase(s.find_last_not_of(" ") + 1);
}

//字符串分隔
void SplitString(const std::string str, const char split, std::vector<std::string> &dst)
{
    if (str.empty())
    {
        return;
    }

    std::string str1 = str + split;

    std::string::size_type pos = str1.find(split);
    while (pos != str1.npos)
    {
        std::string temp = str1.substr(0, pos);
        TrimString(temp);
        dst.push_back(temp);

        str1 = str1.substr(pos+1, str1.size());
        pos = str1.find(split);
    }
}

bool ParseCameraConfigFile(AppConfig* config, const char* cfg_file_path)
{
    libconfig::Config cfg;
    try
    {
        cfg.readFile(cfg_file_path);
    }
    catch(const libconfig::FileIOException& fioex)
    {
        std::cerr << "I/O error while reading file: "<< cfg_file_path << '\n';
        return false;
    }
    catch(const libconfig::ParseException& pex)
    {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
            << " - " << pex.getError() << std::endl;
        
        return false;
    }

    const libconfig::Setting &root = cfg.getRoot();
    // 解析gie组
    try
    {
        const libconfig::Setting &gie = root[CONFIG_GROUP_GIE];

        std::string nms_iou_threshold, confidence_threshold;
        std::string model_engine_file, label_names, model_file;
        int net_mode, npu_idx;
        if(!(gie.lookupValue(MODEL_ENGINE_FILE, model_engine_file) && 
                gie.lookupValue(LABEL_NAMES, label_names) &&
                gie.lookupValue(NMS_THRESH, nms_iou_threshold) &&
                gie.lookupValue(NETWORK_MODE, net_mode) &&
                gie.lookupValue(NPU_IDX, npu_idx) &&
                gie.lookupValue(CLUSTER_THRESH, confidence_threshold)))
        {
            std::cerr << "parse gie lookupvalue error " << std::endl;
            return false;
        }

        if(gie.lookupValue(MODEL_FILE, model_file))
        {
            config->gie_config.model_file = model_file;
        }
        std::string label_file_name;
        //if(gie.lookupValue(LABELFILE_PATH, label_file_name))
        //{
      //      config->gie_config.label_file = label_file_name;
        //}

        config->gie_config.conf_thresh = atof(confidence_threshold.c_str());
        config->gie_config.nms_thresh = atof(nms_iou_threshold.c_str());
        config->gie_config.npu = npu_idx;
        config->gie_config.engine_file = model_engine_file;
        config->gie_config.batch_size = 1;
        config->gie_config.network_mode = network_modes[net_mode];
        
        std::vector<std::string> names;
        SplitString(label_names, ';', names);
        for(auto name : names)
        {
            config->gie_config.class_names.push_back(name);
        }      
        config->gie_config.class_num = config->gie_config.class_names.size();  
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        std::cerr << "gie group error in configuration file." << std::endl;
        return false;
    }      

    // 解析tracker组
    try
    {
        const libconfig::Setting &tracker = root[CONFIG_GROUP_TRACKER];

        std::string low_thresh, high_thresh, max_iou_thresh;
        if(!(tracker.lookupValue(LOW_THRESH, low_thresh) && 
                tracker.lookupValue(HIGH_THRESH, high_thresh) &&
                tracker.lookupValue(MAX_IOU_THRESH, max_iou_thresh)))
        {
            std::cerr << "parse tracker lookupvalue error "<< std::endl;
            return false;
        }

        std::string high_thresh_p, hight_thresh_m;
        bool flag = true;
        if(!(tracker.lookupValue(HIGH_THRESH_PERSON, high_thresh_p) && 
                tracker.lookupValue(HIGH_THRESH_MOTORBIKE, hight_thresh_m)))
        {
            flag = false;
            std::cout << "use one high_thresh "<< std::endl;
        }

        config->tracker_config.low_thresh = atof(low_thresh.c_str());
        config->tracker_config.high_thresh = atof(high_thresh.c_str());
        config->tracker_config.max_iou_thresh = atof(max_iou_thresh.c_str());

        if(flag)
        {
            config->tracker_config.high_thresh_person = atof(high_thresh_p.c_str());
            config->tracker_config.high_thresh_motorbike = atof(hight_thresh_m.c_str());
        }
        else
        {
            config->tracker_config.high_thresh_person = config->tracker_config.high_thresh;
            config->tracker_config.high_thresh_motorbike = config->tracker_config.high_thresh;
        } 
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        std::cerr << "tracker group error in configuration file." << std::endl;
        return false;
    }  

    // 解析location组
    try
    {
        const libconfig::Setting &location = root[LOCATION_GROUP];

        std::string std_measurement_x, std_measurement_y, std_state_ax;
        std::string std_state_ay, speed_thresh, speed_converged_thresh;
        std::string calibration_file;
        int enable;
        std::string heading, camera_location_angle, camera_location_longitude, camera_location_latitude, camera_location_elevation;
        if(!(
            location.lookupValue(ENABLE, enable) &&
            location.lookupValue(JSONNAME, calibration_file) &&
            location.lookupValue(STD_MEAS_X, std_measurement_x) &&
            location.lookupValue(STD_MEAS_Y, std_measurement_y) &&
            location.lookupValue(STD_STATE_X, std_state_ax) &&
            location.lookupValue(STD_STATE_Y, std_state_ay) &&
            location.lookupValue(LOCK_VELO, speed_thresh) &&
            location.lookupValue(CONVERGED_THRESH, speed_converged_thresh) &&
            location.lookupValue(LOCATED_ANGLE, camera_location_angle) &&
            location.lookupValue(LOCATED_LONG, camera_location_longitude) &&
            location.lookupValue(LOCATED_LAT, camera_location_latitude) &&
            location.lookupValue(LOCATED_ELEVATION, camera_location_elevation) &&
            location.lookupValue(HEADING, heading)))
        {
            std::cerr << "parse location lookupvalue error "<< std::endl;
            return false;
        }

        config->loc_config.enable = (bool)enable;
        config->loc_config.std_meas_x = atof(std_measurement_x.c_str());
        config->loc_config.std_meas_y = atof(std_measurement_y.c_str());
        config->loc_config.std_state_ax = atof(std_state_ax.c_str());
        config->loc_config.std_state_ay = atof(std_state_ay.c_str());
        config->loc_config.lock_velo = atof(speed_thresh.c_str());
        config->loc_config.velo_converged_var = atof(speed_converged_thresh.c_str());     

        config->loc_config.calibrator_file = calibration_file;
        config->loc_config.heading = atof(heading.c_str());
        config->loc_config.located_offset_angle = atof(camera_location_angle.c_str());
        config->loc_config.located_longitude = atof(camera_location_longitude.c_str());
        config->loc_config.located_latitude = atof(camera_location_latitude.c_str());
        config->loc_config.located_elevation = atof(camera_location_elevation.c_str());

        //查找多个ROI——HEADING组，每个组对应roi points 和 heading
        try
        {
            const libconfig::Setting &roi_headings = location.lookup(ROI_HEADING_GROUPS);
            for(int i=0; i<roi_headings.getLength(); i++)
            {
                HeadingConfig hcfg;
                std::string points, heading;
                if(!(roi_headings[i].lookupValue(ROI_HEADING_POINTS, points)))
                {
                    std::cerr << "parse roi_headings points lookupvalue error in "<< i+1 << std::endl;
                    return false;
                }
                else
                {
                    std::vector<std::string> pts1;
                    SplitString(points, ';', pts1);
                    if(pts1.size()%2 != 0)
                    {
                        std::cerr << "the number of roi_headings points is wrong in "<< i+1 << std::endl;
                        return false;
                    }
                    for(uint j=0; j<pts1.size(); j+=2)
                    {
                        cv::Point p;
                        p.x = atoi(pts1[j].c_str());
                        p.y = atoi(pts1[j+1].c_str());
                        hcfg.roi.push_back(p);
                    }

                }

                if(!(roi_headings[i].lookupValue(ROI_HEADING, heading)))
                {
                    std::cerr << "parse roi_headings points lookupvalue error in "<< i+1 << std::endl;
                    return false;
                }
                else
                {
                    hcfg.heading = atof(heading.c_str());
                }

                config->loc_config.headingcfg.push_back(hcfg);
            }
        }
        catch(const libconfig::SettingNotFoundException &nfex)
        {
            // std::cout << "there is not link message in configuration file." << std::endl;
            // return false;
        } 

#if 0 
        std::vector<std::string> pts1;
        int dirving_direction;
        std::string right_points, left_points, up_points, down_points;
        if(location.lookupValue(DRIVING_DIRECTION, dirving_direction))
        {
            config->loc_config.dirving_direction = dirving_direction;
        }
        else
            config->loc_config.dirving_direction = -1;
        
        printf("dirving_direction : %d\n", config->loc_config.dirving_direction);
    
        if(location.lookupValue(ROI_RIGHT, right_points))
        {
            SplitString(right_points, ';', pts1);
            if(pts1.size()%2 != 0)
            {
                std::cerr << "ERROR: the number of right_points size %d"<< pts1.size() << std::endl;
                return false;
            }
            
            for(uint i=0; i<pts1.size(); i+=2)
            {
                cv::Point p;
                p.x = atoi(pts1[i].c_str());
                p.y = atoi(pts1[i+1].c_str());
                
                config->loc_config.roi_right.push_back(p);
            }
        }

        if(location.lookupValue(ROI_LEFT, left_points))
        {
            pts1.clear();
            SplitString(left_points, ';', pts1);
            if(pts1.size()%2 != 0)
            {
                std::cerr << "ERROR: the number of left_points size %d"<< pts1.size() << std::endl;
                return false;
            }

            for(uint i=0; i<pts1.size(); i+=2)
            {
                cv::Point p;
                p.x = atoi(pts1[i].c_str());
                p.y = atoi(pts1[i+1].c_str());
                
                config->loc_config.roi_left.push_back(p);
            }
        }

        if(location.lookupValue(ROI_UP, up_points))
        {
            pts1.clear();
            SplitString(up_points, ';', pts1);
            if(pts1.size()%2 != 0)
            {
                std::cerr << "ERROR: the number of up_points size %d"<< pts1.size() << std::endl;
                return false;
            }

            for(uint i=0; i<pts1.size(); i+=2)
            {
                cv::Point p;
                p.x = atoi(pts1[i].c_str());
                p.y = atoi(pts1[i+1].c_str());
                
                config->loc_config.roi_up.push_back(p);
            }
        }

        if(location.lookupValue(ROI_DOWN, down_points))
        {
            pts1.clear();
            SplitString(down_points, ';', pts1);
            if(pts1.size()%2 != 0)
            {
                std::cerr << "ERROR: the number of down_points size %d"<< pts1.size() << std::endl;
                return false;
            }

            for(uint i=0; i<pts1.size(); i+=2)
            {
                cv::Point p;
                p.x = atoi(pts1[i].c_str());
                p.y = atoi(pts1[i+1].c_str());
                
                config->loc_config.roi_down.push_back(p);
            }
        }
#endif

    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        std::cerr << "location group error in configuration file." << std::endl;
        return false;
    } 

    // 解析nl_config组
    try
    {
        const libconfig::Setting &nl_config = root[NL_CONFIG_GROUP];

        std::string camera_device_id, font_file;
        if(!(nl_config.lookupValue(CAM_SENSOR_ID, camera_device_id) && 
                nl_config.lookupValue(FONTNAME, font_file)))
        {
            std::cerr << "parse nl_config lookupvalue error "<< std::endl;
            return false;
        }

        config->nl_config.camera_sensor_id = camera_device_id;
        config->nl_config.font_file = font_file;
 
        const libconfig::Setting &rois = nl_config.lookup(ROI_GROUP);
        int roi_count = rois.getLength();
        for(int i=0; i<roi_count; i++)
        {
            std::string roi_points;
            if(!(rois[i].lookupValue(ROI_POINTS, roi_points)))
            {
                std::cerr << "parse roi_points lookupvalue error in "<< i+1 << std::endl;
                return false;
            }

            Polygon roi;
            std::vector<std::string> pts;
            // std::cout << "************roi: " << roi_points << std::endl;
            SplitString(roi_points, ';', pts);
            if(pts.size()%2 != 0)
            {
                std::cerr << "the number of roi points is wrong in "<< i+1 << std::endl;
                return false;
            }
            for(uint i=0; i<pts.size(); i+=2)
            {
                cv::Point p;
                p.x = atoi(pts[i].c_str());
                p.y = atoi(pts[i+1].c_str());
                // std::cout << p.x <<"  " <<  p.y << std::endl;
                roi.push_back(p);
            }

            config->nl_config.roi_points.push_back(roi);
        }
    
        // plate roi
        std::string plate_roi_points;
        if(!(nl_config.lookupValue(ROI_PLATE_POINTS, plate_roi_points)))
        {
            std::cout << "no plate_roi_points message...." << std::endl;
        }
        else //解析车牌ROI区域顶点
        {
            std::vector<std::string> ptsr;
            SplitString(plate_roi_points, ';', ptsr);
    
            for(uint i=0; i<ptsr.size(); i+=2)
            {
                cv::Point p;
                p.x = atoi(ptsr[i].c_str());
                p.y = atoi(ptsr[i+1].c_str());
                // std::cout << p.x <<"  " <<  p.y << std::endl;
                config->nl_config.plate_roi_points.push_back(p);
            }
        }
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        std::cerr << "nl_config group error in configuration file." << std::endl;
        return false;
    } 

    // 解析link组
    try
    {
        const libconfig::Setting &links = root[CONFIG_GROUP_LINK];
        for(int i=0; i<links.getLength(); i++)
        {
            std::string link_id, heading, link_points;
            int direction;

            if(!(links[i].lookupValue(LINK_ID, link_id) && 
                links[i].lookupValue(HEADING, heading) &&
                links[i].lookupValue(LINK_DIRECTION, direction)))
            {
                std::cerr << "parse link lookupvalue error "<< std::endl;
                return false;
            }

            Link link;
            link.link_id = link_id;
            link.heading = atof(heading.c_str());
            link.direction = direction;

            if(links[i].lookupValue(LINK_POINT, link_points))
            {   //找到link_points, 配置到link级
                std::vector<std::string> link_pts;
                SplitString(link_points, ';', link_pts);
                for(uint k=0; k<link_pts.size(); k+=2)
                {
                    cv::Point p;
                    p.x = atoi(link_pts[k].c_str());
                    p.y = atoi(link_pts[k+1].c_str());
                    link.link_points.push_back(p);
                }
                if(link.link_points.size() < 4)
                {
                    std::cerr << "link_points number < 4 error "<< std::endl;
                    return false;
                }
            }
            else //配置到lane级
            {
                // 解析lane
                const libconfig::Setting &lanes = links[i].lookup(LANE_GROUP);
                
                for(int j=0; j<lanes.getLength(); j++)
                {
                    std::string lane_id, lane_length, lane_points;
                    int lane_type;
                    if(!(lanes[j].lookupValue(LANE_ID, lane_id) && 
                        lanes[j].lookupValue(LANE_TYPE, lane_type) &&
                        lanes[j].lookupValue(LANE_LENGTH, lane_length) &&
                        lanes[j].lookupValue(LANE_POINTS, lane_points)))
                    {
                        std::cerr << "parse lane lookupvalue error "<< std::endl;
                        return false;
                    }

                    Lane lane;
                    lane.lane_id = lane_id;
                    lane.lane_type = lane_type;
                    lane.lane_length = atof(lane_length.c_str());

                    std::vector<std::string> lane_pts;
                    SplitString(lane_points, ';', lane_pts);
                    for(uint k=0; k<lane_pts.size(); k+=2)
                    {
                        cv::Point p;
                        p.x = atoi(lane_pts[k].c_str());
                        p.y = atoi(lane_pts[k+1].c_str());
                        lane.lane_points.push_back(p);
                    }
                    
                    if(lane.lane_points.size() < 4)
                    {
                        std::cerr << "lane_points number < 4 error "<< std::endl;
                        return false;
                    }
                    link.lanes.push_back(lane);
                }
                link.lane_count = link.lanes.size();
            }

            config->nl_config.links.push_back(link);
        }
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        std::cout << "there is not link message in configuration file." << std::endl;
        // return false;
    } 

    return true;
}

void PrintCameraConfigMessage(AppConfig* config)
{
    //打印gie参数
    printf("******print config params*******\n");
    printf("Gie params:\n");
    printf("\tengine_file: %s\n\tmodel_file: %s\n\tnetwork_mode: %s\n\tbatch_size:%d\n\tclass_num:%d\n\tconf_thresh:%.2f\n\tnms_thresh: %.2f\n\tnpu use: %d\n", 
                config->gie_config.engine_file.c_str(), 
                config->gie_config.model_file.c_str(), 
                config->gie_config.network_mode.c_str(),
                config->gie_config.batch_size,
                config->gie_config.class_num,
                config->gie_config.conf_thresh,
                config->gie_config.nms_thresh,
                config->gie_config.npu
                );
    printf("\tclass_names: ");
    for(unsigned int i=0; i<config->gie_config.class_names.size(); i++)
    {
        printf("%s ", config->gie_config.class_names[i].c_str());
    }
    printf("\n");

    printf("\ntracker params:\n");
    printf("\tlow_thresh: %.2f\n\thign_thresh: %.2f\n\thigh_thresh_person: %.2f\n\thigh_thresh_motorbike: %.2f\n\tiou_thresh: %.2f\n", 
        config->tracker_config.low_thresh,
        config->tracker_config.high_thresh,
        config->tracker_config.high_thresh_person,
        config->tracker_config.high_thresh_motorbike,
        config->tracker_config.max_iou_thresh
    );

    printf("\nlocation params:\n");
    if(config->loc_config.enable)
    {
        printf("\tstd_meas_x: %.2f\n\tstd_meas_y: %.2f\n\tstd_state_ax: %.2f\n\tstd_state_ay: %.2f\n\tlock_velo: %.2f\n\tconverged_var: %.2f\n\tjson_file: %s\n\theading: %.2f\n\tlocated_long: %.8f\n\tlocated_lat: %.8f\n\tlocated_elev: %.2f\n\toffset_angle: %.2f\n",
            config->loc_config.std_meas_x,
            config->loc_config.std_meas_y,
            config->loc_config.std_state_ax,
            config->loc_config.std_state_ay,
            config->loc_config.lock_velo,
            config->loc_config.velo_converged_var,
            config->loc_config.calibrator_file.c_str(),
            config->loc_config.heading,
            config->loc_config.located_longitude,
            config->loc_config.located_latitude,
            config->loc_config.located_elevation,
            config->loc_config.located_offset_angle
        );
    }

    //打印NL_config参数
    printf("\nNL_config params:\n");
    printf("\tdevice_id: %s\n\tfontName: %s\n", config->nl_config.camera_sensor_id.c_str(), config->nl_config.font_file.c_str());

    for(unsigned int i=0 ;i<config->nl_config.roi_points.size(); i++)
    {
        printf("\troi%d: ", i);
        for(uint j=0; j<config->nl_config.roi_points[i].size(); j++)
        {
            printf("(%d, %d) ", config->nl_config.roi_points[i].at(j).x, config->nl_config.roi_points[i].at(j).y);
        }
        printf("\n");
    }

    // print plate roi 
    printf("\tplate rec roi: ");
    for(uint j=0; j<config->nl_config.plate_roi_points.size(); j++)
    {
        printf("(%d, %d) ", config->nl_config.plate_roi_points.at(j).x, config->nl_config.plate_roi_points.at(j).y);
    }
    printf("\n");
    
    //打印link相关信息
    printf("\nLink params:\n");
    printf("\tlink count: %lu\n", config->nl_config.links.size());
    for(unsigned int i=0; i<config->nl_config.links.size(); i++)
    {
        printf("\t%d---->link_id:%s  direction:%d  heading=%.2f  lane_count:%d  ", i,
                    config->nl_config.links[i].link_id.c_str(),
                    config->nl_config.links[i].direction,
                    config->nl_config.links[i].heading,
                    config->nl_config.links[i].lane_count
                    );
        
        if(config->nl_config.links[i].link_points.size() > 0)
        {
            printf("link_points: ");
            for(unsigned int j=0; j<config->nl_config.links[i].link_points.size(); j++)
            {
                printf("(%d, %d) ", config->nl_config.links[i].link_points[j].x, config->nl_config.links[i].link_points[j].y);
            }
        }
        printf("\n");
        for(int j=0; j<config->nl_config.links[i].lane_count; j++)
        {
            printf("\t\t%d--> lane_id:%s  length_lane:%.1f  lane_points:", j,
                        config->nl_config.links[i].lanes[j].lane_id.c_str(),
                        config->nl_config.links[i].lanes[j].lane_length
                        );
            for(unsigned int k=0; k<config->nl_config.links[i].lanes[j].lane_points.size(); k++)
            {
                printf("(%d, %d) ", config->nl_config.links[i].lanes[j].lane_points[k].x, config->nl_config.links[i].lanes[j].lane_points[k].y);
            }
            printf("\n");
        }
       
    }
    
    printf("*************************\n");
   
}

} // namespace camera
