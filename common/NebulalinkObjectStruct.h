/*
 *  NebulalinkObjectStruct.h
 *  Created on: 2021.6.21
 *  Author: nebulalink sensor
 *  Description: 雷达、视觉、融合 目标信息通用交互结构体
 *
 */
#ifndef NEBULALINKOBJECTSTRUCT_H
#define NEBULALINKOBJECTSTRUCT_H

#include <vector>
#include <string>

using namespace std;
 
typedef struct Point3 {
	float x;
	float y;
	float z;
}s_Point3;

typedef struct Point4 {
	int camera_x;
	int camera_y;
	int camera_w;
	int camera_h;
}s_Point4;

typedef struct PointGPS{
	double object_longitude;  // 经度，单位度，v2x国标规定为double类型.
	double object_latitude;   // 维度， 单位度，v2x国标规定为double类型.
	double object_altitude;  // 海拔，单位米；
}s_PointGPS;

typedef struct Speed3{
	float speed_x; //单位 m/s
	float speed_y;
	float speed_z;
}s_Speed3;

typedef struct TargetSize{
	float object_length;  // 目标宽度, 单位 m.
	float object_width; // 目标长度, 单位 m.
	float object_height; // 目标高度， 单位 m.
}s_TargetSize;

typedef enum LaneType {
    FOOTWALK = 0,    // 人行道
    MOTORWAY = 1,    // 机动车道
	BICYCLELANE = 2 // 非机动车道
}e_LaneType;

typedef enum LaneDirect {
    CLOSE = 0,
    AWAY = 1
}e_LaneDirect;

// 目标类型,按照协议
typedef enum Type {
    UNKNOWN = 0, 	// 未知
    PEDESTRIAN = 1, // 行人
    BICYCLE = 2,  	// 非机动车
    CAR = 3, 		// 小汽车
    TRUCK = 4,  	// 货车，如果不能区分，就把所有机动车归为CAR = 3.
	BUS = 5, 		// 公交车，如果不能区分，就把所有机动车归为CAR = 3.
	MOTOBIKE = 6, 	// 摩托车，如果不能区分，归为PEDESTRIAN = 1；
	TRICYCLE = 7 	// 三轮车
}e_Type;

typedef enum NS {
    N = 0, 	// 北纬
    S = 1	// 南纬
}s_NS;

typedef enum EW {
    E = 0, 	// 东经
    W = 1	// 西经
}s_EW;

// 默认初始值
typedef struct Object {
	bool is_tracker=true; 			// 物体是否被跟踪成功，如果没有，那么说明跟踪信息是不可靠的，一般都要求目标被追踪，字段暂且为1.
	float object_confidence=0;  // 目标置信度，如果为int的话，最后给*0.01转换成float给出，并给出参考区间.
	string lane_id=""; 			// 目标所在车道id.
	e_Type object_class_type=e_Type::UNKNOWN;  // 目标种类.
	int object_id=2;  		// 目标Id.
	s_Point3 point3f={0,0,0};  		// 目标的几何中心相对传感器坐标位置，单位为米.
	s_Point4 point4f={0,0,0,0}; 			// 图像坐标位置，1单位像素.
	float object_speed=0; 		// 目标速度，单位 m/s.
	s_Speed3 speed3f={0,0,0}; 			// 目标分轴速度，单位 m/s.
	float object_acceleration=0;  // 目标加速度, 单位 m/s2.
	s_TargetSize target_size={0,0,0}; 	// 目标尺寸.
	s_PointGPS point_gps={0,0,0}; 		// 目标经纬度坐标.
	s_NS object_NS=s_NS::N; 		// 南北纬.
	s_EW object_WE=s_EW::E; 		// 东西经.
	float object_direction=0; 	// 车辆车头朝向角度，与正北偏角，度为单位.
	float object_heading=0; 	// 行驶方向朝向航向角，以度为单位.
	int is_head_tail=0; 		// 0车头，1车尾.
	e_LaneType lane_type=e_LaneType::MOTORWAY; 		// 目标所在车道类型.
}s_Object;

typedef struct InfoEndLineValues{
	// 在离开线上的交通信息；
}s_InfoEndLineValues;

typedef struct InfoEntreLineValues{
	// 在进入线上的交通信息
}s_InfoEntreLineValues;

// 单车道
typedef struct LaneJamSenseParams{
	string lane_id="";      	// 车道id，编号
	char type='0';				// 车道类型
	float sense_len=0; 	 	// 车道长度，单位m
	e_LaneDirect direction=e_LaneDirect::CLOSE;		// 车道方向
	float avg_speed=0; 	 	// 车道内车辆的平均速度
	int veh_num=0; 		 	// 车道内车辆总数
	float space_occupancy=0; 	// 该车道空间占有率
	int queue_len=0; 	   	// 排队长度
	int count_time=0; 		// 统计时间，单位秒
	int count_flow=0; 		// 统计车数量
	bool is_count=0; 		// 是否统计成功
	s_InfoEndLineValues info_end_line_values;     // 离开线上的交通信息
	s_InfoEntreLineValues info_entre_line_values; // 在进入线上的交通信息
}s_LaneJamSenseParams;

// 单行驶方向路
typedef struct LinkJamSenseParams{
	string road_id=""; 		// 道路id，编号
	float sense_len=0; 		// 车道长度，单位m
	float avg_speed=0; 		// 平均速度
	int veh_num=0; 			// 车辆数
	e_LaneType type=e_LaneType::MOTORWAY; 			// 车道类型
	e_LaneDirect direction=e_LaneDirect::CLOSE;    	// 车道方向
	float space_occupancy=0; 	// 空间占有率
	int count_time=0; 		// 统计时间，单位秒
	int count_flow=0; 		// 统计车数量
	bool is_count=0; 		// 是否统计成功
	s_InfoEndLineValues info_end_line_values; 	 // 离开线上的交通信息
	s_InfoEntreLineValues info_entre_line_values; // 在进入线上的交通信息
}s_LinkJamSenseParams;

typedef struct ObjectList {
  string devide_id="";  	    // 设备号.
  bool  devide_is_true=true;     // 判断设备数据的有效性.
  long long time_stamp=0;  	    // 时间戳，到1970年1月1日0:00的毫秒数，sint最多10位，只能用int64表示.
  int number_frame=0;  	    // 帧的序列号.
  vector<s_Object> object_list;
  vector<s_LaneJamSenseParams> lane_jam_sense_params;
  vector<s_LinkJamSenseParams> link_jam_sense_params;

}s_ObjectList;

#endif // NEBULALINKOBJECTSTRUCT_H
