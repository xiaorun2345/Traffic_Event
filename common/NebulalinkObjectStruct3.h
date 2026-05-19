/*
 *  NebulalinkObjectStruct3.h
 *  Created on: 2022.07.20
 *  Author: yueyingying
 *  Description: 雷达、视觉、融合 目标信息通用交互结构体,对应nebulalink.perceptron3.0.3
 *
 */
#ifndef __NEBULALINKOBJECTSTRUCT3_H__
#define __NEBULALINKOBJECTSTRUCT3_H__

#include <vector>
#include <string>
#include <stdint.h>
#include <memory>

using namespace std;
namespace perceptron {

// 感知目标列表 成员-目标的几何中心相对传感器坐标位置
typedef struct Point3 {
	float x;
	float y;
	float z;
}s_Point3;

// 感知目标列表 成员-图像坐标位置
typedef struct Point4 {
	int camera_x;
	int camera_y;
	int camera_w;
	int camera_h;
}s_Point4;

typedef struct PointGPS{
	double object_longitude;	// 经度,范围0-180
	double object_latitude;		// 纬度,范围0-90
	double object_elevation;	// 海拔,单位米
}s_PointGPS;

// 感知目标列表 成员-目标分轴速度,单位 m/s
typedef struct Speed3{
	float speed_x; 
	float speed_y;
	float speed_z;
}s_Speed3;

// 感知目标列表 成员-目标尺寸
typedef struct TargetSize{
	float object_length;	// 目标宽度, 单位 m.
	float object_width;		// 目标长度, 单位 m.
	float object_height;	// 目标高度， 单位 m.
}s_TargetSize;

typedef enum LaneType {
	LANE_TYPE_UNKNOWN = 0,			// 未知
    MOTOR_LANE = 1,					// 机动车道
    PEDESTRIAN_LANE = 2,			// 人行横道
	BIKE_LANE = 3					// 非机动车道
}e_LaneType;

typedef enum LaneDirect {
    CLOSE = 1,
    AWAY = 2
}e_LaneDirect;

// 目标大类类型
typedef enum ObjectType {
    OBJECT_TYPE_UNKNOWN = 0,	// 未知
    MOTOR = 1,					// 机动车
    NON_MOTOR = 2,				// 非机动车
    PEDESTRIAN = 3,				// 行人
    RSU = 4,					// RSU自身
	CONE = 5,                   // 锥桶
	THROWS = 6					// 抛洒物
}e_ObjectType;

typedef enum NS {
    N = 0,	// 北纬
    S = 1	// 南纬
}e_NS;

typedef enum EW {
    E = 0,	// 东经
    W = 1	// 西经
}e_EW;

// 目标四轴加速度
typedef struct Acc4Way {
	float acc4WayLon;	// Long：纵向加速度
	float acc4WayLat;	// Lat： 横向加速度
	float acc4WayVert;	// Vert：垂直加速度
	float acc4WayYaw;	// Yaw： 角加速度
}s_Acc4Way;

typedef struct TimeBase {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int millisecond;
}s_TimeBase;

// 经纬度坐标置信度
typedef struct PointGPS_CFD{
	double position_confidence;		// 经纬度置信度
	double plevation_confidence;	// 海拔置信度
}s_PointGPS_CFD;

// 四轴加速度置信度
typedef struct Motion_CFD {
    float speedCfd;		// 速度置信度
    float headingCfd;	// 航向角精度置信度
    float steerCfd;		// 方向盘转向角置信度
}s_Motion_CFD;

typedef enum VehicleType {
    VEHICLE_TYPE_UNKNOWN = 0,	// 未知目标类型
	MOTOR_CAR = 10,			// 乘用车默认类型*
	MOTOR_LIGHTTRUNk= 20,	// 轻型卡车默认类型*
	MOTOR_Truck = 25,		// 重型卡车默认类型*
	MOTOR_Truck_CNT4 = 29,	// 四轴不带拖车的重型卡车
	MOTOR_Truck_NT4 = 30,	// 四轴带单拖车的重型卡车
	MOTOR_MOTOBIKE = 40,	// 摩托车-包含三轮车摩托车
	MOTOR_BUS = 50,			// 公交车默认类型*
	MOTOR_EMERENCY = 60		// 应急车(消防,警用,救护)
}e_VehicleType;

typedef enum VehicleColor {
	VEHICLE_COLOR_UNKNOWN = 0, // 未知, 默认初始值
	WRITE = 1,		// 白
	GREY = 2,		// 灰
	YELLOW = 3,		// 黄
	CYAN = 4,		// 青
	RED = 5,		// 红
	VIOLET = 6,		// 紫
	GREEN = 7,		// 绿
	BLUE = 8,		// 蓝
	BROWN = 9,		// 棕
	BLACK = 10,		// 黑
	ORANGE = 11,	// 橙
	PINK = 12,		// 粉
	OTHER = 13		// 其他
}e_VehicleColor;

//车牌颜色
typedef enum PlateColor {
	PLATE_COLOR_UNKNOWN = 0, 	// 未知, 默认初始值
	PLATE_COLOR_BLUE_WHITE=1,   // 蓝底白字， 常用
	PLATE_COLOR_YELLOW_BLACK=2, // 黄底黑字， 大型货车，摩托车，教练车等
    PLATE_COLOR_WHITE_BLACK=3,	// 白底黑字，政法部门车辆
    PLATE_COLOR_BLACK_WHITE=4, 	// 黑底白字，不常见，外企车辆，港澳台车辆
	PLATE_COLOR_GREEN_WHITE=5,	// 绿底白字，农用车辆，如拖拉机等
	PLATE_COLOR_GREEN_BLACK=6	// 绿底黑字, 新能源车专用，电车
}e_PlateColor; 

// 尺寸置信度
typedef struct TargetSize_CFD {
	float object_width_cfd; 	// 目标宽度置信度
	float object_length_cfd;	// 目标长度置信度
	float object_height_cfd;	// 目标高度置信度
}s_TargetSize_CFD;

typedef struct Acc4Way_CFD {
	float lonAccConfidence;		// Long：纵向加速度置信度
    float latAccConfidence;		// Lat： 横向加速度置信度
    float vertAccConfidence;	// Vert：垂直加速度置信度
    float yawRateCon;			// Yaw： 角加速度置信度
}s_Acc4Way_CFD;

// 感知区域列表
typedef struct PointDesc {
    double p_longitude;				// 经度
    double p_latitude;				// 纬度
    double p_altitude;				// 海拔高度
    e_NS p_NS;						// 南北纬
    e_EW p_EW;						// 东西经
    float p_speed;					// 速度
    float p_heading;				// 航向角
    s_Acc4Way p_accel_4way;			// 目标四轴加速度
    double p_dis2end;				// 距结束点距离
    int64_t p_up_region_node_id;	// 上游节点ID
    int64_t p_down_region_node_id;	// 下游节点ID
    int p_relate_lane_id;			// 点所属车道
    int satellite_num;				// 定位点卫星数
	s_PointGPS_CFD p_pos_cfd;		// 位置置信度
	int  time_offset;			    // 当前描述时刻相对于参考时间点的偏差
}s_PointDesc;

// 轨迹点集合
typedef struct PathPoint {
	s_TimeBase pp_time; 	// 时间
    s_PointDesc pp_point;	// gps，speed, heading
}s_PathPoint;

typedef struct PathPlanning_PB {
	s_PointDesc pplan_pos;		// 轨迹点	
    float pplan_speed_cfd;		// 速度置信度
	float pplan_heading_cfd;	// 航向角置信度
	s_Acc4Way	 pplan_acce;	// 目标四轴加速度
	s_Acc4Way_CFD pplan_acce_cfd;	// 目标四轴加速度 置信度
	int estimated_time;			// 预计时间
	int time_confidence; 		// 时间置信度
}s_PathPlanning_PB;

// 目标行驶规划集合
typedef struct Planning_PB {
	int	duration;		// 持续时间
	int	confid;			
    string	driving_behavior;	// 驾驶行为
	vector<s_PathPlanning_PB>	path_planning;
}s_Planning_PB;

// 目标燃油类型列表
typedef enum FuelType {
	UNKNOWN_FUEL = 0,	// 未知类型, Gasoline Powered
	GASOLINE = 1,		// 汽油, Including blends
	ETHANOL = 2,		// 乙醇燃料
	DIESEL = 3,			// 柴油, All types
	ELECTRIC = 4,		// 电
	HYBRIDS = 5,		// 混合动力汽车, All types
	HYDROGEN = 6,		// 氢气
	NATGASLIQUID = 7,	// 液化天然气 Liquefied
	NATGASCOMP = 8,		// 压缩天然气 Compressed
	PROPANE = 9			// 丙烷气
}e_FuelType;

// 目标车辆档位状态列表
typedef enum PtcTranState{
	NEUTRAL = 0,		// Neutral
	PARK = 1,			// Park 
	FORWARD_GEAR = 2,	// Forward gears
	REVERSE_GEAR = 3,	// Reverse gears
	RESERVED1 = 4,
	RESERVED2 = 5,
	RESERVED3 = 6,
	UNAVAILABLE = 7		// not-equipped or unavailable value, Any related speed is relative to the vehicle reference frame used
}e_PtcTranState;

// 感知目标属性列表.
typedef struct Object {
	bool is_tracker = true;				// 物体是否被跟踪成功，如果没有，那么说明跟踪信息是不可靠的，一般都要求目标被追踪，字段暂且为1.
	float object_confidence = 0;		// 目标置信度，如果为int的话，最后给*0.01转换成float给出，并给出参考区间.
	string lane_id = ""; 				// 目标所在车道id.
	e_ObjectType object_class_type = e_ObjectType::OBJECT_TYPE_UNKNOWN;	// 目标种类.
	int object_id = 0;					// 目标Id.
	s_Point3 point3f = {0,0,0};			// 目标的几何中心相对传感器坐标位置，单位为米.
	s_Point4 point4f = {0,0,0,0};		// 图像坐标位置，1单位像素.
	float object_speed = 0;				// 目标速度，单位 m/s.
	bool vel_converged = false;			// 速度收敛标志
	s_Speed3 speed3f = {0,0,0};			// 目标分轴速度，单位 m/s.
	float object_acceleration = 0;		// 目标加速度, 单位 m/s2.
	s_TargetSize target_size = {0,0,0};	// 目标尺寸.
	s_PointGPS point_gps = {0,0,0};		// 目标经纬度坐标.
	e_NS object_NS = e_NS::N;			// 南北纬.
	e_EW object_WE = e_EW::E;			// 东西经.
	float object_direction = 0;			// 车辆车头朝向角度，与正北偏角，度为单位.
	float object_heading = 0;			// 行驶方向朝向航向角，与正北偏角, 以度为单位, 顺时针[0,360].
	int is_head_tail = 0;				// 车头还是车尾  1-车头，0-车尾
	e_LaneType lane_type = e_LaneType::LANE_TYPE_UNKNOWN;	// 目标所在车道类型.
	string plate_num = "";				// 车牌
	e_PlateColor ptc_veh_plate_color = e_PlateColor::PLATE_COLOR_UNKNOWN;// 车牌颜色
	string objects_identity = "";		// 全域目标id
	e_FuelType fuel_type = e_FuelType::UNKNOWN_FUEL;		// 目标燃油类型
	s_Acc4Way accel_4way = {0,0,0,0};	// 目标四轴加速度
	int64_t obj_time_stamp = 0;			// 目标时间戳-毫秒数
	int  ptc_sourcetype = 0;			// 目标数据来源
	s_TimeBase ptc_time_stamp = {0,0,0,0,0,0,0};				// 目标时间戳-带格式
	s_PointGPS_CFD ptc_gps_cfd = {0,0};							// 目标经纬度坐标置信度
	e_PtcTranState ptc_tran_state = e_PtcTranState::NEUTRAL;	// 目标车辆档位状态
	int ptc_angle = 0;					// 目标转向角
	s_Motion_CFD ptc_motion_cfd = {0,0,0};			// 目标运行状态置信度
	e_VehicleType ptc_veh_type = e_VehicleType::VEHICLE_TYPE_UNKNOWN;		// 目标车辆类型
	e_VehicleColor ptc_veh_color = e_VehicleColor::VEHICLE_COLOR_UNKNOWN;	// 目标车辆颜色
	s_TargetSize_CFD  ptc_size_cfd = {0,0,0};		// 目标尺寸置信度
	int ptc_Exttype = 0;				// 目标类型扩展
	float ptc_Exttype_cfd = 0;			// 目标类型扩展置信度
	s_Acc4Way_CFD ptc_accel_4way_cfd = {0,0,0,0};	// 目标四轴加速度置信度
	int ptc_status_duration = 0;		// 目标状态点偏差
	vector<s_PathPoint> ptc_pathpoint_history;		// 目标历史轨迹
	vector<s_Planning_PB> ptc_planning_list;		// 目标行驶规划集合
	vector<s_PointDesc> ptc_polygonPoint;			// 目标影响感知区域
	int ptc_satellite = 0;				// 目标所在点卫星数量
  	int ptc_regionid = 0;				// 目标所在点区域id
  	int ptc_nodeid = 0;					// 目标所在点区域内节点id
  	int ptc_laneid = 0;					// 目标所在点车道id
  	string ptc_link_name = "";			// 目标所在点道路名称
  	int ptc_link_width = 0;				// 目标所在点道路宽度
}s_Object;

typedef struct InfoEndLineValues{
	// 在离开线上的交通信息；
}s_InfoEndLineValues;

typedef struct InfoEntreLineValues{
	// 在进入线上的交通信息
}s_InfoEntreLineValues;

// 车道级lane 信息列表
typedef struct LaneJamSenseParams{
	string lane_id;				// 车道id*
	int lane_types;				// 车道类型
	float lane_sense_len;		// 车道长度，单位m
	int lane_direction;			// 车道方向
	float lane_avg_speed;		// 车道内车辆的平均速度*
	int lane_veh_num;			// 该车道不区分车型机动车总流量*
	float lane_space_occupancy;	// 车道空间占有率*
	int lane_queue_len;			// 车道排队长度
	int lane_count_time;		// 车道统计时间，单位秒*
	int lane_count_flow;		// 车道统计车数量*
	bool  lane_is_count;		// 是否统计成功
	int lane_ave_distance;		// 车道平均车间距
	int lane_cur_distance;		// 车道实时车间距
	float lane_time_occupancy;	// 车道时间占有率*
	s_InfoEntreLineValues lane_entre_info;	// 进入车道的交通信息	
	s_InfoEndLineValues   lane_end_info;	// 离开车道的交通信息
	int lane_num;				// 车道数量----17以下襄阳项目定义
	int lane_no;				// 车道编号
	int lane_peron_volume;		// 该车道行人流量
	int lane_no_motor_volume;	// 该车道非机动车流量
	int lane_minmotor_volume;	// 该车道小车总流量
	int lane_medmotor_volume;	// 该车道中车流量
	int lane_maxmotor_volume;	// 该车道大车流量
	int lane_pcu;				// 该车道交通当量(pcu)
	float lane_avspeed;			// 该车道平均速度（km/h）
	float lane_headway;			// 该车道平均车头时距（s）
	float lane_gap;				// 该车道平均车身间距（s）
	float lane_avdistance;		// 该车道平均车距(s)
	float lane_avstop;			// 该车道平均停车次数
	float lane_speed85;			// 该车道85%以上的车辆的最高速度（km/h）
	float lane_queueLength;		// 该车道来向车道排队长度（m）
	float lane_stopline;		// 该车道停止线
}s_LaneJamSenseParams;

// 单行驶方向路
typedef struct LinkJamSenseParams{
	string link_id;				// 道路id************************
	float  link_len;			// 道路长度,单位m
	float  link_avgspeed;		// 道路平均速度******************
	int  link_veh_num;			// 道路车辆数
	int  link_type;				// 道路类型
	int  link_direction;		// 道路方向
	float  link_space_occupancy;// 道路空间占有率
	float  link_time_occupancy;	// 道路时间占有率
	int  link_count_time;		// 道路统计时间，单位秒
	int  link_count_flow;		// 道路统计车数量
	bool   link_is_count;		// 是否统计成功*********************
	s_InfoEntreLineValues link_entre_info;	// 进入道路的交通信息
	s_InfoEndLineValues link_end_info;		// 离开道路的交通信息
	string link_deviceid;		// 道路编号--17以下襄阳项目定义
	int link_heading;			// 道路与道路入口中心点与正北方向的顺时针夹角。分辨率为0.0125°
	int link_phaseid;			// 相位ID，只参照直行相位定义
	string link_name;			// 道路名称，如：XXXXX路北方向入口
	int link_no;				// 道路编码
	s_PointGPS link_gps;		// 道路入口坐标(雷达点位坐标)
	int link_measnum;			// 检测线编号
	int link_num;				// 线圈数量
	int link_motor_volume;		// 该道路不区分车型机动车总流量************
	int link_peron_volume;		// 该道路行人流量
	int link_no_motor_volume;	// 该道路非机动车流量
	int link_minmotor_volume;	// 该道路小车总流量
	int link_medmotor_volume;	// 该道路中车流量
	int link_maxmotor_volume;	// 该道路大车流量
	int	link_pcu;				// 该道路交通当量(pcu)******************
	float link_avspeed;			// 该道路平均速度（km/h）
	float link_time_occupany;	// 该道路时间占有率（%）*****************
	float link_headway;			// 该道路平均车头时距（s）***************
	float link_gap;				// 该道路平均车身间距（s）***************
	float link_avdistance;		// 该道路平均车距(s)
	float link_avstop;			// 该道路平均停车次数********************
	float link_speed85;			// 该道路85%以上的车辆的最高速度（km/h）
	float link_queueLength;		// 该道路来向车道排队长度（m）************
	float link_stopline;		// 该道路平均速度（km/h）
	float link_space_occupany;	// 该道路空间占有率（%）******************
	vector<s_LaneJamSenseParams> road_lanelist;	// 感知车道级lane 统计信息
}s_LinkJamSenseParams;

// 感知路口级 统计信息列表
typedef struct OnLineValues{
	int cycleid;				// 线圈id
	s_PointGPS pos;				// 虚拟线的位置
	int vehnum;					// 线上车辆数
	float avgSpeed;				// 线上车的平均速度
	string cross_name;			// 路口名称----5以下为襄阳定义
	int cross_laneno;			// 路口编号
	int cross_volume;			// 该路口不区分车型机动车总流量
	int cross_peron_volume;		// 该路口非机动车流量
	int cross_no_motor_volume;	// 该路口大车流量
	int cross_minmotor_volume;	// 该路口小车总流量
	int cross_medmotor_volume;	// 该路口中车流量
	int cross_maxmotor_volume;	// 该路口大车流量
	int cross_pcu;				// 该路口交通当量(pcu)
	float cross_avspeed;		// 该路口平均速度（km/h）
	float cross_time_occupany;	// 该路口时间占有率（%）
	float cross_headway;		// 该路口平均车头时距（s）
	float cross_gap;			// 该路口平均车身间距（s）
	float cross_avdistance;		// 该路口平均车距(s)
	float cross_avstop;			// 该路口平均停车次数
	float cross_speed85;		// 该路口目标超过85%的目标速度（km/h）
	float cross_queueLength;	// 该路口来向车道排队长度（m）
	float cross_stopline;		// 该路口车道停止线
	float cross_space_occupany;	// 该路口空间占有率（%）
	vector<s_LinkJamSenseParams> road_linklist;	// 感知道路级link 信息统计
}s_OnLineValues;

typedef struct RelateLane{
	int rtl_lane_id = 1;            // 车道ID
}s_RelateLane;

// 关联路段属性
typedef struct RelateLinkDesc {
    int up_region_node_id;			// 上游节点ID   
    int down_region_node_id;		// 下游节点ID
	vector<s_RelateLane> rtl_lanes;	// 关联车道    
}s_RelateLinkDesc;

// 关联路径属性
typedef struct RelatePathDesc {
    vector<s_PointDesc> rpd_pathPoint;	// 点经纬度
    float rpd_radius;					// 关联半径
}s_RelatePathDesc;

// 事件来源
typedef enum RteSource {
	RTE_SOURCE_UNKNOWN = 0,		// 未知
	POLICE = 1,			// 警察
	GOVERMENT = 2,		// 政府
	METEOROLOGICAL = 3,	// 气象局
	INTERNET = 4,		// 网络
	DETECTION = 5		// 检测
}e_RteSource;

// 感知事件列表
typedef enum EventType {
	UNKNOWN = 0,
	LED_1 =1,
	LED_2 = 2,
	LED_3 = 3,
	LED_4 = 4,
	LED_5 = 5,
	LED_6 = 6,
	LED_7 = 7,
	LED_8 = 8,
	// 交通事故
	VEHICLE_BREAKDOWN_ACCIDENT = 101,		// 车辆故障
	PERSON_VEHICLE_ACCIDENT = 102,			// 人-车事故
	VEHICLE_VEHICLE_ACCIDENT = 103,			// 车-车事故
	VEHICLE_INFRASTRUCTURE_ACCIDENT = 104,	// 车-基础设施事故
	OTHER__ACCIDENT = 199,					// 其他交通事故
	// 交通灾害
	VEHICLE_FIRE_DISASTER = 201,		// 车辆火灾
	ROAD_FIRE_DISASTER = 202,			// 路面火灾
	DISASTER_RESERVED1 = 203,			// 预留
	DISASTER_RESERVED2 = 204,			// 预留
	INFRASTRUCTURE_FIRE_DISASTER = 205,	// 道路基础设施火灾
	GEOLOGICAL_DISASTER = 206,			// 地质灾害
	FLOOD_DISASTER = 207,				// 洪水灾害
	OTHRE_DISASTER = 299,				// 其他交通灾害
	// 交通气象
	RAIN = 301,				// 雨
	HAIL = 302, 			// 冰雹
	WEATHER_RESERVED = 303,	// 可扩展
	WIND = 304,				// 风
	FOG = 305,				// 雾
	HIGH_TEMPERATURE = 306,	// 高温
	SNOW = 308,				// 雪
	HAZE = 311,				// 霾
	SLEET = 397,			// 雨夹雪
	HUMITURE = 398,			// 温湿度
	STORM = 399,			// 沙尘暴
	// 路面状况
	ROAD_SPILLAGE = 401,		// 道路抛洒物
	PEDESTRIAN_INSTRUSION = 405,// 行人闯入
	ANIMAL_INSTRUSION = 406,	// 动物闯入
	SURFACE_WATER = 407,		// 路面积水
	ROAD_SLIPPERY = 408,		// 路面湿滑
	ICY_PAVEMENT = 409,			// 路面结冰
	ROAD_SNOW = 410,			// 路面积雪
	ROAD_DAMAGE = 411,			// 路面损毁
	OTHRE_ROAD = 499,			// 其他路面状况
	// 道路施工
	PRESENCE_CONSTRUCTION = 501,		// 占道施工
	ROAD_FRACTURE_CONSTRUCTION = 502,	// 断路施工
	EMERGENCY_RESCUE = 503,				// 救援抢修进行中
	OTHRE_CONSTRUCTION = 599,			// 其他道路施工状况
	// 活动
	COMMERCIAL_ACTIVITIES = 601,// 商业文体活动
	FOREIGN_ACTIVITIES = 602,	// 外交政务活动
	OTHRE_ACTIVITIES = 603,		// 其他活动
	// 重大事件
	GAS_EVENT = 701,			// 煤气事故
	RESERVED_EVENT1 = 702,		// 可扩展
	RESERVED_EVENT2 = 703, 	
	EXPLODE_EVENT = 704,		// 爆炸
	RESERVED_EVENT3 = 705,
	PUBLIC_VIOLENCE_EVENT = 706,// 公共暴力
	TRAFFIC_JAM_EVENT = 707,	// 交通拥堵
	FRONT_ROCKS_EVENT = 708,	// 前方陨石
	FRONT_LANDSLIDE_EVENT = 709,// 前方塌方
	OTHER_EVENT = 799,			// 其他重大事件
	// 其他
	OVERSPEED_VEHICLE = 901,	// 超速车辆
	SLOW_VEHICLE = 902,			// 慢行车辆
	STOP_VEHICLE = 903,			// 停驶车辆
	RETROGRADE_VEHICLE = 904,	// 逆行车辆 
	OCCUPANCY_EMERGENCY_LANE_VEHICLE = 905,	// 占用应急车道车辆
	AVOIDANCE_VEHICLE = 906,	// 大车避让
	ILLEGAL_LANE_CHANGE_VEHICLE = 907,		// 违规变道车辆
	LONG_ONLINE_DRIVING_VEHICLE = 908, 		// 长时间压线行驶车辆
	OTHER_VEHICLE = 998			// 其他
}e_EventType;

// 感知事件属性列表.
typedef struct Eventlist {
	int event_id;			// 事件ID:rte_id
	int event_status;		// 事件状态： 1-发生  2-解除
	e_EventType event_type;	// 交通事件类型.
	e_RteSource rte_source;	// 事件源
	s_PointGPS event_gps;	// 事件发生的经纬度-海拔
	float event_radius;		// 事件影响半径.
	string event_desc;		// 事件描述(根据具体事件可自定义)
	int event_priority;		// 事件优先级
	vector<s_RelateLinkDesc> linklist;	// 事件关联的link路径集合
	vector<s_RelatePathDesc> pathlist;	// 事件关联的lan级路径集合
	int event_confid;					// 事件置信度.
	s_TimeBase event_timestamp_start;	// 事件开始时间戳
	s_TimeBase event_timestamp_end;		// 事件结束时间戳
	vector< vector<s_Point4> > traj_points;// type,traj
}s_Eventlist;

// 南向设备状态数据列表.
typedef struct Heartlist { 
	int  device_status;			// rsu外接设备状态
	string err_device_id;		// rsu外接设备id
	int  err_code;				// rsu外接设备故障码
	int64_t  heart_time;		// rsu外接设备时间戳
	string err_desc;			// rsu外接设备故障描述
	int  err_level;				// rsu外接设备故障等级.
	int  err_device_type;		// rsu外接设备类型
	string err_device_version;	// rsu外接设备版本号
	float  device_temp;			// rsu外接设备温度
}s_Heartlist;

// polygon属性列表.
typedef struct DetectorRegion {
	vector<s_PointDesc> polygonPoint;	// 经纬度
}s_DetectorRegion;

// 道路障碍物类型
typedef enum ObstacleType{
	OBSTACLE_TYPE_UNKNOWN = 0,
	ROCKFALL = 1,			// 落石
	LANDSLIDE = 2,			// 塌方
	ANIMAL_INTRUSION = 3,	// 动物闯入
	LOQUID_SPILL = 4,		// 液体泄露
	GOODS_SCATTERED = 5,	// 物品散落
	TRAFFICCONE = 6,		// 锥筒
	SAFELY_TRIANGLE = 7,	// 三角牌
	TRAFFIC_ROADBLOCK = 8,	// 交通路障
	INSPECTION_SHAFT_WITHOUT_COVER = 9, // 没有盖的窨井
	UNKNOWN_FRAGMENTS = 10,		// 碎片
	UNKNOWN_HARD_OBJECT = 11,	// 硬物
	UNKNOWN_SOFT_OBJECT = 12	// 软物
}e_ObstacleType;

// obs属性列表.
typedef struct Obstacles {
	e_ObstacleType obstype;			// 障碍物 类型
    int obstype_cfd;				// 障碍物 类型置信度
	int obsId;						// 障碍物 id
	int obs_source;					// 障碍物 数据来源
    s_TimeBase obs_time_stamp;		// 障碍物 时间戳* 毫秒数或者UTC?
    s_PointDesc obs_gps;			// 障碍物 相对位置精度
    s_PointGPS_CFD obs_gps_cfd;		// 障碍物 相对位置精度置信度
	float obs_speed;				// 障碍物 速度
	float obs_speed_cfd;			// 障碍物 速度置信度
	float obs_heading;				// 障碍物 航向角
	float obs_heading_cfd;			// 障碍物 航向角置信度
	float obs_verSpeed;				// 障碍物 角加速度
	float obs_verSpeed_cfd;			// 障碍物 角加速度置信度
	s_Acc4Way  obs_accel_4way;		// 障碍物 四轴加速度
	s_TargetSize obs_size;			// 障碍物 尺寸大小
    s_TargetSize_CFD obs_size_cfd;	// 障碍物 尺寸大小置信度
	int    obs_tracking;			// 障碍物 追踪时间
	vector<s_PointDesc> obs_polygonPoint;	// 障碍物 影响区域点集合
}s_Obstacles;

// v2x网联车数据.
typedef struct V2XOBUs{
    s_PointDesc obu_point;		// 车辆基础数据
    float obu_wheel_angle;		// 转向轮角度
    int64_t obu_time_stamp;		// 时间戳
    int obu_veh_type;			// 车辆类型
	int obu_fuel_type;			// 车辆燃油类型
    int obu_light;				// 状态灯
    int obu_brake_state;		// 刹车状态
    int obu_veh_state;			// 车辆状态标志
	float obu_cfd;				// 目标置信度
	s_TargetSize obu_size;		// 车辆长宽高
	string obu_platenum;		// 网联车的车牌号
	string obu_deviceid;		// OBU SN 设备号
}s_V2XOBUs;

// 传感器数据类型
typedef enum SensorType{
	SensorType_UNKNOWN = 0,
	SensorType_CAMERA = 1, 			//相机
	SensorType_CAMERA_SHORT = 2, 	//短焦相机
	SensorType_CAMERA_LONG = 3,		//长焦相机
	SensorType_LIDAR = 4,			//激光雷达
	SensorType_RADAR = 5,			//微波雷达
}e_SensorType;

// 感知共享消息-结构化数据
typedef struct ObjectList {
	string devide_id;				// 设备号.
	bool  devide_is_true;			// 判断设备数据的有效性.
	int64_t time_stamp;				// 时间戳，到1970年1月1日0:00的毫秒数，int最多10位，只能用int64表示.
	e_SensorType sensor_type;
	int number_frame;				// 帧的序列号.
	s_PointGPS perception_gps;		// 感知设备安装位置的GPS信息.
	vector<s_Object> object_list;
	vector<s_LaneJamSenseParams> lane_jam_sense_params;		// lane级道路信息
	vector<s_LinkJamSenseParams> link_jam_sense_params;		// link级道路信息
	vector<s_OnLineValues> on_line_values;					// 交通路口统计类列表
	vector<s_Eventlist> event_list;							// 感知事件列表 rte
	vector<s_Heartlist> heart_list;							// RSU南向感知设备状态
	vector<s_DetectorRegion> polygon;						// 感知区域 polygon
	vector<s_Obstacles> obstacle;							// 感知障碍物列表 obstacle
	vector<s_V2XOBUs> v2x_obus;								// rsu探测的v2x网联车数据
}s_ObjectList;

/** s_Object **/
typedef std::shared_ptr<s_Object> s_ObjectPtr;
typedef std::shared_ptr<const s_Object> s_ObjectConstPtr;

/** s_ObjectList **/
typedef std::shared_ptr<s_ObjectList> s_ObjectListPtr;
typedef std::shared_ptr<const s_ObjectList> s_ObjectListConstPtr;

} // namespace

#endif // NEBULALINKOBJECTSTRUCT_H
