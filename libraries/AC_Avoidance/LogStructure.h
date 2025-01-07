#pragma once

#include <AP_Logger/LogStructure.h>

// 避障相关的日志ID定义
#define LOG_IDS_FROM_AVOIDANCE \
    LOG_OA_BENDYRULER_MSG, \
    LOG_OA_DIJKSTRA_MSG, \
    LOG_SIMPLE_AVOID_MSG, \
    LOG_OD_VISGRAPH_MSG

// @LoggerMessage: OABR
// @Description: Object avoidance (Bendy Ruler) diagnostics
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of BendyRuler currently active
// @Field: Act: True if Bendy Ruler avoidance is being used
// @Field: DYaw: Best yaw chosen to avoid obstacle
// @Field: Yaw: Current vehicle yaw
// @Field: DP: Desired pitch chosen to avoid obstacle
// @Field: RChg: True if BendyRuler resisted changing bearing and continued in last calculated bearing
// @Field: Mar: Margin from path to obstacle on best yaw chosen
// @Field: DLt: Destination latitude
// @Field: DLg: Destination longitude
// @Field: DAlt: Desired alt above EKF Origin
// @Field: OLt: Intermediate location chosen for avoidance
// @Field: OLg: Intermediate location chosen for avoidance
// @Field: OAlt: Intermediate alt chosen for avoidance above EKF origin

// 弯曲标尺避障日志结构体
struct PACKED log_OABendyRuler {
    LOG_PACKET_HEADER;
    uint64_t time_us;          // 系统启动后的时间(微秒)
    uint8_t type;             // 当前激活的弯曲标尺类型
    uint8_t active;           // 弯曲标尺避障是否激活
    uint16_t target_yaw;      // 避障选择的最佳偏航角
    uint16_t yaw;             // 当前车辆偏航角
    uint16_t target_pitch;    // 避障选择的目标俯仰角
    bool resist_chg;          // 弯曲标尺是否抵抗改变方向并继续使用上次计算的方向
    float margin;             // 最佳偏航角下路径到障碍物的边距
    int32_t final_lat;        // 目标纬度
    int32_t final_lng;        // 目标经度
    int32_t final_alt;        // EKF原点以上的目标高度
    int32_t oa_lat;          // 避障选择的中间位置纬度
    int32_t oa_lng;          // 避障选择的中间位置经度
    int32_t oa_alt;          // EKF原点以上避障选择的中间高度
};

// @LoggerMessage: OADJ
// @Description: Object avoidance (Dijkstra) diagnostics
// @Field: TimeUS: Time since system startup
// @Field: State: Dijkstra avoidance library state
// @Field: Err: Dijkstra library error condition
// @Field: CurrPoint: Destination point in calculated path to destination
// @Field: TotPoints: Number of points in path to destination
// @Field: DLat: Destination latitude
// @Field: DLng: Destination longitude
// @Field: OALat: Object Avoidance chosen destination point latitude
// @Field: OALng: Object Avoidance chosen destination point longitude

// Dijkstra避障日志结构体
struct PACKED log_OADijkstra {
    LOG_PACKET_HEADER;
    uint64_t time_us;         // 系统启动后的时间(微秒)
    uint8_t state;            // Dijkstra避障库状态
    uint8_t error_id;         // Dijkstra库错误条件
    uint8_t curr_point;       // 计算路径中的当前目标点
    uint8_t tot_points;       // 到目标路径的总点数
    int32_t final_lat;        // 目标纬度
    int32_t final_lng;        // 目标经度
    int32_t oa_lat;          // 避障选择的目标点纬度
    int32_t oa_lng;          // 避障选择的目标点经度
};

// @LoggerMessage: SA
// @Description: Simple Avoidance messages
// @Field: TimeUS: Time since system startup
// @Field: State: True if Simple Avoidance is active
// @Field: DVelX: Desired velocity, X-Axis (Velocity before Avoidance)
// @Field: DVelY: Desired velocity, Y-Axis (Velocity before Avoidance)
// @Field: DVelZ: Desired velocity, Z-Axis (Velocity before Avoidance)
// @Field: MVelX: Modified velocity, X-Axis (Velocity after Avoidance)
// @Field: MVelY: Modified velocity, Y-Axis (Velocity after Avoidance)
// @Field: MVelZ: Modified velocity, Z-Axis (Velocity after Avoidance)
// @Field: Back: True if vehicle is backing away

// 简单避障日志结构体
struct PACKED log_SimpleAvoid {
  LOG_PACKET_HEADER;
  uint64_t time_us;          // 系统启动后的时间(微秒)
  uint8_t state;             // 简单避障是否激活
  float desired_vel_x;       // 期望X轴速度(避障前)
  float desired_vel_y;       // 期望Y轴速度(避障前)
  float desired_vel_z;       // 期望Z轴速度(避障前)
  float modified_vel_x;      // 修改后X轴速度(避障后)
  float modified_vel_y;      // 修改后Y轴速度(避障后)
  float modified_vel_z;      // 修改后Z轴速度(避障后)
  uint8_t backing_up;        // 车辆是否在后退
};

// @LoggerMessage: OAVG
// @Description: Object avoidance path planning visgraph points
// @Field: TimeUS: Time since system startup
// @Field: version: Visgraph version, increments each time the visgraph is re-generated
// @Field: point_num: point number in visgraph
// @Field: Lat: Latitude
// @Field: Lon: longitude

// 避障路径规划可见性图点日志结构体
struct PACKED log_OD_Visgraph {
  LOG_PACKET_HEADER;
  uint64_t time_us;          // 系统启动后的时间(微秒)
  uint8_t version;           // 可见性图版本,每次重新生成时递增
  uint8_t point_num;         // 可见性图中的点编号
  int32_t Lat;              // 纬度
  int32_t Lon;              // 经度
};

// 避障相关的日志结构定义
#define LOG_STRUCTURE_FROM_AVOIDANCE \
    { LOG_OA_BENDYRULER_MSG, sizeof(log_OABendyRuler), \
      "OABR","QBBHHHBfLLiLLi","TimeUS,Type,Act,DYaw,Yaw,DP,RChg,Mar,DLt,DLg,DAlt,OLt,OLg,OAlt", "s--ddd-mDUmDUm", "F-------GGBGGB" , true }, \
    { LOG_OA_DIJKSTRA_MSG, sizeof(log_OADijkstra), \
      "OADJ","QBBBBLLLL","TimeUS,State,Err,CurrPoint,TotPoints,DLat,DLng,OALat,OALng", "s----DUDU", "F----GGGG" , true }, \
    { LOG_SIMPLE_AVOID_MSG, sizeof(log_SimpleAvoid), \
      "SA",  "QBffffffB","TimeUS,State,DVelX,DVelY,DVelZ,MVelX,MVelY,MVelZ,Back", "s-nnnnnn-", "F--------", true }, \
     { LOG_OD_VISGRAPH_MSG, sizeof(log_OD_Visgraph), \
      "OAVG", "QBBLL", "TimeUS,version,point_num,Lat,Lon", "s--DU", "F--GG", true},
