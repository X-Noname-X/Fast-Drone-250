#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
//Eigen 是一个C++模板库，专门用于线性代数运算  
#include <Eigen/Dense>   // 包含核心矩阵和向量操作

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

class RC_Data_t
{
public:
//模式状态变量
  double mode; //当前飞行模式值
  double gear; // 当前起落架/档位值
  double reboot_cmd;  //当前重启命令值
  double last_mode;   //上一次飞行模式值
  double last_gear;   // 上一次起落架值
  double last_reboot_cmd; // 上一次重启命令值
//初始化标志
  bool have_init_last_mode{false}; // 上次模式值是否已初始化  {}赋初值
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
//遥控器通道数据
  double ch[4];// 存储4个主要控制通道：roll, pitch, yaw, throttle

  mavros_msgs::RCIn msg; // 原始的RC输入消息
  ros::Time rcv_stamp;    // 接收时间戳

//模式判断标志
  bool is_command_mode; // 当前是否处于命令模式
  bool enter_command_mode; // 是否刚刚进入命令模式
  bool is_hover_mode; // 当前是否处于悬停模式 
  bool enter_hover_mode;
  bool toggle_reboot;  // 是否触发重启

//常量定义
  static constexpr double GEAR_SHIFT_VALUE = 0.75;// 档位切换阈值
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;//API模式阈值
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;//重启阈值
  static constexpr double DEAD_ZONE = 0.25; //死区范围

  RC_Data_t();
  void check_validity(); //检查数据有效性
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg); //数据输入：处理新的RC输入消息，解析模式切换等逻辑
  bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  //确保Eigen对象正确内存对齐
  Eigen::Vector3d p;  //3D位置向量 (x, y, z)
  Eigen::Vector3d v;  //三维速度向量 (vx, vy, vz)
  Eigen::Quaterniond q;  //四元数表示姿态
  Eigen::Vector3d w;     // 角速度

  nav_msgs::Odometry msg; //原始里程计消息
  ros::Time rcv_stamp;
  bool recv_new_msg;  //是否接收到新信息

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d a;  //线性加速度 (ax, ay, az)

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t
{
public:
  mavros_msgs::State current_state;  //当前飞行状态
  mavros_msgs::State state_before_offboard;  //进入offboard模式前的状态

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);  //统一的数据处理接口
};

//扩展状态类
class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp;

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

#endif
