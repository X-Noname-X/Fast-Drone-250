#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
	//嵌入结构体
	//控制增益
	struct Gain
	{
		double Kp0, Kp1, Kp2;  //位置比例(P) 对应x,y,z轴
		double Kv0, Kv1, Kv2;  //速度比例(P)
		double Kvi0, Kvi1, Kvi2;  //速度积分(I)
		double Kvd0, Kvd1, Kvd2;  //速度微分(D)
		double KAngR, KAngP, KAngY; //角度比例，对应 横滚、俯仰、偏航
	};
	// 旋翼阻力参数
	struct RotorDrag
	{
		double x, y, z; //阻力系数
		double k_thrust_horz; //水平推力系数
	};
	//定义各传感器/控制消息的超时时间
	struct MsgTimeout
	{
		double odom; //里程计
		double rc;  //遥控器
		double cmd; //控制命令
		double imu; //陀螺仪
		double bat; //电池状态
	};
	// 推力映射模型
	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	};
	//布尔值标识各通道是否需要反向
	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle; //油门
	};
	//自动起降
	struct AutoTakeoffLand
	{
		bool enable; //使能
		bool enable_auto_arm; //使能自动解锁
		bool no_RC; //是否无需遥控器
		double height; //目标高度
		double speed; //起降速度
	};
	//用结构体定义变量
	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;

	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;

	bool use_bodyrate_ctrl;
	// bool print_dbg;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);
	void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif
