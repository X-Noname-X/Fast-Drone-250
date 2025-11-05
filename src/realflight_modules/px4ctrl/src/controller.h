/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h> //自定义的消息类型
#include <queue>

#include "input.h"
#include <Eigen/Dense>

//期望状态结构体
struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	//构造函数实现
	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

//控制器输出结构体
struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame 机体坐标系相对于世界坐标系的姿态四元数
	Eigen::Quaterniond q;

	// Body rates in body frame 机体角速率（弧度/秒），在机体坐标系中表示
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust 归一化推力（集体质量归一化推力）
	double thrust;

	//Eigen::Vector3d des_v_real;
};


class LinearControl
{
public:
  LinearControl(Parameter_t &); //接受参数对象引用来初始化控制器
  //控制方法
  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      Controller_Output_t &u);
  //推力模型估计方法，用于映射推力到加速度的关系
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
  //重置推力映射关系
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0; //最小归一化集体推力常量

  // Thrust-accel mapping params 推力-加速度映射参数
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc); //根据期望加速度计算期望的集体推力信号
  double fromQuaternion2yaw(Eigen::Quaterniond q); //从四元数提取偏航角
};


#endif
