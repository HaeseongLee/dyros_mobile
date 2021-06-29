#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>

extern "C"
{
#include "extApi.h"
}

#ifndef DO_NOT_USE_SHARED_MEMORY
#ifndef SHARED_MEMORY_PORT
#define SHARED_MEMORY_PORT 19997//-3
#endif
#endif

#define JOINT_DOF 8 // fl steer, fl wheel, fr steer, fr wheel, rl steer, rl wheel, rr steer, rr wheel
#define TASK_DOF 3  // X, Y, Theta

class VRepBridge
{
public:
	// TODO: Modify the value of kDOF to fit your application
	enum ControlMode
	{
		CTRL_POSITION,
		CTRL_TORQUE,		
	};

	VRepBridge(ControlMode mode = CTRL_POSITION);
	~VRepBridge();

	bool simConnectionCheck();
	void simLoop();

	void write();
	void read();

	// parameters in the operational space
	void setDesiredTaskPose(const Eigen::Matrix<double, TASK_DOF, 1> &xd);
	void setDesiredTaskTwist(const Eigen::Matrix<double, TASK_DOF, 1> &xd_dot);
	void setDesiredTaskForce(const Eigen::Matrix<double, TASK_DOF, 1> &fd);

	const Eigen::Matrix<double, TASK_DOF, 1> &getCurrentTaskPose();
	const Eigen::Matrix<double, TASK_DOF, 1> &getCurrentTaskTwist();
	const Eigen::Matrix<double, TASK_DOF, 1> &getCurrentTaskForce();

	// paramesters in the joint sapce
	void setDesiredJointAngle(const Eigen::Matrix<double, JOINT_DOF, 1> &qd);
	void setDesiredJointVelocity(const Eigen::Matrix<double, JOINT_DOF, 1> &qd_dot);
	void setDesiredJointTorque(const Eigen::Matrix<double, JOINT_DOF, 1> &taud);

	const Eigen::Matrix<double, JOINT_DOF, 1> &getCurrentJointAngle();
	const Eigen::Matrix<double, JOINT_DOF, 1> &getCurrentJointVelocity();
	const Eigen::Matrix<double, JOINT_DOF, 1> &getCurrentJointTorque();


	const size_t getTick() { return tick_; }

private:
	Eigen::Matrix<double, TASK_DOF, 1> xd_;
	Eigen::Matrix<double, TASK_DOF, 1> xd_dot_;
	Eigen::Matrix<double, TASK_DOF, 1> fd_;
	Eigen::Matrix<double, TASK_DOF, 1> x_;
	Eigen::Matrix<double, TASK_DOF, 1> x_dot_;
	Eigen::Matrix<double, TASK_DOF, 1> f_;

	Eigen::Matrix<double, JOINT_DOF, 1> qd_;
	Eigen::Matrix<double, JOINT_DOF, 1> qd_dot_;
	Eigen::Matrix<double, JOINT_DOF, 1> taud_;
	Eigen::Matrix<double, JOINT_DOF, 1> q_;
	Eigen::Matrix<double, JOINT_DOF, 1> q_dot_;
	Eigen::Matrix<double, JOINT_DOF, 1> tau_;

	simxInt clientID_;
	simxInt motorHandle_[JOINT_DOF]; /// < Depends on simulation envrionment
	simxInt baseHandle_;

	size_t tick_{0};

	ControlMode control_mode_;

	void simxErrorCheck(simxInt error);
	void simInit();
	void getHandle(); /// < Depends on simulation envrionment
};
