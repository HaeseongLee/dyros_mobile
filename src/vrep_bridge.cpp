#include "vrep_bridge.h"

VRepBridge::VRepBridge(ControlMode mode)
{
	control_mode_ = mode;
	simInit();
	getHandle();
	taud_.setZero();
}
VRepBridge::~VRepBridge()
{
	simxStopSimulation(clientID_, simx_opmode_oneshot_wait);
	simxFinish(clientID_);
}

bool VRepBridge::simConnectionCheck()
{
	return (simxGetConnectionId(clientID_) != -1);
}
void VRepBridge::simLoop()
{
	tick_++;
	simxSynchronousTrigger(clientID_);
}
void VRepBridge::simxErrorCheck(simxInt error)
{
	std::string errorMsg;
	switch (error)
	{
		case simx_error_noerror:
			return;	// no error
			break;
		case simx_error_timeout_flag:
			errorMsg = "The function timed out (probably the network is down or too slow)";
			break;
		case simx_error_illegal_opmode_flag:
			errorMsg = "The specified operation mode is not supported for the given function";
			break;
		case simx_error_remote_error_flag:
			errorMsg = "The function caused an error on the server side (e.g. an invalid handle was specified)";
			break;
		case simx_error_split_progress_flag:
			errorMsg = "The communication thread is still processing previous split command of the same type";
			break;
		case simx_error_local_error_flag:
			errorMsg = "The function caused an error on the client side";
			break;
		case simx_error_initialize_error_flag:
			errorMsg = "simxStart was not yet called";
			break;
		default:
			errorMsg = "Unknown error.";
			break;
	}

	std::cout << "[ERROR] An error is occured. code = " << error << std::endl;
	std::cout << " - Description" << std::endl;
	std::cout << " | " << errorMsg << std::endl;

	throw std::string(errorMsg);
}

void VRepBridge::simInit()
{
  simxFinish(-1);
#ifndef DO_NOT_USE_SHARED_MEMORY
  clientID_ = simxStart("127.0.0.1", SHARED_MEMORY_PORT, true, true, 2000, 5);
#elif
  clientID_ = simxStart("127.0.0.1", 19997, true, true, 2000, 5);
#endif
	if (clientID_ < 0)
	{
		throw std::string("Failed connecting to remote API server. Exiting.");
	}

	simxErrorCheck(simxStartSimulation(clientID_, simx_opmode_oneshot_wait));
	simxErrorCheck(simxSynchronous(clientID_, true));

	std::cout << "[INFO] V-Rep connection is established." << std::endl;

}

void VRepBridge::write()
{
	switch (control_mode_)
	{
		case CTRL_POSITION:
		{
			for (size_t i = 0; i < JOINT_DOF; i++)
				{
					simxSetJointTargetPosition(clientID_, motorHandle_[i], qd_(i), simx_opmode_streaming);
				}
				break;
		}
		case CTRL_TORQUE:
		{
			for (size_t i = 0; i < JOINT_DOF; i++)
			{
				simxFloat velocityLimit;

				if (taud_(i) >= 0.0)
					velocityLimit = 10e10f;
				else
					velocityLimit = -10e10f;

				simxSetJointTargetVelocity(clientID_, motorHandle_[i], velocityLimit, simx_opmode_streaming);
				simxSetJointForce(clientID_, motorHandle_[i], static_cast<float>(abs(taud_(i))), simx_opmode_streaming);				
			}	
			break;
		}
	}
}
void VRepBridge::read()
{
	for (size_t i = 0; i < JOINT_DOF; i++)
	{
		simxFloat data;
		simxGetJointPosition(clientID_, motorHandle_[i], &data, simx_opmode_streaming);
		q_(i) = data;
		simxGetObjectFloatParameter(clientID_, motorHandle_[i], 2012, &data, simx_opmode_streaming);
		q_dot_(i) = data;
	}

	simxFloat p[3], r[3], v[3], w[3];
		
	simxGetObjectPosition(clientID_, baseHandle_, -1, p, simx_opmode_streaming);
	simxGetObjectOrientation(clientID_, baseHandle_, -1, r, simx_opmode_streaming);
	simxGetObjectVelocity(clientID_, baseHandle_, v, w, simx_opmode_streaming);

	x_ << p[0], p[1], r[2];
	x_dot_ << v[0], v[1], w[2];

	// std::cout<<x_.transpose()<<std::endl;
	// std::cout<<x_dot_.transpose()<<std::endl;
}

void VRepBridge::setDesiredTaskPose(const Eigen::Matrix<double, TASK_DOF, 1> &xd) 	   { xd_ = xd; } 
void VRepBridge::setDesiredTaskTwist(const Eigen::Matrix<double, TASK_DOF, 1> &xd_dot) { xd_dot_ = xd_dot; } 
void VRepBridge::setDesiredTaskForce(const Eigen::Matrix<double, TASK_DOF, 1> &fd)	   { fd_ = fd; } 
const Eigen::Matrix<double, TASK_DOF, 1> & VRepBridge::getCurrentTaskPose()	{ return x_; }
const Eigen::Matrix<double, TASK_DOF, 1> & VRepBridge::getCurrentTaskTwist(){ return x_dot_; }
const Eigen::Matrix<double, TASK_DOF, 1> & VRepBridge::getCurrentTaskForce(){ return f_; }

void VRepBridge::setDesiredJointAngle(const Eigen::Matrix<double, JOINT_DOF, 1> &qd)		{ qd_ = qd; } 
void VRepBridge::setDesiredJointVelocity(const Eigen::Matrix<double, JOINT_DOF, 1> &qd_dot)	{ qd_dot_ = qd_dot; } 
void VRepBridge::setDesiredJointTorque(const Eigen::Matrix<double, JOINT_DOF, 1> &taud)		{ taud_ = taud; } 

const Eigen::Matrix<double, JOINT_DOF, 1> & VRepBridge::getCurrentJointAngle() 	 { return q_; }
const Eigen::Matrix<double, JOINT_DOF, 1> & VRepBridge::getCurrentJointVelocity(){ return q_dot_; }
const Eigen::Matrix<double, JOINT_DOF, 1> & VRepBridge::getCurrentJointTorque()	 { return tau_; }


void VRepBridge::getHandle()
{
	std::cout << "[INFO] Getting handles." << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "front_left_steer_joint", &motorHandle_[0], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named front_left_steer_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "front_left_rotate_joint", &motorHandle_[1], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named front_left_rotate_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "front_right_steer_joint", &motorHandle_[2], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named front_right_steer_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "front_right_rotate_joint", &motorHandle_[3], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named front_right_rotate_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "rear_left_steer_joint", &motorHandle_[4], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named rear_left_steer_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "rear_left_rotate_joint", &motorHandle_[5], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named rear_left_rotate_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "rear_right_steer_joint", &motorHandle_[6], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named rear_right_steer_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "rear_right_rotate_joint", &motorHandle_[7], simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named rear_right_rotate_joint" << std::endl;

	simxErrorCheck(simxGetObjectHandle(clientID_, "dyros_mobile", &baseHandle_, simx_opmode_oneshot_wait));
	std::cout << "[INFO] Getting a handle named dyros_mobile" << std::endl;

	std::cout << "[INFO] The handle has been imported." << std::endl;
}
