#include <iostream>
#include <string>

#include "vrep_bridge.h"
#include "mobile_controller.h"

#include "linux_terminal_tool.h"
// #define MODE(X,Y) case X: ac.setMode(Y); break;


int main()
{
	// // VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
	VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
	// VRepBridge vb((VRepBridge::ControlMode::CTRL_POSITION)
	const double hz = 1000 ;
	MobileController mc(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
	
		if (is_first)
		{
			vb.simLoop();
			vb.read();
			mc.readJoint(vb.getCurrentJointAngle(), vb.getCurrentJointVelocity(), vb.getCurrentJointTorque());
			mc.readBase(vb.getCurrentTaskPose(), vb.getCurrentTaskTwist(), vb.getCurrentTaskForce());			
			mc.setInitialJoint(vb.getCurrentJointAngle(), vb.getCurrentJointVelocity());
			mc.setInitialBase(vb.getCurrentTaskPose(), vb.getCurrentTaskTwist());			
			is_first = false;
			// ac.initPosition();
		}
		else
		{
			vb.read();
			mc.readJoint(vb.getCurrentJointAngle(), vb.getCurrentJointVelocity(), vb.getCurrentJointTorque());
			mc.readBase(vb.getCurrentTaskPose(), vb.getCurrentTaskTwist(), vb.getCurrentTaskForce());		
		}

		if (kbhit())
		{
			int key = getchar();
			switch (key)
			{
				// Implement with user input
				// MODE('i', "joint_ctrl_init")
				case '\t':
					if (is_simulation_run)
					{
						std::cout << "Simulation Pause" << std::endl;
						is_simulation_run = false;
					}
					else
					{
						std::cout << "Simulation Run" << std::endl;
						is_simulation_run = true;
					}
					break;
				case 'q':
					is_simulation_run = false;
					exit_flag = true;
					break;
				default:
					break;
			}
		}

		if (is_simulation_run) {
			mc.compute();
			// vb.setDesiredPosition(ac.getDesiredPosition());
			// vb.setDesiredTorque(ac.getDesiredTorque());
		
			// vb.write();
			vb.simLoop();
		}
	}
		
	return 0;
}
