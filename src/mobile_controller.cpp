#include "mobile_controller.h"


MobileController::MobileController(const double hz) : 
tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
{
  veh_.Add_Solid(  0.000,  0.000,  XR_Mv, XR_Iv);
  veh_.Add_Caster(0,  XR_l,  XR_w, 0*M_PI_2); // front left 
  veh_.Add_Caster(1,  XR_l, -XR_w, 0*M_PI_2); // front right
  veh_.Add_Caster(2, -XR_l,  XR_w, 0*M_PI_2); // rear left
  veh_.Add_Caster(3, -XR_l, -XR_w, 0*M_PI_2); // rear right
  
  std::cout<<"Load Mobile Controller"<<std::endl;
}



void MobileController::compute()
{
  // update parameters
  veh_.JointRad(q_);
  veh_.Fill_C(C_);
  q_dot_hat_ = C_*x_dot_;
  veh_.Dyn(q_dot_hat_, x_dot_(2));
  Lambda_ = veh_.Lambda_;
  Mu_ = veh_.Mu_;

//   if (is_mode_changed_)
//   {
//     is_mode_changed_ = false;

//     control_start_time_ = play_time_;

//     for(int i=0; i<2; i++)
//     {
//       arm_state_[i].q_0 = arm_state_[i].q;
//       arm_state_[i].qdot_0 = arm_state_[i].qdot;

//       arm_state_[i].xp_0 = arm_state_[i].xp;
//       arm_state_[i].xr_0 = arm_state_[i].xr;
//     }
//   }

//   if (control_mode_ == "joint_ctrl_init")
//   {
//     VectorQd target_position;
//     target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0;
//   }
//   else
//   {
//     for(int i=0; i<2; i++)
//     {
//       torque_desired_.segment<7>(i*7) = arm_state_[i].g;
//     }
//   }

  printState();

  tick_++;
  play_time_ = tick_ / hz_;	// second
}

void MobileController::printState()
{
  // TODO: Modify this method to debug your code

  static int DBG_CNT = 0;
  if (DBG_CNT++ > hz_ / 2.)
  {
    DBG_CNT = 0;

      std::cout << "q   :\t";
      std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
      std::cout << "x   :\t";
      std::cout << std::fixed << std::setprecision(3) << x_.transpose() << std::endl;
      // std::cout << "qd  :\t";
      // std::cout << std::fixed << std::setprecision(3) << qd_.transpose() << std::endl;
      // std::cout << "t desired:\t";
    //		cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << endl;
    //		cout << "x        :\t";
    //		cout << x_.transpose() << endl;
    //		cout << "R        :\t" << endl;
    //		cout << std::fixed << std::setprecision(3) << rotation_ << endl;

    //		Matrix<double, 6, 7>j;
    //    j <<  j_.block <3, kDOF>(0, 0),
    //      j_2_.block<3, kDOF>(0, 0);

    //		cout << "hw 3-1 jacobian:" << endl;
    //		cout << j << endl;

    //		Vector6d x;
    //		x << x_, x_2_;

    //		cout << "hw 3-1 x:\t" << x.transpose() << endl;
  }
}

// Controller Core Methods ----------------------------

// void MobileController::setMode(const std::string & mode)
// {
//   is_mode_changed_ = true;
//   control_mode_ = mode;
//   cout << "Current mode (changed) : " << mode << endl;
// }


void MobileController::readJoint(const VectorQd &q, const VectorQd &q_dot, const VectorQd &tau)
{
    q_ = q;
    q_dot_ = q_dot;
    tau_ = tau;
}

void MobileController::readBase(const Vector3d &x, const Vector3d &x_dot, const Vector3d &f)
{
    x_ = x;
    x_dot_ = x_dot;
    f_ = f;
}

void MobileController::setInitialJoint(const VectorQd &q, const VectorQd &q_dot)
{
  q_init_ = q;
  q_dot_init_ = q_dot;
}

void MobileController::setInitialBase(const Vector3d &x, const Vector3d &x_dot)
{
  x_init_ = x;
  x_dot_init_ = x_dot;
}


// ----------------------------------------------------

