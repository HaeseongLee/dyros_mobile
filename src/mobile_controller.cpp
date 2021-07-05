#include "mobile_controller.h"


MobileController::MobileController(const double hz) : 
tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
{
  veh_.Add_Solid(  0.000,  0.000,  XR_Mv, XR_Iv);
  veh_.Add_Caster(0,  XR_l,  XR_w, 0*M_PI_2); // front left 
  veh_.Add_Caster(1,  XR_l, -XR_w, 0*M_PI_2); // front right
  veh_.Add_Caster(2, -XR_l,  XR_w, 0*M_PI_2); // rear left
  veh_.Add_Caster(3, -XR_l, -XR_w, 0*M_PI_2); // rear right
  
  kp_ = Eigen::Matrix3d::Identity()*15.0;
  kv_ = Eigen::Matrix3d::Identity()*0.01;

  Kjp_ = Eigen::Matrix<double, 8, 8>::Identity()*100.0;;
  Kjv_ = Eigen::Matrix<double, 8, 8>::Identity()*0.01;

  std::cout<<"Load Mobile Controller"<<std::endl;
}

void MobileController::compute()
{
  // update parameters
  veh_.JointRad(q_);
  veh_.Fill_C(C_);
  veh_.Fill_Jcp(J_);
  Jt_ = J_.transpose();
  q_dot_hat_ = C_*x_dot_; // ignore any slip for dynamics
  veh_.Dyn(q_dot_hat_, x_dot_(2));
  Lambda_ = veh_.Lambda_;
  Mu_ = veh_.Mu_;

  if (is_mode_changed_)
  {
    is_mode_changed_ = false;
    control_start_time_ = play_time_;
    init();  
  }

  if(control_mode_ == "op_control")
  {
    Vector3d x_target;
    x_target << 0.1, 0.1, 0.0;//M_PI/2;

    for(int i = 0; i < 3; i ++)
    {
      Eigen::Vector3d x_temp;
      
      x_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + 1.0, x_init_(i), 0.0, 0.0, x_target(i), 0.0, 0.0);     
      
      xd_(i)      = x_temp(0);
      xd_dot_(i)  = x_temp(1);
      xd_ddot_(i) = x_temp(2);
    }

    f_star_zero_ = xd_ddot_ + kp_*(xd_ - x_) + kv_*(xd_dot_ - x_dot_);
    taud_ = Jt_*(Lambda_*f_star_zero_ + Mu_);
    
  }
  else if(control_mode_  == "steer_control")
  {
    VectorQd q_target;
    q_target << -M_PI/2, q_init_(1), -M_PI/2, q_init_(3), -M_PI/2, q_init_(5), -M_PI/2, q_init_(7);

    for(int  i = 0; i < 2*N_CASTERS; i++)
    {
      Eigen::Vector3d q_temp;
      q_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + 1.0,  q_init_(i), 0.0, 0.0, q_target(i), 0.0, 0.0);     
      qd_(i)      = q_temp(0);
      qd_dot_(i)  = q_temp(1);
      qd_ddot_(i) = q_temp(2);
    }
    Kjp_ = Eigen::Matrix<double, 8, 8>::Identity()*300.0;;
    taud_ = qd_ddot_ + Kjp_*(qd_ - q_) + Kjv_*(qd_dot_ -  q_dot_);
  }

  if (control_mode_ == "wheel_control")
  {
    VectorQd q_target;
    double wheel_vel, wheel_ang, dt;
    dt = play_time_ - control_start_time_;

    wheel_vel = 60.0*M_PI/180; // 360 deg/s
    wheel_ang = wheel_vel*dt;
    
    qd_ << q_init_(0), wheel_ang, q_init_(2), wheel_ang, q_init_(3), wheel_ang, q_init_(4), wheel_ang;
    qd_dot_ << 0.0, wheel_vel, 0.0, wheel_vel, 0.0, wheel_vel, 0.0, wheel_vel;
    qd_ddot_.Zero();

    for(int i = 0 ; i < N_CASTERS; i ++)
    {
      int j = 2*i;
      Kjp_(j,j) = 10.0;      
      Kjp_(j+1,j+1) = 100.0;
      Kjv_(j+1,j+1) = 0.1;
    }
    taud_ = qd_ddot_ + Kjp_*(qd_ - q_) + Kjv_*(qd_dot_ -  q_dot_);
  }


  if(control_mode_ != "none") saveState();
  printState();

  tick_++;
  play_time_ = tick_ / hz_;	// second
}

void MobileController::init()
{
  q_init_ = q_;
  q_dot_init_ = q_dot_;

  x_init_ = x_;
  x_dot_init_ = x_dot_;  
}

// Controller Core Methods ----------------------------

void MobileController::setMode(const std::string & mode)
{
  is_mode_changed_ = true;
  control_mode_ = mode;
  std::cout << "Current mode (changed) : " << mode << std::endl;
}
VectorQd MobileController::setDesiredJointTorque(){ return taud_; }

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


void MobileController::printState()
{
  // TODO: Modify this method to debug your code

  static int DBG_CNT = 0;
  if (DBG_CNT++ > hz_ / 10.)
  {
    DBG_CNT = 0;
      std::cout <<"-----------------------"<<std::endl;
      std::cout << "run time :\t";
      std::cout << std::fixed << std::setprecision(3) << play_time_- control_start_time_ << std::endl;
      std::cout << "q   :\t";
      std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
      std::cout << "x   :\t";
      std::cout << std::fixed << std::setprecision(3) << x_.transpose() << std::endl;
      std::cout << "xd  :\t";
      std::cout << std::fixed << std::setprecision(3) << xd_.transpose() << std::endl;
      std::cout << "taud:\t";
      std::cout << std::fixed << std::setprecision(3) << taud_.transpose() << std::endl;
      std::cout << "fd  :\t";
      std::cout << std::fixed << std::setprecision(3) << f_star_zero_.transpose() << std::endl;
      std::cout << "Jt  :\n";
      std::cout << std::fixed << std::setprecision(3) << Jt_ << std::endl;
      std::cout << "L   :\n";
      std::cout << std::fixed << std::setprecision(3) << Lambda_ << std::endl;
      std::cout << "Mu  :\n";
      std::cout << std::fixed << std::setprecision(3) << Mu_ << std::endl;
  }
}

void MobileController::saveState()
{
  pcv_q << q_.transpose() *180/M_PI << std::endl;
  pcv_q_dot << q_dot_.transpose()*180/M_PI << std::endl;

  pcv_qd << qd_.transpose() *180/M_PI << std::endl;
  pcv_qd_dot << qd_dot_.transpose()*180/M_PI << std::endl;
  pcv_taud << taud_.transpose() << std::endl;

  pcv_x << x_.transpose() << std::endl;
  pcv_x_dot << x_dot_.transpose() << std::endl;

  pcv_xd << xd_.transpose() << std::endl;
  pcv_xd_dot << xd_dot_.transpose() << std::endl;
  pcv_fd << f_star_zero_.transpose() << std::endl; 
}
// ----------------------------------------------------

