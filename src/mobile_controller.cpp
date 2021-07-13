#include "mobile_controller.h"


MobileController::MobileController(const double hz) : 
tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
{
  veh_.Add_Solid(  0.000,  0.000,  XR_Mv, XR_Iv);
  veh_.Add_Caster(0,  XR_l,  XR_w, 0*M_PI_2); // front left 
  veh_.Add_Caster(1,  XR_l, -XR_w, 0*M_PI_2); // front right
  veh_.Add_Caster(2, -XR_l,  XR_w, 0*M_PI_2); // rear left
  veh_.Add_Caster(3, -XR_l, -XR_w, 0*M_PI_2); // rear right
  
  kp_ = Eigen::Matrix3d::Identity()*40.0;
  kv_ = Eigen::Matrix3d::Identity()*80.0;

  kp_(2,2) = 300.0;
  kv_(2,2) = 1300.0;

  Kjp_ = Eigen::Matrix<double, 8, 8>::Identity()*100.0;
  Kjv_ = Eigen::Matrix<double, 8, 8>::Identity()*0.01;

  std::cout<<"Load Mobile Controller"<<std::endl;
}

void MobileController::compute()
{
  // get low passed input
  doLowPassFilter();

  // update parameters
  veh_.JointRad(q_);
  veh_.Fill_C(C_);
  veh_.Fill_J(J_);
  veh_.Fill_Jcp(Jcp_);
  Jt_ = J_.transpose();
  Jcpt_ = Jcp_.transpose();
  q_dot_hat_ = C_*x_dot_; // ignore any slip for dynamics
  // veh_.Dyn(q_dot_hat_, x_dot_(2));
  veh_.Dyn(q_dot_, x_dot_(2));
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
    
    Eigen::Vector3d x_target;    
    double dT = 3.0;
    x_target << 0.10, 0.10, M_PI/36;
  
  // for position controller
    Eigen::Vector2d f_star;
    for(int i = 0; i < 2; i ++)
    {
      Eigen::Vector3d x_temp;
      
      x_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + dT, x_init_(i), x_dot_init_(i), 0.0, x_target(i), 0.0, 0.0);     
      
      xd_(i)      = x_temp(0);
      xd_dot_(i)  = x_temp(1);
      xd_ddot_(i) = x_temp(2);
    }

    f_star = xd_ddot_.head<2>() + kp_.block<2,2>(0,0)*(xd_.head<2>() - x_.head<2>()) + kv_.block<2,2>(0,0)*(xd_dot_.head<2>() - x_dot_.head<2>());
    
    // for orientation controller
    double m_star;
    Eigen::Vector3d delphi_delta;
    Eigen::Matrix3d rot_init, rot, rot_d, rot_target;

    Eigen::Vector3d r_temp;    
    r_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + dT, x_init_(2), x_dot_init_(2), 0.0, x_target(2), 0.0, 0.0);     
    xd_(2)      = r_temp(0);
    xd_dot_(2)  = r_temp(1);
    xd_ddot_(2) = r_temp(2);

    // rot_init = DyrosMath::getRotationZ(x_init_(2));
    // rot_target = DyrosMath::getRotationZ(x_target(2));

    rot_d = DyrosMath::getRotationZ(xd_(2));
    rot = DyrosMath::getRotationZ(x_(2));
    delphi_delta = DyrosMath::getPhi(rot, rot_d);

    m_star = xd_ddot_(2) + kp_(2,2)*delphi_delta(2) + kv_(2,2)*(xd_dot_(2) - x_dot_(2));

    f_star_zero_.head<2>() = f_star;
    f_star_zero_(2) = m_star;

    VectorQd q_target;
    double heading;
    heading = atan2(xd_(1), xd_(0));
    q_target << heading, q_(1), heading, q_(3), heading, q_(5), heading, q_(7); 
    // To hold caster modules
    
    taud_ = Jcpt_*(Lambda_*f_star_zero_ + Mu_);
    
    //***** DEBUG *****//
    // VectorQd q_dot_debug;
    // q_dot_debug = C_*xd_dot_;
    // debug << q_dot_debug.transpose()<<std::endl;

  }
  else if(control_mode_  == "steer_control")
  {
    VectorQd q_target;
    q_target << -M_PI/2, q_init_(1), -M_PI/2, q_init_(3), -M_PI/2, q_init_(5), -M_PI/2, q_init_(7);

    for(int  i = 0; i < 2*N_CASTERS; i++)
    {
      Eigen::Vector3d q_temp;
      q_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + 3.0,  q_init_(i), 0.0, 0.0, q_target(i), 0.0, 0.0);     
      qd_(i)      = q_temp(0);
      qd_dot_(i)  = q_temp(1);
      qd_ddot_(i) = q_temp(2);
    }
    // define gains for each steers and wheels
    for(int i = 0; i < N_CASTERS; i++)
    {
      int j = 2*i;
      Kjp_(j,j) = 10.0;
      Kjp_(j+1,j+1) = 7.5;
      Kjv_(j,j) = 1.0;
      Kjv_(j+1,j+1) = 0.5;
    }
    // Kjp_ = Eigen::Matrix<double, 8, 8>::Identity()*10.0;

    taud_ = qd_ddot_ + Kjp_*(qd_ - q_) + Kjv_*(qd_dot_ -  q_dot_);
  }

  else if (control_mode_ == "wheel_control")
  {
    VectorQd q_target;
    double wheel_vel, wheel_ang, dt;
    dt = 6.0;    
    wheel_vel = 0.5; // rad/s //30.0*M_PI/180; // 360 deg/s
    wheel_ang = wheel_vel*dt;

    q_target << q_init_(0), wheel_ang, q_init_(2), wheel_ang, q_init_(3), wheel_ang, q_init_(4), wheel_ang;

    for(int  i = 0; i < 2*N_CASTERS; i++)
    {
      Eigen::Vector3d q_temp;
      q_temp = DyrosMath::quinticSpline(play_time_, control_start_time_, control_start_time_ + dt,  q_init_(i), 0.0, 0.0, q_target(i), 0.0, 0.0);     
      qd_(i)      = q_temp(0);
      qd_dot_(i)  = q_temp(1);
      qd_ddot_(i) = q_temp(2);
    }
    // define gains for each steers and wheels
    for(int i = 0; i < N_CASTERS; i++)
    {
      int j = 2*i;
      Kjp_(j,j) = 5.0;
      Kjp_(j+1,j+1) = 10.0;
      Kjv_(j,j) = 1.0;
      Kjv_(j+1,j+1) = 0.5;
    }
    tau_null_ = 100.0*(q_init_ - q_);
    tau_null_ = (Eigen::Matrix8d::Identity() - Jcpt_*Jcp_)*tau_null_;
    taud_ = qd_ddot_ + Kjp_*(qd_ - q_) + Kjv_*(qd_dot_ -  q_dot_);
  }


  else if (control_mode_ == "none")
  {
    tau_null_ = 5.0*(q_init_ - q_) + 0.1*(q_dot_init_ - q_dot_init_);
    tau_null_ = (Eigen::Matrix8d::Identity() - Jcpt_*Jcp_)*tau_null_;

    f_star_zero_ = 20*(x_init_ - x_) + 10*(x_dot_init_ - x_dot_);
    taud_ = Jcpt_*(Lambda_*f_star_zero_ + Mu_) + tau_null_;    
  }

  prev();

  // if(control_mode_ != "none") saveState();
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

  q_prev_ = q_;
  q_dot_prev_ = q_dot_;

  x_prev_ = x_;
  x_dot_prev_ = x_dot_;

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
  // q_init_ = q;
  // q_dot_init_ = q_dot;
  q_init_.setZero();
  q_dot_init_.setZero();
}

void MobileController::setInitialBase(const Vector3d &x, const Vector3d &x_dot)
{
  // x_init_ = x;
  // x_dot_init_ = x_dot;
  x_init_.setZero();
  x_dot_init_.setZero();
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
      std::cout << "q init:\t";
      std::cout << std::fixed << std::setprecision(3) << q_init_.transpose()*180/M_PI << std::endl;

      // std::cout << "q   :\t";
      // std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
      // std::cout << "qd  :\t";
      // std::cout << std::fixed << std::setprecision(3) << qd_.transpose() << std::endl;
      std::cout << "q dot:\t";
      std::cout << std::fixed << std::setprecision(3) << q_dot_.transpose() << std::endl;
      std::cout << "x init :\t";
      std::cout << std::fixed << std::setprecision(3) << x_init_.transpose() << std::endl;
      std::cout << "x   :\t";
      std::cout << std::fixed << std::setprecision(3) << x_.transpose() << std::endl;
      std::cout << "xd  :\t";
      std::cout << std::fixed << std::setprecision(3) << xd_.transpose() << std::endl;
      std::cout << "taud:\t";
      std::cout << std::fixed << std::setprecision(3) << taud_.transpose() << std::endl;
      // std::cout << "t_n :\t";
      // std::cout << std::fixed << std::setprecision(3) << tau_null_.transpose() << std::endl;
      std::cout << "fd  :\t";
      std::cout << std::fixed << std::setprecision(3) << f_star_zero_.transpose() << std::endl;
      std::cout << "L   :\n";
      std::cout << std::fixed << std::setprecision(3) << Lambda_ << std::endl;      
      std::cout << "Fd  :\t";
      std::cout << std::fixed << std::setprecision(3) << (Lambda_*f_star_zero_).transpose() << std::endl;
      // std::cout << "Jcpt  :\n";
      // std::cout << std::fixed << std::setprecision(3) << Jcpt_ << std::endl;
      std::cout << "Jt  :\n";
      std::cout << std::fixed << std::setprecision(3) << Jt_ << std::endl;
      // std::cout << "C   :\n";
      // std::cout << std::fixed << std::setprecision(3) << C_ << std::endl;
      // std::cout << "Mu  :\n";
      // std::cout << std::fixed << std::setprecision(3) << Mu_ << std::endl;
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

void MobileController::prev()
{
  q_prev_ = q_;
  q_dot_prev_ = q_dot_filter_;

  x_prev_ = x_;
  x_dot_prev_ = x_dot_filter_;

}

void MobileController::doLowPassFilter()
{
  // q_ = DyrosMath::lowPassFilter(q_, q_prev_, 1/hz_, 50.0);
  q_dot_filter_ = DyrosMath::lowPassFilter(q_dot_, q_dot_prev_, 1/hz_, 10.0);
  // x_ = DyrosMath::lowPassFilter(x_, x_prev_, 1/hz_, 50.0);
  x_dot_filter_ = DyrosMath::lowPassFilter(x_dot_, x_dot_prev_, 1/hz_, 10.0);

  q_dot_ = q_dot_filter_;
  x_dot_ = x_dot_filter_;
}

// ----------------------------------------------------

