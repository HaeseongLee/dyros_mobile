#pragma once

#include <iostream>
#include <iomanip>
#include <Eigen/Dense>
#include <memory>
#include <fstream>
#include "math_type_define.h"
#include "pcv_mtx_utils.h"
#include "vehicle.h"

class MobileController
{
    public:
        MobileController(const double hz); 
        ~MobileController(){};

        void readJoint(const VectorQd &q, const VectorQd &q_dot, const VectorQd &tau);
        void readBase(const Vector3d &x, const Vector3d &x_dot, const Vector3d &f);

        void setInitialJoint(const VectorQd &q, const VectorQd &q_dot);
        void setInitialBase(const Vector3d &x, const Vector3d  &x_dot);

        void compute();

        Vehicle veh_;

        // JT-SPACE PARAMETERS(i.e., steer/wheel joint)        
        VectorQd q_, q_dot_, q_ddot_;   // phi, rho
        VectorQd q_init_, q_dot_init_;  // init values
        VectorQd qd_, qd_dot, qd_ddot_;
        VectorQd tau_, taud_;
        VectorQd q_dot_hat_; // computed by Jacobian
        
        // OP-SPACE PARAMETERS(i.e., mobile base)
        Eigen::Vector3d x_, x_dot_, x_ddot_; // x, y, theta
        Eigen::Vector3d x_init_, x_dot_init_;
        Eigen::Vector3d xd_, xd_dot_, xd_ddot_;
        Eigen::Vector3d f_, fd_;

        // DYNAMICS PARAMETERS
        MatrixQtd J_;
        MatrixQd Jt_;
        Matrix3d Lambda_;
        Vector3d Mu_;
        MatrixQd C_; // wheel constraint matrix

        unsigned long tick_;
        double play_time_;
        double hz_;
        double control_start_time_;

        std::string control_mode_;
        bool is_mode_changed_;


    private:        
        void printState();

};
