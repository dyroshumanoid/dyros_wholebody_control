#pragma once

#include <pinocchio/fwd.hpp>
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp> 
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>      
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/contact-jacobian.hpp>

#include "utils.hpp"

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"

#include <filesystem>

class WBMPC
{
public:
    WBMPC(RobotData &rd);

    void updateMPCSolverInput(const Eigen::VectorQVQd &q, const Eigen::VectorVQd &v, const int& current_tick);
    void solve();
    void retractStackedSolution(const Eigen::VectorXd& sol_x);

    void setWeights(const Eigen::VectorXd &Q, const Eigen::VectorXd &R);
    void setReferenceNominalPose(const Eigen::VectorQd q_nom_);
    void setWalkingEnable(const bool enable) { walking_enable = enable; }
    Eigen::VectorQd getWBMPCJointPositionSolution();
    Eigen::VectorQd getWBMPCJointVelocitySolution();
    Eigen::VectorQd getWBMPCJointTorqueSolution();
    double getMpcFrequency() const { return mpc_hz_; }


private:
    RobotData &rd_;

    const double mpc_hz_ = 50.0;

    Eigen::VectorXd stateIntegrate(const Eigen::VectorXd& x, const Eigen::VectorXd& dx);

    //--- MPC Info
    const int mpc_nodes = 10;
    int nf = 12;
    double current_time = 0.0;
    std::vector<int> nu_opt = {42, 42, 30, 30, 30, 30, 30, 30, 30, 30};
    std::vector<int> keep_joint_ids = {
        0, 1, 2, 3, 4, 5,
        6, 7, 8, 9, 10, 11
    };

    std::vector<Eigen::VectorXd> q_sol;
    std::vector<Eigen::VectorXd> v_sol;
    std::vector<Eigen::VectorXd> a_sol;
    std::vector<Eigen::VectorXd> force_sol;
    std::vector<Eigen::VectorXd> torque_sol;
    std::vector<Eigen::VectorXd> DX_prev;
    std::vector<Eigen::VectorXd> U_prev;

    std::vector<casadi::DM> input_mpc;
    Eigen::VectorXd output_mpc;  
    Eigen::VectorXd output_mpc_prev;  
    // Stacked MPC solution vector returned by CasADi solver, corresponds to `sol_x` from solver_function(*solver_params) in ocp.py

        Eigen::VectorXd x_init;

        double dt_min = 0.02; 
        double dt_max = 0.20;
        std::vector<double> dt_vec; 

        Eigen::MatrixXd contact_schedule;
        Eigen::MatrixXd swing_schedule;  

        double n_contacts = 2; 

        double gait_period = 1.0; 
        double swing_period = 0.0; 
        double swing_height = 0.08; 

        Eigen::Vector2d swing_vel_limits = Eigen::Vector2d(-0.3, 0.3);

        Eigen::VectorXd Q_diag;

        Eigen::VectorXd R_diag;

        Eigen::VectorXd base_vel_des;

        Eigen::VectorXd arm_vel_des;

        Eigen::VectorXd arm_force_des;

    //--- Pinocchio
    pinocchio::Model model_;
    pinocchio::Data data_;

    //--- Casadi
    casadi::Function solver;

    std::string current_path = std::filesystem::current_path().parent_path().string();
    std::string prefix_lib = current_path + "/catkin_ws/src/tocabi_cc/lib/";
    std::string lib_name = "libsolver_function.so";
    std::string lib_path = prefix_lib + lib_name; 

    //--- etc
    Eigen::VectorQd q_nom;
    bool walking_enable = false;
};