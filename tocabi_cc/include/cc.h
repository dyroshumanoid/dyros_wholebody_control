#pragma once
#include "mpc_solver.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>



class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();
    double getMpcFrequency() { return wb_mpc_.getMpcFrequency(); }

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;

    void loadParams();

    //--- Thread
    void computeSlow();
    void computeFast();
    void computeThread3();
    void computePlanner();


    RobotData &rd_;
    RobotData rd_cc_;

    //--- Initial Values
    Eigen::VectorQd q_init_;
    Eigen::VectorQd q_init_des;

    void mapGlobalToBase();
    void applyTorqueSmoothingOnce(Eigen::VectorQd &torque_target);
private:
    WBMPC wb_mpc_;

    Eigen::VectorQd ControlVal_;
    double hz_ = 2000.0;
    double lpf_cutoff_ = 50.0;

    std::mutex mpc_mutex;

    //--- Robot State for MPC
    Eigen::VectorQVQd q_container_;
    Eigen::VectorQVQd q_;
    Eigen::VectorVQd  v_container_;
    Eigen::VectorVQd  v_;

    //--- Robot Control Input for MPC
    bool mpc_update = false;
    int mpc_count = 0;

    Eigen::VectorQd q_mpc_;
    Eigen::VectorQd v_mpc_;
    Eigen::VectorQd a_mpc_;
    Eigen::VectorQd torque_mpc_;

    Eigen::VectorQd q_mpc_prev_;
    Eigen::VectorQd v_mpc_prev_;
    Eigen::VectorQd a_mpc_prev_;
    Eigen::VectorQd torque_mpc_prev_;

    Eigen::VectorQd q_mpc_container_;
    Eigen::VectorQd v_mpc_container_;
    Eigen::VectorQd a_mpc_container_;
    Eigen::VectorQd torque_mpc_container_;

    Eigen::VectorQd q_mpc_interpol_;
    Eigen::VectorQd v_mpc_interpol_;
    Eigen::VectorQd torque_mpc_interpol_;
    double mpc_update_time_ = 0.0;
};