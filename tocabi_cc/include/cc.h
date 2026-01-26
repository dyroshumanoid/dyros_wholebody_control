#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <list>
#include <iomanip> 

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"

#include "control_manager.h"
#include "task_manager.h"
#include "kin_wbc.h"
#include "dyn_wbc.h"
#include "cbf_manager/cbf_manager.h"
#include "teleop_manager.h"
#include "utils.h"

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::Subscriber joy_sub_;
    ros::Subscriber xbox_joy_sub_;

    double move_forward = 0.0;
    double move_lateral = 0.0;
    double rotate_yaw =   0.0;

    Eigen::Vector3d v_cmd;
    Eigen::Vector3d w_cmd;

    void loadParams();
    double traj_time_, pelv_dist_, hand_dist_, step_length_, step_yaw_, foot_height_, step_duration_, dsp_duration_;

    //--- Thread
    void computeSlow();
    void computeFast();
    void computePlanner();

    
    //--- Robot Model
    RigidBodyDynamics::Model model;  
    ControlManager cm_;
    TaskManager tm_;
    KinWBC kin_wbc_;  
    DynWBC dyn_wbc_;  
    CbfManager cbf_mgr_;  
    TeleOperationManager teleop_;  
    
    RobotData &rd_;
    RobotData rd_cc_;

    Eigen::VectorXd Kp;  Eigen::VectorXd Kd; 
    Eigen::VectorXd Kp_virtual; Eigen::VectorXd Kd_virtual; 
    Eigen::VectorQd joint_pos_limit_l_;
    Eigen::VectorQd joint_pos_limit_h_;
    Eigen::VectorQd joint_vel_limit_l_;
    Eigen::VectorQd joint_vel_limit_h_;

    //--- Initial Values
    Eigen::VectorQd q_init_;
    Eigen::VectorQd q_init_des;
    void CustomControllerInit();
    void moveInitialPose();

    //--- Test Function
    TaskMotionType motion_mode_ = TaskMotionType::None;
    CbfType cbf_mode_ = CbfType::None;

    Eigen::VectorQd torque_pd;
    Eigen::VectorQd torque_idn;
    Eigen::VectorQd torque_sum;

    //--- Data Exchange
    void pubDataFromSlowToFast();
    void subDataFromSlowToFast();
    void pubDataFromFastToSlow();
    void subDataFromFastToSlow();

    Eigen::VectorQd torque_idn_fast;
    Eigen::VectorQd torque_idn_container;

    Eigen::MatrixVVd M_fast;
    Eigen::MatrixVVd M_container;
    Eigen::VectorVQd G_fast;
    Eigen::VectorVQd G_container;
    Eigen::MatrixXd J_C_fast;
    Eigen::MatrixXd J_C_container;
    Eigen::VectorVQd qddot_cmd_container;
    Eigen::VectorVQd qddot_cmd_fast;
    Eigen::Vector12d contact_wrench_cmd_container;
    Eigen::Vector12d contact_wrench_cmd_fast;

    std::atomic<bool> atb_control_command_update_{false};
    std::atomic<bool> atb_torque_update_{false};

    void applyTorqueSmoothingOnce(Eigen::VectorQd &torque_target);

private:
    Eigen::VectorQd ControlVal_;
    double hz_ = 2000;
    bool is_joy_enable = false;

    // Fast loop is ready to run after the first slow-loop update
    bool fast_loop_ready = false;
    bool control_mode_changed = false;
    bool is_6_init = true;
    bool is_7_init = true;
};