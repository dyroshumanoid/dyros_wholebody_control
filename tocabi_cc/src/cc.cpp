#include "cc.h"

using namespace TOCABI;

std::string current_path = std::filesystem::current_path().parent_path().string();
std::string data_path = current_path + "/catkin_ws/src/tocabi_cc/log/";

ofstream calc_time_checker(data_path + "cc_calc_time_log.txt");
ofstream mpc_qpos_checker(data_path + "mpc_qpos_log.txt");
ofstream current_qpos_checker(data_path + "current_qpos_log.txt");
ofstream mpc_torque_checker(data_path + "mpc_torque_log.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd), wb_mpc_(rd)
{
    //--- ROS Node Handle
    nh_cc_.setCallbackQueue(&queue_cc_);
    ControlVal_.setZero();
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::computeSlow()
{
    queue_cc_.callAvailable(ros::WallDuration());

    static VectorVQd q_dot_virtual_lpf;
    for (int i = 0; i < MODEL_DOF_VIRTUAL; i++)
    {
        q_dot_virtual_lpf(i) = DyrosMath::lpf(rd_.q_dot_virtual_(i), q_dot_virtual_lpf(i), hz_, lpf_cutoff_);
    }

    {
        std::lock_guard<std::mutex> lock(mpc_mutex);
        q_container_ = rd_.q_virtual_;
        v_container_ = q_dot_virtual_lpf;
    }

    if (rd_.tc_.mode == 6)
    {
        static bool is_cc_init = true;
        if (is_cc_init == true) 
        {
            WBC::SetContact(rd_, true, true);

            loadParams();

            q_init_ = rd_.q_;

            is_cc_init = false;
        }

        static int initial_tick = 0;

        rd_.q_desired = DyrosMath::cubicVector<MODEL_DOF>(initial_tick, 0, 2.0 * hz_, q_init_, q_init_des, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero());
        rd_.torque_desired = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);

        initial_tick++;
    }
    else if (rd_.tc_.mode == 7)
    {
        static bool is_cc_init = true;
        if (is_cc_init == true) 
        {
            q_mpc_prev_ = q_init_des;
            v_mpc_prev_.setZero();
            torque_mpc_prev_ = rd_.torque_desired;

            q_mpc_ = q_init_des;
            v_mpc_.setZero();
            torque_mpc_ = rd_.torque_desired;

            is_cc_init = false;
        }

        if(mpc_update)
        {
            std::lock_guard<std::mutex> lock(mpc_mutex);

            q_mpc_prev_ = q_mpc_;
            v_mpc_prev_ = v_mpc_;
            torque_mpc_prev_ = torque_mpc_;
            
            q_mpc_ = q_mpc_container_;
            v_mpc_ = v_mpc_container_;
            torque_mpc_ = torque_mpc_container_;

            mpc_count = 1;

            mpc_update = false;
        }

        double mpc_hz_ = wb_mpc_.getMpcFrequency();
        q_mpc_interpol_ = q_mpc_prev_ + (q_mpc_ - q_mpc_prev_) * (mpc_hz_ / hz_) * mpc_count;
        torque_mpc_interpol_ = torque_mpc_prev_ + (torque_mpc_ - torque_mpc_prev_) * (mpc_hz_ / hz_) * mpc_count;

        mpc_qpos_checker   << q_mpc_interpol_.transpose() << std::endl;
        current_qpos_checker   << rd_.q_virtual_.transpose() << std::endl;
        mpc_torque_checker << torque_mpc_interpol_.transpose() << std::endl;

        mpc_count++;

        Eigen::VectorQd torque_sum; torque_sum.setZero();
        for(int i = 0; i < 12; i++) 
        {
            torque_sum(i) = torque_mpc_interpol_(i) + 0.0 * (q_mpc_interpol_(i) - rd_.q_(i)) + 100.0 * (0.0 - rd_.q_dot_(i));
        }
        for(int i = 12; i < MODEL_DOF; i++) 
        {
            torque_sum(i) = rd_.Kp_diag(i, i) * (rd_.q_desired(i) - rd_.q_(i)) + rd_.Kd_diag(i, i) * (0.0 - rd_.q_dot_(i));
        }

        // Eigen::VectorQd torque_sum = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);

        applyTorqueSmoothingOnce(torque_sum);

        rd_.torque_desired = torque_sum;
    }
    else
    {
        rd_.torque_desired = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);
    }
}

void CustomController::computeThread3()
{
    auto t1 = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lock(mpc_mutex);
        q_ = q_container_;
        v_ = v_container_;
    }

    static int current_tick = 0;

    wb_mpc_.updateMPCSolverInput(q_, v_, current_tick);

    if (rd_.tc_.mode == 7)
    {

        wb_mpc_.solve();

        Eigen::VectorQd q_mpc_local = wb_mpc_.getWBMPCJointPositionSolution();
        Eigen::VectorQd v_mpc_local = wb_mpc_.getWBMPCJointVelocitySolution();
        Eigen::VectorQd torque_mpc_local = wb_mpc_.getWBMPCJointTorqueSolution();  

        {
            std::lock_guard<std::mutex> lock(mpc_mutex);
            q_mpc_container_ = q_mpc_local;
            v_mpc_container_ = v_mpc_local;
            torque_mpc_container_ = torque_mpc_local;

            mpc_update = true;
        }

        current_tick++;

        auto t2 = std::chrono::steady_clock::now();

        auto dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

        calc_time_checker << dur << std::endl;
    }
}

void CustomController::computeFast()
{

}

void CustomController::computePlanner()
{

}

//--- First Torque Initialization
void CustomController::applyTorqueSmoothingOnce(Eigen::VectorQd &torque_target)
{
    static bool is_torque_save_init = true;
    if (is_torque_save_init == true)
    {
        rd_.torque_init = rd_.torque_desired;

        is_torque_save_init = false;
    }

    static bool is_torque_desired_init = true;
    static int tick_torque_desired_init = 0;
    if (is_torque_desired_init == true)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            torque_target(i) = DyrosMath::cubic(tick_torque_desired_init, 0, 1000, rd_.torque_init(i), torque_target(i), 0.0, 0.0);
        }

        tick_torque_desired_init++;

        if (tick_torque_desired_init >= 1000)
        {
            is_torque_desired_init = false;
            std::cout << "========== INFO: INITIAL TORQUE SMOOTHING COMPLETE ==========" << std::endl;
        }
    }
}

void CustomController::loadParams()
{
    ///////////////////////
    //--- Joint Gains ---//
    std::vector<double> kp_vec, kd_vec;

    nh_cc_.getParam("/tocabi_controller/joint_gains/Kp", kp_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kd", kd_vec);

    if(kp_vec.size() != MODEL_DOF || kd_vec.size() != MODEL_DOF)
        ROS_ERROR("Gains vector size mismatch (Kp size: %lu, Kd size: %lu, expected: %d)", kp_vec.size(), kd_vec.size(), MODEL_DOF);
    assert(kp_vec.size() == MODEL_DOF && kd_vec.size() == MODEL_DOF);

    Eigen::Map<Eigen::VectorQd> Kp(kp_vec.data(), MODEL_DOF);
    Eigen::Map<Eigen::VectorQd> Kd(kd_vec.data(), MODEL_DOF);

    rd_.Kp_diag = Kp.asDiagonal();
    rd_.Kd_diag = Kd.asDiagonal();

    ////////////////////////////////
    //--- Desired Initial Pose ---//
    std::vector<double> q_init_des_vec;
    nh_cc_.getParam("/tocabi_controller/q_init_des", q_init_des_vec);

    if(q_init_des_vec.size() != MODEL_DOF)
        ROS_ERROR("Initial desired position vector size mismatch (size: %lu, expected: %d)", q_init_des_vec.size(), MODEL_DOF);
    assert(q_init_des_vec.size() == MODEL_DOF);

    Eigen::Map<Eigen::VectorQd> q_init_des_map(q_init_des_vec.data(), MODEL_DOF);
    q_init_des = q_init_des_map;  
    wb_mpc_.setReferenceNominalPose(q_init_des);

    ///////////////////////
    //--- MPC Weights ---//
    std::vector<double> Q_pos_vec, Q_vel_vec;
    std::vector<double> R_acc_vec, R_force_vec, R_torque_vec;

    nh_cc_.getParam("/tocabi_controller/mpc_parameters/Q_pos_diag", Q_pos_vec);
    nh_cc_.getParam("/tocabi_controller/mpc_parameters/Q_vel_diag", Q_vel_vec);
    nh_cc_.getParam("/tocabi_controller/mpc_parameters/R_acc_diag", R_acc_vec);
    nh_cc_.getParam("/tocabi_controller/mpc_parameters/R_force_diag", R_force_vec);
    nh_cc_.getParam("/tocabi_controller/mpc_parameters/R_torque_diag", R_torque_vec);

    // --- Build Q_diag = [Q_pos, Q_vel]
    Eigen::VectorXd Q_diag, R_diag;
    Q_diag.resize(Q_pos_vec.size() + Q_vel_vec.size());
    Q_diag << Eigen::Map<Eigen::VectorXd>(Q_pos_vec.data(), Q_pos_vec.size()),
              Eigen::Map<Eigen::VectorXd>(Q_vel_vec.data(), Q_vel_vec.size());

    // --- Build R_diag = [R_acc, R_force, R_torque]
    R_diag.resize(R_acc_vec.size() + R_force_vec.size() + R_torque_vec.size());
    R_diag << Eigen::Map<Eigen::VectorXd>(R_acc_vec.data(), R_acc_vec.size()),
              Eigen::Map<Eigen::VectorXd>(R_force_vec.data(), R_force_vec.size()),
              Eigen::Map<Eigen::VectorXd>(R_torque_vec.data(), R_torque_vec.size());
    wb_mpc_.setWeights(Q_diag, R_diag);
}