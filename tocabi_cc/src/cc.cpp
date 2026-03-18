#include "cc.h"

using namespace TOCABI;

// ofstream torque_sum_log(      "/home/dyros/catkin_ws/src/tocabi_cc/data/torque_sum_log.txt");
// ofstream torque_idn_log(      "/home/dyros/catkin_ws/src/tocabi_cc/data/torque_idn_log.txt");
// ofstream torque_pd_log(       "/home/dyros/catkin_ws/src/tocabi_cc/data/torque_pd_log.txt");
// ofstream joint_desired_log(   "/home/dyros/catkin_ws/src/tocabi_cc/data/joint_desired_log.txt");
// ofstream joint_position_log(  "/home/dyros/catkin_ws/src/tocabi_cc/data/joint_position_log.txt");
// ofstream joint_velocity_log(  "/home/dyros/catkin_ws/src/tocabi_cc/data/joint_velocity_log.txt");
// ofstream computation_time_log(       "/home/dyros/catkin_ws/src/tocabi_cc/data/computation_time_log.txt");

// ofstream torque_sum_log(             "/home/kwan/catkin_ws/src/tocabi_cc/data/torque_sum_log.txt");
// ofstream torque_idn_log(             "/home/kwan/catkin_ws/src/tocabi_cc/data/torque_idn_log.txt");
// ofstream torque_pd_log(              "/home/kwan/catkin_ws/src/tocabi_cc/data/torque_pd_log.txt");
// ofstream joint_desired_log(          "/home/kwan/catkin_ws/src/tocabi_cc/data/joint_desired_log.txt");
// ofstream joint_position_log(         "/home/kwan/catkin_ws/src/tocabi_cc/data/joint_position_log.txt");
// ofstream joint_velocity_log(         "/home/kwan/catkin_ws/src/tocabi_cc/data/joint_velocity_log.txt");
// ofstream computation_time_log(       "/home/kwan/catkin_ws/src/tocabi_cc/data/computation_time_log.txt");

CustomController::CustomController(RobotData &rd) : rd_(rd),
                                                    cm_(rd, model),
                                                    tm_(rd),
                                                    cbf_mgr_(rd, model),
                                                    kin_wbc_(rd, cbf_mgr_),
                                                    dyn_wbc_(rd, cbf_mgr_, model),
                                                    teleop_(rd)
{
    //--- ROS Node Handle
    nh_cc_.setCallbackQueue(&queue_cc_);
    ControlVal_.setZero();

    //--- Load Model
    std::string urdf_path;
    ros::param::get("/tocabi_controller/urdf_path", urdf_path);
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model, true, false);

    //--- Joy Callback
    xbox_joy_sub_ = nh_cc_.subscribe<sensor_msgs::Joy>("/joy", 10, &CustomController::xBoxJoyCallback, this);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

void CustomController::computeSlow()
{
    queue_cc_.callAvailable(ros::WallDuration());
    cbf_mgr_.callAvailableQueue();
    teleop_.callAvailableQueue();

    static bool filter_init = true;
    if(filter_init)
    {
        q_dot_lpf_.setZero();
        q_dot_lpf_ = rd_.q_dot_;
        
        filter_init = false;
    }

    q_dot_lpf_ = DyrosMath::lpf<MODEL_DOF>(rd_.q_dot_, q_dot_lpf_, hz_, cutoff_freq);

    if (rd_.tc_.mode == 6)  // INIT MODE
    {
        static bool cm_init_save_trigger = true;
        cm_.update(cm_init_save_trigger);

        cbf_mgr_.update();

        CustomControllerInit();
        moveInitialPose();
    }
    else if (rd_.tc_.mode == 7) // TASK MODE
    {
        if (tc_mode_prev == 6)
        {
            static bool cm_init_save_trigger = true;
            cm_.update(cm_init_save_trigger);

            cbf_mgr_.update();

            tm_.runTestMotion(motion_mode_);

            kin_wbc_.computeTaskSpaceKinematicWBC();

            pubDataFromSlowToFast();

            torque_pd.setZero();
            torque_pd = rd_.Kd_diag * (rd_.q_dot_desired - rd_.q_dot_);

            torque_idn.setZero();
            subDataFromFastToSlow();

            FrictionCompensationTorques();
            
            torque_sum.setZero();
            torque_sum = torque_pd + torque_idn;

            // applyTorqueSmoothingOnce(torque_sum);

            // torque_sum_log << torque_sum.transpose() << std::endl;
            // joint_desired_log << rd_.q_desired.transpose() << std::endl;
            // joint_position_log << rd_.q_.transpose() << std::endl;
            // torque_idn_log << torque_idn.transpose() << std::endl;
            // torque_pd_log << torque_pd.transpose() << std::endl;

            rd_.torque_desired = torque_sum;

            is_slow_loop_once = true;
        }
        else
        {
            static bool is_warning_printed = true;
            static Eigen::VectorQd q_init;
            if (is_warning_printed)
            {
                ROS_ERROR("Press the Mode 6 to initialize the controller before running Mode 7.");
                q_init = rd_.q_;
                is_warning_printed = false;
            }

            for(int i = 0; i < MODEL_DOF; i++) {
                rd_.torque_desired(i) = rd_.pos_kp_v[i] * (q_init(i) - rd_.q_(i)) +  rd_.pos_kv_v[i] * (0.0 - rd_.q_dot_(i));
            }
        }
    }
    else if (rd_.tc_.mode == 8) // FRICTION COMPENSATION TEST
    {
        if (tc_mode_prev == 6)
        {
            static bool init_trigger = true;
            if(init_trigger){
                std::cout << "================================================" << std::endl;
                std::cout << "========== Friction Compensation Mode ==========" << std::endl;
                init_trigger = false;
            }
            rd_.torque_desired.setZero();
            rd_.torque_desired = (rd_.Kp_diag * (q_init_des - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);

            FrictionCompensationTorques();

            rd_.torque_desired(sinusoid_joint_target_) = torque_fric(sinusoid_joint_target_);
        }
        else
        {
            static bool is_warning_printed = true;
            static Eigen::VectorQd q_init;
            if (is_warning_printed)
            {
                ROS_ERROR("Press the Mode 6 to tune the friction compensation params before running Mode 8.");
                q_init = rd_.q_;
                is_warning_printed = false;
            }

            for(int i = 0; i < MODEL_DOF; i++) {
                rd_.torque_desired(i) = rd_.pos_kp_v[i] * (q_init(i) - rd_.q_(i)) +  rd_.pos_kv_v[i] * (0.0 - rd_.q_dot_(i));
            }
        }
    }
    else if (rd_.tc_.mode == 9) // PD TUNE TEST
    {
        if (tc_mode_prev == 6)
        {
            rd_.torque_desired.setZero();

            static bool init_trigger = true;
            if(init_trigger){
                std::cout << "==================================" << std::endl;
                std::cout << "========== PD TUNE Mode ==========" << std::endl;
                init_trigger = false;
            }

            static bool cm_init_save_trigger = true;
            cm_.update(cm_init_save_trigger);

            static bool is_cc_init = true;
            static double start_time = 0.0;
            static double phase = 0.0;

            double current_time = rd_.control_time_;

            static Eigen::VectorQd q_init_;
            if (is_cc_init)
            {
                q_init_ = rd_.q_;
                start_time = current_time;

                const double A = sinusoid_joint_max_;
                const double B = sinusoid_joint_min_;

                const double q0 = q_init_(sinusoid_joint_target_);

                const double c = q0 + 0.5 * (A - B);
                const double a = 0.5 * (A + B);

                double sin_phi = (q0 - c) / a;
                sin_phi = std::min(1.0, std::max(-1.0, sin_phi));

                phase = std::asin(sin_phi);

                is_cc_init = false;
            }
            rd_.q_desired = q_init_;

            if (is_step)
            {
                //--- Step Joint Trajectory
                rd_.q_desired(sinusoid_joint_target_) = q_init_(sinusoid_joint_target_) + sinusoid_joint_max_;
            }
            else
            {
                //--- Sinusoidal Joint Trajectory
                const double t = current_time - start_time;
                const double w = 2.0 * M_PI / sinusoid_period_;

                const double A = sinusoid_joint_max_;
                const double B = sinusoid_joint_min_;

                const double q0 = q_init_(sinusoid_joint_target_);
                const double c = q0 + 0.5 * (A - B);
                const double a = 0.5 * (A + B);

                rd_.q_desired(sinusoid_joint_target_) =c + a * std::sin(w * t + phase);
            }

            rd_.q_ddot_desired_virtual.setZero();
            rd_.q_ddot_desired_virtual.segment(6, MODEL_DOF) = rd_.Kp_virtual_diag.bottomRightCorner(MODEL_DOF, MODEL_DOF) * (rd_.q_desired - rd_.q_) 
                                                             - rd_.Kd_virtual_diag.bottomRightCorner(MODEL_DOF, MODEL_DOF) * rd_.q_dot_;
            
            pubDataFromSlowToFast();

            rd_.LF_FT_DES.setZero();
            rd_.RF_FT_DES.setZero();
            Eigen::Vector12d contact_wrench_cmd;
            contact_wrench_cmd.setZero();

            // dyn_wbc_.recursiveNewtonEulerAlgorithm(rd_.q_ddot_desired_virtual, contact_wrench_cmd, torque_idn);

            subDataFromFastToSlow();

            for (int i = 0; i < MODEL_DOF; i++)
                rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (0.0 - rd_.q_dot_[i]);
            rd_.torque_desired(sinusoid_joint_target_) = torque_idn(sinusoid_joint_target_) 
                                                       + rd_.Kp_diag(sinusoid_joint_target_, sinusoid_joint_target_) * (rd_.q_desired - rd_.q_)(sinusoid_joint_target_)
                                                       - rd_.Kd_diag(sinusoid_joint_target_, sinusoid_joint_target_) * rd_.q_dot_(sinusoid_joint_target_);

            FrictionCompensationTorques();
            rd_.torque_desired += torque_fric;

            for (int i = 0; i < MODEL_DOF; i++)
            {
                rd_.torque_desired(i) = DyrosMath::minmax_cut(rd_.torque_desired(i), -rd_.torque_limit(i), rd_.torque_limit(i));
            }

            // joint_desired_log << rd_.q_desired(sinusoid_joint_target_) << std::endl;
            // joint_position_log << rd_.q_(sinusoid_joint_target_) << std::endl;
            // joint_velocity_log << rd_.q_dot_(sinusoid_joint_target_) << std::endl;
            // joint_velocity_filter_log << q_dot_lpf_(sinusoid_joint_target_) << std::endl;
            // torque_sum_log << rd_.torque_desired(sinusoid_joint_target_) << std::endl;
        }
        else
        {
            static bool is_warning_printed = true;
            static Eigen::VectorQd q_init;
            if (is_warning_printed)
            {
                ROS_ERROR("Press the Mode 6 to tune the PD gain before running Mode 9.");
                q_init = rd_.q_;
                is_warning_printed = false;
            }

            for(int i = 0; i < MODEL_DOF; i++) {
                rd_.torque_desired(i) = rd_.pos_kp_v[i] * (q_init(i) - rd_.q_(i)) +  rd_.pos_kv_v[i] * (0.0 - rd_.q_dot_(i));
            }        
        }
    }
    else
    {
        rd_.torque_desired = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);
    }
}

void CustomController::computeFast()
{
    if (rd_.tc_.mode == 7 || rd_.tc_.mode == 9)
    {
        if (tc_mode_prev == 6)
        {
            if(is_slow_loop_once)
            {
                auto t1 = std::chrono::steady_clock::now();

                subDataFromSlowToFast();

                dyn_wbc_.updateRobotStates(M_fast, G_fast, J_C_fast);
                dyn_wbc_.updateControlCommands(contact_wrench_cmd_fast, qddot_cmd_fast);
                torque_idn_fast.setZero();
                torque_idn_fast = dyn_wbc_.computeDynamicWBC();

                pubDataFromFastToSlow();

                auto t2 = std::chrono::steady_clock::now();

                auto dt = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

                // computation_time_log << dt << std::endl;
            }
        }
    }
}

void CustomController::computePlanner()
{
}

void CustomController::CustomControllerInit()
{
    static bool is_cc_init = true;

    if (is_cc_init == true)
    {
        loadParams();

        q_init_ = rd_.q_;
        WBC::SetContact(rd_, true, true);

        M_container.setZero();
        M_fast.setZero();

        G_container.setZero();
        G_fast.setZero();

        J_C_container.setZero(12, MODEL_DOF_VIRTUAL);
        J_C_fast.setZero(12, MODEL_DOF_VIRTUAL);

        contact_wrench_cmd_fast.setZero();
        contact_wrench_cmd_container.setZero();

        qddot_cmd_fast.setZero();
        qddot_cmd_container.setZero();

        torque_idn_fast.setZero();
        torque_idn_container.setZero();

        tc_mode_prev = 6;

        is_cc_init = false;
    }
}

void CustomController::moveInitialPose()
{
    static int initial_tick = 0;

    q_init_des;
    q_init_des.setZero();
    q_init_des = q_init_;

    if (motion_mode_ == TaskMotionType::Walking || motion_mode_ == TaskMotionType::TeleOperation)
    {
        q_init_des(0) = 0.0;
        q_init_des(1) = 0.0;
        q_init_des(2) = -0.24;
        q_init_des(3) = 0.6;
        q_init_des(4) = -0.36;
        q_init_des(5) = 0.0;

        q_init_des(6) = 0.0;
        q_init_des(7) = 0.0;
        q_init_des(8) = -0.24;
        q_init_des(9) = 0.6;
        q_init_des(10) = -0.36;
        q_init_des(11) = 0.0;

        q_init_des(12) = 0.0;
        q_init_des(13) = 0.0;
        q_init_des(14) = 0.0;

        q_init_des(15) = +15.0 * DEG2RAD;
        q_init_des(16) = +10.0 * DEG2RAD;
        q_init_des(17) = +80.0 * DEG2RAD;
        q_init_des(18) = -70.0 * DEG2RAD;
        q_init_des(19) = -30.0 * DEG2RAD;
        q_init_des(21) = 0.0 * DEG2RAD;

        q_init_des(25) = -15.0 * DEG2RAD;
        q_init_des(26) = -10.0 * DEG2RAD;
        q_init_des(27) = -80.0 * DEG2RAD;
        q_init_des(28) = +70.0 * DEG2RAD;
        q_init_des(29) = +30.0 * DEG2RAD;
        q_init_des(31) = -0.0 * DEG2RAD;
    }
    else
    {
        q_init_des(12) = 0.0; // yaw
        q_init_des(13) = 0.0; // pitch
        q_init_des(14) = 0.0; // roll

        q_init_des(15) = 0.53;
        q_init_des(16) =-0.60;
        q_init_des(17) = 1.70;
        q_init_des(18) = 0.00;
        q_init_des(19) =-1.20; // elbow
        q_init_des(20) = 0.6;
        q_init_des(21) =-0.65;
        q_init_des(22) = 0.0;

        q_init_des(23) = 0.0; // yaw
        q_init_des(24) = 0.0; // pitch

        q_init_des(25) =-0.53;
        q_init_des(26) = 0.60;
        q_init_des(27) =-1.70;
        q_init_des(28) = 0.0;
        q_init_des(29) = 1.20; // elbow
        q_init_des(30) =-0.6;
        q_init_des(31) = 0.65;
        q_init_des(32) = 0.0;
    }

    kin_wbc_.setInitialConfiguration(q_init_des);

    rd_.q_desired = DyrosMath::cubicVector<MODEL_DOF>(initial_tick, 0, 2.0 * hz_, q_init_, q_init_des, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero());
    for (int i = 0; i < MODEL_DOF; i++){
        rd_.torque_desired[i] = rd_.pos_kp_v[i] * (rd_.q_desired[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (0.0 - rd_.q_dot_[i]);
    }

    initial_tick++;
}

//--- Joy Utils
void CustomController::xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    double threshold = 1.0;

    move_forward = DyrosMath::minmax_cut(joy->axes[1] * threshold, -threshold, threshold);
    move_lateral = DyrosMath::minmax_cut(joy->axes[0] * threshold, -threshold, threshold);
    rotate_yaw = DyrosMath::minmax_cut(joy->axes[3] * threshold, -threshold, threshold);

    static int zz = 0;
    if (zz % 1000 == 0)
    {
        std::cout << "move_forward: " << move_forward << std::endl;
        std::cout << "move_lateral: " << move_lateral << std::endl;
        std::cout << "rotate_yaw: " << rotate_yaw << std::endl;
    }
    zz++;

    if (is_joy_enable == true)
    {
        tm_.setStepStride(move_forward * step_length_);
        tm_.setStepLateral(move_lateral * step_lateral_);
        tm_.setStepYaw(rotate_yaw * step_yaw_);
    }
}

//--- Data Transfer
void CustomController::pubDataFromSlowToFast()
{
    if (atb_control_command_update_ == false)
    {
        atb_control_command_update_ = true;

        M_container = rd_.local_A;
        G_container = rd_.local_G;
        J_C_container = rd_.local_J_C;

        qddot_cmd_container = rd_.q_ddot_desired_virtual;

        contact_wrench_cmd_container.head(6) = rd_.LF_FT_DES;
        contact_wrench_cmd_container.tail(6) = rd_.RF_FT_DES;

        if (cbf_mgr_.getCbfMode() == CbfType::Dyn)
        {
            cbf_mgr_.pubDataFromSlowToFast();
        }

        atb_control_command_update_ = false;
    }
}

void CustomController::subDataFromSlowToFast()
{
    if (atb_control_command_update_ == false)
    {
        atb_control_command_update_ = true;

        M_fast = M_container;
        G_fast = G_container;
        J_C_fast = J_C_container;

        contact_wrench_cmd_fast = contact_wrench_cmd_container;
        qddot_cmd_fast = qddot_cmd_container;

        if (cbf_mgr_.getCbfMode() == CbfType::Dyn)
        {
            cbf_mgr_.subDataFromSlowToFast();
        }

        atb_control_command_update_ = false;
    }
}

void CustomController::pubDataFromFastToSlow()
{
    if (atb_torque_update_ == false)
    {
        atb_torque_update_ = true;
        torque_idn_container = torque_idn_fast;
        atb_torque_update_ = false;
    }
}

void CustomController::subDataFromFastToSlow()
{
    if (atb_torque_update_ == false)
    {
        atb_torque_update_ = true;
        torque_idn = torque_idn_container;
        atb_torque_update_ = false;
    }
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

    const double interpol_time = 0.5;
    double interpol_tick_end = interpol_time * hz_;

    static bool is_torque_desired_init = true;
    static int tick_torque_desired_init = 0;
    if (is_torque_desired_init == true)
    {
        for (int i = 0; i < MODEL_DOF; i++)
        {
            torque_target(i) = DyrosMath::cubic(tick_torque_desired_init, 0, interpol_tick_end, rd_.torque_init(i), torque_target(i), 0.0, 0.0);
        }

        tick_torque_desired_init++;

        if (tick_torque_desired_init >= interpol_tick_end)
        {
            is_torque_desired_init = false;
            std::cout << "========== INFO: INITIAL TORQUE SMOOTHING COMPLETE ==========" << std::endl;
        }
    }
}

//--- Parameter Loader
void CustomController::loadParams()
{
    Kp.setZero(MODEL_DOF);
    Kd.setZero(MODEL_DOF);
    Kp_virtual.setZero(MODEL_DOF_VIRTUAL);
    Kd_virtual.setZero(MODEL_DOF_VIRTUAL);

    std::vector<double> kp_vec, kd_vec;
    std::vector<double> kp_dyn_vec, kd_dyn_vec;
    std::vector<double> pos_low_deg, pos_high_deg;
    std::vector<double> vel_low, vel_high;

    nh_cc_.getParam("/tocabi_controller/joint_gains/Kp", kp_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kd", kd_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kp_dyn", kp_dyn_vec);
    nh_cc_.getParam("/tocabi_controller/joint_gains/Kd_dyn", kd_dyn_vec);
    nh_cc_.getParam("/tocabi_controller/joint_limits/pos_low_deg", pos_low_deg);
    nh_cc_.getParam("/tocabi_controller/joint_limits/pos_high_deg", pos_high_deg);
    nh_cc_.getParam("/tocabi_controller/joint_limits/vel_low", vel_low);
    nh_cc_.getParam("/tocabi_controller/joint_limits/vel_high", vel_high);

    // Check Vector Dimension
    if (kp_vec.size() != MODEL_DOF)
        ROS_ERROR("Kp vector size mismatch: got %lu, expected %d", kp_vec.size(), MODEL_DOF);
    assert(kp_vec.size() == MODEL_DOF);

    if (kd_vec.size() != MODEL_DOF)
        ROS_ERROR("Kd vector size mismatch: got %lu, expected %d", kd_vec.size(), MODEL_DOF);
    assert(kd_vec.size() == MODEL_DOF);

    if (kp_dyn_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kp_dyn vector size mismatch: got %lu, expected %d", kp_dyn_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kp_dyn_vec.size() == MODEL_DOF_VIRTUAL);

    if (kd_dyn_vec.size() != MODEL_DOF_VIRTUAL)
        ROS_ERROR("Kd_dyn vector size mismatch: got %lu, expected %d", kd_dyn_vec.size(), MODEL_DOF_VIRTUAL);
    assert(kd_dyn_vec.size() == MODEL_DOF_VIRTUAL);

    if (pos_low_deg.size() != MODEL_DOF)
        ROS_ERROR("Joint position lower limit vector size mismatch: got %lu, expected %d", pos_low_deg.size(), MODEL_DOF);
    assert(pos_low_deg.size() == MODEL_DOF);

    if (pos_high_deg.size() != MODEL_DOF)
        ROS_ERROR("Joint position upper limit vector size mismatch: got %lu, expected %d", pos_high_deg.size(), MODEL_DOF);
    assert(pos_high_deg.size() == MODEL_DOF);

    // if (pos_low.size() != MODEL_DOF)
    //     ROS_ERROR("Joint position lower limit vector size mismatch: got %lu, expected %d", pos_low_deg.size(), MODEL_DOF);
    // assert(pos_low.size() == MODEL_DOF);

    // if (pos_high.size() != MODEL_DOF)
    //     ROS_ERROR("Joint position upper limit vector size mismatch: got %lu, expected %d", pos_high_deg.size(), MODEL_DOF);
    // assert(pos_high.size() == MODEL_DOF);

    if (vel_low.size() != MODEL_DOF)
        ROS_ERROR("Joint velocity lower limit vector size mismatch: got %lu, expected %d", pos_low_deg.size(), MODEL_DOF);
    assert(vel_low.size() == MODEL_DOF);

    if (vel_high.size() != MODEL_DOF)
        ROS_ERROR("Joint velocity upper limit vector size mismatch: got %lu, expected %d", pos_high_deg.size(), MODEL_DOF);
    assert(vel_high.size() == MODEL_DOF);

    // Assign each vector into Eigen Vec or Mat
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        Kp(i) = kp_vec[i];
        Kd(i) = kd_vec[i];
    }
    for (int i = 0; i < MODEL_DOF_VIRTUAL; ++i)
    {
        Kp_virtual(i) = kp_dyn_vec[i];
        Kd_virtual(i) = kd_dyn_vec[i];
    }

    rd_.Kp_virtual_diag = Kp_virtual.asDiagonal();
    rd_.Kd_virtual_diag = Kd_virtual.asDiagonal();
    rd_.Kp_diag = Kp.asDiagonal();
    rd_.Kd_diag = Kd.asDiagonal();

    // Position Limits (convert deg to rad)
    for (int i = 0; i < MODEL_DOF; ++i)
    {
        rd_.q_pos_l_lim(i) = pos_low_deg[i] * DEG2RAD;
        rd_.q_pos_h_lim(i) = pos_high_deg[i] * DEG2RAD;
        // rd_.q_pos_l_lim(i) = pos_low[i] * DEG2RAD;
        // rd_.q_pos_h_lim(i) = pos_high[i] * DEG2RAD;
        rd_.q_vel_l_lim(i) = vel_low[i];
        rd_.q_vel_h_lim(i) = vel_high[i];
    }

    // Motion mode
    int motion_mode_idx = 0;
    nh_cc_.getParam("/tocabi_controller/motion_mode", motion_mode_idx);
    if (motion_mode_idx == 0)
    {
        motion_mode_ = TaskMotionType::None;
    }
    else if (motion_mode_idx == 1)
    {
        motion_mode_ = TaskMotionType::PelvHand;
    }
    else if (motion_mode_idx == 2)
    {
        motion_mode_ = TaskMotionType::Taichi;
    }
    else if (motion_mode_idx == 3)
    {
        motion_mode_ = TaskMotionType::Walking;
    }
    else if (motion_mode_idx == 4)
    {
        motion_mode_ = TaskMotionType::TeleOperation;
    }
    else
    {
        ROS_ERROR("Motion mode idx error: got %d", motion_mode_idx);
        assert(motion_mode_idx == 0 || motion_mode_idx == 1 || motion_mode_idx == 2 || motion_mode_idx == 3 || motion_mode_idx == 4);
    }

    const char *mode_name =
        (motion_mode_ == TaskMotionType::None)            ? "None"
        : (motion_mode_ == TaskMotionType::PelvHand)      ? "PelvHand"
        : (motion_mode_ == TaskMotionType::Taichi)        ? "Taichi"
        : (motion_mode_ == TaskMotionType::Walking)       ? "Walking"
        : (motion_mode_ == TaskMotionType::TeleOperation) ? "TeleOperation"
                                                          : "Unknown";
    std::cout << " " << std::endl;
    std::cout << "===== Motion Mode : " << mode_name << " =====" << std::endl;
    std::cout << " " << std::endl;

    kin_wbc_.setTaskHierarchy(motion_mode_);

    if (motion_mode_ == TaskMotionType::Walking)
    {
        // rd_.q_pos_l_lim.segment(12, MODEL_DOF - 12).setZero();
        // rd_.q_pos_h_lim.segment(12, MODEL_DOF - 12).setZero();
        // rd_.q_vel_l_lim.segment(12, MODEL_DOF - 12).setZero();
        // rd_.q_vel_h_lim.segment(12, MODEL_DOF - 12).setZero();

        std::cout << " " << std::endl;
        std::cout << "rd_.q_pos_l_lim: " << rd_.q_pos_l_lim.transpose() << std::endl;
        std::cout << "rd_.q_pos_h_lim: " << rd_.q_pos_h_lim.transpose() << std::endl;
        std::cout << "rd_.q_vel_l_lim: " << rd_.q_vel_l_lim.transpose() << std::endl;
        std::cout << "rd_.q_vel_h_lim: " << rd_.q_vel_h_lim.transpose() << std::endl;
        std::cout << " " << std::endl;
    }

    //--- Task Parameter
    nh_cc_.getParam("/tocabi_controller/task_param/traj_time", traj_time_);
    nh_cc_.getParam("/tocabi_controller/task_param/pelv_dist", pelv_dist_);
    nh_cc_.getParam("/tocabi_controller/task_param/hand_dist", hand_dist_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_length", step_length_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_lateral", step_lateral_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_yaw", step_yaw_);
    nh_cc_.getParam("/tocabi_controller/task_param/foot_height", foot_height_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_duration", step_duration_);
    nh_cc_.getParam("/tocabi_controller/task_param/dsp_duration", dsp_duration_);

    tm_.setTrajectoryDuration(traj_time_);
    tm_.setPelvisDistance(pelv_dist_);
    tm_.setHandDistance(hand_dist_);

    if(is_joy_enable == false){
        tm_.setStepStride(step_length_);
        tm_.setStepLateral(step_lateral_);
        tm_.setStepYaw(step_yaw_);
    }
    else{
        tm_.setStepStride(0.0);
        tm_.setStepLateral(0.0);
        tm_.setStepYaw(0.0);
    }
        
    tm_.setFootHeight(foot_height_);
    tm_.setStepDuration(step_duration_);
    tm_.setDspDuration(dsp_duration_);

    std::cout << " " << std::endl;
    std::cout << "======== Task Parameters ========== " << std::endl;
    std::cout << "Trajectory Time : " << traj_time_ << " sec" << std::endl;
    std::cout << "Pelvis Distance : " << pelv_dist_ << " m" << std::endl;
    std::cout << "Hand Distance : " << hand_dist_ << " m" << std::endl;
    std::cout << "Step Length : " << step_length_ << " m" << std::endl;
    std::cout << "Step Lateral : " << step_lateral_ << " m" << std::endl;
    std::cout << "Foot Height : " << foot_height_ << " m" << std::endl;
    std::cout << "Step Duration : " << step_duration_ << " sec" << std::endl;
    std::cout << "Double Support Duration : " << dsp_duration_ << " sec" << std::endl;
    std::cout << " " << std::endl;

    //--- Whole-body Inverse Dynamics
    double W_qddot, W_cwr, W_torque, W_energy;
    nh_cc_.getParam("/tocabi_controller/wbid/W_qddot", W_qddot);
    nh_cc_.getParam("/tocabi_controller/wbid/W_cwr", W_cwr);
    nh_cc_.getParam("/tocabi_controller/wbid/W_torque", W_torque);
    nh_cc_.getParam("/tocabi_controller/wbid/W_energy", W_energy);

    dyn_wbc_.setJointTrackingWeight(W_qddot);
    dyn_wbc_.setContactWrenchTrackingWeight(W_cwr);
    dyn_wbc_.setTorqueMinimizationWeight(W_torque);
    dyn_wbc_.setAccelEnergyMinimizationWeight(W_energy);

    double friction_coeff, foot_size, foot_width;
    nh_cc_.getParam("/tocabi_controller/wbid/friction_coeff", friction_coeff);
    nh_cc_.getParam("/tocabi_controller/wbid/foot_size", foot_size);
    nh_cc_.getParam("/tocabi_controller/wbid/foot_width", foot_width);
    dyn_wbc_.setFrictionCoefficient(friction_coeff);
    dyn_wbc_.setFootDimension(foot_size, foot_width);

    std::cout << " " << std::endl;
    std::cout << "========== WBID Parameters ========== " << std::endl;
    std::cout << "W_qddot : " << W_qddot << std::endl;
    std::cout << "W_cwr : " << W_cwr << std::endl;
    std::cout << "W_energy : " << W_energy << std::endl;
    std::cout << "friction_coeff : " << friction_coeff << std::endl;
    std::cout << " " << std::endl;

    //--- CBF Parameters
    int issf_mode_idx = 0;
    nh_cc_.getParam("/tocabi_controller/cbf/issf_mode", issf_mode_idx);
    if (issf_mode_idx == 0)
    {
        issf_mode_ = false;
    }
    else if (issf_mode_idx == 1)
    {
        issf_mode_ = true;
    }
    else    
    {
        ROS_ERROR("ISSF mode idx error: got %d", issf_mode_idx);
        assert(issf_mode_idx == 0 || issf_mode_idx == 1);
    }
    cbf_mgr_.setIssfMode(issf_mode_);

    int cbf_mode_idx = 0;
    nh_cc_.getParam("/tocabi_controller/cbf/cbf_mode", cbf_mode_idx);
    if (cbf_mode_idx == 0)
    {
        cbf_mode_ = CbfType::None;
    }
    else if (cbf_mode_idx == 1)
    {
        cbf_mode_ = CbfType::Kin;
    }
    else if (cbf_mode_idx == 2)
    {
        cbf_mode_ = CbfType::Dyn;
    }
    else
    {
        ROS_ERROR("Cbf mode idx error: got %d", cbf_mode_idx);
        assert(cbf_mode_idx == 0 || cbf_mode_idx == 1 || cbf_mode_idx == 2);
    }
    const char *cbf_mode_name =
        (cbf_mode_ == CbfType::Kin)   ? "Kin"
        : (cbf_mode_ == CbfType::Dyn) ? "Dyn"
                                      : "None";
    std::cout << " " << std::endl;
    std::cout << "========== CBF Parameters ==========" << std::endl;
    std::cout << "CBF Mode : " << cbf_mode_name << std::endl;
    if(issf_mode_){
        std::cout << "ISSF Mode : ON" << std::endl;
    }
    else{
        std::cout << "ISSF Mode : OFF" << std::endl;
    }

    cbf_mgr_.setCbfMode(cbf_mode_);

    std::string cbf_ns;
    if (cbf_mode_ == CbfType::Dyn)
        cbf_ns = "/tocabi_controller/cbf/dyn/";
    else if (cbf_mode_ == CbfType::Kin)
        cbf_ns = "/tocabi_controller/cbf/kin/";

    double joint_limit_cbf_alpha, joint_limit_cbf_epsilon;
    nh_cc_.getParam(cbf_ns + "joint_limit_cbf_alpha", joint_limit_cbf_alpha);
    nh_cc_.getParam(cbf_ns + "joint_limit_cbf_epsilon", joint_limit_cbf_epsilon);
    cbf_mgr_.setJointLimitCbfParameters(joint_limit_cbf_alpha, joint_limit_cbf_epsilon);
    cbf_mgr_.setJointLimitBoundaries(rd_.q_pos_l_lim, rd_.q_pos_h_lim);

    double workspace_boundary_cbf_alpha, workspace_boundary_cbf_epsilon, workspace_boundary_cbf_hand;
    nh_cc_.getParam(cbf_ns + "workspace_boundary_cbf_alpha", workspace_boundary_cbf_alpha);
    nh_cc_.getParam(cbf_ns + "workspace_boundary_cbf_epsilon", workspace_boundary_cbf_epsilon);
    nh_cc_.getParam("/tocabi_controller/cbf/workspace_boundary_cbf_hand", workspace_boundary_cbf_hand);
    cbf_mgr_.setWorkspaceBoundaryCbfParameters(workspace_boundary_cbf_alpha, workspace_boundary_cbf_epsilon);
    std::vector<WorkspaceBoundaryPair> workspace_pairs = {
        {Left_Hand, Left_Hand - 5, workspace_boundary_cbf_hand},
        {Right_Hand, Right_Hand - 5, workspace_boundary_cbf_hand},
    };
    cbf_mgr_.setWorkspaceBoundaryPairs(workspace_pairs);

    double self_collision_cbf_alpha, self_collision_cbf_epsilon;
    nh_cc_.getParam(cbf_ns + "self_collision_cbf_alpha", self_collision_cbf_alpha);
    nh_cc_.getParam(cbf_ns + "self_collision_cbf_epsilon", self_collision_cbf_epsilon);
    cbf_mgr_.setSelfCollisionCbfParameters(self_collision_cbf_alpha, self_collision_cbf_epsilon);

    double obstacle_avoidance_cbf_alpha, obstacle_avoidance_cbf_epsilon;
    nh_cc_.getParam(cbf_ns + "obstacle_avoidance_cbf_alpha", obstacle_avoidance_cbf_alpha);
    nh_cc_.getParam(cbf_ns + "obstacle_avoidance_cbf_epsilon", obstacle_avoidance_cbf_epsilon);
    cbf_mgr_.setObstacleAvoidanceCbfParameters(obstacle_avoidance_cbf_alpha, obstacle_avoidance_cbf_epsilon);

    std::cout << " Joint Limit CBF: " << std::endl;
    std::cout << "   alpha   : " << joint_limit_cbf_alpha << ", epsilon : " << joint_limit_cbf_epsilon << std::endl;
    std::cout << " Workspace Boundary CBF: " << std::endl;
    std::cout << "   alpha   : " << workspace_boundary_cbf_alpha << ", epsilon : " << workspace_boundary_cbf_epsilon << ", hand_dist : " << workspace_boundary_cbf_hand << std::endl;
    std::cout << " Self Collision CBF: " << std::endl;
    std::cout << "   alpha   : " << self_collision_cbf_alpha << ", epsilon : " << self_collision_cbf_epsilon << std::endl;
    std::cout << " Obstacle Avoidance CBF: " << std::endl;
    std::cout << "   alpha   : " << obstacle_avoidance_cbf_alpha << ", epsilon : " << obstacle_avoidance_cbf_epsilon << std::endl;
    std::cout << "====================================" << std::endl;
    std::cout << " " << std::endl;

    //--- Joint Tuning
    nh_cc_.getParam("/tocabi_controller/sinusoidal_joint_test/joint_target", sinusoid_joint_target_);
    nh_cc_.getParam("/tocabi_controller/sinusoidal_joint_test/joint_min", sinusoid_joint_min_);
    nh_cc_.getParam("/tocabi_controller/sinusoidal_joint_test/joint_max", sinusoid_joint_max_);
    nh_cc_.getParam("/tocabi_controller/sinusoidal_joint_test/period", sinusoid_period_);
    int is_step_idx = 0;
    nh_cc_.getParam("/tocabi_controller/sinusoidal_joint_test/is_step", is_step_idx);
    if (is_step_idx == 0)
    {
        is_step = false;
    }
    else if (is_step_idx == 1)
    {
        is_step = true;
    }
    else
    {
        std::cout << "Sinusoidal joint test step_or_not error: got " << is_step_idx << std::endl;
        assert(is_step_idx == 0 || is_step_idx == 1);
    }

    std::cout << "=====================================" << std::endl;
    std::cout << "==== Sinusoidal Joint Test Params ====" << std::endl;
    std::cout << "joint_target : " << sinusoid_joint_target_ << std::endl;
    std::cout << "joint_min    : " << sinusoid_joint_min_ << std::endl;
    std::cout << "joint_max    : " << sinusoid_joint_max_ << std::endl;
    std::cout << "period [s]   : " << sinusoid_period_ << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << " " << std::endl;

    //--- Friction Compensation
    nh_cc_.getParam("/tocabi_controller/friction_compensation/tau_coulomb", tau_coulomb);
    nh_cc_.getParam("/tocabi_controller/friction_compensation/tau_viscous", tau_viscous);
    std::cout << "======================================" << std::endl;
    std::cout << "==== Friction Compensation Params ====" << std::endl;
    for(int i = 0; i < MODEL_DOF; i++){
        std::cout << "tau_coulomb[" << i << "] : " << tau_coulomb[i] << ", tau_viscous[" << i << "] : " << tau_viscous[i] << std::endl;
    }
    std::cout << "======================================" << std::endl;
    std::cout << " " << std::endl;
}

//--- Friction Compensation
void CustomController::FrictionCompensationTorques()
{
    double alpha_fric = 0.95;
    Eigen::VectorQd torque_coulomb; torque_coulomb.setZero();
    Eigen::VectorQd torque_viscous; torque_viscous.setZero();

    for(int i = 0; i < MODEL_DOF; i++){
        torque_coulomb(i) = tau_coulomb[i] * tanh(alpha_fric * rd_.q_dot_(i)); 
        torque_viscous(i) = tau_viscous[i] * tanh(alpha_fric * rd_.q_dot_(i)) * sqrt(abs(rd_.q_dot_(i))); 
    }

    torque_fric.setZero();
    torque_fric = torque_coulomb + torque_viscous;
}