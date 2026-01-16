#include "cc.h"

using namespace TOCABI;

CustomController::CustomController(RobotData &rd) : rd_(rd), cm_(rd, model), tm_(rd), cbf_mgr_(rd, model), kin_wbc_(rd), dyn_wbc_(rd, cbf_mgr_), teleop_(rd)
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

    if (rd_.tc_.mode == 6)
    {   
        control_mode_changed = std::exchange(is_6_init, false);

        cm_.update(control_mode_changed);

        cbf_mgr_.update();

        CustomControllerInit();
        moveInitialPose();
    }
    else if (rd_.tc_.mode == 7)
    {   
        control_mode_changed = std::exchange(is_7_init, false);

        cm_.update(control_mode_changed);

        cbf_mgr_.update();
#ifdef COMPILE_SIMULATION
            cbf_mgr_.col_mgr_.pubQRObstaclePose(sim_tick_, hz_);
            sim_tick_++;
#endif
        tm_.runTestMotion(motion_mode_); 

        kin_wbc_.computeTaskSpaceKinematicWBC();

        pubDataFromSlowToFast();

        torque_pd.setZero();
        torque_pd = (rd_.Kd_diag) * (Eigen::VectorQd::Zero() - rd_.q_dot_);

        torque_idn.setZero();
        subDataFromFastToSlow();

        torque_sum.setZero();
        torque_sum = torque_pd + torque_idn;

        applyTorqueSmoothingOnce(torque_sum);

        rd_.torque_desired = torque_sum;
    }
    else
    {
        rd_.torque_desired = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);
    }
}

void CustomController::computeFast()
{
    if (fast_loop_ready){
        if (rd_.tc_.mode == 7)
        {
            subDataFromSlowToFast();

            dyn_wbc_.updateRobotStates(M_fast, G_fast, J_C_fast);
            dyn_wbc_.updateControlCommands(contact_wrench_cmd_fast, qddot_cmd_fast);
            torque_idn_fast.setZero();
            torque_idn_fast = dyn_wbc_.computeDynamicWBC();

            pubDataFromFastToSlow();
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

        is_cc_init = false;
    }
}

void CustomController::moveInitialPose()
{
    static int initial_tick = 0;

    q_init_des; q_init_des.setZero();
    q_init_des = q_init_;

    if(motion_mode_ == TaskMotionType::Walking)
    {
        q_init_des(0) = 0.0; 
        q_init_des(1) = 0.0; 
        q_init_des(2) = -0.24; 
        q_init_des(3) = 0.6; 
        q_init_des(4) = -0.36; 
        q_init_des(5) = 0.0; 

        q_init_des(6)  = 0.0; 
        q_init_des(7)  = 0.0; 
        q_init_des(8)  = -0.24; 
        q_init_des(9)  = 0.6; 
        q_init_des(10) = -0.36; 
        q_init_des(11) = 0.0;
        
        q_init_des(12) = 0.0;
        q_init_des(13) = 0.0;
        q_init_des(14) = 0.0;

        q_init_des(15) = + 15.0 * DEG2RAD; 
        q_init_des(16) = + 10.0 * DEG2RAD; 
        q_init_des(17) = + 80.0 * DEG2RAD; 
        q_init_des(18) = - 70.0 * DEG2RAD; 
        q_init_des(19) = - 45.0 * DEG2RAD; 
        q_init_des(21) =   0.0 * DEG2RAD; 

        q_init_des(25) = - 15.0 * DEG2RAD; 
        q_init_des(26) = - 10.0 * DEG2RAD;            
        q_init_des(27) = - 80.0 * DEG2RAD;  
        q_init_des(28) = + 70.0 * DEG2RAD; 
        q_init_des(29) = + 45.0 * DEG2RAD;       
        q_init_des(31) = - 0.0 * DEG2RAD; 
    }
    else
    {
        q_init_des(12) = 0.0;
        q_init_des(13) = 0.0;
        q_init_des(14) = 0.0;

        q_init_des(15) = 0.0;
        q_init_des(16) = -0.3;
        q_init_des(17) = 1.57;
        q_init_des(18) = -1.2;
        q_init_des(19) = -1.57; // elbow
        q_init_des(20) = 1.5;
        q_init_des(21) = 0.4;
        q_init_des(22) = -0.2;

        q_init_des(23) = 0.0; // yaw
        q_init_des(24) = 0.0; // pitch

        q_init_des(25) = 0.0;
        q_init_des(26) = 0.3;
        q_init_des(27) = -1.57;
        q_init_des(28) = 1.2;
        q_init_des(29) = 1.57; // elbow
        q_init_des(30) = -1.5;
        q_init_des(31) = -0.4;
        q_init_des(32) = 0.2;
    }

    kin_wbc_.setInitialConfiguration(q_init_des);
    
    rd_.q_desired = DyrosMath::cubicVector<MODEL_DOF>(initial_tick, 0, 2.0 * hz_, q_init_, q_init_des, Eigen::VectorQd::Zero(), Eigen::VectorQd::Zero()); 
    rd_.torque_desired = (rd_.Kp_diag * (rd_.q_desired - rd_.q_)) - (rd_.Kd_diag * rd_.q_dot_);

    initial_tick++;
}

//--- Joy Utils
void CustomController::xBoxJoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    double threshold = 1.0;

    move_forward = DyrosMath::minmax_cut(joy->axes[1] * threshold, -threshold, threshold);
    move_lateral = DyrosMath::minmax_cut(joy->axes[0] * threshold, -threshold, threshold);
    rotate_yaw   = DyrosMath::minmax_cut(joy->axes[3] * threshold, -threshold, threshold);

    if (is_joy_enable == true)
    {
        tm_.setStepStride(move_forward * step_length_);
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

        cbf_mgr_.pubDataFromSlowToFast();

        fast_loop_ready = true;

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

        cbf_mgr_.subDataFromSlowToFast();

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
    if(is_torque_save_init == true)
    {
        rd_.torque_init = rd_.torque_desired;

        is_torque_save_init = false;
    }

    static bool is_torque_desired_init = true;
    static int tick_torque_desired_init = 0;
    if(is_torque_desired_init == true)
    {
        for (int i = 0; i < MODEL_DOF; i++) {
            torque_target(i) = DyrosMath::cubic(tick_torque_desired_init, 0, 1000, rd_.torque_init(i), torque_target(i), 0.0, 0.0);
        }

        tick_torque_desired_init++;

        if(tick_torque_desired_init >= 1000) {
            is_torque_desired_init = false;
            std::cout << "========== INFO: INITIAL TORQUE SMOOTHING COMPLETE ==========" << std::endl;
        }
    }
}


//--- Parameter Loader
void CustomController::loadParams()
{
    Kp.setZero(MODEL_DOF); Kd.setZero(MODEL_DOF);
    Kp_virtual.setZero(MODEL_DOF_VIRTUAL); Kd_virtual.setZero(MODEL_DOF_VIRTUAL);
    
    
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
    if      (motion_mode_idx == 0){ motion_mode_ = TaskMotionType::None; }
    else if (motion_mode_idx == 1){ motion_mode_ = TaskMotionType::PelvHand;}
    else if (motion_mode_idx == 2){ motion_mode_ = TaskMotionType::Taichi; }
    else if (motion_mode_idx == 3){ motion_mode_ = TaskMotionType::Walking; }
    else if (motion_mode_idx == 4){ motion_mode_ = TaskMotionType::TeleOperation; }
    else {
        ROS_ERROR("Motion mode idx error: got %d", motion_mode_idx);
        assert(motion_mode_idx == 0 || motion_mode_idx == 1 || motion_mode_idx == 2 || motion_mode_idx == 3 || motion_mode_idx == 4);
    }
    const char *mode_name =
        (motion_mode_ == TaskMotionType::None) ? "None" 
      : (motion_mode_ == TaskMotionType::PelvHand) ? "PelvHand"
      : (motion_mode_ == TaskMotionType::Taichi)   ? "Taichi"
      : (motion_mode_ == TaskMotionType::Walking)  ? "Walking"
      : (motion_mode_ == TaskMotionType::TeleOperation)  ? "TeleOperation"
      : "Unknown";

    kin_wbc_.setTaskHierarchy(motion_mode_);

    if (motion_mode_ == TaskMotionType::Walking)
    {
        // rd_.q_pos_l_lim.segment(12, MODEL_DOF - 12).setZero();
        // rd_.q_pos_h_lim.segment(12, MODEL_DOF - 12).setZero();
        // rd_.q_vel_l_lim.segment(12, MODEL_DOF - 12).setZero();
        // rd_.q_vel_h_lim.segment(12, MODEL_DOF - 12).setZero();

        std::cout << "rd_.q_pos_l_lim: " << rd_.q_pos_l_lim.transpose() << std::endl;
        std::cout << "rd_.q_pos_h_lim: " << rd_.q_pos_h_lim.transpose() << std::endl; 
        std::cout << "rd_.q_vel_l_lim: " << rd_.q_vel_l_lim.transpose() << std::endl;
        std::cout << "rd_.q_vel_h_lim: " << rd_.q_vel_h_lim.transpose() << std::endl;
    }

    std::cout << "=====================================" << std::endl;
    std::cout << "===== Motion Mode : " << mode_name << " =====" << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << " " << std::endl;

    //--- Task Parameter
    nh_cc_.getParam("/tocabi_controller/task_param/traj_time", traj_time_);
    nh_cc_.getParam("/tocabi_controller/task_param/pelv_dist", pelv_dist_);
    nh_cc_.getParam("/tocabi_controller/task_param/hand_dist", hand_dist_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_length", step_length_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_yaw", step_yaw_);
    nh_cc_.getParam("/tocabi_controller/task_param/foot_height", foot_height_);
    nh_cc_.getParam("/tocabi_controller/task_param/step_duration", step_duration_);
    nh_cc_.getParam("/tocabi_controller/task_param/dsp_duration", dsp_duration_);

    tm_.setTrajectoryDuration(traj_time_);
    tm_.setPelvisDistance(pelv_dist_);
    tm_.setHandDistance(hand_dist_);
    tm_.setStepStride(step_length_);
    tm_.setFootHeight(foot_height_);
    tm_.setStepDuration(step_duration_);
    tm_.setDspDuration(dsp_duration_);

    std::cout << "====================================" << std::endl;
    std::cout << "======== Task Parameters ========== " << std::endl;
    std::cout << "Trajectory Time : " << traj_time_ << " sec" << std::endl;
    std::cout << "Pelvis Distance : " << pelv_dist_ << " m" << std::endl;
    std::cout << "Hand Distance : " << hand_dist_ << " m" << std::endl;
    std::cout << "Step Length : " << step_length_ << " m" << std::endl;
    std::cout << "Foot Height : " << foot_height_ << " m" << std::endl;
    std::cout << "Step Duration : " << step_duration_ << " sec" << std::endl;
    std::cout << "Double Support Duration : " << dsp_duration_ << " sec" << std::endl;
    std::cout << "====================================" << std::endl;
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

    std::cout << "=====================================" << std::endl;
    std::cout << "========== WBID Parameters ========== " << std::endl;
    std::cout << "W_qddot : " << W_qddot  << std::endl;
    std::cout << "W_cwr : " << W_cwr  << std::endl;
    std::cout << "W_energy : " << W_energy  << std::endl;
    std::cout << "friction_coeff : " << friction_coeff  << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << " " << std::endl;

    //--- CBF Parameters
    double joint_limit_cbf_alpha1, joint_limit_cbf_alpha2, joint_limit_cbf_epsilon;
    nh_cc_.getParam("/tocabi_controller/cbf/joint_limit_cbf_alpha1", joint_limit_cbf_alpha1);
    nh_cc_.getParam("/tocabi_controller/cbf/joint_limit_cbf_alpha2", joint_limit_cbf_alpha2);
    nh_cc_.getParam("/tocabi_controller/cbf/joint_limit_cbf_epsilon", joint_limit_cbf_epsilon);
    cbf_mgr_.setJointLimitCbfParameters(joint_limit_cbf_alpha1, joint_limit_cbf_alpha2, joint_limit_cbf_epsilon);
    cbf_mgr_.setJointLimitBoundaries(rd_.q_pos_l_lim, rd_.q_pos_h_lim);

    double workspace_boundary_cbf_alpha1, workspace_boundary_cbf_alpha2, workspace_boundary_cbf_epsilon, workspace_boundary_cbf_lhand, workspace_boundary_cbf_rhand;
    nh_cc_.getParam("/tocabi_controller/cbf/workspace_boundary_cbf_alpha1", workspace_boundary_cbf_alpha1);
    nh_cc_.getParam("/tocabi_controller/cbf/workspace_boundary_cbf_alpha2", workspace_boundary_cbf_alpha2);
    nh_cc_.getParam("/tocabi_controller/cbf/workspace_boundary_cbf_epsilon", workspace_boundary_cbf_epsilon);
    nh_cc_.getParam("/tocabi_controller/cbf/workspace_boundary_cbf_lhand", workspace_boundary_cbf_lhand);
    nh_cc_.getParam("/tocabi_controller/cbf/workspace_boundary_cbf_rhand", workspace_boundary_cbf_rhand);
    cbf_mgr_.setWorkspaceBoundaryCbfParameters(workspace_boundary_cbf_alpha1, workspace_boundary_cbf_alpha2, workspace_boundary_cbf_epsilon);
    std::vector<WorkspaceBoundaryPair> workspace_pairs = {
        {Left_Hand,  Left_Hand  - 5, workspace_boundary_cbf_lhand},
        {Right_Hand, Right_Hand - 5, workspace_boundary_cbf_rhand},
    };
    cbf_mgr_.setWorkspaceBoundaryPairs(workspace_pairs);

    double self_collision_cbf_alpha1, self_collision_cbf_alpha2, self_collision_cbf_epsilon;
    nh_cc_.getParam("/tocabi_controller/cbf/self_collision_cbf_alpha1",  self_collision_cbf_alpha1);
    nh_cc_.getParam("/tocabi_controller/cbf/self_collision_cbf_alpha2",  self_collision_cbf_alpha2);
    nh_cc_.getParam("/tocabi_controller/cbf/self_collision_cbf_epsilon", self_collision_cbf_epsilon);
    cbf_mgr_.setSelfCollisionCbfParameters(self_collision_cbf_alpha1, self_collision_cbf_alpha2, self_collision_cbf_epsilon);

    double obstacle_avoidance_cbf_alpha1, obstacle_avoidance_cbf_alpha2, obstacle_avoidance_cbf_epsilon;
    nh_cc_.getParam("/tocabi_controller/cbf/obstacle_avoidance_cbf_alpha1",  obstacle_avoidance_cbf_alpha1);
    nh_cc_.getParam("/tocabi_controller/cbf/obstacle_avoidance_cbf_alpha2",  obstacle_avoidance_cbf_alpha2);
    nh_cc_.getParam("/tocabi_controller/cbf/obstacle_avoidance_cbf_epsilon", obstacle_avoidance_cbf_epsilon);
    cbf_mgr_.setObstacleAvoidanceCbfParameters(obstacle_avoidance_cbf_alpha1, obstacle_avoidance_cbf_alpha2, obstacle_avoidance_cbf_epsilon);
}
