#include "task_manager.h"
#include <algorithm>
#include <cmath>

using namespace TOCABI;


// ofstream hand_log(              "/home/dyros/catkin_ws/src/tocabi_cc/data/hand_log.txt");
// ofstream hand_traj_log(         "/home/dyros/catkin_ws/src/tocabi_cc/data/hand_traj_log.txt");

ofstream hand_log(              "/home/kwan/ubuntu-20-04/catkin_ws/src/tocabi_cc/data/hand_log.txt");
ofstream hand_traj_log(         "/home/kwan/ubuntu-20-04/catkin_ws/src/tocabi_cc/data/hand_traj_log.txt");

ofstream com_log(              "/home/kwan/ubuntu-20-04/catkin_ws/src/tocabi_cc/data/com_log.txt");
ofstream com_traj_log(         "/home/kwan/ubuntu-20-04/catkin_ws/src/tocabi_cc/data/com_traj_log.txt");

ofstream force_log(              "/home/kwan/ubuntu-20-04/catkin_ws/src/tocabi_cc/data/force_log.txt");
ofstream force_traj_log(         "/home/kwan/ubuntu-20-04/catkin_ws/src/tocabi_cc/data/force_traj_log.txt");

TaskManager::TaskManager(RobotData& rd) : rd_(rd)
{
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_desired.setZero();
        rd_.link_[idx].rot_desired.setIdentity();
    }
}

void TaskManager::runTestMotion(const TaskMotionType& motion_mode)
{
    base_pos = rd_.link_[Pelvis].xpos;
    base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(2));
    
    mapBaseToSupport();
    
    switch (motion_mode)
    {
        case TaskMotionType::PelvHand:
            movePelvHandPose();
            break;
        case TaskMotionType::Taichi:
            moveTaichiMotion();
            break;
        case TaskMotionType::Walking:
            bipedalWalkingController();
            break;
        case TaskMotionType::TeleOperation:
            teleOperationController();
            break;
        case TaskMotionType::None:
            break;
        default:
            break;
    }
}

void TaskManager::movePelvHandPose()
{
    //--- Initialization
    static int sim_tick = 0;
    static bool is_transitioning = true;
    static int transition_tick = 0;
    const double transition_time = 1.0;
    const int transition_tick_end = std::max(1, static_cast<int>(transition_time * hz_));
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_desired = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj.setIdentity();
    }

    // --- (1) Reachability
    // //--- COM_id Trajectory (Support Frame)
    // rd_.link_[COM_id].x_desired    = rd_.link_[COM_id].support_xpos_init;
    // rd_.link_[COM_id].x_desired.head(2).setZero();
    // rd_.link_[COM_id].x_desired(1) = rd_.link_[COM_id].support_xpos_init(1) + pelv_dist;
    // rd_.link_[COM_id].x_desired(2) = rd_.link_[COM_id].support_xpos_init(2) - pelv_dist;
    // rd_.link_[COM_id].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].support_xpos_init, rd_.link_[COM_id].x_desired);

    // --- Hand Trajectory (Support Frame)
    // rd_.link_[Left_Hand].x_desired(0)  = rd_.link_[Left_Hand].support_xpos_init(0)  + 0.0;
    // rd_.link_[Left_Hand].x_desired(1)  = rd_.link_[Left_Hand].support_xpos_init(1)  + hand_dist;
    // rd_.link_[Left_Hand].x_desired(2)  = rd_.link_[Left_Hand].support_xpos_init(2)  + hand_dist;
    // rd_.link_[Right_Hand].x_desired(0) = rd_.link_[Right_Hand].support_xpos_init(0) + 0.0;
    // rd_.link_[Right_Hand].x_desired(1) = rd_.link_[Right_Hand].support_xpos_init(1) - hand_dist;
    // rd_.link_[Right_Hand].x_desired(2) = rd_.link_[Right_Hand].support_xpos_init(2) - hand_dist;

    // rd_.link_[Left_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Left_Hand].support_xpos_init, rd_.link_[Left_Hand].x_desired);
    // rd_.link_[Right_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Right_Hand].support_xpos_init, rd_.link_[Right_Hand].x_desired);

    // --- (2) Self Collision Avoidance
    //--- COM, Hand Trajectory (Support Frame)
    const double total_ticks = std::max(1.0, traj_time * hz_);
    const double phase = std::fmod(static_cast<double>(sim_tick), total_ticks) / total_ticks;
    const double theta = 2.0 * std::acos(-1.0) * phase;

    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].support_xpos_init;
    rd_.link_[COM_id].x_traj(2) += pelv_dist * std::sin(theta);

    rd_.link_[Left_Hand].x_traj = rd_.link_[Left_Hand].support_xpos_init;
    rd_.link_[Right_Hand].x_traj = rd_.link_[Right_Hand].support_xpos_init;

    rd_.link_[Left_Hand].x_traj(0) += hand_dist * (std::cos(theta) - 1.0);
    rd_.link_[Left_Hand].x_traj(2) += hand_dist * std::sin(theta);

    rd_.link_[Right_Hand].x_traj(0) += hand_dist * (std::cos(theta) - 1.0);
    rd_.link_[Right_Hand].x_traj(2) -= hand_dist * std::sin(theta);

    hand_log << rd_.link_[Left_Hand].support_xpos.transpose() << " " << rd_.link_[Right_Hand].support_xpos.transpose() << endl;
    hand_traj_log << rd_.link_[Left_Hand].x_traj.transpose() << " " << rd_.link_[Right_Hand].x_traj.transpose() << endl;

    // --- Convert to Base Frame Trajectory
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].x_traj - rd_.link_[Pelvis].support_xpos_init;
    rd_.link_[Left_Hand].x_traj = rd_.link_[Left_Hand].x_traj - rd_.link_[Pelvis].support_xpos_init;
    rd_.link_[Right_Hand].x_traj = rd_.link_[Right_Hand].x_traj - rd_.link_[Pelvis].support_xpos_init;

    rd_.link_[Pelvis].r_traj = DyrosMath::rotationCubic(sim_tick, 0, traj_time * hz_, rd_.link_[Pelvis].local_rotm_init, Eigen::Matrix3d::Identity());
    rd_.link_[Left_Hand].r_traj  = rd_.link_[Left_Hand].local_rotm_init;
    rd_.link_[Right_Hand].r_traj = rd_.link_[Right_Hand].local_rotm_init;

    if (is_transitioning)
    {
        const double s = std::min(1.0, std::max(0.0, static_cast<double>(transition_tick) / transition_tick_end));
        for (int i = 0; i < LINK_NUMBER + 1; ++i)
        {
            Eigen::Isometry3d T_from = Eigen::Isometry3d::Identity();
            Eigen::Isometry3d T_to = Eigen::Isometry3d::Identity();

            T_from.translation() = rd_.link_[i].local_xpos_init;
            T_from.linear() = rd_.link_[i].local_rotm_init;
            T_to.translation() = rd_.link_[i].x_traj;
            T_to.linear() = rd_.link_[i].r_traj;

            Eigen::Isometry3d T = blendIsometry(T_from, T_to, s);
            rd_.link_[i].x_traj = T.translation();
            rd_.link_[i].r_traj = T.linear();
        }

        transition_tick++;
        if (transition_tick >= transition_tick_end)
        {
            is_transitioning = false;
        }
    }

    //--- Increment Tick
    const int circling_number = 10;
    if(sim_tick <= circling_number * traj_time * hz_)    {
        sim_tick++;
    }
}

void TaskManager::moveTaichiMotion()
{
    //--- Initialization
    static int sim_tick = 0;
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_desired = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj = rd_.link_[idx].local_rotm_init;
    }

    //--- COM_id Trajectory (Support Frame)
    rd_.link_[COM_id].x_desired    = rd_.link_[COM_id].support_xpos_init;
    rd_.link_[COM_id].x_desired.head(2) = rd_.link_[Left_Foot].support_xpos_init.head(2);
    // rd_.link_[COM_id].x_desired(1) = rd_.link_[COM_id].support_xpos_init(1) + pelv_dist;

    rd_.link_[COM_id].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].support_xpos_init, rd_.link_[COM_id].x_desired);

    //--- Contact Wrench
    static double wn = sqrt(GRAVITY / rd_.link_[COM_id].support_xpos_init(2));
    Eigen::Vector2d cp_measured = (rd_.link_[COM_id].support_xpos + rd_.link_[COM_id].support_v / wn).head(2);
    Eigen::Vector2d cp_desired  = (rd_.link_[COM_id].x_traj + rd_.link_[COM_id].v_traj / wn).head(2);

    Eigen::Vector6d lfoot_contact_wrench; lfoot_contact_wrench.setZero();
    Eigen::Vector6d rfoot_contact_wrench; rfoot_contact_wrench.setZero();
    Eigen::Vector2d del_zmp; del_zmp.setZero();
    del_zmp = 1.4 * (cp_measured - cp_desired);
    
    double alpha = 0.0;
    double F_R = 0.0, F_L = 0.0;
    double Tau_all_y = 0.0, Tau_R_y = 0.0, Tau_L_y = 0.0;
    double Tau_all_x = 0.0, Tau_R_x = 0.0, Tau_L_x = 0.0;

    Eigen::Vector2d pL_sharp; pL_sharp.setZero(); pL_sharp = rd_.link_[Left_Foot].support_xpos.segment(0, 2);  
    Eigen::Vector2d pR_sharp; pR_sharp.setZero(); pR_sharp = rd_.link_[Right_Foot].support_xpos.segment(0, 2); 

    double zmp_x_ref = rd_.link_[COM_id].x_traj(0);
    double zmp_y_ref = rd_.link_[COM_id].x_traj(1);
    alpha = (zmp_y_ref + del_zmp(1) - pR_sharp(1)) / (pL_sharp(1) - pR_sharp(1));
    alpha = DyrosMath::minmax_cut(alpha, 0.0, 1.0);

    // double real_robot_mass_offset_ = 0.0; 
    double real_robot_mass_offset_ = 6.17805; // 20250305: TOCABI 101.8 kg 
    F_R = -(1 - alpha) * (rd_.link_[COM_id].mass + real_robot_mass_offset_) * GRAVITY;
    F_L =     - alpha  * (rd_.link_[COM_id].mass + real_robot_mass_offset_) * GRAVITY;

    Eigen::Vector2d pL; pL.setZero(); pL = rd_.link_[Left_Foot].support_xpos.segment(0, 2);
    Eigen::Vector2d pR; pR.setZero(); pR = rd_.link_[Right_Foot].support_xpos.segment(0, 2);

    Tau_all_x = -((pR(1) - (zmp_y_ref + del_zmp(1))) * F_R + (pL(1) - (zmp_y_ref + del_zmp(1))) * F_L);
    Tau_all_y = +((pR(0) - (zmp_x_ref + del_zmp(0))) * F_R + (pL(0) - (zmp_x_ref + del_zmp(0))) * F_L);

    Tau_R_x =(1 - alpha) * Tau_all_x;
    Tau_R_y =(1 - alpha) * Tau_all_y;
    Tau_L_x =     alpha  * Tau_all_x;
    Tau_L_y =     alpha  * Tau_all_y;

    lfoot_contact_wrench << 0.0, 0.0, F_L, Tau_L_x, Tau_L_y, 0.0;
    rfoot_contact_wrench << 0.0, 0.0, F_R, Tau_R_x, Tau_R_y, 0.0;

    lfoot_contact_wrench *= (-1.0);
    rfoot_contact_wrench *= (-1.0);

    rd_.LF_FT_DES = lfoot_contact_wrench;
    rd_.RF_FT_DES = rfoot_contact_wrench;

    static Vector6d LF_FT_LPF = rd_.LF_FT;
    static Vector6d RF_FT_LPF = rd_.RF_FT;

    for (int i = 0; i < 6; i++)
    {
        LF_FT_LPF(i) = DyrosMath::lpf(rd_.LF_FT(i), LF_FT_LPF(i), 2000, 73);
        RF_FT_LPF(i) = DyrosMath::lpf(rd_.RF_FT(i), RF_FT_LPF(i), 2000, 73);
    }

    //---Force Z axis feedback
    rd_.LF_FT_DES(2) += 0.73 * (rd_.LF_FT_DES(2) + LF_FT_LPF(2));
    rd_.RF_FT_DES(2) += 0.73 * (rd_.RF_FT_DES(2) + RF_FT_LPF(2));

    // //---Moment X axis feedback
    // rd_.LF_FT_DES(3) += 1.0 * (rd_.LF_FT_DES(3) + LF_FT_LPF(3));
    // rd_.RF_FT_DES(3) += 1.0 * (rd_.RF_FT_DES(3) + RF_FT_LPF(3));

    // //---Moment Y axis feedback
    rd_.LF_FT_DES(4) += 0.73 * (rd_.LF_FT_DES(4) + LF_FT_LPF(4));
    rd_.RF_FT_DES(4) += 0.73 * (rd_.RF_FT_DES(4) + RF_FT_LPF(4));

    com_log << rd_.link_[COM_id].support_xpos.transpose() << endl;
    com_traj_log << rd_.link_[COM_id].x_traj.transpose() << endl;

    force_log << rd_.LF_FT.transpose() << " " << rd_.RF_FT.transpose() << endl;
    force_traj_log << rd_.LF_FT_DES.transpose() << " " << rd_.RF_FT_DES.transpose() << endl;

    //--- COM_id Trajectory (Base Frame)
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].x_traj - rd_.link_[Pelvis].support_xpos_init;

    // --- Hand Trajectory (Support Frame)
    for (int idx = 1; idx < 3; idx++)
    {
        rd_.link_[Left_Hand].x_desired(idx)  = rd_.link_[Left_Hand].support_xpos_init(idx) + 2.0 * hand_dist;
        rd_.link_[Right_Hand].x_desired(idx) = rd_.link_[Right_Hand].support_xpos_init(idx) - hand_dist;
    }

    rd_.link_[Left_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Left_Hand].support_xpos_init, rd_.link_[Left_Hand].x_desired);
    rd_.link_[Right_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Right_Hand].support_xpos_init, rd_.link_[Right_Hand].x_desired);

    hand_log << rd_.link_[Left_Hand].support_xpos.transpose() << " " << rd_.link_[Right_Hand].support_xpos.transpose() << endl;
    hand_traj_log << rd_.link_[Left_Hand].x_traj.transpose() << " " << rd_.link_[Right_Hand].x_traj.transpose() << endl;

    // --- Hand Trajectory (Base Frame)
    rd_.link_[Left_Hand].x_traj = rd_.link_[Left_Hand].x_traj - rd_.link_[Pelvis].support_xpos_init;
    rd_.link_[Right_Hand].x_traj = rd_.link_[Right_Hand].x_traj - rd_.link_[Pelvis].support_xpos_init;
    
    rd_.link_[Left_Hand].r_traj  = rd_.link_[Left_Hand].local_rotm_init;
    rd_.link_[Right_Hand].r_traj = rd_.link_[Right_Hand].local_rotm_init;

    //--- Swing Foot Trajectory (Base Frame)
    rd_.link_[Right_Foot].x_desired = rd_.link_[Right_Foot].support_xpos_init;
    rd_.link_[Right_Foot].x_desired(2) = rd_.link_[Right_Foot].support_xpos_init(2) + foot_height;
    rd_.link_[Right_Foot].SetTrajectoryQuintic(sim_tick, 1.5 * traj_time * hz_, 2.5 * traj_time * hz_, rd_.link_[Right_Foot].support_xpos_init, rd_.link_[Right_Foot].x_desired);

    rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].x_traj - rd_.link_[Pelvis].support_xpos_init;

    // rd_.link_[Left_Foot].r_traj  = rd_.link_[Left_Foot].local_rotm_init;
    // rd_.link_[Right_Foot].r_traj = rd_.link_[Right_Foot].local_rotm_init;

    //--- Upper Body Trajectory (Base Frame)
    // rd_.link_[Pelvis].r_traj = DyrosMath::rotationCubic(sim_tick, 1.5 * traj_time * hz_, 2.5 * traj_time * hz_, 
    //                                                         rd_.link_[Pelvis].local_rotm_init, 
    //                                                         DyrosMath::rotateWithX(-10 * DEG2RAD));
    // rd_.link_[Upper_Body].r_traj = DyrosMath::rotationCubic(sim_tick, 1.5 * traj_time * hz_, 2.5 * traj_time * hz_, 
    //                                                         rd_.link_[Upper_Body].local_rotm_init, 
    //                                                         DyrosMath::rotateWithX(-10 * DEG2RAD));

    //--- Increment Tick
    sim_tick++;

    //--- contact transition
    if (sim_tick == traj_time * hz_ - 1)
    {
        // if (support_phase_indicator_ == ContactIndicator::DoubleSupport)
        // {
        //     rd_.is_left_contact_transition = true;
        
        //     support_phase_indicator_ = ContactIndicator::LeftSingleSupport;
        // }
    }
}

void TaskManager::bipedalWalkingController()
{
    static WalkingManager wm_(rd_); 
    
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj = rd_.link_[idx].local_rotm_init;
    }

    static bool is_wm_init = true;
    if(is_wm_init == true)
    {
        wm_.setControlFrequency(hz_);
        wm_.setCenterOfMassHeight(rd_.link_[COM_id].support_xpos_init(2));
        wm_.setTransferDuration(2.0);
        wm_.findPreviewParameter(1.0 / hz_, 1.6 * hz_);

        is_wm_init = false;
    } 

    wm_.updateContactState(rd_.ee_[0].contact, rd_.ee_[1].contact);

    wm_.updateSupportPhaseIndicator();
    support_phase_indicator_ = wm_.getSupportPhaseIndicator();
    mapBaseToSupport();

    wm_.setWalkingParameter(step_length, step_lateral, step_yaw, foot_height);
    wm_.setStepDuration(step_duration);
    wm_.setDspDuration(dsp_duration);

    wm_.computeWalkingMotion();
}

void TaskManager::teleOperationController()
{    
    static TeleOperationManager teleop_(rd_);
    teleop_.setWalkingParameter(step_length, foot_height, step_duration);
    static bool calibration_done = false;
    static bool ready_pose_mode = false;
    static bool avatar_mode = false;

    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj = rd_.link_[idx].local_rotm_init;
    }

    //--- User Command
    if(teleop_.leftSecondaryPressedOnce())
    {
        std::cout << "========== CALIBRATION MODE ==========" << std::endl;

        calibration_done = teleop_.transformAllOK();    // IF ALL TFs ARE RECEIVED, THEN TRUE; ELSE FALSE
        if(!calibration_done){
            std::cout << "TF MISSING DURING CALIBRATION!" << std::endl;
            std::cout << "PLEASE TRY RETARGET AGAIN!" << std::endl;
        }
    }
    else if(teleop_.leftPrimaryPressedOnce())
    {
        if(calibration_done) {
            std::cout << "========== READY POSE ===========" << std::endl;
            
            ready_pose_mode = true;
        }
        else{
            std::cout << "========== PLEASE DO CALIBRATION FIRST ==========" << std::endl;
        }
    }
    else if(teleop_.rightSecondaryPressedOnce())
    {
        if(calibration_done) {
            std::cout << "========== PAUSE MODE ==========" << std::endl;

            for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
            {
                //--- Base frame
                rd_.link_[idx].local_xpos_init = rd_.link_[idx].local_xpos;
                rd_.link_[idx].local_rotm_init = rd_.link_[idx].local_rotm;          
                
                //--- Support frame
                rd_.link_[idx].support_xpos_init = rd_.link_[idx].support_xpos;
                rd_.link_[idx].support_rotm_init = rd_.link_[idx].support_rotm;
            }

            avatar_mode = false;        
        }
        else{
            std::cout << "========== PLEASE DO CALIBRATION FIRST ==========" << std::endl;
        }
    }
    else if(teleop_.rightPrimaryPressedOnce())
    {
        if(calibration_done) {
            std::cout << "========== AVATAR MODE ===========" << std::endl;
            avatar_mode = true;
        }
        else{
            std::cout << "========== PLEASE DO CALIBRATION FIRST ==========" << std::endl;
        }
    }

    //--- Teleoperation Control
    teleop_.updateTrackerFromPoseArray();
    support_phase_indicator_ = teleop_.getSupportPhaseIndicator();
    mapBaseToSupport();
    teleop_.calibrationFunction(calibration_done);
    teleop_.motionRetargeting(avatar_mode);
    teleop_.sendReadyPoseToRobot(ready_pose_mode);
}


//--- Frame Transformation
void TaskManager::mapBaseToSupport()
{
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        //--- Support frame
        if(support_phase_indicator_ == ContactIndicator::DoubleSupport)
        {
            rd_.link_[idx].support_xpos = rd_.link_[idx].local_xpos - rd_.link_[Left_Foot].local_xpos;
            rd_.link_[idx].support_rotm = rd_.link_[idx].local_rotm;
            rd_.link_[idx].support_v    = rd_.link_[idx].local_v;
            rd_.link_[idx].support_w    = rd_.link_[idx].local_w;
        }
        else if(support_phase_indicator_ == ContactIndicator::LeftSingleSupport)
        {
            rd_.link_[idx].support_xpos = rd_.link_[idx].local_xpos - rd_.link_[Left_Foot].local_xpos;
            rd_.link_[idx].support_rotm = rd_.link_[idx].local_rotm;
            rd_.link_[idx].support_v    = rd_.link_[idx].local_v;
            rd_.link_[idx].support_w    = rd_.link_[idx].local_w;
        }
        else if(support_phase_indicator_ == ContactIndicator::RightSingleSupport)
        {
            rd_.link_[idx].support_xpos = rd_.link_[idx].local_xpos - rd_.link_[Right_Foot].local_xpos;
            rd_.link_[idx].support_rotm = rd_.link_[idx].local_rotm;
            rd_.link_[idx].support_v    = rd_.link_[idx].local_v;
            rd_.link_[idx].support_w    = rd_.link_[idx].local_w;
        }
        else
        {
            ROS_ERROR("CONTACT MISSING");
            assert((support_phase_indicator_ == ContactIndicator::DoubleSupport)
                || (support_phase_indicator_ == ContactIndicator::LeftSingleSupport)
                || (support_phase_indicator_ == ContactIndicator::RightSingleSupport));
        }
    }

    static bool is_tm_init = true;
    if(is_tm_init)
    {
        for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
        {
            //--- Support frame
            rd_.link_[idx].support_xpos_init = rd_.link_[idx].support_xpos;
            rd_.link_[idx].support_rotm_init = rd_.link_[idx].support_rotm;                             
            rd_.link_[idx].support_v_init    = rd_.link_[idx].support_v;                               
            rd_.link_[idx].support_w_init    = rd_.link_[idx].support_w;
        }

        is_tm_init = false;
    }
}

//--- Class Setter
void TaskManager::setTrajectoryDuration(double &traj_time_)
{
    traj_time = traj_time_;
}

void TaskManager::setPelvisDistance(double &pelv_dist_)
{
    pelv_dist = pelv_dist_;
}

void TaskManager::setHandDistance(double &hand_dist_)
{
    hand_dist = hand_dist_;
}

void TaskManager::setStepStride(double step_length_)
{
    step_length = step_length_;
}

void TaskManager::setStepLateral(double step_lateral_)
{
    step_lateral = step_lateral_;
}

void TaskManager::setStepYaw(double step_yaw_)
{
    step_yaw = step_yaw_;
}

void TaskManager::setFootHeight(double &foot_height_)
{
    foot_height = foot_height_;
}

void TaskManager::setStepDuration(double &step_duration_)
{
    step_duration = step_duration_;
}

void TaskManager::setDspDuration(double &dsp_duration_)
{
    dsp_duration = dsp_duration_;
}
