#include "task_manager.h"
#include <algorithm>
#include <cmath>

using namespace TOCABI;

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
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_desired = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj.setIdentity();
    }

    // //--- COM_id Trajectory (Support Frame)
    rd_.link_[COM_id].x_desired    = rd_.link_[COM_id].support_xpos_init;
    rd_.link_[COM_id].x_desired.head(2).setZero();
    rd_.link_[COM_id].x_desired(1) = rd_.link_[COM_id].support_xpos_init(1) + pelv_dist;
    rd_.link_[COM_id].x_desired(2) = rd_.link_[COM_id].support_xpos_init(2) - pelv_dist;
    rd_.link_[COM_id].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].support_xpos_init, rd_.link_[COM_id].x_desired);

    rd_.link_[Pelvis].r_traj = DyrosMath::rotationCubic(sim_tick, 0, traj_time * hz_, rd_.link_[Pelvis].local_rotm_init, Eigen::Matrix3d::Identity());

    //--- COM_id Trajectory (Base Frame)
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].x_traj - rd_.link_[Pelvis].support_xpos;

    // --- Hand Trajectory (Base Frame)
    // rd_.link_[Left_Hand].x_desired(1)  = rd_.link_[Left_Hand].local_xpos_init(1) +  hand_dist;
    // rd_.link_[Right_Hand].x_desired(1) = rd_.link_[Right_Hand].local_xpos_init(1) - hand_dist;
    // rd_.link_[Left_Hand].x_desired(2)  = rd_.link_[Left_Hand].local_xpos_init(2) +  hand_dist;
    // rd_.link_[Right_Hand].x_desired(2) = rd_.link_[Right_Hand].local_xpos_init(2) - hand_dist;
    //
    // rd_.link_[Left_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Left_Hand].local_xpos_init, rd_.link_[Left_Hand].x_desired);
    // rd_.link_[Right_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Right_Hand].local_xpos_init, rd_.link_[Right_Hand].x_desired);

    const double total_ticks = std::max(1.0, traj_time * hz_);
    const double phase = std::fmod(static_cast<double>(sim_tick), total_ticks) / total_ticks;
    const double theta = 2.0 * std::acos(-1.0) * phase;

    rd_.link_[Left_Hand].x_traj = rd_.link_[Left_Hand].local_xpos_init;
    rd_.link_[Right_Hand].x_traj = rd_.link_[Right_Hand].local_xpos_init;

    rd_.link_[Left_Hand].x_traj(0) += hand_dist * (std::cos(theta) - 1.0);
    rd_.link_[Left_Hand].x_traj(2) += hand_dist * std::sin(theta);

    rd_.link_[Right_Hand].x_traj(0) += hand_dist * (std::cos(theta) - 1.0);
    rd_.link_[Right_Hand].x_traj(2) -= hand_dist * std::sin(theta);

    rd_.link_[Left_Hand].r_traj  = rd_.link_[Left_Hand].local_rotm_init;
    rd_.link_[Right_Hand].r_traj = rd_.link_[Right_Hand].local_rotm_init;

    //--- Increment Tick
    const int circling_number = 3;
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
        rd_.link_[idx].r_traj.setIdentity();
    }

    //--- COM_id Trajectory (Support Frame)
    rd_.link_[COM_id].x_desired    = rd_.link_[COM_id].support_xpos_init;
    rd_.link_[COM_id].x_desired.head(2).setZero();
    rd_.link_[COM_id].x_desired(1) = rd_.link_[COM_id].support_xpos_init(1) + pelv_dist;
    rd_.link_[COM_id].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].support_xpos_init, rd_.link_[COM_id].x_desired);
    
    //--- COM_id Trajectory (Base Frame)
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].x_traj - rd_.link_[Pelvis].support_xpos;

    // --- Hand Trajectory (Base Frame)
    for (int idx = 1; idx < 3; idx++)
    {
        rd_.link_[Left_Hand].x_desired(idx)  = rd_.link_[Left_Hand].local_xpos_init(idx) + hand_dist;
        rd_.link_[Right_Hand].x_desired(idx) = rd_.link_[Right_Hand].local_xpos_init(idx) - hand_dist;
    }

    rd_.link_[Left_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Left_Hand].local_xpos_init, rd_.link_[Left_Hand].x_desired);
    rd_.link_[Right_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Right_Hand].local_xpos_init, rd_.link_[Right_Hand].x_desired);
    
    rd_.link_[Left_Hand].r_traj  = rd_.link_[Left_Hand].local_rotm_init;
    rd_.link_[Right_Hand].r_traj = rd_.link_[Right_Hand].local_rotm_init;

    //--- Swing Foot Trajectory (Base Frame)
    rd_.link_[Right_Foot].x_desired(2) = rd_.link_[Right_Foot].local_xpos_init(2) + foot_height;
    rd_.link_[Right_Foot].SetTrajectoryQuintic(sim_tick, traj_time * hz_, 2.0 * traj_time * hz_, rd_.link_[Right_Foot].local_xpos_init, rd_.link_[Right_Foot].x_desired);

    //--- Increment Tick
    sim_tick++;

    //--- contact transition
    if (sim_tick == traj_time * hz_ - 1)
    {
        if (support_phase_indicator_ == ContactIndicator::DoubleSupport)
        {
            rd_.is_left_contact_transition = true;
        
            support_phase_indicator_ == ContactIndicator::LeftSingleSupport;
        }
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
