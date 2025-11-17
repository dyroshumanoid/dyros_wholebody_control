#include "task_manager.h"

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

    //--- COM_id Trajectory (Support Frame)
    rd_.link_[COM_id].x_desired    = rd_.link_[COM_id].support_xpos_init;
    rd_.link_[COM_id].x_desired.head(2).setZero();
    rd_.link_[COM_id].x_desired(1) = rd_.link_[COM_id].support_xpos_init(1) + pelv_dist;
    rd_.link_[COM_id].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].support_xpos_init, rd_.link_[COM_id].x_desired);

    static WalkingManager wm_(rd_); 
    wm_.setCenterOfMassHeight(rd_.link_[COM_id].support_xpos_init(2));
    wm_.cp_desired_ = rd_.link_[COM_id].x_traj.head(2) + rd_.link_[COM_id].v_traj.head(2) / wm_.wn;
    wm_.zmp_x_ref = rd_.link_[COM_id].x_traj(0);
    wm_.zmp_y_ref = rd_.link_[COM_id].x_traj(1);
    wm_.contactWrenchCalculator();

    //--- COM_id Trajectory (Base Frame)
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].x_traj - rd_.link_[Pelvis].support_xpos;

    //--- Both Hand Trajectories (Base Frame)
    for (int idx = 1; idx < 3; idx++)
    {
        rd_.link_[Left_Hand].x_desired(idx)  = rd_.link_[Left_Hand].local_xpos_init(idx)  + hand_dist;
        rd_.link_[Right_Hand].x_desired(idx) = rd_.link_[Right_Hand].local_xpos_init(idx) - hand_dist;
    }

    rd_.link_[Left_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Left_Hand].local_xpos_init, rd_.link_[Left_Hand].x_desired);
    rd_.link_[Right_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Right_Hand].local_xpos_init, rd_.link_[Right_Hand].x_desired);

    //--- Increment Tick
    sim_tick++;
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
    
    //--- Contact Wrench
    static WalkingManager wm_(rd_); 
    wm_.setCenterOfMassHeight(rd_.link_[COM_id].support_xpos_init(2));
    wm_.cp_measured_ = (rd_.link_[COM_id].support_xpos + rd_.link_[COM_id].support_v / wm_.wn).head(2);
    wm_.cp_desired_ = rd_.link_[COM_id].x_traj.head(2) + rd_.link_[COM_id].v_traj.head(2) / wm_.wn;
    wm_.zmp_x_ref = DyrosMath::cubic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].support_xpos_init(0), 0.0, 0.0, 0.0);
    wm_.zmp_y_ref = DyrosMath::cubic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].support_xpos_init(1), 0.0, 0.0, 0.0);

    if(support_phase_indicator_ == ContactIndicator::DoubleSupport)
    {
        wm_.contactWrenchCalculator();
    }
    else if(support_phase_indicator_ == ContactIndicator::LeftSingleSupport)
    {
        rd_.LF_FT_DES.setZero();
        rd_.RF_FT_DES.setZero();

        rd_.LF_FT_DES.setZero();
        rd_.LF_FT_DES(2) = rd_.link_[COM_id].mass * GRAVITY;
        
        Eigen::Vector2d del_zmp = 1.4 * (wm_.cp_measured_ - wm_.cp_desired_);

        rd_.LF_FT_DES(3) = -(0.0 - (wm_.zmp_y_ref + del_zmp(1)) * rd_.link_[COM_id].mass * GRAVITY);
        rd_.LF_FT_DES(4) = +(0.0 - (wm_.zmp_x_ref + del_zmp(0)) * rd_.link_[COM_id].mass * GRAVITY);
    }

    //--- COM_id Trajectory (Base Frame)
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].x_traj - rd_.link_[Pelvis].support_xpos;

    //--- Hand Trajectory (Base Frame)
    for (int idx = 1; idx < 3; idx++)
    {
        rd_.link_[Left_Hand].x_desired(idx)  = rd_.link_[Left_Hand].local_xpos_init(idx) + hand_dist;
        rd_.link_[Right_Hand].x_desired(idx) = rd_.link_[Right_Hand].local_xpos_init(idx) - hand_dist;
    }
    rd_.link_[Left_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Left_Hand].local_xpos_init, rd_.link_[Left_Hand].x_desired);
    rd_.link_[Right_Hand].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[Right_Hand].local_xpos_init, rd_.link_[Right_Hand].x_desired);

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
        wm_.isForceTorqueSensorAvailable(is_ft_sensor_available);

        is_wm_init = false;
    } 

    wm_.updateContactState(rd_.ee_[0].contact, rd_.ee_[1].contact);

    wm_.updateSupportPhaseIndicator();
    support_phase_indicator_ = wm_.getSupportPhaseIndicator();
    mapBaseToSupport();

    wm_.setWalkingParameter(step_length, 0.0, foot_height);
    wm_.setStepDuration(step_duration);
    wm_.setDspDuration(dsp_duration);

    wm_.computeWalkingMotion();
}

void TaskManager::teleOperationController()
{    
    static WalkingManager wm_(rd_); 
    static TeleOperationManager teleop_(rd_); 

    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj = rd_.link_[idx].local_rotm_init;
    }

    teleop_.motionRetargeting();

    wm_.setCenterOfMassHeight(rd_.link_[COM_id].support_xpos_init(2));
    wm_.cp_desired_ = rd_.link_[COM_id].x_traj.head(2) + rd_.link_[COM_id].v_traj.head(2) / wm_.wn;
    wm_.zmp_x_ref = rd_.link_[COM_id].x_traj(0);
    wm_.zmp_y_ref = rd_.link_[COM_id].x_traj(1);
    wm_.contactWrenchCalculator();
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
void TaskManager::setControlFrequency(double &hz)
{
    hz_ = hz;
}

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

void TaskManager::setStepStride(double &step_length_)
{
    step_length = step_length_;
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

void TaskManager::isForceTorqueSensorAvailable(const bool &is_ft_sensor_available_)
{
    is_ft_sensor_available = is_ft_sensor_available_;
}