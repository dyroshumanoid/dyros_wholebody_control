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
    double T = traj_time; // period
    double wn = (2.0 * M_PI) / T;
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].support_xpos_init;
    rd_.link_[COM_id].x_traj(1) = rd_.link_[COM_id].support_xpos_init(1) + pelv_dist * sin(wn * sim_tick / hz_);
    rd_.link_[COM_id].v_traj(1) = wn * pelv_dist * cos(wn * sim_tick / hz_);

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

    //--- Map Trajectory from Base frame to Global frame
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_traj = base_rot * (rd_.link_[idx].x_traj + base_pos);
        rd_.link_[idx].r_traj = base_rot * rd_.link_[idx].r_traj;
    }

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
    rd_.link_[COM_id].x_desired(1) = rd_.link_[COM_id].support_xpos_init(1) + pelv_dist;
    rd_.link_[COM_id].x_desired(2) = rd_.link_[COM_id].support_xpos_init(2) - pelv_dist;

    //--- COM_id Trajectory (Base Frame)
    rd_.link_[COM_id].x_desired    = rd_.link_[COM_id].x_desired - rd_.link_[Pelvis].support_xpos;
    rd_.link_[COM_id].SetTrajectoryQuintic(sim_tick, 0, traj_time * hz_, rd_.link_[COM_id].local_xpos_init, rd_.link_[COM_id].x_desired);

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

    //--- Map Trajectory from Base frame to Global frame
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_traj = base_rot * (rd_.link_[idx].x_traj + base_pos);
        rd_.link_[idx].r_traj = base_rot * rd_.link_[idx].r_traj;
    }

    //--- Increment Tick
    sim_tick++;

    //--- contact transition
    bool local_LF_contact = rd_.ee_[0].contact;
    bool local_RF_contact = rd_.ee_[1].contact;
    if (sim_tick == traj_time * hz_ - 1)
    {
        if (local_LF_contact == true && local_RF_contact == true)
        {
            rd_.is_left_contact_transition = true;
            rd_.is_right_contact_transition = false;
        }
        else
        {
            ROS_ERROR("CONTACT MISSING");
            assert((local_LF_contact == true && local_RF_contact == true)
                || (local_LF_contact == true && local_RF_contact != true)
                || (local_LF_contact != true && local_RF_contact == true));
        }
    }
}

void TaskManager::bipedalWalkingController()
{
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].x_traj = rd_.link_[idx].local_xpos_init;
        rd_.link_[idx].r_traj = rd_.link_[idx].local_rotm_init;
    }

    static WalkingManager wm_(rd_); 

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
    wm_.setWalkingParameter(step_length, 0.0, foot_height);
    wm_.setStepDuration(step_duration);

    wm_.computeWalkingMotion();
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

void TaskManager::isForceTorqueSensorAvailable(const bool &is_ft_sensor_available_)
{
    is_ft_sensor_available = is_ft_sensor_available_;
}