#include "control_manager.h"

using namespace TOCABI;

ControlManager::ControlManager(RobotData& rd) : rd_(rd)
{
    // TODO : RBDL -> Pinocchio
    std::string urdf_path;
    ros::param::get("/tocabi_controller/urdf_path", urdf_path);
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, true, false);
}

void ControlManager::update()
{
    contactStateMachine();
    mapGlobalToBase();
    mapBaseToSupport();

    static bool is_cm_init = true;
    if(is_cm_init == true)
    {           
        saveInitialState();
        is_cm_init = false;
    }
}

void ControlManager::contactStateMachine()
{
    //--- Contact Change Trigger
    if(rd_.is_left_contact_transition == true)
    {
        WBC::SetContact(rd_, true, false);
        
        rd_.is_left_contact_transition = false;
    }
    else if(rd_.is_right_contact_transition == true)
    {
        WBC::SetContact(rd_, false, true);

        rd_.is_right_contact_transition = false;
    }
    else
    {
        // No contact change
    }

    local_LF_contact = rd_.ee_[0].contact;
    local_RF_contact = rd_.ee_[1].contact;
}

void ControlManager::mapGlobalToBase()
{
    //--- Robot States
    base_pos = rd_.link_[Pelvis].xpos; 
    base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(2)); 
    
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        rd_.link_[idx].local_xpos = base_rot.transpose() * (rd_.link_[idx].xpos - base_pos);
        rd_.link_[idx].local_rotm = base_rot.transpose() *  rd_.link_[idx].rotm;                             
        rd_.link_[idx].local_v    = base_rot.transpose() *  rd_.link_[idx].v;                               
        rd_.link_[idx].local_w    = base_rot.transpose() *  rd_.link_[idx].w;
    }
}

void ControlManager::mapBaseToSupport()
{
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        //--- Support frame
        if (local_LF_contact == true && local_RF_contact == true)
        {
            rd_.link_[idx].support_xpos = rd_.link_[idx].local_xpos - rd_.link_[Left_Foot].local_xpos;
            rd_.link_[idx].support_rotm = rd_.link_[idx].local_rotm;
            rd_.link_[idx].support_v    = rd_.link_[idx].local_v;
            rd_.link_[idx].support_w    = rd_.link_[idx].local_w;
        }
        else if (local_LF_contact == true && local_RF_contact != true)
        {
            rd_.link_[idx].support_xpos = rd_.link_[idx].local_xpos - rd_.link_[Left_Foot].local_xpos;
            rd_.link_[idx].support_rotm = rd_.link_[idx].local_rotm;
            rd_.link_[idx].support_v    = rd_.link_[idx].local_v;
            rd_.link_[idx].support_w    = rd_.link_[idx].local_w;
        }
        else if (local_LF_contact != true && local_RF_contact == true)
        {
            rd_.link_[idx].support_xpos = rd_.link_[idx].local_xpos - rd_.link_[Right_Foot].local_xpos;
            rd_.link_[idx].support_rotm = rd_.link_[idx].local_rotm;
            rd_.link_[idx].support_v    = rd_.link_[idx].local_v;
            rd_.link_[idx].support_w    = rd_.link_[idx].local_w;
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


void ControlManager::saveInitialState()
{
    rd_.q_desired_virtual = rd_.local_q_virtual_.head(MODEL_DOF_VIRTUAL);
    rd_.q_dot_desired_virtual.setZero();

    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        //--- Global frame
        rd_.link_[idx].rot_init = rd_.link_[idx].local_rotm;                             

        //--- Base frame
        rd_.link_[idx].local_xpos_init = rd_.link_[idx].local_xpos;
        rd_.link_[idx].local_rotm_init = rd_.link_[idx].local_rotm;                             
        rd_.link_[idx].local_v_init    = rd_.link_[idx].local_v;                               
        rd_.link_[idx].local_w_init    = rd_.link_[idx].local_w;

        //--- Support frame
        rd_.link_[idx].support_xpos_init = rd_.link_[idx].support_xpos;
        rd_.link_[idx].support_rotm_init = rd_.link_[idx].support_rotm;                             
        rd_.link_[idx].support_v_init    = rd_.link_[idx].support_v;                               
        rd_.link_[idx].support_w_init    = rd_.link_[idx].support_w;
    }
}