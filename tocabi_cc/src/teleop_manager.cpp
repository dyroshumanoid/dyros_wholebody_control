#include "teleop_manager.h"

TeleOperationManager::TeleOperationManager(RobotData &rd) : rd_(rd) {
    xr_pose_sub_ = nh_.subscribe(
        "/xr_pose",
        10,
        &TeleOperationManager::xrPoseCallback,
        this,
        ros::TransportHints().tcpNoDelay(true)
    );
}

void TeleOperationManager::callAvailableQueue()
{
    queue_.callAvailable(ros::WallDuration());
}

//----------------------//
//--- TF Data Update ---//
void TeleOperationManager::updateTrackerFromTF()
{
    ok_pelvis   = lookupTF("map", "pico_pelvis",        tracker_pelv_pose_raw_);
    ok_chest    = lookupTF("map", "pico_spine3",        tracker_chest_pose_raw_);
    ok_head     = lookupTF("map", "pico_head",          tracker_head_pose_raw_);
    ok_lhand    = lookupTF("map", "pico_left_hand",     tracker_lhand_pose_raw_);
    ok_lshould  = lookupTF("map", "pico_left_shoulder", tracker_lshoulder_pose_raw_);
    ok_lelbow   = lookupTF("map", "pico_left_elbow",    tracker_lelbow_pose_raw_);
    ok_rhand    = lookupTF("map", "pico_right_hand",    tracker_rhand_pose_raw_);
    ok_rshould  = lookupTF("map", "pico_right_shoulder",tracker_rshoulder_pose_raw_);
    ok_relbow   = lookupTF("map", "pico_right_elbow",   tracker_relbow_pose_raw_);
    ok_lhip     = lookupTF("map", "pico_left_hip",      tracker_lhip_pose_raw_);
    ok_rhip     = lookupTF("map", "pico_right_hip",     tracker_rhip_pose_raw_);
    ok_lfoot    = lookupTF("map", "pico_left_foot",     tracker_lfoot_pose_raw_);
    ok_rfoot    = lookupTF("map", "pico_right_foot",    tracker_rfoot_pose_raw_);

    tf_all_ok_ = ok_pelvis && ok_chest && ok_head &&
                 ok_lhand && ok_lshould && ok_lelbow &&
                 ok_rhand && ok_rshould && ok_relbow &&
                 ok_lfoot && ok_rfoot;

    if (!tf_all_ok_)
    {
        std::stringstream ss;
        ss << "[TF MISSING] ";

        if (!ok_pelvis)  ss << "pelvis ";
        if (!ok_chest)   ss << "chest ";
        if (!ok_head)    ss << "head ";
        if (!ok_lhand)   ss << "lhand ";
        if (!ok_lshould) ss << "lshoulder ";
        if (!ok_lelbow)  ss << "lelbow ";
        if (!ok_rhand)   ss << "rhand ";
        if (!ok_rshould) ss << "rshoulder ";
        if (!ok_relbow)  ss << "relbow ";
        if (!ok_lhip)    ss << "lhip ";
        if (!ok_rhip)    ss << "rhip ";
        if (!ok_lfoot)   ss << "lfoot ";
        if (!ok_rfoot)   ss << "rfoot ";

        ROS_WARN_THROTTLE(1.0, "%s", ss.str().c_str());
        return;
    }

    //--- Tracker Mapping for Local Frame
    Eigen::Isometry3d T_world_;
    T_world_.linear().setZero();
    T_world_.translation().setZero();
    T_world_.translation() = tracker_pelv_pose_raw_.translation();
    T_world_.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(tracker_pelv_pose_raw_.linear())(2)); 

    tracker_head_pose_mapped_      = DyrosMath::inverseIsometry3d(T_world_) * tracker_head_pose_raw_;
    tracker_chest_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_chest_pose_raw_;
    tracker_pelv_pose_mapped_      = DyrosMath::inverseIsometry3d(T_world_) * tracker_pelv_pose_raw_;
    tracker_lshoulder_pose_mapped_ = DyrosMath::inverseIsometry3d(T_world_) * tracker_lshoulder_pose_raw_;
    tracker_rshoulder_pose_mapped_ = DyrosMath::inverseIsometry3d(T_world_) * tracker_rshoulder_pose_raw_;
    tracker_lelbow_pose_mapped_    = DyrosMath::inverseIsometry3d(T_world_) * tracker_lelbow_pose_raw_;
    tracker_relbow_pose_mapped_    = DyrosMath::inverseIsometry3d(T_world_) * tracker_relbow_pose_raw_;
    tracker_lhand_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_lhand_pose_raw_;
    tracker_rhand_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_rhand_pose_raw_;
    tracker_lhip_pose_mapped_      = DyrosMath::inverseIsometry3d(T_world_) * tracker_lhip_pose_raw_;
    tracker_rhip_pose_mapped_      = DyrosMath::inverseIsometry3d(T_world_) * tracker_rhip_pose_raw_;
    tracker_lfoot_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_lfoot_pose_raw_;
    tracker_rfoot_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_rfoot_pose_raw_;

    //--- Human Contact State Estimation
    double lift_threshold = 0.05; 
    double foot_height_gap = tracker_lfoot_pose_mapped_.translation()(2) - tracker_rfoot_pose_mapped_.translation()(2);
    if (foot_height_gap > lift_threshold){
        human_contact_indicator_ = ContactIndicator::RightSingleSupport;
    }
    else if (foot_height_gap < -lift_threshold){
        human_contact_indicator_ = ContactIndicator::LeftSingleSupport;
    }
    else{
        human_contact_indicator_ = ContactIndicator::DoubleSupport;
    }

    //--- Tracker Mapping for Support Foot
    if(human_contact_indicator_ == ContactIndicator::DoubleSupport)
    {
        tracker_pelv_pos_mapped_support_ = tracker_pelv_pose_mapped_.translation()   - tracker_lfoot_pose_mapped_.translation();
        tracker_lfoot_pos_mapped_support_ = tracker_lfoot_pose_mapped_.translation() - tracker_lfoot_pose_mapped_.translation();
        tracker_rfoot_pos_mapped_support_ = tracker_rfoot_pose_mapped_.translation() - tracker_lfoot_pose_mapped_.translation();
    }
    else if(human_contact_indicator_ == ContactIndicator::LeftSingleSupport)
    {
        tracker_pelv_pos_mapped_support_ = tracker_pelv_pose_mapped_.translation()   - tracker_lfoot_pose_mapped_.translation();
        tracker_lfoot_pos_mapped_support_ = tracker_lfoot_pose_mapped_.translation() - tracker_lfoot_pose_mapped_.translation();
        tracker_rfoot_pos_mapped_support_ = tracker_rfoot_pose_mapped_.translation() - tracker_lfoot_pose_mapped_.translation();
    }
    else if(human_contact_indicator_ == ContactIndicator::RightSingleSupport)
    {
        tracker_pelv_pos_mapped_support_ = tracker_pelv_pose_mapped_.translation()   - tracker_rfoot_pose_mapped_.translation();
        tracker_lfoot_pos_mapped_support_ = tracker_lfoot_pose_mapped_.translation() - tracker_rfoot_pose_mapped_.translation();
        tracker_rfoot_pos_mapped_support_ = tracker_rfoot_pose_mapped_.translation() - tracker_rfoot_pose_mapped_.translation();
    }
    else
    {
        ROS_ERROR("CONTACT MISSING");
        assert((human_contact_indicator_ == ContactIndicator::DoubleSupport)
            || (human_contact_indicator_ == ContactIndicator::LeftSingleSupport)
            || (human_contact_indicator_ == ContactIndicator::RightSingleSupport));
    }


}

void TeleOperationManager::calibrationFunction(const bool &calibration_done)
{
    static bool save_initial_configuration = false;
    static bool calibration_done_prev = false;

    if (calibration_done && !calibration_done_prev){
        save_initial_configuration = true;
    }
    calibration_done_prev = calibration_done;

    if(tf_all_ok_){
        if(save_initial_configuration){
            //--- Orientation Initial Configuration
            tracker_head_rotm_init_   = tracker_head_pose_mapped_.linear();    
            tracker_chest_rotm_init_  = tracker_chest_pose_mapped_.linear();
            tracker_pelv_rotm_init_   = tracker_pelv_pose_mapped_.linear();
            tracker_lelbow_rotm_init_ = tracker_lelbow_pose_mapped_.linear();
            tracker_relbow_rotm_init_ = tracker_relbow_pose_mapped_.linear();
            tracker_lhand_rotm_init_  = tracker_lhand_pose_mapped_.linear();
            tracker_rhand_rotm_init_  = tracker_rhand_pose_mapped_.linear();
            tracker_lfoot_rotm_init_  = tracker_lfoot_pose_mapped_.linear();
            tracker_rfoot_rotm_init_  = tracker_rfoot_pose_mapped_.linear();

            robot_head_rotm_init_   = rd_.link_[Head].local_rotm;
            robot_chest_rotm_init_  = rd_.link_[Upper_Body].local_rotm;
            robot_pelv_rotm_init_   = rd_.link_[Pelvis].local_rotm; 
            robot_lelbow_rotm_init_ = rd_.link_[Left_Hand - 5].local_rotm;
            robot_relbow_rotm_init_ = rd_.link_[Right_Hand - 5].local_rotm;
            robot_lhand_rotm_init_  = rd_.link_[Left_Hand].local_rotm; 
            robot_rhand_rotm_init_  = rd_.link_[Right_Hand].local_rotm;
            robot_lfoot_rotm_init_  = rd_.link_[Left_Foot].local_rotm;
            robot_rfoot_rotm_init_  = rd_.link_[Right_Foot].local_rotm;

            tracker_pelv_pos_mapped_support_init_ = tracker_pelv_pos_mapped_support_;
            tracker_lfoot_pos_mapped_support_init_ = tracker_lfoot_pos_mapped_support_;
            tracker_rfoot_pos_mapped_support_init_ = tracker_rfoot_pos_mapped_support_;

            //--- Pelvis Height Ratio
            double robot_pelv_height = rd_.link_[Pelvis].support_xpos_init(2);
            double human_pelv_height = tracker_pelv_pos_mapped_support_init_(2);
            pelvis_height_ratio = robot_pelv_height / human_pelv_height;

            std::cout << "[Pelvis Calibration Info]" << std::endl;
            std::cout << "Robot_pelv_height: " << robot_pelv_height << std::endl;
            std::cout << "Human_pelv_height: " << human_pelv_height << std::endl;
            std::cout << "Pelvis Height Ratio: " << pelvis_height_ratio << std::endl;

            //--- Arm Length Ratio
            double robot_larm_length = (rd_.link_[Left_Hand].local_xpos - rd_.link_[Left_Hand - 5].local_xpos).norm();
            double human_larm_length = (tracker_lshoulder_pose_mapped_.translation() - tracker_lhand_pose_mapped_.translation()).norm();
            larm_length_ratio = robot_larm_length / human_larm_length;

            double robot_rarm_length = (rd_.link_[Right_Hand].local_xpos - rd_.link_[Right_Hand - 5].local_xpos).norm();
            double human_rarm_length = (tracker_rshoulder_pose_mapped_.translation() - tracker_rhand_pose_mapped_.translation()).norm();
            rarm_length_ratio = robot_rarm_length / human_rarm_length;

            std::cout << "[Arm Length Calibration Info]" << std::endl;
            std::cout << "Robot_larm_length: " << robot_larm_length << std::endl;
            std::cout << "Human_larm_length: " << human_larm_length << std::endl;
            std::cout << "Larm Length Ratio: " << larm_length_ratio << std::endl;
            std::cout << "Robot_rarm_length: " << robot_rarm_length << std::endl;
            std::cout << "Human_rarm_length: " << human_rarm_length << std::endl;
            std::cout << "Rarm Length Ratio: " << rarm_length_ratio << std::endl;

            //--- Leg Length Ratio
            double robot_lleg_length = (rd_.link_[Left_Foot].local_xpos - rd_.link_[Left_Foot - 5].local_xpos).norm();
            double human_lleg_length = (tracker_lfoot_pose_mapped_.translation() - tracker_lhip_pose_mapped_.translation()).norm();
            lleg_length_ratio = robot_lleg_length / human_lleg_length;

            double robot_rleg_length = (rd_.link_[Right_Foot].local_xpos - rd_.link_[Right_Foot - 5].local_xpos).norm();
            double human_rleg_length = (tracker_rfoot_pose_mapped_.translation() - tracker_rhip_pose_mapped_.translation()).norm();
            rleg_length_ratio = robot_rleg_length / human_rleg_length;

            std::cout << "[Leg Length Calibration Info]" << std::endl;
            std::cout << "Robot_lleg_length: " << robot_lleg_length << std::endl;
            std::cout << "Human_lleg_length: " << human_lleg_length << std::endl;
            std::cout << "Lleg Length Ratio: " << lleg_length_ratio << std::endl;
            std::cout << "Robot_rleg_length: " << robot_rleg_length << std::endl;
            std::cout << "Human_rleg_length: " << human_rleg_length << std::endl;
            std::cout << "Rleg Length Ratio: " << rleg_length_ratio << std::endl;

            save_initial_configuration = false;
        }
    }
    else{
        return;
    }

    //--- Orientation Retargeting
    robot_head_pose_retarget_.linear()   = robot_head_rotm_init_   * tracker_head_pose_mapped_.linear()   * tracker_head_rotm_init_.transpose();
    robot_chest_pose_retarget_.linear()  = robot_chest_rotm_init_  * tracker_chest_pose_mapped_.linear()  * tracker_chest_rotm_init_.transpose();
    // robot_pelv_pose_retarget_.linear()   = robot_pelv_rotm_init_   * tracker_pelv_pose_mapped_.linear()   * tracker_pelv_rotm_init_.transpose();
    robot_pelv_pose_retarget_.linear().setIdentity();
    robot_lelbow_pose_retarget_.linear() = robot_lelbow_rotm_init_ * tracker_lelbow_pose_mapped_.linear() * tracker_lelbow_rotm_init_.transpose();
    robot_relbow_pose_retarget_.linear() = robot_relbow_rotm_init_ * tracker_relbow_pose_mapped_.linear() * tracker_relbow_rotm_init_.transpose();
    robot_lhand_pose_retarget_.linear()  = robot_lhand_rotm_init_  * tracker_lhand_pose_mapped_.linear()  * tracker_lhand_rotm_init_.transpose();
    robot_rhand_pose_retarget_.linear()  = robot_rhand_rotm_init_  * tracker_rhand_pose_mapped_.linear()  * tracker_rhand_rotm_init_.transpose();
    robot_lfoot_pose_retarget_.linear()  = robot_lfoot_rotm_init_  * tracker_lfoot_pose_mapped_.linear()  * tracker_lfoot_rotm_init_.transpose();
    robot_rfoot_pose_retarget_.linear()  = robot_rfoot_rotm_init_  * tracker_rfoot_pose_mapped_.linear()  * tracker_rfoot_rotm_init_.transpose();

    //--- Hand Position Retargeting
    robot_lhand_pose_retarget_.translation() = rd_.link_[Left_Hand - 6].local_xpos  + larm_length_ratio * (tracker_lhand_pose_mapped_.translation() - tracker_lshoulder_pose_mapped_.translation());
    robot_rhand_pose_retarget_.translation() = rd_.link_[Right_Hand - 6].local_xpos + rarm_length_ratio * (tracker_rhand_pose_mapped_.translation() - tracker_rshoulder_pose_mapped_.translation());

    //--- Foot Position Retargeting
    // TODO

    //--- CoM Position Retargeting
    robot_com_pose_retarget_.translation()(2) = rd_.link_[COM_id].support_xpos_init(2) + pelvis_height_ratio * (tracker_pelv_pos_mapped_support_(2) - tracker_pelv_pos_mapped_support_init_(2));
    robot_com_pose_retarget_.translation() -= rd_.link_[Pelvis].support_xpos;
    double com_horizontal_offset =((tracker_pelv_pose_mapped_.translation().head(2)  - tracker_lfoot_pose_mapped_.translation().head(2)).transpose() * (tracker_rfoot_pose_mapped_.translation().head(2) - tracker_lfoot_pose_mapped_.translation().head(2)))(0)
                                 /((tracker_rfoot_pose_mapped_.translation().head(2) - tracker_lfoot_pose_mapped_.translation().head(2)).transpose() * (tracker_rfoot_pose_mapped_.translation().head(2) - tracker_lfoot_pose_mapped_.translation().head(2)))(0);
    robot_com_pose_retarget_.translation().head(2) = rd_.link_[Left_Foot].local_xpos.head(2) + com_horizontal_offset * (rd_.link_[Right_Foot].local_xpos.head(2) - rd_.link_[Left_Foot].local_xpos.head(2)); 

}

void TeleOperationManager::sendReadyPoseToRobot(const bool &ready_pose_mode)
{
    static bool go_ready_pose = false;
    static bool ready_pose_mode_prev = false;
    //--- Tracker Info

    if (ready_pose_mode && !ready_pose_mode_prev){
        go_ready_pose = true;
    }
    ready_pose_mode_prev = ready_pose_mode;

    if(go_ready_pose){
        static double control_time_init = 0; 
        static bool go_ready_pose_first = true;
        static double trajectory_duration = 3.0; // seconds

        double target_x = 0.3;
        double target_z = 0.3;

        if(go_ready_pose_first){
            control_time_init = rd_.control_time_;

            rd_.link_[Left_Hand].x_init    = rd_.link_[Left_Hand].local_xpos;
            rd_.link_[Right_Hand].x_init   = rd_.link_[Right_Hand].local_xpos;
            rd_.link_[Left_Hand].rot_init  = rd_.link_[Left_Hand].local_rotm;
            rd_.link_[Right_Hand].rot_init = rd_.link_[Right_Hand].local_rotm;

            rd_.link_[Left_Hand].x_desired    = rd_.link_[Left_Hand].x_init; 
            rd_.link_[Right_Hand].x_desired   = rd_.link_[Right_Hand].x_init; 
            rd_.link_[Left_Hand].rot_desired  = rd_.link_[Left_Hand].rot_init  * DyrosMath::rotateWithX(M_PI / 2.0);
            rd_.link_[Right_Hand].rot_desired = rd_.link_[Right_Hand].rot_init * DyrosMath::rotateWithX(-M_PI / 2.0);

            rd_.link_[Left_Hand].x_desired(0)  += target_x;  
            rd_.link_[Right_Hand].x_desired(0) += target_x;
            rd_.link_[Left_Hand].x_desired(2)  += target_z;  
            rd_.link_[Right_Hand].x_desired(2) += target_z;

            go_ready_pose_first = false;
        }

        rd_.link_[Left_Hand].SetTrajectoryQuintic(rd_.control_time_, control_time_init, control_time_init + trajectory_duration , rd_.link_[Left_Hand].x_init,  rd_.link_[Left_Hand].x_desired);
        rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, control_time_init, control_time_init + trajectory_duration, rd_.link_[Right_Hand].x_init, rd_.link_[Right_Hand].x_desired);
        rd_.link_[Left_Hand].SetTrajectoryRotation(rd_.control_time_, control_time_init, control_time_init + trajectory_duration);
        rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, control_time_init, control_time_init + trajectory_duration);

        if(rd_.control_time_ >= control_time_init + trajectory_duration){
            go_ready_pose = false;
            std::cout << "========== READY POSE DONE ===========" << std::endl;

            for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
            {
                //--- Base frame
                rd_.link_[idx].local_xpos_init = rd_.link_[idx].local_xpos;
                rd_.link_[idx].local_rotm_init = rd_.link_[idx].local_rotm;                             
            }
        }
    }
    else{
        return;
    }
}

void TeleOperationManager::motionRetargeting(const bool &avatar_mode)
{
    //--- interpolation function
    double now = ros::Time::now().toSec();
    static bool avatar_mode_prev_ = false;
    static bool is_transitioning_ = false;
    static double transition_start_time_ = 0.0;
    
    const double transition_duration_ = 3.0; 

    static Eigen::Isometry3d T_from_[LINK_NUMBER + 1];
    static Eigen::Isometry3d T_to_[LINK_NUMBER + 1];

    if (avatar_mode != avatar_mode_prev_)
    {
        transition_start_time_ = now;

        is_transitioning_ = true;
    }

    avatar_mode_prev_ = avatar_mode;

    if (is_transitioning_)
    {
        if (avatar_mode)
        {
            for (int i = 0; i < LINK_NUMBER + 1; ++i)
            {
                T_from_[i].translation() = rd_.link_[i].local_xpos_init;
                T_from_[i].linear()      = rd_.link_[i].local_rotm_init;
            }

            T_to_[Head]       = robot_head_pose_retarget_;
            T_to_[Upper_Body] = robot_chest_pose_retarget_;
            T_to_[Pelvis]     = robot_pelv_pose_retarget_;
            T_to_[Left_Hand]  = robot_lhand_pose_retarget_;
            T_to_[Right_Hand] = robot_rhand_pose_retarget_;
            T_to_[Left_Hand - 5]  = robot_lelbow_pose_retarget_;
            T_to_[Right_Hand - 5] = robot_relbow_pose_retarget_;
            if(lowerbody_disable_){
                T_to_[COM_id].translation() = rd_.link_[COM_id].local_xpos_init;
                T_to_[Left_Foot].translation()  = rd_.link_[Left_Foot].local_xpos_init;
                T_to_[Right_Foot].translation() = rd_.link_[Right_Foot].local_xpos_init;
            }
            else{
                T_to_[COM_id].translation() = robot_com_pose_retarget_.translation();
                T_to_[Left_Foot].translation()  = robot_lfoot_pose_retarget_.translation();
                T_to_[Right_Foot].translation() = robot_rfoot_pose_retarget_.translation();
            }

            T_to_[Left_Foot].linear()  = rd_.link_[Left_Foot].local_rotm_init;
            T_to_[Right_Foot].linear() = rd_.link_[Right_Foot].local_rotm_init;
        }
        else
        {
            T_from_[Head]       = robot_head_pose_retarget_;
            T_from_[Upper_Body] = robot_chest_pose_retarget_;
            T_from_[Pelvis]     = robot_pelv_pose_retarget_;
            T_from_[Left_Hand]  = robot_lhand_pose_retarget_;
            T_from_[Right_Hand] = robot_rhand_pose_retarget_;
            T_from_[Left_Hand - 5]  = robot_lelbow_pose_retarget_;
            T_from_[Right_Hand - 5] = robot_relbow_pose_retarget_;
            if(lowerbody_disable_){
                T_from_[COM_id].translation() = rd_.link_[COM_id].local_xpos_init;
                T_from_[Left_Foot].translation()  = rd_.link_[Left_Foot].local_xpos_init;
                T_from_[Right_Foot].translation() = rd_.link_[Right_Foot].local_xpos_init;
            }
            else{
                T_from_[COM_id].translation() = robot_com_pose_retarget_.translation();
                T_from_[Left_Foot].translation()  = rd_.link_[Left_Foot].local_xpos_init;
                T_from_[Right_Foot].translation() = rd_.link_[Right_Foot].local_xpos_init;
            }

            T_from_[Left_Foot].linear()  = rd_.link_[Left_Foot].local_rotm_init;
            T_from_[Right_Foot].linear() = rd_.link_[Right_Foot].local_rotm_init;

            for (int i = 0; i < LINK_NUMBER + 1; ++i)
            {
                T_to_[i].translation() = rd_.link_[i].local_xpos_init;
                T_to_[i].linear()      = rd_.link_[i].local_rotm_init;
            }
        }

        double s = (now - transition_start_time_) / transition_duration_;
        s = std::min(1.0, std::max(0.0, s));

        for (int i = 0; i < LINK_NUMBER + 1; ++i)
        {
            Eigen::Isometry3d T = blendIsometry(T_from_[i], T_to_[i], s);
            rd_.link_[i].x_traj = T.translation();
            rd_.link_[i].r_traj = T.linear();
        }

        if (s >= 1.0)
            is_transitioning_ = false;

        return;
    }

    if (avatar_mode)
    {
        //--- Rotation
        rd_.link_[Head].r_traj       = robot_head_pose_retarget_.linear();
        rd_.link_[Upper_Body].r_traj = robot_chest_pose_retarget_.linear();
        rd_.link_[Pelvis].r_traj     = robot_pelv_pose_retarget_.linear();

        rd_.link_[Left_Hand].r_traj  = robot_lhand_pose_retarget_.linear();
        rd_.link_[Right_Hand].r_traj = robot_rhand_pose_retarget_.linear();

        rd_.link_[Left_Hand - 5].r_traj  = robot_lelbow_pose_retarget_.linear();
        rd_.link_[Right_Hand - 5].r_traj = robot_relbow_pose_retarget_.linear();

        //--- Translation
        rd_.link_[Left_Hand].x_traj  = robot_lhand_pose_retarget_.translation();
        rd_.link_[Right_Hand].x_traj = robot_rhand_pose_retarget_.translation();
        if(lowerbody_disable_){
            rd_.link_[COM_id].x_traj = rd_.link_[COM_id].local_xpos_init;
            rd_.link_[Left_Foot].x_traj  = rd_.link_[Left_Foot].local_xpos_init;
            rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].local_xpos_init;
        }
        else{
            rd_.link_[COM_id].x_traj = robot_com_pose_retarget_.translation();
            rd_.link_[Left_Foot].x_traj  = rd_.link_[Left_Foot].local_xpos_init;
            rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].local_xpos_init;
        }
    }
    else
    {
        for (int i = 0; i < LINK_NUMBER + 1; ++i)
        {
            rd_.link_[i].x_traj = rd_.link_[i].local_xpos_init;
            rd_.link_[i].r_traj = rd_.link_[i].local_rotm_init;
        }
    }
}


//-------------//
//--- Utils ---//
Eigen::Isometry3d TeleOperationManager::tfToEigen(const tf::StampedTransform& tf_msg)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    // translation
    T.translation() << tf_msg.getOrigin().x(),
                       tf_msg.getOrigin().y(),
                       tf_msg.getOrigin().z();

    // rotation
    tf::Quaternion q = tf_msg.getRotation();
    Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
    T.linear() = q_eigen.toRotationMatrix();

    return T;
}


bool TeleOperationManager::lookupTF(const std::string &parent, const std::string &child, Eigen::Isometry3d &T_out)
{
    tf::StampedTransform tf_msg;

    try
    {
        tf_listener_.lookupTransform(parent, child, ros::Time(0), tf_msg);
        T_out = tfToEigen(tf_msg);
        return true;
    }
    catch (tf::TransformException &)
    {
        return false;
    }
}

//----------------------//
//--- XR Pose Update ---//
void TeleOperationManager::xrPoseCallback(const xr_msgs::Custom::ConstPtr& msg)
{
    right_controller_primary_button   = msg->right_controller.primary_button;
    right_controller_secondary_button = msg->right_controller.secondary_button;
    right_controller_trigger          = msg->right_controller.trigger;
    
    left_controller_primary_button    = msg->left_controller.primary_button;
    left_controller_secondary_button  = msg->left_controller.secondary_button;
    left_controller_trigger           = msg->left_controller.trigger;
}

bool TeleOperationManager::leftPrimaryPressedOnce()
{
    static bool prev = false;
    bool curr = left_controller_primary_button;

    if (curr && !prev)
    {
        prev = curr;
        return true;
    }

    prev = curr;
    return false;
}

bool TeleOperationManager::leftSecondaryPressedOnce()
{
    static bool prev = false;
    bool curr = left_controller_secondary_button;

    if (curr && !prev)
    {
        prev = curr;
        return true;
    }

    prev = curr;
    return false;
}

bool TeleOperationManager::rightPrimaryPressedOnce()
{
    static bool prev = false;
    bool curr = right_controller_primary_button;

    if (curr && !prev)
    {
        prev = curr;
        return true;
    }

    prev = curr;
    return false;
}

bool TeleOperationManager::rightSecondaryPressedOnce()
{
    static bool prev = false;
    bool curr = right_controller_secondary_button;

    if (curr && !prev)
    {
        prev = curr;
        return true;
    }

    prev = curr;
    return false;
}
