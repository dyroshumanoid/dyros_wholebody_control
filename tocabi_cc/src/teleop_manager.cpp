#include "teleop_manager.h"

TeleOperationManager::TeleOperationManager(RobotData &rd) : rd_(rd)
{
    xr_pose_sub_ = nh_.subscribe(
        "/xr_pose",
        10,
        &TeleOperationManager::xrPoseCallback,
        this,
        ros::TransportHints().tcpNoDelay(true));

    tracker_pose_sub_ = nh_.subscribe(
        "/tracker_pose",
        1,
        &TeleOperationManager::trackerPoseCallback,
        this,
        ros::TransportHints().tcpNoDelay(true));

    retarget_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
        "/retarget_pose", 1);
}

void TeleOperationManager::trackerPoseCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    tracker_pose_ = *msg;
    tracker_pose_received_ = true;
}

void TeleOperationManager::callAvailableQueue()
{
    queue_.callAvailable(ros::WallDuration());
}

//----------------------//
void TeleOperationManager::updateTrackerFromPoseArray()
{
    if (!tracker_pose_received_)
    {
        ROS_WARN_THROTTLE(1.0, "[WARN] No tracker_pose received");
        return;
    }

    const auto& poses = tracker_pose_.poses;

    tracker_pelv_pose_raw_      = poseToEigen(poses[0]);
    tracker_chest_pose_raw_     = poseToEigen(poses[1]);
    tracker_lfoot_pose_raw_     = poseToEigen(poses[2]);
    tracker_rfoot_pose_raw_     = poseToEigen(poses[3]);
    tracker_head_pose_raw_      = poseToEigen(poses[4]);
    tracker_lshoulder_pose_raw_ = poseToEigen(poses[5]);
    tracker_rshoulder_pose_raw_ = poseToEigen(poses[6]);
    tracker_lelbow_pose_raw_    = poseToEigen(poses[7]);
    tracker_relbow_pose_raw_    = poseToEigen(poses[8]);
    tracker_lhand_pose_raw_     = poseToEigen(poses[9]);
    tracker_rhand_pose_raw_     = poseToEigen(poses[10]);

    
    //--- Tracker Mapping for Local Frame
    Eigen::Isometry3d T_world_;
    T_world_.linear().setZero();
    T_world_.translation().setZero();
    T_world_.translation() = tracker_pelv_pose_raw_.translation();
    T_world_.linear() = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(tracker_pelv_pose_raw_.linear())(2));

    tracker_pelv_pose_mapped_      = DyrosMath::inverseIsometry3d(T_world_) * tracker_pelv_pose_raw_;
    tracker_chest_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_chest_pose_raw_;
    tracker_head_pose_mapped_      = DyrosMath::inverseIsometry3d(T_world_) * tracker_head_pose_raw_;

    tracker_lhand_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_lhand_pose_raw_;
    tracker_lshoulder_pose_mapped_ = DyrosMath::inverseIsometry3d(T_world_) * tracker_lshoulder_pose_raw_;
    tracker_lelbow_pose_mapped_    = DyrosMath::inverseIsometry3d(T_world_) * tracker_lelbow_pose_raw_;

    tracker_rhand_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_rhand_pose_raw_;
    tracker_rshoulder_pose_mapped_ = DyrosMath::inverseIsometry3d(T_world_) * tracker_rshoulder_pose_raw_;
    tracker_relbow_pose_mapped_    = DyrosMath::inverseIsometry3d(T_world_) * tracker_relbow_pose_raw_;

    tracker_lfoot_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_lfoot_pose_raw_;
    tracker_rfoot_pose_mapped_     = DyrosMath::inverseIsometry3d(T_world_) * tracker_rfoot_pose_raw_;

    Eigen::Matrix3d human_robot_lelbow_map;
    human_robot_lelbow_map.setIdentity();
    human_robot_lelbow_map << 0, 0, -1,
        0, 1, 0,
        1, 0, 0;
    tracker_lelbow_pose_mapped_.linear() = tracker_lelbow_pose_mapped_.linear() * human_robot_lelbow_map;

    Eigen::Matrix3d human_robot_lhand_map;
    human_robot_lhand_map.setIdentity();
    human_robot_lhand_map << 0, 1, 0,
        0, 0, -1,
        -1, 0, 0;
    tracker_lhand_pose_mapped_.linear() = tracker_lhand_pose_mapped_.linear() * human_robot_lhand_map;

    Eigen::Matrix3d human_robot_relbow_map;
    human_robot_relbow_map.setIdentity();
    human_robot_relbow_map << 0, 0, -1,
        0, 1, 0,
        1, 0, 0;
    tracker_relbow_pose_mapped_.linear() = tracker_relbow_pose_mapped_.linear() * human_robot_relbow_map;

    Eigen::Matrix3d human_robot_rhand_map;
    human_robot_rhand_map.setIdentity();
    human_robot_rhand_map << 0, -1, 0,
        0, 0, 1,
        -1, 0, 0;
    tracker_rhand_pose_mapped_.linear() = tracker_rhand_pose_mapped_.linear() * human_robot_rhand_map;

    tracker_pelv_pos_mapped_support_  = tracker_pelv_pose_mapped_.translation() - tracker_lfoot_pose_mapped_.translation();
    tracker_lfoot_pos_mapped_support_ = tracker_lfoot_pose_mapped_.translation() - tracker_lfoot_pose_mapped_.translation();
    tracker_rfoot_pos_mapped_support_ = tracker_rfoot_pose_mapped_.translation() - tracker_lfoot_pose_mapped_.translation();
}

void TeleOperationManager::calibrationFunction(const bool &calibration_done)
{
    static bool save_initial_configuration = false;
    static bool calibration_done_prev = false;

    if (calibration_done && !calibration_done_prev)
    {
        save_initial_configuration = true;
    }
    calibration_done_prev = calibration_done;

    if (tracker_pose_received_)
    {
        if (save_initial_configuration)
        {
            //--- Orientation Initial Configuration
            tracker_head_rotm_init_ = tracker_head_pose_mapped_.linear();
            tracker_chest_rotm_init_ = tracker_chest_pose_mapped_.linear();
            tracker_pelv_rotm_init_ = tracker_pelv_pose_mapped_.linear();
            tracker_lelbow_rotm_init_ = tracker_lelbow_pose_mapped_.linear();
            tracker_relbow_rotm_init_ = tracker_relbow_pose_mapped_.linear();
            tracker_lhand_rotm_init_ = tracker_lhand_pose_mapped_.linear();
            tracker_rhand_rotm_init_ = tracker_rhand_pose_mapped_.linear();
            tracker_lfoot_rotm_init_ = tracker_lfoot_pose_mapped_.linear();
            tracker_rfoot_rotm_init_ = tracker_rfoot_pose_mapped_.linear();

            tracker_lfoot_pos_init_ = tracker_lfoot_pose_mapped_.translation();
            tracker_rfoot_pos_init_ = tracker_rfoot_pose_mapped_.translation();

            robot_head_rotm_init_ = rd_.link_[Head].local_rotm;
            robot_chest_rotm_init_ = rd_.link_[Upper_Body].local_rotm;
            robot_pelv_rotm_init_ = rd_.link_[Pelvis].local_rotm;
            robot_lelbow_rotm_init_ = rd_.link_[Left_Hand - 4].local_rotm;
            robot_relbow_rotm_init_ = rd_.link_[Right_Hand - 4].local_rotm;
            robot_lhand_rotm_init_ = rd_.link_[Left_Hand].local_rotm;
            robot_rhand_rotm_init_ = rd_.link_[Right_Hand].local_rotm;
            robot_lfoot_rotm_init_ = rd_.link_[Left_Foot].local_rotm;
            robot_rfoot_rotm_init_ = rd_.link_[Right_Foot].local_rotm;

            tracker_pelv_pos_mapped_support_init_  = tracker_pelv_pos_mapped_support_ ;
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
            std::cout << " " << std::endl;

            //--- Arm Length Ratio
            double robot_larm_length = (rd_.link_[Left_Hand].local_xpos - rd_.link_[Left_Hand - 4].local_xpos).norm();
            double human_larm_length = (tracker_lshoulder_pose_mapped_.translation() - tracker_lhand_pose_mapped_.translation()).norm();
            larm_length_ratio = robot_larm_length / human_larm_length;

            double robot_rarm_length = (rd_.link_[Right_Hand].local_xpos - rd_.link_[Right_Hand - 4].local_xpos).norm();
            double human_rarm_length = (tracker_rshoulder_pose_mapped_.translation() - tracker_rhand_pose_mapped_.translation()).norm();
            rarm_length_ratio = robot_rarm_length / human_rarm_length;

            std::cout << "[Arm Length Calibration Info]" << std::endl;
            std::cout << "Robot_larm_length: " << robot_larm_length << std::endl;
            std::cout << "Human_larm_length: " << human_larm_length << std::endl;
            std::cout << "Larm Length Ratio: " << larm_length_ratio << std::endl;
            std::cout << "Robot_rarm_length: " << robot_rarm_length << std::endl;
            std::cout << "Human_rarm_length: " << human_rarm_length << std::endl;
            std::cout << "Rarm Length Ratio: " << rarm_length_ratio << std::endl;
            std::cout << " " << std::endl;

            save_initial_configuration = false;
        }
    }
    else
    {
        ROS_WARN_THROTTLE(1.0, "[WARN] Tracker pose not received yet, cannot calibrate");
        return;
    }

    //--- Orientation Retargeting
    robot_head_pose_retarget_.linear() = clampRotation(robot_head_rotm_init_ * tracker_head_pose_mapped_.linear() * tracker_head_rotm_init_.transpose(), 15 * DEG2RAD);
    robot_chest_pose_retarget_.linear() = clampRotation(robot_chest_rotm_init_ * tracker_chest_pose_mapped_.linear() * tracker_chest_rotm_init_.transpose(), 15 * DEG2RAD);
    robot_pelv_pose_retarget_.linear() = clampRotation(robot_pelv_rotm_init_ * tracker_pelv_pose_mapped_.linear() * tracker_pelv_rotm_init_.transpose(), 15 * DEG2RAD);

    robot_lelbow_pose_retarget_.linear() = tracker_lelbow_pose_mapped_.linear();
    robot_relbow_pose_retarget_.linear() = tracker_relbow_pose_mapped_.linear();
    robot_lhand_pose_retarget_.linear() = tracker_lhand_pose_mapped_.linear();
    robot_rhand_pose_retarget_.linear() = tracker_rhand_pose_mapped_.linear();

    //--- Hand Position Retargeting
    robot_lhand_pose_retarget_.translation() = rd_.link_[Left_Hand - 5].local_xpos + larm_length_ratio * (tracker_lhand_pose_mapped_.translation() - tracker_lshoulder_pose_mapped_.translation());
    robot_rhand_pose_retarget_.translation() = rd_.link_[Right_Hand - 5].local_xpos + rarm_length_ratio * (tracker_rhand_pose_mapped_.translation() - tracker_rshoulder_pose_mapped_.translation());

    //--- Foot Position Retargeting
    //TODO
    robot_lfoot_pose_retarget_.translation() = rd_.link_[Left_Foot].local_xpos_init;
    robot_rfoot_pose_retarget_.translation() = rd_.link_[Right_Foot].local_xpos_init;

    //--- CoM Position Retargeting
    robot_com_pose_retarget_.translation()(2) = rd_.link_[COM_id].support_xpos_init(2) + pelvis_height_ratio * (tracker_pelv_pos_mapped_support_(2) - tracker_pelv_pos_mapped_support_init_(2));
    robot_com_pose_retarget_.translation() -= rd_.link_[Pelvis].support_xpos;
    double com_horizontal_offset = ((tracker_pelv_pose_mapped_.translation().head(2) - tracker_lfoot_pose_mapped_.translation().head(2)).transpose() * (tracker_rfoot_pose_mapped_.translation().head(2) - tracker_lfoot_pose_mapped_.translation().head(2)))(0) / ((tracker_rfoot_pose_mapped_.translation().head(2) - tracker_lfoot_pose_mapped_.translation().head(2)).transpose() * (tracker_rfoot_pose_mapped_.translation().head(2) - tracker_lfoot_pose_mapped_.translation().head(2)))(0);
    robot_com_pose_retarget_.translation().head(2) = rd_.link_[Left_Foot].local_xpos.head(2) + com_horizontal_offset * (rd_.link_[Right_Foot].local_xpos.head(2) - rd_.link_[Left_Foot].local_xpos.head(2));

    //--- else
    robot_head_pose_retarget_.translation() = rd_.link_[Head].local_xpos_init;
    robot_com_pose_retarget_.linear().setIdentity();
    robot_lfoot_pose_retarget_.linear().setIdentity();
    robot_rfoot_pose_retarget_.linear().setIdentity();
    robot_lelbow_pose_retarget_.translation() = rd_.link_[Left_Hand - 4].local_xpos_init;
    robot_relbow_pose_retarget_.translation() = rd_.link_[Right_Hand - 4].local_xpos_init;
    publishRetargetPoseArray();
}

void TeleOperationManager::sendReadyPoseToRobot(const bool &ready_pose_mode)
{
    static bool go_ready_pose = false;
    static bool ready_pose_mode_prev = false;
    //--- Tracker Info

    if (ready_pose_mode && !ready_pose_mode_prev)
    {
        go_ready_pose = true;
    }
    ready_pose_mode_prev = ready_pose_mode;

    if (go_ready_pose)
    {
        static double control_time_init = 0;
        static bool go_ready_pose_first = true;
        static double trajectory_duration = 3.0; // seconds

        double target_x = 0.3;
        double target_z = 0.3;

        if (go_ready_pose_first)
        {
            control_time_init = rd_.control_time_;

            rd_.link_[Left_Hand].x_init = rd_.link_[Left_Hand].local_xpos;
            rd_.link_[Right_Hand].x_init = rd_.link_[Right_Hand].local_xpos;
            rd_.link_[Left_Hand].rot_init = rd_.link_[Left_Hand].local_rotm;
            rd_.link_[Right_Hand].rot_init = rd_.link_[Right_Hand].local_rotm;

            rd_.link_[Left_Hand].x_desired = rd_.link_[Left_Hand].x_init;
            rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init;
            rd_.link_[Left_Hand].rot_desired = rd_.link_[Left_Hand].rot_init * DyrosMath::rotateWithX(M_PI / 2.0);
            rd_.link_[Right_Hand].rot_desired = rd_.link_[Right_Hand].rot_init * DyrosMath::rotateWithX(-M_PI / 2.0);

            rd_.link_[Left_Hand].x_desired(0) += target_x;
            rd_.link_[Right_Hand].x_desired(0) += target_x;
            rd_.link_[Left_Hand].x_desired(2) += target_z;
            rd_.link_[Right_Hand].x_desired(2) += target_z;

            go_ready_pose_first = false;
        }

        rd_.link_[Left_Hand].SetTrajectoryQuintic(rd_.control_time_, control_time_init, control_time_init + trajectory_duration, rd_.link_[Left_Hand].x_init, rd_.link_[Left_Hand].x_desired);
        rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, control_time_init, control_time_init + trajectory_duration, rd_.link_[Right_Hand].x_init, rd_.link_[Right_Hand].x_desired);
        rd_.link_[Left_Hand].SetTrajectoryRotation(rd_.control_time_, control_time_init, control_time_init + trajectory_duration);
        rd_.link_[Right_Hand].SetTrajectoryRotation(rd_.control_time_, control_time_init, control_time_init + trajectory_duration);

        if (rd_.control_time_ >= control_time_init + trajectory_duration)
        {
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
    else
    {
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

    const double transition_duration_ = 1.0;

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
                T_from_[i].linear() = rd_.link_[i].local_rotm_init;
            }

            T_to_[Head] = robot_head_pose_retarget_;
            T_to_[Upper_Body] = robot_chest_pose_retarget_;
            T_to_[Pelvis] = robot_pelv_pose_retarget_;
            T_to_[Left_Hand] = robot_lhand_pose_retarget_;
            T_to_[Right_Hand] = robot_rhand_pose_retarget_;
            T_to_[Left_Hand - 4] = robot_lelbow_pose_retarget_;
            T_to_[Right_Hand - 4] = robot_relbow_pose_retarget_;
            if (lowerbody_disable_)
            {
                T_to_[COM_id].translation() = rd_.link_[COM_id].local_xpos_init;
                T_to_[Left_Foot].translation() = rd_.link_[Left_Foot].local_xpos_init;
                T_to_[Right_Foot].translation() = rd_.link_[Right_Foot].local_xpos_init;
            }
            else
            {
                T_to_[COM_id].translation() = robot_com_pose_retarget_.translation();
                T_to_[Left_Foot].translation() = rd_.link_[Left_Foot].local_xpos_init;
                T_to_[Right_Foot].translation() = rd_.link_[Right_Foot].local_xpos_init;
            }

            T_to_[Left_Foot].linear() = rd_.link_[Left_Foot].local_rotm_init;
            T_to_[Right_Foot].linear() = rd_.link_[Right_Foot].local_rotm_init;
        }
        else
        {
            T_from_[Head] = robot_head_pose_retarget_;
            T_from_[Upper_Body] = robot_chest_pose_retarget_;
            T_from_[Pelvis] = robot_pelv_pose_retarget_;
            T_from_[Left_Hand] = robot_lhand_pose_retarget_;
            T_from_[Right_Hand] = robot_rhand_pose_retarget_;
            T_from_[Left_Hand - 4] = robot_lelbow_pose_retarget_;
            T_from_[Right_Hand - 4] = robot_relbow_pose_retarget_;
            if (lowerbody_disable_)
            {
                T_from_[COM_id].translation() = rd_.link_[COM_id].local_xpos_init;
                T_from_[Left_Foot].translation() = rd_.link_[Left_Foot].local_xpos_init;
                T_from_[Right_Foot].translation() = rd_.link_[Right_Foot].local_xpos_init;
            }
            else
            {
                T_from_[COM_id].translation() = robot_com_pose_retarget_.translation();
                T_from_[Left_Foot].translation() = rd_.link_[Left_Foot].local_xpos_init;
                T_from_[Right_Foot].translation() = rd_.link_[Right_Foot].local_xpos_init;
            }

            T_from_[Left_Foot].linear() = rd_.link_[Left_Foot].local_rotm_init;
            T_from_[Right_Foot].linear() = rd_.link_[Right_Foot].local_rotm_init;

            for (int i = 0; i < LINK_NUMBER + 1; ++i)
            {
                T_to_[i].translation() = rd_.link_[i].local_xpos_init;
                T_to_[i].linear() = rd_.link_[i].local_rotm_init;
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
        rd_.link_[Head].r_traj = robot_head_pose_retarget_.linear();
        rd_.link_[Upper_Body].r_traj = robot_chest_pose_retarget_.linear();
        rd_.link_[Pelvis].r_traj = robot_pelv_pose_retarget_.linear();

        rd_.link_[Left_Hand].r_traj = robot_lhand_pose_retarget_.linear();
        rd_.link_[Right_Hand].r_traj = robot_rhand_pose_retarget_.linear();

        rd_.link_[Left_Hand - 4].r_traj = robot_lelbow_pose_retarget_.linear();
        rd_.link_[Right_Hand - 4].r_traj = robot_relbow_pose_retarget_.linear();

        //--- Translation
        rd_.link_[Left_Hand].x_traj = robot_lhand_pose_retarget_.translation();
        rd_.link_[Right_Hand].x_traj = robot_rhand_pose_retarget_.translation();
        if (lowerbody_disable_)
        {
            rd_.link_[COM_id].x_traj = rd_.link_[COM_id].local_xpos_init;
            rd_.link_[Left_Foot].x_traj = rd_.link_[Left_Foot].local_xpos_init;
            rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].local_xpos_init;
        }
        else
        {
            rd_.link_[COM_id].x_traj = robot_com_pose_retarget_.translation();
            rd_.link_[Left_Foot].x_traj = robot_lfoot_pose_retarget_.translation();
            rd_.link_[Right_Foot].x_traj = robot_rfoot_pose_retarget_.translation();
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
void TeleOperationManager::publishRetargetPoseArray()
{
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.poses.reserve(10);

    msg.poses.push_back(eigenToPose(robot_head_pose_retarget_));
    msg.poses.push_back(eigenToPose(robot_chest_pose_retarget_));
    msg.poses.push_back(eigenToPose(robot_pelv_pose_retarget_));

    msg.poses.push_back(eigenToPose(robot_lelbow_pose_retarget_));
    msg.poses.push_back(eigenToPose(robot_relbow_pose_retarget_));

    msg.poses.push_back(eigenToPose(robot_lhand_pose_retarget_));
    msg.poses.push_back(eigenToPose(robot_rhand_pose_retarget_));

    msg.poses.push_back(eigenToPose(robot_lfoot_pose_retarget_));
    msg.poses.push_back(eigenToPose(robot_rfoot_pose_retarget_));

    msg.poses.push_back(eigenToPose(robot_com_pose_retarget_));

    retarget_pose_pub_.publish(msg);
}

geometry_msgs::Pose TeleOperationManager::eigenToPose(const Eigen::Isometry3d &T)
{
    geometry_msgs::Pose pose;

    // position
    pose.position.x = T.translation()(0);
    pose.position.y = T.translation()(1);
    pose.position.z = T.translation()(2);

    // orientation
    Eigen::Quaterniond q(T.linear());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

Eigen::Isometry3d TeleOperationManager::poseToEigen(const geometry_msgs::Pose& p)
{
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() << p.position.x, p.position.y, p.position.z;
    Eigen::Quaterniond q(p.orientation.w,
                         p.orientation.x,
                         p.orientation.y,
                         p.orientation.z);
    T.linear() = q.toRotationMatrix();
    return T;
}

//----------------------//
//--- XR Pose Update ---//
void TeleOperationManager::xrPoseCallback(const xr_msgs::Custom::ConstPtr &msg)
{
    right_controller_primary_button = msg->right_controller.primary_button;
    right_controller_secondary_button = msg->right_controller.secondary_button;
    right_controller_trigger = msg->right_controller.trigger;

    left_controller_primary_button = msg->left_controller.primary_button;
    left_controller_secondary_button = msg->left_controller.secondary_button;
    left_controller_trigger = msg->left_controller.trigger;
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

Eigen::Matrix3d TeleOperationManager::clampRotation(const Eigen::Matrix3d &R_target, double max_angle_rad)
{
    Eigen::Vector3d target_euler = DyrosMath::rot2Euler(R_target);
    for (int i = 0; i < 3; ++i)
    {
        if (target_euler(i) > max_angle_rad)
            target_euler(i) = max_angle_rad;
        else if (target_euler(i) < -max_angle_rad)
            target_euler(i) = -max_angle_rad;
    }

    Eigen::Matrix3d R_limited = DyrosMath::Euler2rot(target_euler(0), target_euler(1), target_euler(2));

    return R_limited;
}

void TeleOperationManager::setWalkingParameter(const double &step_length_, const double &foot_height_, const double &step_duration_)
{
    step_length = step_length_;
    foot_height = foot_height_;
    step_duration = step_duration_;
}

Eigen::Vector3d TeleOperationManager::generateSwingFootTrajectory(
    const int swing_foot_link_idx,
    const Eigen::Isometry3d &tracker_swing_pose,
    const Eigen::Vector3d &tracker_swing_init,
    const Eigen::Vector3d &robot_swing_init,
    double &control_time_init,
    bool &first_lift_flag)
{
    Eigen::Vector3d robot_swing_target = Eigen::Vector3d::Zero();

    if (first_lift_flag)
    {
        control_time_init = rd_.control_time_;
        if(swing_foot_link_idx == Left_Foot)
            std::cout << "[Teleop Manager] : Left Foot Lifting Triggered" << std::endl;
        else if (swing_foot_link_idx == Right_Foot)
            std::cout << "[Teleop Manager] : Right Foot Lifting Triggered" << std::endl;
        first_lift_flag = false;
    }

    Eigen::Vector2d swing_vec = (tracker_swing_pose.translation() - tracker_swing_init).head<2>();

    double swing_norm = swing_vec.norm();
    if (swing_norm > 1e-6)
        swing_vec /= swing_norm;
    else
        swing_vec.setZero();

    Eigen::Vector2d swing_location = swing_vec;
    swing_location.setZero();

    double t = rd_.control_time_ - control_time_init;

    for (int i = 0; i < 2; i++)
    {
        robot_swing_target(i) =
            DyrosMath::cubic(
                t,
                0.0,
                step_duration,
                robot_swing_init(i),
                robot_swing_init(i) + swing_location(i),
                0.0, 0.0);
    }

    if (t <= step_duration * 0.5)
    {
        // Swing up phase
        robot_swing_target(2) =
            DyrosMath::cubic(
                t,
                0.0,
                step_duration * 0.5,
                robot_swing_init(2),
                robot_swing_init(2) + foot_height,
                0.0,
                0.0);
    }
    else
    {
        // Swing down phase
        robot_swing_target(2) =
            DyrosMath::cubic(
                t,
                step_duration * 0.5,
                step_duration,
                robot_swing_init(2) + foot_height,
                robot_swing_init(2),
                0.0,
                0.0);
    }

    std::cout << "robot_swing_target: " << robot_swing_target.transpose() << std::endl;

    return robot_swing_target;
}