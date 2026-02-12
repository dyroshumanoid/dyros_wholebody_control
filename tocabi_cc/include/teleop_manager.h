#pragma once
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include "xr_msgs/Custom.h"
#include <tf2_msgs/TFMessage.h>

class TeleOperationManager 
{
public:
    TeleOperationManager(RobotData &rd);
    void callAvailableQueue();

    void updateTrackerFromTF();
    void calibrationFunction(const bool &calibration_done);
    void sendReadyPoseToRobot(const bool &ready_pose_mode);
    void motionRetargeting(const bool &avatar_mode);


    bool leftPrimaryPressedOnce();
    bool leftSecondaryPressedOnce();
    bool rightPrimaryPressedOnce();
    bool rightSecondaryPressedOnce();

    bool transformAllOK() { return tf_all_ok_; }

private:
    RobotData &rd_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;

    tf::TransformListener tf_listener_;
    ros::Subscriber xr_pose_sub_;

    void xrPoseCallback(const xr_msgs::Custom::ConstPtr& msg);    
    bool lookupTF(const std::string &parent, const std::string &child, Eigen::Isometry3d &T_out);
    Eigen::Isometry3d tfToEigen(const tf::StampedTransform& tf_msg);

private:
    double pelvis_height_ratio = 0.0;
    double larm_length_ratio = 0.0;
    double rarm_length_ratio = 0.0;

    //--- Tracker
    Eigen::Isometry3d tracker_head_pose_raw_;
    Eigen::Isometry3d tracker_chest_pose_raw_;
    Eigen::Isometry3d tracker_pelv_pose_raw_;
    Eigen::Isometry3d tracker_lshoulder_pose_raw_;
    Eigen::Isometry3d tracker_rshoulder_pose_raw_;
    Eigen::Isometry3d tracker_lelbow_pose_raw_;
    Eigen::Isometry3d tracker_relbow_pose_raw_;
    Eigen::Isometry3d tracker_lhand_pose_raw_;
    Eigen::Isometry3d tracker_rhand_pose_raw_;
    Eigen::Isometry3d tracker_lfoot_pose_raw_;
    Eigen::Isometry3d tracker_rfoot_pose_raw_;

    Eigen::Isometry3d tracker_head_pose_mapped_;
    Eigen::Isometry3d tracker_chest_pose_mapped_;
    Eigen::Isometry3d tracker_pelv_pose_mapped_;
    Eigen::Isometry3d tracker_lshoulder_pose_mapped_;
    Eigen::Isometry3d tracker_rshoulder_pose_mapped_;
    Eigen::Isometry3d tracker_lelbow_pose_mapped_;
    Eigen::Isometry3d tracker_relbow_pose_mapped_;
    Eigen::Isometry3d tracker_lhand_pose_mapped_;
    Eigen::Isometry3d tracker_rhand_pose_mapped_;
    Eigen::Isometry3d tracker_lfoot_pose_mapped_;
    Eigen::Isometry3d tracker_rfoot_pose_mapped_;

    Eigen::Matrix3d tracker_pelv_pos_init_;

    Eigen::Matrix3d tracker_head_rotm_init_;
    Eigen::Matrix3d tracker_chest_rotm_init_;
    Eigen::Matrix3d tracker_pelv_rotm_init_;
    Eigen::Matrix3d tracker_lelbow_rotm_init_;
    Eigen::Matrix3d tracker_relbow_rotm_init_;
    Eigen::Matrix3d tracker_lhand_rotm_init_;
    Eigen::Matrix3d tracker_rhand_rotm_init_;
    Eigen::Matrix3d tracker_lfoot_rotm_init_;
    Eigen::Matrix3d tracker_rfoot_rotm_init_;

    //--- Robot
    Eigen::Matrix3d robot_head_rotm_init_;
    Eigen::Matrix3d robot_chest_rotm_init_;
    Eigen::Matrix3d robot_pelv_rotm_init_;
    Eigen::Matrix3d robot_lelbow_rotm_init_;
    Eigen::Matrix3d robot_relbow_rotm_init_;
    Eigen::Matrix3d robot_lhand_rotm_init_;
    Eigen::Matrix3d robot_rhand_rotm_init_;
    Eigen::Matrix3d robot_lfoot_rotm_init_;
    Eigen::Matrix3d robot_rfoot_rotm_init_;
    
    Eigen::Matrix3d robot_head_rotm_offset_;
    Eigen::Matrix3d robot_chest_rotm_offset_;
    Eigen::Matrix3d robot_pelv_rotm_offset_;
    Eigen::Matrix3d robot_lelbow_rotm_offset_;
    Eigen::Matrix3d robot_relbow_rotm_offset_;
    Eigen::Matrix3d robot_lhand_rotm_offset_;
    Eigen::Matrix3d robot_rhand_rotm_offset_;
    Eigen::Matrix3d robot_lfoot_rotm_offset_;
    Eigen::Matrix3d robot_rfoot_rotm_offset_;
    
    Eigen::Isometry3d robot_head_pose_retarget_;
    Eigen::Isometry3d robot_chest_pose_retarget_;
    Eigen::Isometry3d robot_pelv_pose_retarget_;
    Eigen::Isometry3d robot_lelbow_pose_retarget_;
    Eigen::Isometry3d robot_relbow_pose_retarget_;
    Eigen::Isometry3d robot_lhand_pose_retarget_;
    Eigen::Isometry3d robot_rhand_pose_retarget_;
    Eigen::Isometry3d robot_lfoot_pose_retarget_;
    Eigen::Isometry3d robot_rfoot_pose_retarget_;

    //--- Retargeting
    bool tf_all_ok_ = false;
        bool ok_pelvis = false;
        bool ok_chest = false;
        bool ok_head = false;
        bool ok_lhand = false;
        bool ok_lshould = false;
        bool ok_lelbow = false;
        bool ok_rhand = false;
        bool ok_rshould = false;
        bool ok_relbow = false;
        bool ok_lfoot = false;
        bool ok_rfoot = false;

    //--- From PICO
    bool right_controller_primary_button = false;
    bool right_controller_secondary_button = false;
    double right_controller_trigger = 0.0;

    bool left_controller_primary_button = false;
    bool left_controller_secondary_button = false;
    double left_controller_trigger = 0.0;
};