#include "collision_manager/collision_manager.h"

using namespace TOCABI;

ofstream data1("/home/sanghyuk/MATLAB/data/col_mgr_data1.txt");
ofstream data2("/home/sanghyuk/MATLAB/data/col_mgr_data2.txt");

CollisionManager::CollisionManager(const RobotData &rd) : rd_(rd)
{
    nh_cm_.setCallbackQueue(&queue_cm_);

    // initialize Pinocchio model and data
    std::string urdf_path;
    ros::param::get("/tocabi_controller/urdf_path", urdf_path);

    pinocchio::JointModelFreeFlyer floating_base;
    pinocchio::urdf::buildModel(urdf_path, floating_base, model_);
    data_ = pinocchio::Data(model_);

    // // initialize HPP-FCL collision objects related struct(CollisionBody) for the robot links
    initColBody();

    // Initialize variables related to self-collision avoidance constraints
    num_pairs_ = col_id_pairs_.size();
    J_self_col_.setZero(num_pairs_, MODEL_DOF_VIRTUAL);
    min_distances_.setZero(num_pairs_);

    // set DistanceRequest to enable computing nearest points
    request_enable_nearest_point_.enable_nearest_points = true;


    base_to_head_pose_pub_ = nh_cm_.advertise<geometry_msgs::PoseStamped>("/tocabi_cc/base_to_head", 1);
    aruco_detect_sub_ = nh_cm_.subscribe("/camera/qr_pose", 1, &CollisionManager::BasetoQRTransformCallback, this);
    ros::param::get("/kalman_filter/tracking_duration", tracking_duration);
    ros::param::get("/kalman_filter/process_variance", process_var);
    ros::param::get("/kalman_filter/process_rate_variance", process_rate_var);
    ros::param::get("/kalman_filter/measurement_variance", measurement_var);

    TrackedObstacle::setSamplingTime(1.0/hz_);
    TrackedObstacle::setCounterSize(static_cast<int>(hz_* tracking_duration));
    TrackedObstacle::setCovariances(process_var, process_rate_var, measurement_var);
#ifdef COMPILE_SIMULATION
    aruco_pose_pub_ = nh_cm_.advertise<geometry_msgs::PoseStamped>("/tocabi_cc/aruco_pose", 10);
    col_status_pub_ = nh_cm_.advertise<std_msgs::UInt8MultiArray>("/tocabi_cc/collision_status", 10);
    col_status_msg_.data.resize(Col_Obj_Count);
#endif
}

void CollisionManager::queueCallAvailable()
{
    queue_cm_.callAvailable(ros::WallDuration());
}

//========================================== Pinocchio ===========================================//

void CollisionManager::convertQVirtualRBDLtoPin(const Eigen::VectorQVQd &q_virtual_rbdl)
{
    q_virtual_pin_.segment<6>(0) = q_virtual_rbdl.segment<6>(0);
    q_virtual_pin_(6) = q_virtual_rbdl(MODEL_DOF_VIRTUAL);
    q_virtual_pin_.segment<MODEL_DOF>(7) = q_virtual_rbdl.segment<MODEL_DOF>(6);
}

//________________________________________________________________________________________________//

//====================================== Collision Related =======================================//

void CollisionManager::pubSelfCollisionStatus()
{
    for(unsigned int id = 0; id < Col_Obj_Count; id++){
        col_status_msg_.data[id] = static_cast<uint8_t>(cb_robot_[id].in_collision);
    }
    col_status_pub_.publish(col_status_msg_);

    // reset collision flags
    std::fill(collision_flags_.begin(), collision_flags_.end(), 0);
}

void CollisionManager::updateRobotCObjsTF()
{
    for(unsigned int i = Left_Pelvis_Col_ID; i < Col_Obj_Count; i++)
    {
        cb_robot_[i].trans_global = rd_.link_[cb_robot_[i].link_id].xpos + rd_.link_[cb_robot_[i].link_id].rotm * cb_robot_[i].trans_local;
        cb_robot_[i].rot_global = rd_.link_[cb_robot_[i].link_id].rotm * cb_robot_[i].rot_local;

        // cb_robot_[i].obj->setTransform(hpp::fcl::Transform3f(cb_robot_[i].rot_global, cb_robot_[i].trans_global));
    }
}


void CollisionManager::computeSelfColAvoidJac()
{
    convertQVirtualRBDLtoPin(rd_.q_virtual_);

    for(unsigned int i = 0; i < num_pairs_; i++)
    {
        CollisionObjectIdx id1 = col_id_pairs_[i].first;
        CollisionObjectIdx id2 = col_id_pairs_[i].second;
        
        Vector3d nearest_point1, nearest_point2;

        if(cb_robot_[id1].type == CollisionBody::Type::Sphere && cb_robot_[id2].type == CollisionBody::Type::Sphere){
            ColResultSphere2Sphere(id1, id2, min_distances_(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].type == CollisionBody::Type::Sphere && cb_robot_[id2].type == CollisionBody::Type::Capsule){
            ColResultSphere2Capsule(id1, id2, min_distances_(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].type == CollisionBody::Type::Capsule && cb_robot_[id2].type == CollisionBody::Type::Sphere){
            ColResultSphere2Capsule(id2, id1, min_distances_(i), nearest_point2, nearest_point1);
        }
        else if(cb_robot_[id1].type == CollisionBody::Type::Capsule && cb_robot_[id2].type == CollisionBody::Type::Capsule){
            ColResultCapsule2Capsule(id1, id2, min_distances_(i), nearest_point1, nearest_point2);
        }

        if (min_distances_(i) < 0.0){
            collision_flags_[id1] += 1;
            collision_flags_[id2] += 1;
        }

        J_self_col_.block<1, MODEL_DOF_VIRTUAL>(i, 0) = computeColPairJac(id1, nearest_point1,
                                                                          id2, nearest_point2,
                                                                          DyrosMath::sign(min_distances_(i))
                                                                          );
    }

    for(unsigned int id = 0; id < Col_Obj_Count; id++) cb_robot_[id].in_collision = collision_flags_[id] > 0;

}

// using HPP-FCL
// void CollisionManager::computeSelfColAvoidJac()
// {   
//     convertQVirtualRBDLtoPin(rd_.q_virtual_);

//     for(unsigned int i = 0; i < num_pairs_; i++)
//     {
//         CollisionObjectIdx id1 = col_id_pairs_[i].first;
//         CollisionObjectIdx id2 = col_id_pairs_[i].second;

//         hpp::fcl::DistanceResult dist_res_tmp = getDistanceResultBetweenCObjs(id1, id2);

//         min_distances_(i) = dist_res_tmp.min_distance;
        
//         J_self_col_.block(i,0, 1,MODEL_DOF_VIRTUAL) = computeColPairJacHPPFCL(id1, dist_res_tmp.nearest_points[0],
//                                                                               id2, dist_res_tmp.nearest_points[1],
//                                                                               DyrosMath::sign(min_distances_(i))
//                                                                               );
        
//     }
// }

void CollisionManager::checkSelfCollision()
{
    for(unsigned int i = 0; i < num_pairs_; i++)
    {
        CollisionObjectIdx id1 = col_id_pairs_[i].first;
        CollisionObjectIdx id2 = col_id_pairs_[i].second;
        
        Vector3d nearest_point1, nearest_point2;

        if(cb_robot_[id1].type == CollisionBody::Type::Sphere && cb_robot_[id2].type == CollisionBody::Type::Sphere){
            ColResultSphere2Sphere(id1, id2, min_distances_(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].type == CollisionBody::Type::Sphere && cb_robot_[id2].type == CollisionBody::Type::Capsule){
            ColResultSphere2Capsule(id1, id2, min_distances_(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].type == CollisionBody::Type::Capsule && cb_robot_[id2].type == CollisionBody::Type::Sphere){
            ColResultSphere2Capsule(id2, id1, min_distances_(i), nearest_point2, nearest_point1);
        }
        else if(cb_robot_[id1].type == CollisionBody::Type::Capsule && cb_robot_[id2].type == CollisionBody::Type::Capsule){
            ColResultCapsule2Capsule(id1, id2, min_distances_(i), nearest_point1, nearest_point2);
        }
        
        if (min_distances_(i) < 0.0){
            collision_flags_[id1] += 1;
            collision_flags_[id2] += 1;
        }
    }

    for(unsigned int id = 0; id < Col_Obj_Count; id++) cb_robot_[id].in_collision = collision_flags_[id] > 0;

}

void CollisionManager::initColBody()
{
    cb_robot_.resize(Col_Obj_Count);

    // Left Pelvis
    cb_robot_[Left_Pelvis_Col_ID].link_name = "Pelvis_Link";
    cb_robot_[Left_Pelvis_Col_ID].link_id = TOCABI::Pelvis;
    cb_robot_[Left_Pelvis_Col_ID].joint_id = model_.frames[model_.getFrameId("Pelvis_Link")].parent;
    cb_robot_[Left_Pelvis_Col_ID].trans_local  = Eigen::Vector3d(0.05, 0.02, 0.1);
    cb_robot_[Left_Pelvis_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Pelvis_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Left_Pelvis_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.18, 0.05); // radius, height

    // Right Pelvis
    cb_robot_[Right_Pelvis_Col_ID].link_name = "Pelvis_Link";
    cb_robot_[Right_Pelvis_Col_ID].link_id = TOCABI::Pelvis;
    cb_robot_[Right_Pelvis_Col_ID].joint_id = model_.frames[model_.getFrameId("Pelvis_Link")].parent;
    cb_robot_[Left_Pelvis_Col_ID].trans_local  = Eigen::Vector3d(0.05, -0.02, 0.1);
    cb_robot_[Right_Pelvis_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Pelvis_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Right_Pelvis_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.18, 0.05); // radius, height

    // Left Upper Leg
    cb_robot_[Left_Upper_Leg_Col_ID].link_name = "L_Thigh_Link";
    cb_robot_[Left_Upper_Leg_Col_ID].link_id = TOCABI::Left_Thigh;
    cb_robot_[Left_Upper_Leg_Col_ID].joint_id = model_.frames[model_.getFrameId("L_Thigh_Link")].parent;
    cb_robot_[Left_Upper_Leg_Col_ID].trans_local = Eigen::Vector3d(0.0, 0.0, -0.1323);
    cb_robot_[Left_Upper_Leg_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Upper_Leg_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Left_Upper_Leg_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.075, 0.34); // radius, height

    // Left Lower Leg
    cb_robot_[Left_Lower_Leg_Col_ID].link_name = "L_Knee_Link";
    cb_robot_[Left_Lower_Leg_Col_ID].link_id = TOCABI::Left_Knee;
    cb_robot_[Left_Lower_Leg_Col_ID].joint_id = model_.frames[model_.getFrameId("L_Knee_Link")].parent;
    cb_robot_[Left_Lower_Leg_Col_ID].trans_local = Eigen::Vector3d(0.0, 0.0, -0.23);
    cb_robot_[Left_Lower_Leg_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Lower_Leg_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Left_Lower_Leg_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.075, 0.3); // radius, height

    // Left Inner Foot
    cb_robot_[Left_Inner_Foot_Col_ID].link_name = "L_Foot_Link";
    cb_robot_[Left_Inner_Foot_Col_ID].link_id = TOCABI::Left_Foot;
    cb_robot_[Left_Inner_Foot_Col_ID].joint_id = model_.frames[model_.getFrameId("L_Foot_Link")].parent;
    cb_robot_[Left_Inner_Foot_Col_ID].trans_local = Eigen::Vector3d(0.03, -0.0455, -0.15);
    cb_robot_[Left_Inner_Foot_Col_ID].rot_local = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Left_Inner_Foot_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Left_Inner_Foot_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.0432, 0.24); // radius, height
    
    // Left Outer Foot
    cb_robot_[Left_Outer_Foot_Col_ID].link_name = "L_Foot_Link";
    cb_robot_[Left_Outer_Foot_Col_ID].link_id = TOCABI::Left_Foot;
    cb_robot_[Left_Outer_Foot_Col_ID].joint_id = model_.frames[model_.getFrameId("L_Foot_Link")].parent;
    cb_robot_[Left_Outer_Foot_Col_ID].trans_local = Eigen::Vector3d(0.03, 0.0455, -0.15);
    cb_robot_[Left_Outer_Foot_Col_ID].rot_local = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Left_Outer_Foot_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Left_Outer_Foot_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.0432, 0.24); // radius, height

    // Right Upper Leg
    cb_robot_[Right_Upper_Leg_Col_ID].link_name = "R_Thigh_Link";
    cb_robot_[Right_Upper_Leg_Col_ID].link_id = TOCABI::Right_Thigh;
    cb_robot_[Right_Upper_Leg_Col_ID].joint_id = model_.frames[model_.getFrameId("R_Thigh_Link")].parent;
    cb_robot_[Right_Upper_Leg_Col_ID].trans_local = Eigen::Vector3d(0.0, 0.0, -0.1323);
    cb_robot_[Right_Upper_Leg_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Upper_Leg_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Right_Upper_Leg_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.075, 0.34); // radius, height

    // Right Lower Leg
    cb_robot_[Right_Lower_Leg_Col_ID].link_name = "R_Knee_Link";
    cb_robot_[Right_Lower_Leg_Col_ID].link_id = TOCABI::Right_Knee;
    cb_robot_[Right_Lower_Leg_Col_ID].joint_id = model_.frames[model_.getFrameId("R_Knee_Link")].parent;
    cb_robot_[Right_Lower_Leg_Col_ID].trans_local = Eigen::Vector3d(0.0, 0.0, -0.23);
    cb_robot_[Right_Lower_Leg_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Lower_Leg_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Right_Lower_Leg_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.075, 0.30); // radius, height

    // Right Inner Foot
    cb_robot_[Right_Inner_Foot_Col_ID].link_name = "R_Foot_Link";
    cb_robot_[Right_Inner_Foot_Col_ID].link_id = TOCABI::Right_Foot;
    cb_robot_[Right_Inner_Foot_Col_ID].joint_id = model_.frames[model_.getFrameId("R_Foot_Link")].parent;
    cb_robot_[Right_Inner_Foot_Col_ID].trans_local = Eigen::Vector3d(0.03, 0.0455, -0.15);
    cb_robot_[Right_Inner_Foot_Col_ID].rot_local = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Right_Inner_Foot_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Right_Inner_Foot_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.0432, 0.24); // radius, height

    // Right Outer Foot
    cb_robot_[Right_Outer_Foot_Col_ID].link_name = "R_Foot_Link";
    cb_robot_[Right_Outer_Foot_Col_ID].link_id = TOCABI::Right_Foot;
    cb_robot_[Right_Outer_Foot_Col_ID].joint_id = model_.frames[model_.getFrameId("R_Foot_Link")].parent;
    cb_robot_[Right_Outer_Foot_Col_ID].trans_local = Eigen::Vector3d(0.03, -0.0455, -0.15);
    cb_robot_[Right_Outer_Foot_Col_ID].rot_local = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Right_Outer_Foot_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Right_Outer_Foot_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.0432, 0.24); // radius, height

    // // Left Upper Body
    // cb_robot_[Left_Upper_Body_Col_ID].link_name = "Upperbody_Link";
    // cb_robot_[Left_Upper_Body_Col_ID].link_id = TOCABI::Upper_Body;
    // cb_robot_[Left_Upper_Body_Col_ID].joint_id = model_.frames[model_.getFrameId("Upperbody_Link")].parent;
    // cb_robot_[Left_Upper_Body_Col_ID].trans_local = Eigen::Vector3d(0.023, 0.154, 0.19);
    // cb_robot_[Left_Upper_Body_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    // cb_robot_[Left_Upper_Body_Col_ID].type = CollisionBody::Type::Capsule;
    // cb_robot_[Left_Upper_Body_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.06, 0.03); // radius, height

    // // Right Upper Body
    // cb_robot_[Right_Upper_Body_Col_ID].link_name = "Upperbody_Link";
    // cb_robot_[Right_Upper_Body_Col_ID].link_id = TOCABI::Upper_Body;
    // cb_robot_[Right_Upper_Body_Col_ID].joint_id = model_.frames[model_.getFrameId("Upperbody_Link")].parent;
    // cb_robot_[Right_Upper_Body_Col_ID].trans_local = Eigen::Vector3d(0.023, -0.154, 0.19);
    // cb_robot_[Right_Upper_Body_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    // cb_robot_[Right_Upper_Body_Col_ID].type = CollisionBody::Type::Capsule;
    // cb_robot_[Right_Upper_Body_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.06, 0.03); // radius, height

    // Left Upper Arm
    cb_robot_[Left_Upper_Arm_Col_ID].link_name = "L_Armlink_Link";
    cb_robot_[Left_Upper_Arm_Col_ID].link_id = TOCABI::Left_Upper_Arm;
    cb_robot_[Left_Upper_Arm_Col_ID].joint_id = model_.frames[model_.getFrameId("L_Armlink_Link")].parent;
    cb_robot_[Left_Upper_Arm_Col_ID].trans_local = Eigen::Vector3d(0.005, 0.12, -0.01);
    cb_robot_[Left_Upper_Arm_Col_ID].rot_local = DyrosMath::rotateWithX(75 * DEG2RAD);
    cb_robot_[Left_Upper_Arm_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Left_Upper_Arm_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.062, 0.2); // radius, height

    // Left ForeArm
    cb_robot_[Left_ForeArm_Col_ID].link_name = "L_Forearm_Link";
    cb_robot_[Left_ForeArm_Col_ID].link_id = TOCABI::Left_Forearm;
    cb_robot_[Left_ForeArm_Col_ID].joint_id = model_.frames[model_.getFrameId("L_Forearm_Link")].parent;
    cb_robot_[Left_ForeArm_Col_ID].trans_local = Eigen::Vector3d(0.0, 0.05, 0.0);
    cb_robot_[Left_ForeArm_Col_ID].rot_local = DyrosMath::rotateWithX(M_PI / 2.0);
    cb_robot_[Left_ForeArm_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Left_ForeArm_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.055, 0.2); // radius, height

    // Left Hand
    cb_robot_[Left_Hand_Col_ID].link_name = "L_Wrist2_Link";
    cb_robot_[Left_Hand_Col_ID].link_id = TOCABI::Left_Hand;
    cb_robot_[Left_Hand_Col_ID].joint_id = model_.frames[model_.getFrameId("L_Wrist2_Link")].parent;
    cb_robot_[Left_Hand_Col_ID].trans_local = Eigen::Vector3d(0.0, 0.0, 0.0);
    cb_robot_[Left_Hand_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Hand_Col_ID].type = CollisionBody::Type::Sphere;
    cb_robot_[Left_Hand_Col_ID].sphere = std::make_shared<hpp::fcl::Sphere>(0.06); // radius

    // Head
    cb_robot_[Head_Col_ID].link_name = "Head_Link";
    cb_robot_[Head_Col_ID].link_id = TOCABI::Head;
    cb_robot_[Head_Col_ID].joint_id = model_.frames[model_.getFrameId("Head_Link")].parent;
    cb_robot_[Head_Col_ID].trans_local = Eigen::Vector3d(0.035, 0.0, 0.0735);
    cb_robot_[Head_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Head_Col_ID].type = CollisionBody::Type::Sphere;
    cb_robot_[Head_Col_ID].sphere =  std::make_shared<hpp::fcl::Sphere>(0.142); // radius

    // Right Upper Arm
    cb_robot_[Right_Upper_Arm_Col_ID].link_name = "R_Armlink_Link";
    cb_robot_[Right_Upper_Arm_Col_ID].link_id = TOCABI::Right_Upper_Arm;
    cb_robot_[Right_Upper_Arm_Col_ID].joint_id = model_.frames[model_.getFrameId("R_Armlink_Link")].parent;
    cb_robot_[Right_Upper_Arm_Col_ID].trans_local = Eigen::Vector3d(0.005, -0.12, -0.01);
    cb_robot_[Right_Upper_Arm_Col_ID].rot_local = DyrosMath::rotateWithX(-75 * DEG2RAD);
    cb_robot_[Right_Upper_Arm_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Right_Upper_Arm_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.062, 0.2); // radius, height

    // Right ForeArm
    cb_robot_[Right_ForeArm_Col_ID].link_name = "R_Forearm_Link";
    cb_robot_[Right_ForeArm_Col_ID].link_id = TOCABI::Right_Forearm;
    cb_robot_[Right_ForeArm_Col_ID].joint_id = model_.frames[model_.getFrameId("R_Forearm_Link")].parent;
    cb_robot_[Right_ForeArm_Col_ID].trans_local = Eigen::Vector3d(0.0, -0.05, 0.0);
    cb_robot_[Right_ForeArm_Col_ID].rot_local = DyrosMath::rotateWithX(M_PI / 2.0);
    cb_robot_[Right_ForeArm_Col_ID].type = CollisionBody::Type::Capsule;
    cb_robot_[Right_ForeArm_Col_ID].capsule = std::make_shared<hpp::fcl::Capsule>(0.055, 0.2); // radius, height

    // Right Hand
    cb_robot_[Right_Hand_Col_ID].link_name = "R_Wrist2_Link";
    cb_robot_[Right_Hand_Col_ID].link_id = TOCABI::Right_Hand;
    cb_robot_[Right_Hand_Col_ID].joint_id = model_.frames[model_.getFrameId("R_Wrist2_Link")].parent;
    cb_robot_[Right_Hand_Col_ID].trans_local = Eigen::Vector3d(0.0, 0.0, 0.0);
    cb_robot_[Right_Hand_Col_ID].rot_local = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Hand_Col_ID].type = CollisionBody::Type::Sphere;
    cb_robot_[Right_Hand_Col_ID].sphere = std::make_shared<hpp::fcl::Sphere>(0.06); // radius

    assignRobotCObjs();
}

void CollisionManager::assignRobotCObjs()
{
    for(unsigned int i = 0; i < Col_Obj_Count; i++)
    {
        cb_robot_[i].trans_global = rd_.link_[cb_robot_[i].link_id].xpos + rd_.link_[cb_robot_[i].link_id].rotm * cb_robot_[i].trans_local;
        cb_robot_[i].rot_global = rd_.link_[cb_robot_[i].link_id].rotm * cb_robot_[i].rot_local;
        hpp::fcl::Transform3f obj_tf(cb_robot_[i].rot_global, cb_robot_[i].trans_global);

        // if(cb_robot_[i].type == CollisionBody::Type::Sphere)
        // {
        //     cb_robot_[i].obj = std::make_shared<hpp::fcl::CollisionObject>(cb_robot_[i].sphere, obj_tf);
        // }
        // else if(cb_robot_[i].type == CollisionBody::Type::Capsule)
        // {
        //     cb_robot_[i].obj = std::make_shared<hpp::fcl::CollisionObject>(cb_robot_[i].capsule, obj_tf);
        // }
    }
}


hpp::fcl::DistanceResult CollisionManager::getDistanceResultBetweenCObjs(const CollisionObjectIdx obj_id1, const CollisionObjectIdx obj_id2, DistanceResultOption option)
{
    hpp::fcl::DistanceResult dist_result;
    
    if(option == Links) {
        distance(cb_robot_[obj_id1].obj.get(), cb_robot_[obj_id2].obj.get(), request_enable_nearest_point_, dist_result);
    }
    else if(option == Link2Env) {
        distance(cb_robot_[obj_id1].obj.get(), cb_obstacles_[obj_id2].obj.get(), request_enable_nearest_point_, dist_result);
    }
    else {
        distance(cb_obstacles_[obj_id1].obj.get(), cb_obstacles_[obj_id2].obj.get(), request_enable_nearest_point_, dist_result);
    }
    cb_obstacles_[obj_id1].obj->getRotation();

    // for debugging/verification: print minimum distance and closest points
    // cout << "Minimum distance: " << dist_result.min_distance << endl;
    // cout << "Closest point on object 1: " << dist_result.nearest_points[0].transpose() << endl;
    // cout << "Closest point on object 2: " << dist_result.nearest_points[1].transpose() << endl;

    return dist_result;
}

void CollisionManager::ColResultSphere2Sphere(const CollisionObjectIdx obj_id1, const CollisionObjectIdx obj_id2, double &min_distance, Eigen::Vector3d &nearest_point1, Eigen::Vector3d &nearest_point2)
{
    // center point: center of sphere
    Vector3d center_point1 = cb_robot_[obj_id1].trans_global;
    Vector3d center_point2 = cb_robot_[obj_id2].trans_global;

    min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].sphere->radius + cb_robot_[obj_id2].sphere->radius);

    nearest_point1= center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].sphere->radius;
    nearest_point2= center_point2 + (center_point1 - center_point2).normalized() * cb_robot_[obj_id2].sphere->radius;
}

void CollisionManager::ColResultSphere2Capsule(const CollisionObjectIdx obj_id1, const CollisionObjectIdx obj_id2, double &min_distance, Eigen::Vector3d &nearest_point1, Eigen::Vector3d &nearest_point2)
{
    // center_point1: center of sphere, center_point2: position of the nearest point on the centerline(i.e., the central axis) of capsule 
    Vector3d center_point1, center_point2;
    center_point1 =  cb_robot_[obj_id1].trans_global;
    // p_capsule1, p_capsule2: endpoints of capsule centerline
    Vector3d p_capsule1 = cb_robot_[obj_id2].trans_global + cb_robot_[obj_id2].rot_global * Eigen::Vector3d(0, 0, cb_robot_[obj_id2].capsule->halfLength);
    Vector3d p_capsule2 = cb_robot_[obj_id2].trans_global + cb_robot_[obj_id2].rot_global * Eigen::Vector3d(0, 0, -cb_robot_[obj_id2].capsule->halfLength);

    Vector3d vec_capsule1_to_sphere = nearest_point1 - p_capsule1;
    Vector3d vec_capsule2_to_sphere = nearest_point1 - p_capsule2;
    Vector3d vec_capsule1_to_capsule2 = p_capsule2 - p_capsule1;

    // case 1: the sphere center lies to the left of the capsule’s centerline (before the first endpoint)
    if(vec_capsule1_to_sphere.dot(vec_capsule1_to_capsule2) <= 0){
        center_point2 = p_capsule1;
    }

    // case 2: the sphere center lies to the right of the capsule’s centerline (beyond the second endpoint)
    else if(vec_capsule2_to_sphere.dot(vec_capsule1_to_capsule2) >= 0){
        center_point2 = p_capsule2;
    }

    // case 3: the sphere center lies between the two endpoints of the capsule’s centerline
    else{
        center_point2 = p_capsule1 + (vec_capsule1_to_sphere.dot(vec_capsule1_to_capsule2) / vec_capsule1_to_capsule2.squaredNorm()) * vec_capsule1_to_capsule2;
    }

    min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].sphere->radius + cb_robot_[obj_id2].capsule->radius);

    nearest_point1 = center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].sphere->radius;
    nearest_point2 = center_point2 + (center_point1 - center_point2).normalized() * cb_robot_[obj_id2].capsule->radius;
}

void CollisionManager::ColResultCapsule2Capsule(const CollisionObjectIdx obj_id1, const CollisionObjectIdx obj_id2, double &min_distance, Eigen::Vector3d &nearest_point1, Eigen::Vector3d &nearest_point2)
{
    // reference paper: On fast computation of distance between line segments(1985)
    // link: https://www.sciencedirect.com/science/article/pii/0020019085900328

    // center_point: position of the nearest point on the centerline(i.e., the central axis) of capsule
    Vector3d center_point1, center_point2;

    // pA, pB: endpoints of capsule 1 centerline
    // pC, pD: endpoints of capsule 2 centerline
    Vector3d pA, pB, pC, pD;

    pA = cb_robot_[obj_id1].trans_global + cb_robot_[obj_id1].rot_global * Eigen::Vector3d(0, 0, cb_robot_[obj_id1].capsule->halfLength);
    pB = cb_robot_[obj_id1].trans_global + cb_robot_[obj_id1].rot_global * Eigen::Vector3d(0, 0, -cb_robot_[obj_id1].capsule->halfLength);
    pC = cb_robot_[obj_id2].trans_global + cb_robot_[obj_id2].rot_global * Eigen::Vector3d(0, 0, cb_robot_[obj_id2].capsule->halfLength);
    pD = cb_robot_[obj_id2].trans_global + cb_robot_[obj_id2].rot_global * Eigen::Vector3d(0, 0, -cb_robot_[obj_id2].capsule->halfLength);

    // center_point1 = pA * (1-t) + pB * t (0 <= t <=1)
    // center_point2 = pC * (1-u) + pD * u (0 <= u <=1)
    double t, u;

    double R, S1, S2, D1, D2;

    // Step 1
    R = (pB - pA).dot(pD - pC);
    S1 = (pB - pA).dot(pC - pA);
    S2 = (pD - pC).dot(pC - pA);
    D1 = (pB - pA).squaredNorm();
    D2 = (pD - pC).squaredNorm();
    
    // parallel case(set t = 0 & directly go to Step 3)
    if((D1 * D2 - pow(R,2)) == 0){
        t = 0;
        // Step 3
        u = - S2 / D2;
        if(u < 0.0){
            u = 0.0;
            // Step 4                          
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
        else if (u > 1.0){
            u = 1.0;
            // Step 4                          
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
    }

    else{
        // Step 2
        t = DyrosMath::minmax_cut((S1 * D2 - S2 * R) / (D1 * D2 - pow(R,2)),
                                  0.0,
                                  1.0
                                  );
        // Step 3
        u = (t * R - S2) / D2;
        if(u < 0.0){
            u = 0.0;
            // Step 4
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
        else if (u > 1.0){
            u = 1.0;
            // Step 4                          
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
    }

    // Step 5
    center_point1 = pA * (1 - t) + pB * t;
    center_point2 = pC * (1 - u) + pD * u;

    min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].capsule->radius + cb_robot_[obj_id2].capsule->radius);
    
    nearest_point1 = center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].capsule->radius;
    nearest_point2 = center_point2 + (center_point1 - center_point2).normalized() * cb_robot_[obj_id2].capsule->radius;
}

Eigen::MatrixXd CollisionManager::computeColPairJac(const CollisionObjectIdx obj_id1, const Eigen::Vector3d nearest_point1, const CollisionObjectIdx obj_id2, const Eigen::Vector3d nearest_point2, const int sign)
{
    // reference paper: Humanoid Self-Collision Avoidance Using Whole-Body Control with Control Barrier Functions(2022)
    // link: https://ieeexplore.ieee.org/abstract/document/10000235

    pinocchio::computeJointJacobians(model_, data_, q_virtual_pin_);
    pinocchio::framesForwardKinematics(model_, data_, q_virtual_pin_);

    // Matrix3d base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(2)); 

    // Jacobian used for the self-collision avoidance constraint
    MatrixXd J_col_pair;
    J_col_pair.setZero(1, MODEL_DOF_VIRTUAL);

    // unit vector connecting the two closest points between two collision objects
    Vector3d normal_vec = (nearest_point1 - nearest_point2) / (nearest_point1 - nearest_point2).norm();

    // Jacobians from base to trans_joint_to_nearestcenter
    MatrixXd J_col_obj1, J_col_obj2;
    MatrixXd Jv_col_obj1, Jv_col_obj2;  // linear part
    J_col_obj1.setZero(6,MODEL_DOF_VIRTUAL); J_col_obj2.setZero(6,MODEL_DOF_VIRTUAL);
    Jv_col_obj1.setZero(3,MODEL_DOF_VIRTUAL); Jv_col_obj2.setZero(3,MODEL_DOF_VIRTUAL);

    // first collision object
    // vector stores local translation vector from the joint to the nearest point on the collision object's center (sphere: center or capsule: axis point)
    Vector3d trans_joint_to_nearestcenter_local1 = rd_.link_[cb_robot_[obj_id1].link_id].rotm.transpose() * (nearest_point1 - rd_.link_[cb_robot_[obj_id1].link_id].xpos);

    // transformation matrix contains translation from the joint to the nearest point on the collision object's center
    pinocchio::SE3 placement1(Eigen::Matrix3d::Identity(), trans_joint_to_nearestcenter_local1);
    pinocchio::getFrameJacobian(model_, data_, cb_robot_[obj_id1].joint_id, placement1, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_col_obj1);
    Jv_col_obj1 =  J_col_obj1.block<3, MODEL_DOF_VIRTUAL>(0, 0);

    // second collision object
    // vector stores local translation vector from the joint to the nearest point on the collision object's center (sphere: center or capsule: axis point)
    Vector3d trans_joint_to_nearestcenter_local2 = rd_.link_[cb_robot_[obj_id2].link_id].rotm.transpose() * (nearest_point2 - rd_.link_[cb_robot_[obj_id2].link_id].xpos);

    // transformation matrix contains translation from the joint to the nearest point on the collision object's center
    pinocchio::SE3 placement2(Eigen::Matrix3d::Identity(), trans_joint_to_nearestcenter_local2);
    pinocchio::getFrameJacobian(model_, data_, cb_robot_[obj_id2].joint_id, placement2, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_col_obj2);
    Jv_col_obj2 = J_col_obj2.block<3, MODEL_DOF_VIRTUAL>(0, 0);

    J_col_pair = sign * normal_vec.transpose() * (Jv_col_obj1 - Jv_col_obj2);

    return J_col_pair;
}

Eigen::MatrixXd CollisionManager::computeColPairJacHPPFCL(const CollisionObjectIdx obj_id1, const Eigen::Vector3d nearest_point1, const CollisionObjectIdx obj_id2, const Eigen::Vector3d nearest_point2, const int sign)
{   
    // reference paper: Humanoid Self-Collision Avoidance Using Whole-Body Control with Control Barrier Functions(2022)
    // link: https://ieeexplore.ieee.org/abstract/document/10000235
    pinocchio::computeJointJacobians(model_, data_, q_virtual_pin_);
    pinocchio::framesForwardKinematics(model_, data_, q_virtual_pin_);
    // pinocchio::updateFramePlacements(model_, data_);

    // Jacobian used for the self-collision avoidance constraint
    MatrixXd J_col_pair;
    J_col_pair.setZero(1, MODEL_DOF_VIRTUAL);

    // unit vector connecting the two closest points between two collision objects
    Vector3d normal_vec = (nearest_point1 - nearest_point2) / (nearest_point1 - nearest_point2).norm();

    // Jacobians from base to trans_joint_to_nearestcenter
    MatrixXd J_col_obj1, J_col_obj2;
    MatrixXd Jv_col_obj1, Jv_col_obj2;  // linear part
    J_col_obj1.setZero(6,MODEL_DOF_VIRTUAL); J_col_obj2.setZero(6,MODEL_DOF_VIRTUAL);
    Jv_col_obj1.setZero(3,MODEL_DOF_VIRTUAL); Jv_col_obj2.setZero(3,MODEL_DOF_VIRTUAL);


    // vector stores local translation vector from the joint to the nearest point on the collision object's center (sphere: center or capsule: axis point)
    Vector3d trans_joint_to_nearestcenter_local;

    // first collision object
    if(cb_robot_[obj_id1].type == CollisionBody::Type::Sphere){
        trans_joint_to_nearestcenter_local = cb_robot_[obj_id1].trans_local;
    }

    else if(cb_robot_[obj_id1].type == CollisionBody::Type::Capsule){
        Vector3d trans_object_to_nearestcenter_local;
        trans_object_to_nearestcenter_local.setZero();
        trans_object_to_nearestcenter_local(2) =  DyrosMath::minmax_cut((cb_robot_[obj_id1].rot_global.transpose() * (nearest_point1 - cb_robot_[obj_id1].trans_global))(2),
                                                                        -cb_robot_[obj_id1].capsule->halfLength,
                                                                        cb_robot_[obj_id1].capsule->halfLength);

        trans_joint_to_nearestcenter_local = cb_robot_[obj_id1].trans_local + cb_robot_[obj_id1].rot_local * trans_object_to_nearestcenter_local;
    }

    // transformation matrix contains translation from the joint to the nearest point on the collision object's center
    pinocchio::SE3 placement1(Eigen::Matrix3d::Identity(), trans_joint_to_nearestcenter_local);
    pinocchio::getFrameJacobian(model_, data_, cb_robot_[obj_id1].joint_id, placement1, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_col_obj1);
    Jv_col_obj1 = J_col_obj1.block<3, MODEL_DOF_VIRTUAL>(0, 0);

    // second collision object
    if(cb_robot_[obj_id2].type == CollisionBody::Type::Sphere){
        trans_joint_to_nearestcenter_local = cb_robot_[obj_id2].trans_local;
    }

    else if(cb_robot_[obj_id2].type == CollisionBody::Type::Capsule){
        Vector3d trans_object_to_nearestcenter_local;
        trans_object_to_nearestcenter_local.setZero();
        trans_object_to_nearestcenter_local(2) =  DyrosMath::minmax_cut((cb_robot_[obj_id2].rot_global.transpose() * (nearest_point2 - cb_robot_[obj_id2].trans_global))(2),
                                                                        -cb_robot_[obj_id2].capsule->halfLength,
                                                                        cb_robot_[obj_id2].capsule->halfLength);

        trans_joint_to_nearestcenter_local = cb_robot_[obj_id2].trans_local + cb_robot_[obj_id2].rot_local * trans_object_to_nearestcenter_local;
    }

    // transformation matrix contains translation from the joint to the nearest point on the collision object's center
    pinocchio::SE3 placement2(Eigen::Matrix3d::Identity(), trans_joint_to_nearestcenter_local);
    pinocchio::getFrameJacobian(model_, data_, cb_robot_[obj_id2].joint_id, placement2, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_col_obj2);
    Jv_col_obj2 = J_col_obj2.block<3, MODEL_DOF_VIRTUAL>(0, 0);

    J_col_pair = sign * normal_vec.transpose() * (Jv_col_obj1 - Jv_col_obj2);

    return J_col_pair;
}

std::shared_ptr<hpp::fcl::CollisionObject> CollisionManager::assignSphereCObj(const Eigen::Matrix3d obj_rot, const Eigen::Vector3d obj_trans, const double radius)
{
    // Create geometry
    // The type of geometry should be shared pointer since it is required by CollisionObject
    auto sphere = std::make_shared<hpp::fcl::Sphere>(radius);

    // Build transform
    hpp::fcl::Transform3f obj_tf(obj_rot, obj_trans);

    // Create collision object with geometry + transform
    auto col_obj = std::make_shared<hpp::fcl::CollisionObject>(sphere, obj_tf);

    return col_obj;
}

std::shared_ptr<hpp::fcl::CollisionObject> CollisionManager::assignCapsuleCObj(const Eigen::Matrix3d obj_rot, const Eigen::Vector3d obj_trans, const double radius, const double height)
{
    // Create geometry
    // The type of geometry should be shared pointer since it is required by CollisionObject
    auto capsule = std::make_shared<hpp::fcl::Capsule>(radius, height);

    // Build transform
    hpp::fcl::Transform3f obj_tf(obj_rot, obj_trans);

    // Create collision object with geometry + transform
    auto col_obj = std::make_shared<hpp::fcl::CollisionObject>(capsule, obj_tf);

    return col_obj;
}

std::shared_ptr<hpp::fcl::CollisionObject> CollisionManager::assignBoxCObj(const Eigen::Matrix3d obj_rot, const Eigen::Vector3d obj_trans, const double size_x, const double size_y, const double size_z)
{
    // Create geometry
    // The type of geometry should be shared pointer since it is required by CollisionObject
    auto box = std::make_shared<hpp::fcl::Box>(size_x, size_y, size_z);

    // Build transform
    hpp::fcl::Transform3f obj_tf(obj_rot, obj_trans);

    // Create collision object with geometry + transform
    auto col_obj = std::make_shared<hpp::fcl::CollisionObject>(box, obj_tf);

    return col_obj;
}

//________________________________________________________________________________________________//

//================================== Communication with Camera ===================================//

void CollisionManager::pubBasetoHeadTransform()
{
    // ts_base_to_head stores the transform data between base_link and head_link
    base_to_head_pose_msg_.header.stamp = ros::Time::now();

    // translation
    base_to_head_pose_msg_.pose.position.x = rd_.link_[Head].local_xpos(0);
    base_to_head_pose_msg_.pose.position.y = rd_.link_[Head].local_xpos(1);
    base_to_head_pose_msg_.pose.position.z = rd_.link_[Head].local_xpos(2);

    // rotation matrix -> quaternion
    Eigen::Quaterniond quat_head(rd_.link_[Head].local_rotm);
    base_to_head_pose_msg_.pose.orientation.x = quat_head.x();
    base_to_head_pose_msg_.pose.orientation.y = quat_head.y();
    base_to_head_pose_msg_.pose.orientation.z = quat_head.z();
    base_to_head_pose_msg_.pose.orientation.w = quat_head.w();

    // publishes the transform data
    base_to_head_pose_pub_.publish(base_to_head_pose_msg_);
}

Eigen::Isometry3d CollisionManager::getBasetoQRTransform()
{
    Eigen::Isometry3d base_to_qr_transform;

    try
    {
        // listener looks for the transform data between base_link and qr_code(object_frame)
        base_to_qr_tf_msg_ = tf_buffer_.lookupTransform("base_link", "object_frame", ros::Time(0));

        // store the translation data
        base_to_qr_transform.translation() << base_to_qr_tf_msg_.transform.translation.x,
                                              base_to_qr_tf_msg_.transform.translation.y,
                                              base_to_qr_tf_msg_.transform.translation.z;
        // store the rotation data (quaternion -> rotation matrix)
        Eigen::Quaterniond quat_qr(base_to_qr_tf_msg_.transform.rotation.w,
                                   base_to_qr_tf_msg_.transform.rotation.x,
                                   base_to_qr_tf_msg_.transform.rotation.y,
                                   base_to_qr_tf_msg_.transform.rotation.z);
        base_to_qr_transform.linear() = quat_qr.toRotationMatrix();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("Failed to lookup transform form base_link to object_frame: %s", ex.what());
        base_to_qr_transform.setIdentity();
    }
    
    return base_to_qr_transform;
}

void CollisionManager::BasetoQRTransformCallback(const geometry_msgs::PoseStamped &msg)
{
    std::lock_guard<std::mutex> lk(meas_mutex_);
    
    base_to_qr_transform_.translation() << msg.pose.position.x,
                                           msg.pose.position.y,
                                           msg.pose.position.z;
    
    Eigen::Quaterniond quat_qr(msg.pose.orientation.w,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z);
    
    base_to_qr_transform_.linear() = quat_qr.toRotationMatrix();                           

    has_new_measure_ = true;
}

void CollisionManager::updateObstacle()
{
    if(!tracked_obstacles_.empty()) tracked_obstacles_[0].updateState();

    {
        std::lock_guard<std::mutex> lk(meas_mutex_);
        if(has_new_measure_){
            if(tracked_obstacles_.empty()){
                TrackedObstacle to(base_to_qr_transform_.translation());
                tracked_obstacles_.push_back(to);
            }
            else{
                tracked_obstacles_[0].correctState(base_to_qr_transform_.translation());
            }

            has_new_measure_ = false;
        }
    }
    
    if(!tracked_obstacles_.empty()){
        if(tracked_obstacles_[0].hasFaded()){
            tracked_obstacles_.clear();
            cb_obstacles_.clear();
        }
        else{
            if(cb_obstacles_.empty()){
                CollisionBody cb;
                cb.trans_global = tracked_obstacles_[0].getObstacle().pos_;
                cb_obstacles_.push_back(cb);
            }
            else{
                cb_obstacles_[0].trans_global = tracked_obstacles_[0].getObstacle().pos_;
                cb_obstacles_[0].vel_global = tracked_obstacles_[0].getObstacle().vel_;
                // data1 << cb_obstacles_[0].trans_global(1) << "," << cb_obstacles_[0].vel_global(1) << endl;
            }
        }
    }
}

//________________________________________________________________________________________________//

//====================================== Obstacle in MuJoCo ======================================//

void CollisionManager::pubQRObstaclePose(const int sim_tick, const double hz)
{
    // ts_QR stores the position and orientation data of the QR obstacle in MuJoCo environment
    aruco_pose_msg_.header.stamp = ros::Time::now();
    aruco_pose_msg_.header.frame_id = "world";

    // translation
    aruco_pose_msg_.pose.position.x = 0.7;
    aruco_pose_msg_.pose.position.y = 0.2 * sin(M_PI * (sim_tick/hz));
    // aruco_pose_msg_.pose.position.y = 0.5;
    aruco_pose_msg_.pose.position.z = 1.5;

    // orientation (Quaternion)
    aruco_pose_msg_.pose.orientation.x = 0.5;
    aruco_pose_msg_.pose.orientation.y = -0.5;
    aruco_pose_msg_.pose.orientation.z = -0.5;
    aruco_pose_msg_.pose.orientation.w = 0.5;

    // tf_broadcaster publishes the transform data
    aruco_pose_pub_.publish(aruco_pose_msg_);
}

//________________________________________________________________________________________________//


// ugly initialization of static members of tracked obstacles
int    TrackedObstacle::s_fade_counter_size_     = 0;
double TrackedObstacle::s_sampling_time_         = 100.0;
double TrackedObstacle::s_process_variance_      = 0.0;
double TrackedObstacle::s_process_rate_variance_ = 0.0;
double TrackedObstacle::s_measurement_variance_  = 0.0;