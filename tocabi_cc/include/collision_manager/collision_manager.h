/**
 * @file collision_manager.h
 * @brief CollisionManager class for integrating environment perception with HPP-FCL.
 * 
 * This class handles:
 *  - Updating the robot's collision objects based on joint configurations.
 *  - Representing and updating collision objects of the surrounding environment
 *    detected via camera or sensors.
 *  - Performing collision checks and computing minimum distances and nearest points
 *    between robot links and environmental objects using HPP-FCL.
 *  - Broadcasting and accessing transformation matrices for robot frames
 *    and camera-detected objects using ROS tf2.
 *  - Sending position and orientation of QR code (ArUCo) attached obstacles
 *    to the MuJoCo simulation, allowing real-time update of obstacle positions.
 * 
 * It is intended to be used within the control loop to ensure that
 * collision objects always reflect the current robot and environment state.
 */

#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H

// Must inlude while using Pinocchio in noetic
// to avoid compilation errors from differing Boost-variant sizes.
#include <pinocchio/fwd.hpp>

// ROS Headers
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <std_msgs/UInt8MultiArray.h>           // For collision status of collision objects
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>     // For TF information of camera frame
#include <geometry_msgs/PoseStamped.h>          // For sending obstacle poses to MuJoCo

// Pinocchio Headers
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/parsers/urdf.hpp>

// Collision Libarary(HPP-FCL) Headers
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/collision.h>

#include "math_type_define.h"
#include "tocabi_lib/robot_data.h"
#include "collision_manager/tracked_obstacle.h"

class CollisionManager
{
public:
    CollisionManager(const RobotData &rd);

    void callAvailableQueue();

private:
    // robot data
    const RobotData &rd_;

    // control frequency
    double hz_ = 2000;

    // ROS
    ros::NodeHandle nh_cm_;
    ros::CallbackQueue queue_cm_;
    // publisher
    ros::Publisher col_status_pub_;                         // send collision status of collision objects to MuJoCo
    ros::Publisher base_to_head_pose_pub_;                  // send base_to_head_pose_pub_ to camera
    ros::Publisher aruco_pose_pub_;                         // send aruco_pose_msg to MuJoCo
    // subscriber
    ros::Subscriber aruco_detect_sub_;
    // tf2_ros(coordinate transform communication)
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener{tf_buffer_};
    // msg
    std_msgs::UInt8MultiArray col_status_msg_;              // contain collision status of collision objects
    geometry_msgs::TransformStamped base_to_head_tf_msg_;   // contain transform from base to head
    geometry_msgs::TransformStamped base_to_qr_tf_msg_;     // contain transform from base to qr object
    geometry_msgs::PoseStamped aruco_pose_msg_;             // contain pos.&ori. of the obstacle
    geometry_msgs::PoseStamped base_to_head_pose_msg_;      // contain transform from base to head

    //================================ Pinocchio =================================//
public:
    Eigen::VectorQVQd q_virtual_pin_;

private:
    // pinocchio model of the Robot
    pinocchio::Model model_;
    // pinocchio data of the Robot
    pinocchio::Data data_;

    /**
     * @brief Convert the joint configuration including floating base from RBDL format to Pinocchio format
     * 
     * @param q_virtual_rbdl the joint configuration of the robot + the virtual joints of the floating base in RBDL format
     * 
     * @note - q_virtual format of RBDL : [base pos.(x,y,z), base ori. quat(x,y,z), joint pos.(n), base ori. quat(w)]
     * 
     *       - q_virtual format of Pinocchio : [base pos.(x,y,z), base ori. quat(x,y,z,w), joint pos.(n)]
     */
    void convertQVirtualRBDLtoPin(
        const Eigen::VectorQVQd &q_virtual_rbdl
        );

    //============================ Collision Related =============================//
public:
    /**
     * @brief Struct representing a single collision body,
     *        including its geometry, transform, and link association.
     */
    struct CollisionBody{
        // collision object using HPP-FCL
        std::shared_ptr<hpp::fcl::CollisionObject> obj;

        // name the link where the collision object is attached
        std::string link_name;
        // link id of robot data(rd)
        int link_id;
        // pinocchio parent joint ID of the link
        int joint_id;

        // collision status of each collision object (true: in collision, false: safe(no collision))
        bool in_collision = false; 

        // translation vector
        Eigen::Vector3d trans_local;    // from link frame to collision object frame
        Eigen::Vector3d trans_global;   // from world frame to collision object frame

        // rotation matrix
        Eigen::Matrix3d rot_local;      // from link frame to collision object frame
        Eigen::Matrix3d rot_global;     // from world frame to collision object frame

        // for obstacle
        Eigen::Vector3d pos_base;       // obstacle position w.r.t. base frmae
        Eigen::Vector3d vel_global;     // obstacle velocity w.r.t. world frmae

        // type of collision geometry
        enum class Type
        {
            Sphere,
            Capsule,
            Box
        } type;

        // collision geometry shapes
        std::shared_ptr<hpp::fcl::Sphere>  sphere;    // valid if type == Sphere
        std::shared_ptr<hpp::fcl::Capsule> capsule;   // valid if type == Capsule
        std::shared_ptr<hpp::fcl::Box>     box;       // valid if type == Box
    };

    // enum for the ID of collision objects for the robot links
    enum CollisionObjectIdx
    {
        Left_Pelvis_Col_ID,
        Right_Pelvis_Col_ID,
        Left_Upper_Leg_Col_ID,
        Left_Lower_Leg_Col_ID,
        Left_Inner_Foot_Col_ID,
        Left_Outer_Foot_Col_ID,
        Right_Upper_Leg_Col_ID,
        Right_Lower_Leg_Col_ID,
        Right_Inner_Foot_Col_ID,
        Right_Outer_Foot_Col_ID,
        // Left_Upper_Body_Col_ID,
        // Right_Upper_Body_Col_ID,
        Left_Upper_Arm_Col_ID,
        Left_ForeArm_Col_ID,
        Left_Hand_Col_ID,
        Head_Col_ID,
        Right_Upper_Arm_Col_ID,
        Right_ForeArm_Col_ID,
        Right_Hand_Col_ID,
        Col_Obj_Count  // total number of collision Objects in Robot (used for cb_robot_ vector sizing)
    };

    // options for choosing object pairs in getDistanceResultBetweenObjects()
    enum DistanceResultOption
    {
        Links,
        Link2Env,
        Envs
    };

    // collisionBody vectors of the robot
    std::vector<CollisionBody> cb_robot_;

    // collisionBody vectors of the obstacles in the environment
    std::vector<CollisionBody> cb_obstacles_;

    // the number of collision pairs
    int num_pairs_;                         // self collision
    int num_pairs_w_obs_;                   // between robot and obstacle
    
    // self-collision avoidance Jacobian for all defined collision pairs
    MatrixXd J_self_col_;                   // self collision
    MatrixXd J_obs_col_;                    // between robot and obstacle

    // minimum distance
    VectorXd min_distances_;                // self collision
    VectorXd min_distances_w_obs_;          // between robot and obstacle
    VectorXd obs_vel_projections_;          // obstacle velocity projection to the normal vector

    /**
     * @brief Publishes the self-collision status of all collision objects of the robot
     *        to the MuJoCo simulation
     * 
     * @note This method constructs and sends a `std_msgs::UInt8MultiArray` message
     *       where each element represents the collision status of a corresponding collision object:
     * 
     *       - 0: safe (no collision)
     * 
     *       - 1: in collision
     *
     */
    void pubSelfCollisionStatus();

    /**
     * @brief Update the transformation(TF) matrices of the robot's collision objects(CObjs)
     * 
     * @note This method should be called after updating the robot's model and computing forward kinematics,
     *       so that the collision objects correctly reflect the current joint configuration
     */
    void updateRobotCObjsTF();

    /**
     * @brief Compute the Jacobian-based constraint matrix for self-collision avoidance
     *        and minimum distances between collision pairs
     * 
     * @note This method calculates the Jacobian based on the predefined collision pairs
     *       specified in col_pair_ids_
     */
    void computeSelfColAvoidJac();

    void computeObstacleAvoidJac();

    /**
     * @brief Check for self-collisions among the robot's collision objects
     * 
     * @note This method is used to detect collisions for visualization purposes, particularly to highlight
     *       collisions in red when the self-collision avoidance constraint is not applied.
     */
    void checkSelfCollision();

private:
    // pairs of collision object IDs for self-collision checks
    std::vector<std::pair<CollisionObjectIdx, CollisionObjectIdx>> col_id_pairs_ =
    {
        {Left_Pelvis_Col_ID, Left_Upper_Arm_Col_ID},
        {Left_Pelvis_Col_ID, Left_ForeArm_Col_ID},
        {Left_Pelvis_Col_ID, Left_Hand_Col_ID},
        {Right_Pelvis_Col_ID, Right_Upper_Arm_Col_ID},
        {Right_Pelvis_Col_ID, Right_ForeArm_Col_ID},
        {Right_Pelvis_Col_ID, Right_Hand_Col_ID},
        {Left_Upper_Leg_Col_ID, Right_Upper_Leg_Col_ID},
        {Left_Upper_Leg_Col_ID, Right_Lower_Leg_Col_ID},
        {Left_Upper_Leg_Col_ID, Left_ForeArm_Col_ID},
        {Left_Upper_Leg_Col_ID, Left_Hand_Col_ID},
        {Left_Lower_Leg_Col_ID, Right_Upper_Leg_Col_ID},
        {Left_Lower_Leg_Col_ID, Right_Lower_Leg_Col_ID},
        {Left_Lower_Leg_Col_ID, Right_Inner_Foot_Col_ID},
        {Left_Inner_Foot_Col_ID, Right_Lower_Leg_Col_ID},
        {Left_Inner_Foot_Col_ID, Right_Inner_Foot_Col_ID},
        {Right_Upper_Leg_Col_ID, Right_ForeArm_Col_ID},
        {Right_Upper_Leg_Col_ID, Right_Hand_Col_ID},
        // {Left_Upper_Arm_Col_ID, Right_Hand_Col_ID},
        // {Left_Hand_Col_ID, Right_Upper_Arm_Col_ID},
        {Left_ForeArm_Col_ID, Right_ForeArm_Col_ID},
        {Left_ForeArm_Col_ID, Right_Hand_Col_ID},
        {Left_Hand_Col_ID, Right_ForeArm_Col_ID},
        {Left_Hand_Col_ID, Right_Hand_Col_ID},
        {Head_Col_ID, Left_ForeArm_Col_ID},
        {Head_Col_ID, Left_Hand_Col_ID}
    };

    // collision status flags for each collision object (0: safe, 0>=: in collision)
    std::vector<unsigned int> collision_flags_ = std::vector<unsigned int>(Col_Obj_Count, 0);

    // DistanceRequest configured to compute nearest points (used in getDistanceResultBetweenCObjs)
    hpp::fcl::DistanceRequest request_enable_nearest_point_;

    /**
     * @brief Initialize the CollisionBody struct information of the robot
     * 
     * @note This method sets the predefined values of cgeom_frame_, joint_to_cgeom_trans_,
     *       and joint_to_cgeom_rot_ for each robot link
     */
    void initColBody();

    /**
     * @brief Assign collision objects(CObjs) to the robot links based on the predefined CollisionBody information
     * 
     * @note This method creates HPP-FCL collision objects for each robot link
     *       using the parameters defined in the CollisionBody structs in cb_robot_
     */
    void assignRobotCObjs();

    /**
     * @brief Get the hpp::fcl::DistanceResult between two collision objects(CObjs)
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param option specifies which pair of objects to include in the returned DistanceResult:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * 
     * @return A DistanceResult structure containing the minimum distance and the closest points between the two objects
     * 
     * @note - minimum distance: <Name-of-DistanceResult>.min_distance
     * 
     *       - closest point on object 1: <Name-of-DistanceResult>.nearest_points[0]
     * 
     *       - closest point on object 2: <Name-of-DistanceResult>.nearest_points[1]
     */                                                                
    hpp::fcl::DistanceResult getDistanceResultBetweenCObjs(const CollisionObjectIdx obj_id1,
                                                           const CollisionObjectIdx obj_id2,
                                                           DistanceResultOption option = Links
                                                           );
    
    /**
     * @brief Get the collision result(ColResult) between two sphere collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * 
     * @note The collision result contains the minimum distance between the nearest points
     *       and the corresponding points on each object's surface
     */                                                          
    void ColResultSphere2Sphere(const CollisionObjectIdx obj_id1, 
                                const CollisionObjectIdx obj_id2, 
                                double &min_distance,
                                Eigen::Vector3d &nearest_point1,
                                Eigen::Vector3d &nearest_point2
                                );
    
    void obsColResultSphere2Sphere(const unsigned int obj_id1, 
                                   const unsigned int obj_id2, 
                                   double &min_distance,
                                   Eigen::Vector3d &nearest_point1,
                                   Eigen::Vector3d &nearest_point2
                                   );

    void obsColResultSphere2Capsule(const unsigned int obj_id1, 
                                    const unsigned int obj_id2,
                                    double &min_distance, 
                                    Eigen::Vector3d &nearest_point1,
                                    Eigen::Vector3d &nearest_point2
                                    );

    /**
     * @brief Get the collision result(ColResult) between sphere and capsule collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * 
     * @note The sphere object must be assigned to obj_id1
     *      
     *       The collision result contains the minimum distance between the nearest points
     *       and the corresponding points on each object's surface
     *       
     */                               
    void ColResultSphere2Capsule(const CollisionObjectIdx obj_id1, 
                                 const CollisionObjectIdx obj_id2,
                                 double &min_distance, 
                                 Eigen::Vector3d &nearest_point1,
                                 Eigen::Vector3d &nearest_point2
                                 );
    
    /**
     * @brief Get the collision result(ColResult) between two capsule collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * 
     * @note The collision result contains the minimum distance between the nearest points
     *       and the corresponding points on each object's surface
     */                                
    void ColResultCapsule2Capsule(const CollisionObjectIdx obj_id1, 
                                  const CollisionObjectIdx obj_id2, 
                                  double &min_distance, 
                                  Eigen::Vector3d &nearest_point1, 
                                  Eigen::Vector3d &nearest_point2
                                  );

    /**
     * @brief Compute the Jacobian-based constraint matrix for a single collision pair(ColPair) in the robot
     * 
     * @param obj_id1 index of the first collision object
     * @param center_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param obj_id2 index of the second collision object
     * @param center_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param sign sign of the minimum distance(positive(objects separated): 1, negative(penetration): -1)
     * 
     * @return the Jacobian-based constraint matrix for the single collision pair
     */                                 
    Eigen::MatrixXd computeRobotColPairJac(const CollisionObjectIdx obj_id1,
                                           const Eigen::Vector3d nearest_point1,
                                           const CollisionObjectIdx obj_id2,
                                           const Eigen::Vector3d nearest_point2,
                                           const int sign
                                           );

    
    Eigen::RowVectorXd computeColPairWithObsJac(const unsigned int obj_id,
                                                const Eigen::Vector3d nearest_point_obj,
                                                const unsigned int obs_id,
                                                const Eigen::Vector3d nearest_point_obs,
                                                double &obs_vel_projection,
                                                const int sign
                                                );

    /**
     * @brief Compute the Jacobian-based constraint matrix for a single collision pair(ColPair) using HPP-FCL collision library
     * 
     * @param obj_id1 index of the first collision object
     * @param nearest_point1 the point on object 1 returned by HPP-FCL that is closest to object 2, expressed in the world frame
     * @param obj_id2 index of the second collision object
     * @param nearest_point2 the point on object 2 returned by HPP-FCL that is closest to object 1, expressed in the world frame
     * @param sign sign of the minimum distance(positive(objects separated): 1, negative(penetration): -1)
     * 
     * @return the Jacobian-based constraint matrix for the single collision pair
     */                                     
    Eigen::MatrixXd computeColPairJacHPPFCL(const CollisionObjectIdx obj_id1,
                                            const Eigen::Vector3d nearest_point1,
                                            const CollisionObjectIdx obj_id2,
                                            const Eigen::Vector3d nearest_point2,
                                            const int sign
                                            );

    /**
     * @brief Assign a collision object(CObj) as a sphere shape to the target object
     * 
     * @param obj_rot the rotation matrix of the coordinate of the collision object
     * @param obj_trans the translation of the coordinate of the collision object
     * @param radius the radius of the sphere
     * 
     * @return The assigned collision object as a sphere shape
     */
    std::shared_ptr<hpp::fcl::CollisionObject> assignSphereCObj(const Eigen::Matrix3d obj_rot,
                                                                const Eigen::Vector3d obj_trans,
                                                                const double radius
                                                                );
    
    /**
     * @brief Assign a collision object(CObj) as a capsule shape to the target object
     * 
     * @param obj_rot the rotation matrix of the coordinate of the collision object
     * @param obj_trans the translation of the coordinate of the collision object
     * @param radius the radius of the capsule
     * @param height the height of the capsule
     * 
     * @return The assigned collision object as a capsule shape
     */
    std::shared_ptr<hpp::fcl::CollisionObject> assignCapsuleCObj(const Eigen::Matrix3d obj_rot,
                                                                 const Eigen::Vector3d obj_trans,
                                                                 const double radius,
                                                                 const double height
                                                                 );
    
    /**
     * @brief Assign a collision object(CObj) as a box shape to the target object
     * 
     * @param obj_rot the rotation matrix of the coordinate of the collision object
     * @param obj_trans the translation of the coordinate of the collision object
     * @param size_x the size of the box in x direction
     * @param size_y the size of the box in y direction
     * @param size_z the size of the box in z direction
     * 
     * @return The assigned collision object as a box shape
     */
    std::shared_ptr<hpp::fcl::CollisionObject> assignBoxCObj(const Eigen::Matrix3d obj_rot,
                                                             const Eigen::Vector3d obj_trans,
                                                             const double size_x,
                                                             const double size_y,
                                                             const double size_z
                                                             );

    //____________________________________________________________________________//
                                                             
    //======================== Communication with Camera =========================//
public:
    // transformation matrices
    Eigen::Isometry3d base_to_qr_transform_;    // from base frame to object frame
    Eigen::Isometry3d base_to_head_transform_;  // from base frame to head frame

    // rotation matrix and translation vector from world frame to base frame
    Eigen::Matrix3d world_to_base_rot_yaw_only_;
    Eigen::Vector3d world_to_base_trans_;

    std::mutex meas_mutex_;
    bool has_new_measure_ = false;
    ros::Time last_detect_time_;
    
    double tracking_duration, process_var, process_rate_var, measurement_var;

    /**
     * @brief Publish the transformation matrix from base frame to head frame
     * 
     * @note This method uses ROS to publish the transformation matrix
     */
    void pubBasetoHeadTransform();

    /**
     * @brief Get the transformation matrix from base frame to QR code(ArUCo) frame
     * 
     * @return The transformation matrix from base frame to QR code(ArUCo) frame(object frame)
     * 
     * @note This method uses ROS tf2 to lookup the transformation matrix
     */
    Eigen::Isometry3d getBasetoQRTransform();

    void BasetoQRTransformCallback(const geometry_msgs::PoseStamped &msg);
    
    void updateObstacle();

private:
    std::vector<TrackedObstacle> tracked_obstacles_;

    //____________________________________________________________________________//

    //============================ Obstacle in MuJoCo ============================//
public:
    /**
     * @brief Publish the desired position and orientation of a QR code (ArUCo) obstacle
     *        in the MuJoCo simulation environment.
     * 
     * @param sim_tick the simulation tick at which to apply the new obstacle position.
     * @param hz control frequency
     */
    void pubQRObstaclePose(const int sim_tick, 
                           const double hz);
};

#endif // COLLISION_MANAGER_H