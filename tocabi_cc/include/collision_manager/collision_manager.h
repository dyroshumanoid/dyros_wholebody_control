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
    ros::Subscriber aruco_detect_sub_;                      // receive the pose of the QR code(ArUCo) obstacle
    // msg
    std_msgs::UInt8MultiArray col_status_msg_;              // contain collision status of collision objects
    geometry_msgs::TransformStamped base_to_head_tf_msg_;   // contain transform from base to head
    geometry_msgs::TransformStamped base_to_qr_tf_msg_;     // contain transform from base to qr object
    geometry_msgs::PoseStamped aruco_pose_msg_;             // contain pos.&ori. of the obstacle
    geometry_msgs::PoseStamped base_to_head_pose_msg_;      // contain transform from base to head

    //================================ Pinocchio =================================//
public:
    // pinocchio joint configuration including floating base
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
    // collision geometry shapes
    // sphere
    struct Sphere{
        double radius;

        Sphere() = default;
        Sphere(double r) : radius(r) {}
    };

    // capsule
    struct Capsule{
        double radius;
        double half_length;

        Capsule() = default;
        Capsule(double r, double lz) : radius(r), half_length(lz / 2) {}
    };

    // box
    struct Box{
        Vector3d half_side;

        Box() = default;
        Box(double x, double y, double z) : half_side(x / 2, y / 2, z / 2) {}
    };

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
        Eigen::Vector3d xpos_link;       // from link frame to collision object frame
        Eigen::Vector3d xpos_local;      // from floating base to collision object frame (used for obstacle)
        Eigen::Vector3d xpos;            // from world frame to collision object frame

        // velocity vector
        Eigen::Vector3d v;               // velocity of collision object in world frame

        // rotation matrix
        Eigen::Matrix3d rotm_link;       // from link frame to collision object frame
        Eigen::Matrix3d rotm;            // from world frame to collision object frame

        // type of collision geometry
        enum class Type
        {
            Sphere,
            Capsule,
            Box
        } type;

        // collision geometry shapes
        std::shared_ptr<Sphere> sphere;     // valid if type == Sphere
        std::shared_ptr<Capsule> capsule;   // valid if type == Capsule
        std::shared_ptr<Box> box;           // valid if type == Box
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

    // obstacle 인식 안됐을 때
    bool check1 = false, check2 = false, check3 = false, check4 = false;
    // obstacle 인식 됐을 때
    bool check5 = false, check6 = false;

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
     * @note This method calculates the constraint matrix for self-collision avoidance
     *       based on the predefined collision pairs specified in col_pair_ids_
     */
    void computeSelfColAvoidJac();

    /**
     * @brief Compute the Jacobian-based constraint matrix for obstacle avoidance
     *        , minimum distances between collision pairs, and obstacle velocity projections
     * 
     * @note This method calculates the constraint matrix for obstacle avoidance
     *       between the robot's collision objects and the environment obstacles tracked in cb_obstacles_
     */
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
                                                           DistanceResultOption option = Links);
    
    /**
     * @brief Get the collision result(ColResult) between two sphere collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param option  specifies which pair of objects to include in the collision result:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * 
     * @note - The collision result contains the minimum distance between the nearest points
     *         and the corresponding points on each object's surface
     * 
     *       - When the option is set to Link2Env, obj_id1 must correspond to a robot link,
     *         and obj_id2 must correspond to an environment obstacle.
     */                                                          
    void ColResultSphere2Sphere(const unsigned int obj_id1, 
                                const unsigned int obj_id2, 
                                double &min_distance,
                                Eigen::Vector3d &nearest_point1,
                                Eigen::Vector3d &nearest_point2,
                                DistanceResultOption option = Links);

    /**
     * @brief Get the collision result(ColResult) between sphere and capsule collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param option  specifies which pair of objects to include in the collision result:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * @param obs_is_sphere indicates whether the obstacle is a sphere (true) or capsule (false)
     * 
     * @note - The sphere object must be assigned to obj_id1
     *      
     *       - The collision result contains the minimum distance between the nearest points
     *         and the corresponding points on each object's surface
     *       
     */                               
    void ColResultSphere2Capsule(const unsigned int obj_id1, 
                                 const unsigned int obj_id2,
                                 double &min_distance, 
                                 Eigen::Vector3d &nearest_point1,
                                 Eigen::Vector3d &nearest_point2,
                                 DistanceResultOption option = Links,
                                 bool obs_is_sphere = true);
    
    /**
     * @brief Get the collision result(ColResult) between two capsule collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param option  specifies which pair of objects to include in the collision result:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * 
     * @note - The collision result contains the minimum distance between the nearest points
     *         and the corresponding points on each object's surface
     * 
     *      - When the option is set to Link2Env, obj_id1 must correspond to a robot link,
     *        and obj_id2 must correspond to an environment obstacle.
     */                                
    void ColResultCapsule2Capsule(const unsigned int obj_id1, 
                                  const unsigned int obj_id2, 
                                  double &min_distance, 
                                  Eigen::Vector3d &nearest_point1, 
                                  Eigen::Vector3d &nearest_point2,
                                  DistanceResultOption option = Links);

    /**
     * @brief Compute the Jacobian-based constraint row for a single collision pair in the robot
     * 
     * @param obj_id1 index of the first collision object
     * @param center_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param obj_id2 index of the second collision object
     * @param center_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param sign sign of the minimum distance(positive(objects separated): 1, negative(penetration): -1)
     * 
     * @return the Jacobian-based constraint row for the single collision pair
     */                                 
    Eigen::RowVectorXd computeSelfColAvoidJacRow(const unsigned int obj_id1,
                                                 const Eigen::Vector3d nearest_point1,
                                                 const unsigned int obj_id2,
                                                 const Eigen::Vector3d nearest_point2,
                                                 const int sign);
    
    /**
     * @brief Compute the Jacobian-based constraint row for a single collision pair between the robot and an obstacle
     * 
     * @param obj_id index of the collision object in the robot
     * @param nearest_point_obj the point on the robot object that is closest to the obstacle, expressed in the world frame
     * @param obs_id index of the collision object in the obstacle
     * @param nearest_point_obs the point on the obstacle that is closest to the robot object, expressed in the world frame
     * @param obs_vel_projection projection of the obstacle velocity onto the normal vector between the two nearest points
     * @param sign sign of the minimum distance(positive(objects separated): 1, negative(penetration): -1)
     * 
     * @return the Jacobian-based constraint row for the single collision pair
     */
    Eigen::RowVectorXd computeObstacleAvoidJacRow(const unsigned int obj_id,
                                                  const Eigen::Vector3d nearest_point_obj,
                                                  const unsigned int obs_id,
                                                  const Eigen::Vector3d nearest_point_obs,
                                                  double &obs_vel_projection,
                                                  const int sign);

    /**
     * @brief Compute the Jacobian-based constraint row for a single collision pair using HPP-FCL collision library
     * 
     * @param obj_id1 index of the first collision object
     * @param nearest_point1 the point on object 1 returned by HPP-FCL that is closest to object 2, expressed in the world frame
     * @param obj_id2 index of the second collision object
     * @param nearest_point2 the point on object 2 returned by HPP-FCL that is closest to object 1, expressed in the world frame
     * @param sign sign of the minimum distance(positive(objects separated): 1, negative(penetration): -1)
     * 
     * @return the Jacobian-based constraint row for the single collision pair
     */                                     
    Eigen::RowVectorXd computeSelfColAvoidJacRowHPPFCL(const CollisionObjectIdx obj_id1,
                                            const Eigen::Vector3d nearest_point1,
                                            const CollisionObjectIdx obj_id2,
                                            const Eigen::Vector3d nearest_point2,
                                            const int sign);

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
                                                                const double radius);
    
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
                                                                 const double height);
    
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
                                                             const double size_z);

    //____________________________________________________________________________//
                                                             
    //======================== Communication with Camera =========================//
public:
    // transformation matrices
    Eigen::Isometry3d base_to_qr_transform_;    // from base frame to object frame
    Eigen::Isometry3d base_to_head_transform_;  // from base frame to head frame

    // rotation matrix and translation vector from world frame to base frame
    Eigen::Matrix3d world_to_base_rot_yaw_only_;
    Eigen::Vector3d world_to_base_trans_;
    
    // mutex for measurement update
    std::mutex meas_mutex_;

    // flag to indicate new measurement arrival
    bool has_new_measure_ = false;
    
    // Kalman filter parameters
    double tracking_duration;                   // duration for tracking
    double process_var;                         // process variance
    double process_rate_var;                    // process rate variance (time derivative of process)
    double measurement_var;                     // measurement variance

    /**
     * @brief Publish the transformation matrix from base frame to head frame
     * 
     * @note This method uses ROS to publish the transformation matrix
     */
    void pubBasetoHeadTransform();

    /**
     * @brief Callback function to process the received pose of the QR code(ArUCo) obstacle
     * 
     * @param msg the received PoseStamped message containing the pose of the QR code(ArUCo) obstacle
     */
    void BasetoQRTransformCallback(const geometry_msgs::PoseStamped &msg);
    
    /**
     * @brief Update the obstacle information based on the latest measurement
     * 
     * @note This method updates the position and velocity of the tracked obstacles
     *       using a Kalman filter
     */
    void updateObstacle();

private:
    // vector of tracked obstacles
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