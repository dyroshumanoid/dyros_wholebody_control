#pragma once

#include <Eigen/Dense>
#include "tocabi_lib/robot_data.h"
#include "utils.h"     
#include "cbf_manager/collision_manager.h"     

namespace TOCABI {

class CbfManager
{
public:
    CbfManager(RobotData& rd, RigidBodyDynamics::Model& model);
    ~CbfManager();

    void callAvailableQueue();

    void update();
    
    void pubDataFromSlowToFast();
    void subDataFromSlowToFast();

private:
    RobotData& rd_;
    RigidBodyDynamics::Model& model_;

public:
    CollisionManager col_mgr_;

    // =========================
    // Joint limit CBF
    // =========================
    void setJointLimitCbfParameters(const double& alpha1,
                                    const double& alpha2,
                                    const double& epsilon);
    void setJointLimitBoundaries(const Eigen::VectorQd& q_pos_lb,
                                 const Eigen::VectorQd& q_pos_ub);
    void computeJointLimitCbfConstraint();
    void getJointLimitCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A,
                                    Eigen::VectorQd &lbA, 
                                    Eigen::VectorQd &ubA) const;

    // =========================
    // Workspace boundary CBF
    // =========================
    void setWorkspaceBoundaryCbfParameters(const double& alpha1,
                                           const double& alpha2,
                                           const double& epsilon);
    void setWorkspaceBoundaryPairs(const std::vector<WorkspaceBoundaryPair>& pairs);
    void computeWorkspaceBoundaryCbfConstraint();
    void getWorkspaceBoundaryCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A,
                                           Eigen::VectorXd &lbA,
                                           Eigen::VectorXd &ubA) const;
    int getNumWorkspaceBoundaryPairs() const;

    // =========================
    // Self-collision avoidance CBF
    // =========================
    void setSelfCollisionCbfParameters(const double &alpha1, 
                                       const double &alpha2, 
                                       const double &epsilon);
    void computeSelfCollisionCbfConstraint();
    void getSelfCollisionCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A,
                                       Eigen::VectorXd &lbA,
                                       Eigen::VectorXd &ubA) const;
    int getNumSelfCollisionPairs() const;

    // =========================
    // Obstacle avoidance CBF
    // =========================
    void setObstacleAvoidanceCbfParameters(const double &alpha1, 
                                           const double &alpha2, 
                                           const double &epsilon);
    void computeObstacleAvoidanceCbfConstraint();
    void getObstacleAvoidanceCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A,
                                           Eigen::VectorXd &lbA,
                                           Eigen::VectorXd &ubA) const;
    int getNumObstacleAvoidancePairs() const;

private:
    // Signed distance function between two links (point-to-point distance)
    // Outputs:
    //   - dist_AB: ||pA - pB||
    //   - J_AB:    1 x MODEL_DOF_VIRTUAL, derivative of dist wrt qdot_virtual (via linear jacobians)
    //   - Jqdot_AB:1 x 1, time derivative term (normal^T * (Jdot*qdot)_diff)
    double getSignedDistanceFunction(const LinkData& linkA_,
                                     const LinkData& linkB_,
                                     Eigen::MatrixXd& J_AB,
                                     Eigen::MatrixXd& Jqdot_AB) const;

private:
    // --- Joint-limit CBF params & bounds
    double alpha1_joint_limit{0.0};
    double alpha2_joint_limit{0.0};
    double epsilon_joint_limit{1.0};

    Eigen::VectorQd q_pos_lb;
    Eigen::VectorQd q_pos_ub;

    // --- Workspace boundary CBF params & pairs
    double alpha1_workspace{0.0};
    double alpha2_workspace{0.0};
    double epsilon_workspace{1.0};

    std::vector<WorkspaceBoundaryPair> workspace_boundary_pairs_;

    // --- Self-collision avoidance CBF params
    double alpha1_self_collision{0.0};
    double alpha2_self_collision{0.0};
    double epsilon_self_collision{1.0};

    // --- Obstacle avoidance CBF params
    double alpha1_obstacle_avoidance{0.0};
    double alpha2_obstacle_avoidance{0.0};
    double epsilon_obstacle_avoidance{1.0};

    // --- Data computed in computeSlow() loop (2 kHz)
    ConstraintMatrix joint_limit_constraints;
    ConstraintMatrix workspace_boundary_constraints;
    ConstraintMatrix self_collision_avoidance_constraints;
    ConstraintMatrix obstacle_avoidance_constraints;

    // --- Storage for computeSlow() loop
    //     Used to transfer data to computeFast() loop
    ConstraintMatrix joint_limit_constraints_container;
    ConstraintMatrix workspace_boundary_constraints_container;
    ConstraintMatrix self_collision_avoidance_constraints_container;
    ConstraintMatrix obstacle_avoidance_constraints_container;

    // --- Data consumed by the computeFast() loop
    ConstraintMatrix joint_limit_constraints_fast;
    ConstraintMatrix workspace_boundary_constraints_fast;
    ConstraintMatrix self_collision_avoidance_constraints_fast;
    ConstraintMatrix obstacle_avoidance_constraints_fast;

};

} // namespace TOCABI
