#pragma once

#include <Eigen/Dense>
#include "tocabi_lib/robot_data.h"
#include "utils.h"     
// #include "cbf_manager/collision_manager.h"     

namespace TOCABI {

class CbfManager
{
public:
    CbfManager(RobotData& rd);
    ~CbfManager();

    // =========================
    // Joint limit CBF
    // =========================
    Eigen::MatrixQQd getJointLimitCbfConstraint(Eigen::VectorQd& lbA, Eigen::VectorQd& ubA);

    void setJointLimitCbfParameters(const double& alpha1,
                                    const double& alpha2,
                                    const double& epsilon);

    void setJointLimitBoundaries(const Eigen::VectorQd& q_pos_lb,
                                 const Eigen::VectorQd& q_pos_ub);

    // =========================
    // Workspace boundary CBF
    // =========================
    Eigen::MatrixXd getWorkspaceBoundaryCbfConstraint(Eigen::VectorXd& lbA, Eigen::VectorXd& ubA) const;
    void setWorkspaceBoundaryCbfParameters(double alpha1, double alpha2, double epsilon);
    void setWorkspaceBoundaryPairs(const std::vector<WorkspaceBoundaryPair>& pairs);
    int getNumWorkspaceBoundaryPairs() const;

    // =========================
    // Self-collision avoidance CBF
    // =========================
    void setSelfCollisionCbfParameters(double alpha1, double alpha2, double epsilon);

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
    RobotData& rd_;
    RigidBodyDynamics::Model model_;  

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

    // --- Self-collision avoidance CBF params & pairs
    double alpha1_self_collision{0.0};
    double alpha2_self_collision{0.0};
    double epsilon_self_collision{1.0};
};

} // namespace TOCABI
