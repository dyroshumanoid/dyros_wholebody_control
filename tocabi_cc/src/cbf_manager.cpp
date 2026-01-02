#include "cbf_manager/cbf_manager.h"

using namespace TOCABI;

CbfManager::CbfManager(RobotData &rd) : rd_(rd){}
CbfManager::~CbfManager(){}

// =========================
// Joint limit CBF
// =========================
Eigen::MatrixQQd CbfManager::getJointLimitCbfConstraint(Eigen::VectorQd &lbA, Eigen::VectorQd &ubA)
{
    // Gradient
    Eigen::MatrixQQd A; A.setIdentity();    

    // Lower and upper bounds
    lbA = (-1.0) * (alpha1_joint_limit + alpha2_joint_limit) * rd_.q_dot_ 
        + (alpha1_joint_limit * alpha2_joint_limit) * (q_pos_lb - rd_.q_) 
        + (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();

    ubA = (-1.0) * (alpha1_joint_limit + alpha2_joint_limit) * rd_.q_dot_ 
        + (alpha1_joint_limit * alpha2_joint_limit) * (q_pos_ub - rd_.q_) 
        - (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();

    return A;
}

void CbfManager::setJointLimitCbfParameters(const double& alpha1, const double& alpha2, const double& epsilon)
{
    alpha1_joint_limit = alpha1;
    alpha2_joint_limit = alpha2;
    epsilon_joint_limit = epsilon;
}

void CbfManager::setJointLimitBoundaries(const Eigen::VectorQd& q_pos_lb_, const Eigen::VectorQd& q_pos_ub_)
{
    q_pos_lb = q_pos_lb_;
    q_pos_ub = q_pos_ub_;
}

// =========================
// Workspace Boundary CBF
// =========================
void CbfManager::setWorkspaceBoundaryCbfParameters(double alpha1, double alpha2, double epsilon)
{
    alpha1_workspace  = alpha1;
    alpha2_workspace  = alpha2;
    epsilon_workspace = epsilon;
}

void CbfManager::setWorkspaceBoundaryPairs(const std::vector<WorkspaceBoundaryPair>& pairs)
{
    workspace_boundary_pairs_ = pairs;
}

Eigen::MatrixXd CbfManager::getWorkspaceBoundaryCbfConstraint(Eigen::VectorXd& lbA, Eigen::VectorXd& ubA) const
{
    const int m = static_cast<int>(workspace_boundary_pairs_.size());

    Eigen::MatrixXd A;
    A.setZero(m, MODEL_DOF_VIRTUAL);

    lbA.setZero(m);
    ubA.setZero(m);

    for (int i = 0; i < m; ++i)
    {
        const auto& pr = workspace_boundary_pairs_[i];

        const LinkData& linkA = rd_.link_[pr.idx_A];
        const LinkData& linkB = rd_.link_[pr.idx_B];

        Eigen::MatrixXd J_AB, Jqdot_AB;
        const double dist_AB = getSignedDistanceFunction(linkA, linkB, J_AB, Jqdot_AB);

        A.block(i, 0, 1, MODEL_DOF_VIRTUAL) = -J_AB;

        const double Jdotqdot = Jqdot_AB(0, 0);
        const double Jqdot    = (J_AB * rd_.local_q_dot_virtual_)(0, 0);

        const double h = - Jdotqdot
                         - (alpha1_workspace + alpha2_workspace) * Jqdot
                         + (alpha1_workspace * alpha2_workspace) * (pr.max_dist - dist_AB)
                         - (1.0 / epsilon_workspace) * (J_AB * J_AB.transpose())(0,0);


        lbA(i) = (-1.0) * h;
        ubA(i) = 1e5;
    }

    return A;
}

int CbfManager::getNumWorkspaceBoundaryPairs() const
{
    return static_cast<int>(workspace_boundary_pairs_.size());
}

// ===============================
// Self-collision constraints CBF
// ===============================
void CbfManager::setSelfCollisionCbfParameters(double alpha1, double alpha2, double epsilon)
{
    alpha1_self_collision  = alpha1;
    alpha2_self_collision  = alpha2;
    epsilon_self_collision = epsilon;
}

// Eigen::MatrixXd CbfManager::getSelfCollisionCbfConstraint(Eigen::VectorXd& lbA, Eigen::VectorXd& ubA) const
// {
//     static CollisionManager col_mgr_(rd_); 
    // col_mgr_.updateRobotCObjsTF();
//     col_mgr_.computeSelfColAvoidJac();

//     const int m = col_mgr_.num_pairs_;
//     Eigen::MatrixXd A;
//     A.setZero(m, MODEL_DOF_VIRTUAL);

//     lbA.setZero(m);
//     ubA.setZero(m);

//     A = col_mgr.J_self_col_;

//     lbA = - (alpha1_workspace + alpha2_workspace) * (A * rd_.local_q_dot_virtual_) 
//           + (alpha1_workspace * alpha2_workspace) * (col_mgr_.min_distances_) 
//           - (1.0 / epsilon_workspace) * (A * A.transpose());
//     ubA.setConstant(1e5);

//     return A;
// }

// ===============================
// Utils
// ===============================
double CbfManager::getSignedDistanceFunction(const LinkData& linkA_,
                                             const LinkData& linkB_,
                                             Eigen::MatrixXd& J_AB,
                                             Eigen::MatrixXd& Jqdot_AB) const
{
    // Initialization
    double sd_AB = 0.0;

    sd_AB = (linkA_.local_xpos - linkB_.local_xpos).norm(); 

    Eigen::Vector3d normal_vector_btw_AB; normal_vector_btw_AB.setZero();
    normal_vector_btw_AB = (linkA_.local_xpos - linkB_.local_xpos) / (linkA_.local_xpos - linkB_.local_xpos).norm();

    J_AB.setZero(1, MODEL_DOF_VIRTUAL);
    J_AB = normal_vector_btw_AB.transpose() * (linkA_.local_Jac_v - linkB_.local_Jac_v);

    Jqdot_AB.setZero(1, 1);
    Jqdot_AB = normal_vector_btw_AB.transpose() * (linkA_.local_Jqdot.head(3) - linkB_.local_Jqdot.head(3));

    return sd_AB;
}