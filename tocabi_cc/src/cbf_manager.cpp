#include "cbf_manager/cbf_manager.h"

using namespace TOCABI;

CbfManager::CbfManager(RobotData &rd, RigidBodyDynamics::Model &model) : rd_(rd), model_(model), col_mgr_(rd_, model_) {}
CbfManager::~CbfManager(){}

void CbfManager::callAvailableQueue()
{
    col_mgr_.callAvailableQueue();
}

void CbfManager::update()
{
    col_mgr_.updateObstacle();
    col_mgr_.updateRobotCollisionObjectsPose();

    col_mgr_.computeSelfColAvoidConstraintTerms();
    col_mgr_.computeObstacleAvoidConstraintTerms();

    computeJointLimitCbfConstraint();
    computeWorkspaceBoundaryCbfConstraint();
    computeSelfCollisionCbfConstraint();
    computeObstacleAvoidanceCbfConstraint();
}

void CbfManager::pubDataFromSlowToFast()
{
    joint_limit_constraints_container              = joint_limit_constraints;
    workspace_boundary_constraints_container       = workspace_boundary_constraints;
    self_collision_avoidance_constraints_container = self_collision_avoidance_constraints;
    obstacle_avoidance_constraints_container       = obstacle_avoidance_constraints;
}

void CbfManager::subDataFromSlowToFast()
{
    joint_limit_constraints_fast              = joint_limit_constraints_container;
    workspace_boundary_constraints_fast       = workspace_boundary_constraints_container;
    self_collision_avoidance_constraints_fast = self_collision_avoidance_constraints_container;
    obstacle_avoidance_constraints_fast       = obstacle_avoidance_constraints_container;
}

// =========================
// Joint limit CBF
// =========================
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

void CbfManager::computeJointLimitCbfConstraint()
{
    // Gradient
    joint_limit_constraints.A.setIdentity(MODEL_DOF, MODEL_DOF);

    // Lower and upper bounds
    joint_limit_constraints.lbA = (-1.0) * (alpha1_joint_limit + alpha2_joint_limit) * rd_.q_dot_ 
                                + (alpha1_joint_limit * alpha2_joint_limit) * (q_pos_lb - rd_.q_) 
                                + (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();
    
    joint_limit_constraints.ubA = (-1.0) * (alpha1_joint_limit + alpha2_joint_limit) * rd_.q_dot_ 
                                + (alpha1_joint_limit * alpha2_joint_limit) * (q_pos_ub - rd_.q_) 
                                - (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();
}

void CbfManager::getJointLimitCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorQd &lbA, Eigen::VectorQd &ubA) const
{
    // Gradient
    A = joint_limit_constraints_fast.A;

    // Lower and upper bounds
    lbA = joint_limit_constraints_fast.lbA;
    ubA = joint_limit_constraints_fast.ubA;
}

// =========================
// Workspace Boundary CBF
// =========================
void CbfManager::setWorkspaceBoundaryCbfParameters(const double &alpha1, const double &alpha2, const double &epsilon)
{
    alpha1_workspace  = alpha1;
    alpha2_workspace  = alpha2;
    epsilon_workspace = epsilon;
}

void CbfManager::setWorkspaceBoundaryPairs(const std::vector<WorkspaceBoundaryPair>& pairs)
{
    workspace_boundary_pairs_ = pairs;
}

void CbfManager::computeWorkspaceBoundaryCbfConstraint()
{
    const int m = static_cast<int>(workspace_boundary_pairs_.size());

    // Gradient
    workspace_boundary_constraints.A.setZero(m, MODEL_DOF_VIRTUAL);

    // Lower and upper bounds
    workspace_boundary_constraints.lbA.setZero(m);
    workspace_boundary_constraints.ubA.setZero(m);

    for (int i = 0; i < m; ++i)
    {
        const auto& pr = workspace_boundary_pairs_[i];

        const LinkData& linkA = rd_.link_[pr.idx_A];
        const LinkData& linkB = rd_.link_[pr.idx_B];

        Eigen::MatrixXd J_AB, Jqdot_AB;
        const double dist_AB = getSignedDistanceFunction(linkA, linkB, J_AB, Jqdot_AB);

        workspace_boundary_constraints.A.block(i, 0, 1, MODEL_DOF_VIRTUAL) = -J_AB;

        const double Jdotqdot = Jqdot_AB(0, 0);
        const double Jqdot    = (J_AB * rd_.local_q_dot_virtual_)(0, 0);

        const double h = - Jdotqdot
                         - (alpha1_workspace + alpha2_workspace) * Jqdot
                         + (alpha1_workspace * alpha2_workspace) * (pr.max_dist - dist_AB)
                         - (1.0 / epsilon_workspace) * (J_AB * J_AB.transpose())(0,0);

        workspace_boundary_constraints.lbA(i) = (-1.0) * h;
        workspace_boundary_constraints.ubA(i) = 1e5;
    }
}

void CbfManager::getWorkspaceBoundaryCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorXd &lbA, Eigen::VectorXd &ubA) const
{
    // Gradient
    A = workspace_boundary_constraints_fast.A;

    // Lower and upper bounds
    lbA = workspace_boundary_constraints_fast.lbA;
    ubA = workspace_boundary_constraints_fast.ubA;
}

int CbfManager::getNumWorkspaceBoundaryPairs() const
{
    return static_cast<int>(workspace_boundary_pairs_.size());
}

// ===============================
// Self-collision constraints CBF
// ===============================
void CbfManager::setSelfCollisionCbfParameters(const double &alpha1, const double &alpha2, const double &epsilon)
{
    alpha1_self_collision  = alpha1;
    alpha2_self_collision  = alpha2;
    epsilon_self_collision = epsilon;
}

void CbfManager::computeSelfCollisionCbfConstraint()
{
    // Gradient
    self_collision_avoidance_constraints.A.setZero(col_mgr_.self_collision_avoid_terms_.num_pairs, MODEL_DOF_VIRTUAL);

    self_collision_avoidance_constraints.A = col_mgr_.self_collision_avoid_terms_.J;

    self_collision_avoidance_constraints.lbA.setZero(col_mgr_.self_collision_avoid_terms_.num_pairs);
    self_collision_avoidance_constraints.ubA.setZero(col_mgr_.self_collision_avoid_terms_.num_pairs);

    // Lower and upper bounds
    self_collision_avoidance_constraints.lbA = - col_mgr_.self_collision_avoid_terms_.Jdotqdot
                                               - (alpha1_self_collision + alpha2_self_collision) * (self_collision_avoidance_constraints.A * rd_.local_q_dot_virtual_) 
                                               - (alpha1_self_collision * alpha2_self_collision) * (col_mgr_.self_collision_avoid_terms_.min_distances) 
                                               + (1.0 / epsilon_self_collision) * self_collision_avoidance_constraints.A.rowwise().squaredNorm();

    self_collision_avoidance_constraints.ubA.setConstant(1e5);
}

void CbfManager::getSelfCollisionCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorXd &lbA, Eigen::VectorXd &ubA) const
{
    // Gradient
    A = self_collision_avoidance_constraints_fast.A;

    // Lower and upper bounds
    lbA = self_collision_avoidance_constraints_fast.lbA;
    ubA = self_collision_avoidance_constraints_fast.ubA;
}

int CbfManager::getNumSelfCollisionPairs() const
{
    return col_mgr_.self_collision_avoid_terms_.num_pairs;
}

// ===============================
// Obstacle avoidance constraints CBF
// ===============================
void CbfManager::setObstacleAvoidanceCbfParameters(const double &alpha1, const double &alpha2, const double &epsilon)
{
    alpha1_obstacle_avoidance  = alpha1;
    alpha2_obstacle_avoidance  = alpha2;
    epsilon_obstacle_avoidance = epsilon;
}

void CbfManager::computeObstacleAvoidanceCbfConstraint()
{
    // Gradient
    obstacle_avoidance_constraints.A.setZero(col_mgr_.obstacle_avoid_terms_.num_pairs, MODEL_DOF_VIRTUAL);

    obstacle_avoidance_constraints.A = col_mgr_.obstacle_avoid_terms_.J;

    // Lower and upper bounds
    obstacle_avoidance_constraints.lbA.setZero(col_mgr_.obstacle_avoid_terms_.num_pairs);
    obstacle_avoidance_constraints.ubA.setZero(col_mgr_.obstacle_avoid_terms_.num_pairs);

    obstacle_avoidance_constraints.lbA = - col_mgr_.obstacle_avoid_terms_.Jdotqdot
                                         - (alpha1_obstacle_avoidance + alpha2_obstacle_avoidance) * 
                                           (obstacle_avoidance_constraints.A * rd_.local_q_dot_virtual_ - col_mgr_.obstacle_avoid_terms_.obs_vel_projections)
                                         - (alpha1_obstacle_avoidance * alpha2_obstacle_avoidance) * (col_mgr_.obstacle_avoid_terms_.min_distances)
                                         + (1.0 / epsilon_obstacle_avoidance) * obstacle_avoidance_constraints.A.rowwise().squaredNorm();

    obstacle_avoidance_constraints.ubA.setConstant(1e5);
}

void CbfManager::getObstacleAvoidanceCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorXd &lbA, Eigen::VectorXd &ubA) const
{
    // Gradient
    A = obstacle_avoidance_constraints_fast.A;

    // Lower and upper bounds
    lbA = obstacle_avoidance_constraints_fast.lbA;
    ubA = obstacle_avoidance_constraints_fast.ubA;
}

int CbfManager::getNumObstacleAvoidancePairs() const
{
    return col_mgr_.obstacle_avoid_terms_.num_pairs;
}

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
    Jqdot_AB = normal_vector_btw_AB.transpose() * (linkA_.local_Jdotqdot.head(3) - linkB_.local_Jdotqdot.head(3));

    return sd_AB;
}