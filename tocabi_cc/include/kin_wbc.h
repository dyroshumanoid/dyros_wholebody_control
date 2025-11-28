// Must inlude while using Pinocchio in noetic
// to avoid compilation errors from differing Boost-variant sizes.
#include <pinocchio/fwd.hpp>

#pragma once

#include <eigen3/Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include "collision_manager/collision_manager.h"
#include "wholebody_functions.h"
#include "utils.h"

class KinWBC {
public:
    KinWBC(RobotData& rd, CollisionManager& col_mgr);

    void computeTaskSpaceKinematicWBC();
    void setTaskHierarchy(const TaskMotionType& motion_mode_);
    void setInitialConfiguration(const Eigen::VectorQd &q_init_des_);
    
private:
    RobotData &rd_;

    CollisionManager &col_mgr_;

    std::vector<std::vector<TaskInfo>> task_hierarchy;
    TaskMotionType motion_mode;

    bool is_cannot_solve_qp_ = true;

    Eigen::VectorVQd safetyFilter();
    CQuadraticProgram QP_safety_filter;

        void calcCostHess();
        void calcCostGrad();
        void calcEqualityConstraint();
        void calcInequalityConstraint();
        void checkGradHessSize();
    
    void getReachabilityConstraints(const std::vector<Eigen::MatrixXd> &J_reachability_, const std::vector<double> &h_reachability_);
    std::vector<Eigen::MatrixXd> grad_reachability_;       
    std::vector<double>          cbf_reachability_;    

    Eigen::MatrixXd Hess; 
    Eigen::VectorXd grad;
    Eigen::MatrixXd A_const;   
    Eigen::VectorXd lbA_const;
    Eigen::VectorXd ubA_const;
    std::vector<ConstraintMatrix> constraints_;
    int total_num_state = 0;
    int total_num_constraints = 0;

    Eigen::VectorVQd qdot_des;
    Eigen::VectorVQd qdot_safety;

    Eigen::VectorQd q_init_des;

};