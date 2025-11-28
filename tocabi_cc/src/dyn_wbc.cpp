// dyn_wbc.cpp
#include "dyn_wbc.h"
#include <iostream>

using namespace Eigen;
using namespace qpOASES;

DynWBC::DynWBC(RobotData& rd) : rd_(rd) { }

Eigen::VectorQd DynWBC::computeDynamicWBC()
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    local_LF_contact = rd_.ee_[0].contact;
    local_RF_contact = rd_.ee_[1].contact;

    local_LF_contact = true;    // TODO
    local_RF_contact = true;    // TODO

    updateContactState();
    updateRobotStates();

    constraints_.clear();
    calcCostHess();
    calcCostGrad();
    calcEqualityConstraint();
    calcInequalityConstraint();

    if (contact_dim_prev != contact_dim)
    {
        is_wbc_init_ = true;
        is_gradhess_init_ = true;
        std::cout << "[CONTACT TRIGGER] QP-based WBC formulation has been updated." << std::endl;
    }

    total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();

    if(is_wbc_init_ == true)
    {
        total_num_constraints = 0;
        for (const auto& c : constraints_) {total_num_constraints += c.A.rows();}

        //--- Initialization
        QP_Dyn_Wbc.InitializeProblemSize(total_num_state, total_num_constraints);

        A_const   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_const = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_const = Eigen::VectorXd::Zero(total_num_constraints);

        torque_prev.setZero();

        is_wbc_init_ = false;
    }

    //--- Stack Constraints
    int row_idx  = 0;
    for (const auto& c : constraints_) {
        int rows = c.A.rows();
        A_const.block(row_idx, 0, rows, total_num_state) = c.A;
        lbA_const.segment(row_idx , rows)    = c.lbA;
        ubA_const.segment(row_idx , rows)    = c.ubA;
        row_idx  += rows;
    }

    checkGradHessSize();

    QP_Dyn_Wbc.EnableEqualityCondition(1e-8);
    QP_Dyn_Wbc.UpdateMinProblem(Hess, grad);
    QP_Dyn_Wbc.DeleteSubjectToAx();
    QP_Dyn_Wbc.UpdateSubjectToAx(A_const, lbA_const, ubA_const);

    bool qp_status = true;
    Eigen::VectorXd X_; X_.setZero(total_num_state);
    if(QP_Dyn_Wbc.SolveQPoases(500, X_, true))
    {
        contact_wrench_qp  = X_.segment(0, contact_dim);
        qddot_qp           = X_.segment(contact_dim, MODEL_DOF_VIRTUAL);
        torque_qp          = X_.segment(contact_dim + MODEL_DOF_VIRTUAL, MODEL_DOF);
        qp_status = true;
    }
    else
    {
        //--- CONSTRAINTS VIOLATION CHECKER
        if(is_cannot_solve_qp_ == true)   
        {
            Eigen::VectorXd Ax = A_const * X_; 

            for (int i = 0; i < A_const.rows(); ++i)
            {
                double val = Ax(i);
                double l = lbA_const(i);
                double u = ubA_const(i);

                double eps = 0.0;

                if (val < l - eps)
                {
                    std::cerr << "[Constraint Violation] Row " << i << ": " << val << " < lbA = " << l << std::endl;
                }
                else if (val > u + eps)
                {
                    std::cerr << "[Constraint Violation] Row " << i << ": " << val << " > ubA = " << u << std::endl;
                }
            }
            is_cannot_solve_qp_ = false;
        }

        std::cout << "Dyn WBC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        qp_status = false;
    }

    torque_prev = torque_qp;

    return(torque_qp);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
}

void DynWBC::updateControlCommands(const Eigen::Vector12d& contact_wrench_cmd_, const Eigen::VectorVQd &qddot_cmd_)
{
    contact_wrench_cmd = contact_wrench_cmd_;
    qddot_cmd = qddot_cmd_;
}

/////////////////////////////////////////////
//--- Quadratic Programming Formulation ---//
/////////////////////////////////////////////
void DynWBC::calcCostHess()
{
    Hess.setZero(contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
    
    unsigned int start_idx = 0;
    Hess.block(start_idx, start_idx, contact_dim, contact_dim) = W_cwr * Eigen::MatrixXd::Identity(contact_dim, contact_dim);
    start_idx += contact_dim; 
    
    Hess.block(start_idx, start_idx, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = W_qddot * Eigen::MatrixVVd::Identity() + W_energy * M;
    start_idx += MODEL_DOF_VIRTUAL; 

    Hess.block(start_idx, start_idx, MODEL_DOF, MODEL_DOF) = W_torque * Eigen::MatrixQQd::Identity();
    start_idx += MODEL_DOF; 
}

void DynWBC::calcCostGrad()
{
    grad.setZero(contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
    unsigned int start_idx = 0;

    grad.segment(start_idx, contact_dim) -= W_cwr * contact_wrench_cmd;
    start_idx += contact_dim; 

    grad.segment(start_idx, MODEL_DOF_VIRTUAL) -= W_qddot * qddot_cmd;
    start_idx += MODEL_DOF_VIRTUAL; 

    grad.segment(start_idx, MODEL_DOF) -= W_torque * torque_prev;
    start_idx += MODEL_DOF; 
}

void DynWBC::calcEqualityConstraint()
{
    //--- (1) Floating base dynamics
    Eigen::MatrixXd A_wb; A_wb.setZero(MODEL_DOF_VIRTUAL, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
    Eigen::VectorXd lbA_wb; lbA_wb.setZero(MODEL_DOF_VIRTUAL);
    Eigen::VectorXd ubA_wb; ubA_wb.setZero(MODEL_DOF_VIRTUAL);

    unsigned int start_idx = 0;
    A_wb.block(0, start_idx, MODEL_DOF_VIRTUAL, contact_dim) = J_C_T;
    start_idx += contact_dim;
    A_wb.block(0, start_idx, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = (-1.0) * M;
    start_idx += MODEL_DOF_VIRTUAL;
    A_wb.block(0, start_idx, MODEL_DOF_VIRTUAL, MODEL_DOF) = Sa_T;
    start_idx += MODEL_DOF; 

    lbA_wb = G;
    ubA_wb = G;
    constraints_.push_back({A_wb, lbA_wb, ubA_wb});
}

void DynWBC::calcInequalityConstraint()
{
    //--- (2) Friction cone constraints
    constraints_.push_back({   
        A_fric,
        Eigen::VectorXd::Constant(A_fric.rows(), -std::numeric_limits<double>::infinity()),
        ubA_fric
    });
}

void DynWBC::checkGradHessSize()
{
    if(is_gradhess_init_ == true)
    {
        std::cout << "==============================================" << std::endl;
        std::cout << "===== DynWBC COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "==============================================" << std::endl;

        std::cout << "total_num_state: " << total_num_state << std::endl;
        std::cout << "total_num_constraints: " << total_num_constraints << std::endl;
        std::cout << std::endl;

        std::cout << "Hess size: " << Hess.rows() << " x " << Hess.cols() << std::endl;
        std::cout << "grad size: " << grad.size() << std::endl;
        std::cout << std::endl;

        std::cout << "A: " << A_const.rows() << " x " << A_const.cols() << std::endl;
        std::cout << "lbA size: " << lbA_const.size() << std::endl;
        std::cout << "ubA size: " << ubA_const.size() << std::endl;
        std::cout << std::endl;

        is_gradhess_init_ = false;
    }
}

///////////////////////////////////////////
//--- Quadratic Programming Variables ---//
///////////////////////////////////////////
void DynWBC::updateContactState()
{
    contact_dim_prev = contact_dim;
    contact_dim = rd_.contact_index * 6;
    contact_dim = 12;    // TODO

    contact_wrench_cmd.setZero(contact_dim);

    if (local_LF_contact == true && local_RF_contact == true)
    {
        contact_wrench_cmd.head(6) = rd_.LF_FT_DES;
        contact_wrench_cmd.tail(6) = rd_.RF_FT_DES;
    }
    else if (local_LF_contact == true || local_RF_contact != true)
    {
        contact_wrench_cmd = rd_.LF_FT_DES;
    }
    else if (local_LF_contact != true || local_RF_contact == true)
    {
        contact_wrench_cmd = rd_.RF_FT_DES;
    }
    else
    {
        ROS_ERROR("DynWBC::updateContactState() ERROR: No contact detected!");
    }
}

void DynWBC::updateRobotStates() 
{
    //--- Robot States
    // M = rd_.local_A;
    // M_inv = rd_.local_A_inv;
    // G = rd_.local_G;
    M = rd_.A_;
    M_inv = rd_.A_inv_;
    G = rd_.G;

    // J_C.setZero(rd_.local_J_C.rows(), rd_.local_J_C.cols());
    // J_C = rd_.local_J_C;

    J_C.setZero(12, MODEL_DOF_VIRTUAL);  // TODO
    J_C.topRows(6) = rd_.ee_[0].jac_contact.cast<double>();
    J_C.bottomRows(6) = rd_.ee_[1].jac_contact.cast<double>();

    J_C_T.setZero(J_C.cols(), J_C.rows());
    J_C_T = J_C.transpose();

    // N_C.setZero(rd_.local_N_C.rows(), rd_.local_N_C.cols());
    // N_C = rd_.local_N_C;

    Sa_T.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF); Sa_T.bottomRows(MODEL_DOF).setIdentity();
    Sa.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);   Sa = Sa_T.transpose();
    Sf.setZero(6, MODEL_DOF_VIRTUAL);    Sf.leftCols(6).setIdentity();

    //--- Friction cone constraints (https://scaron.info/robotics/wrench-friction-cones.html)
    Eigen::MatrixXd U_fric_dsp; U_fric_dsp.setZero(34, 12);
    Eigen::MatrixXd U_fric_ssp; U_fric_ssp.setZero(17, 6);
    double X = foot_size  / 2.0; 
    double Y = foot_width / 2.0; 
    U_fric_ssp <<  0,  0,            -1,   0,   0,  0,
                  -1,  0,           -mu,   0,   0,  0,
                  +1,  0,           -mu,   0,   0,  0,
                   0, -1,           -mu,   0,   0,  0,
                   0, +1,           -mu,   0,   0,  0,
                   0,  0,            -Y,  -1,   0,  0,
                   0,  0,            -Y,  +1,   0,  0,
                   0,  0,            -X,   0,  -1,  0,
                   0,  0,            -X,   0,  +1,  0,
                  -Y, -X, -(X + Y) * mu, -mu, +mu, -1,
                  +Y, +X, -(X + Y) * mu, +mu, -mu, -1,
                  +Y, -X, -(X + Y) * mu, +mu, +mu, -1,
                  +Y, +X, -(X + Y) * mu, +mu, +mu, -1,
                  +Y, -X, -(X + Y) * mu, +mu, +mu, +1,
                  +Y, +X, -(X + Y) * mu, +mu, -mu, +1,
                  -Y, -X, -(X + Y) * mu, -mu, -mu, +1,
                  -Y, +X, -(X + Y) * mu, -mu, +mu, +1;
    U_fric_dsp.topLeftCorner(17, 6) = U_fric_ssp;
    U_fric_dsp.bottomRightCorner(17, 6) = U_fric_ssp;

    if (local_LF_contact == true && local_RF_contact == true)
    {
        A_fric.setZero(34, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
        lbA_fric.setZero(34);
        ubA_fric.setZero(34);

        A_fric.leftCols(contact_dim) = U_fric_dsp;
    }
    else
    {
        A_fric.setZero(17, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
        lbA_fric.setZero(17);
        ubA_fric.setZero(17);

        A_fric.leftCols(contact_dim) = U_fric_ssp;
    }
}

//--- Setter
void DynWBC::setFrictionCoefficient(const double& mu_)
{
    //--- Friction, Contact, Torque limit constraints
    mu = mu_;
}

void DynWBC::setFootDimension(const double& foot_size_, const double& foot_width_)
{
    foot_size = foot_size_; 
    foot_width = foot_width_; 
}

void DynWBC::setJointTrackingWeight(const double &W_qddot_)
{
    W_qddot = W_qddot_;
}

void DynWBC::setContactWrenchTrackingWeight(const double &W_cwr_)
{
    W_cwr = W_cwr_;
}

void DynWBC::setTorqueMinimizationWeight(const double &W_torque_)
{
    W_torque = W_torque_;
}

void DynWBC::setAccelEnergyMinimizationWeight(const double &W_energy_)
{
    W_energy = W_energy_;
}
