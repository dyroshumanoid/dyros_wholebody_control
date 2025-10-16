#ifndef WBF_H
#define WBF_H

#include "tocabi_lib/robot_data.h"
#include "qp.h"
#include <fstream>

using namespace TOCABI;

namespace WBC
{
    void SetContactInit(RobotData &rd_);

    void CalcContact(RobotData &rd_);
    bool GravMinMax(VectorQd torque);

    void ContactCalcDefault(RobotData &Robot);
    void SetContact(RobotData &Robot, bool left_foot, bool right_foot, bool left_hand = 0, bool right_hand = 0);

    Vector3d GetFstarPos(LinkData &link_, bool a_traj_switch = false, bool com_switch = false);

    Vector3d GetFstarRot(LinkData &link_);

    Vector6d GetFstar6d(LinkData &link_, bool a_traj_switch = false, bool com_switch = false);

    VectorQd GravityCompensationTorque(RobotData &rd_);

    VectorQd GravityCompenstationTorque_Isolated(RobotData &rd_, bool contact_left_foot_, bool contact_right_foot_);

    MatrixXd GetJKT1(RobotData &rd_, MatrixXd &J_task);
    MatrixXd GetJKT1(RobotData &rd_, MatrixXd &J_task, MatrixXd &lambda);

    MatrixXd GetJKT2(RobotData &rd_, MatrixXd &J_task);

    VectorQd TaskControlTorque(RobotData &rd_, VectorXd f_star);

    VectorQd ContactTorqueQP(RobotData &rd_, VectorQd Control_Torque);

    VectorQd TaskControlTorqueQP(RobotData &rd_, VectorXd f_star, bool init);

    void ForceRedistributionTwoContactMod2(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector12d &F12, Eigen::Vector6d &ResultantForce, Eigen::Vector12d &ForceRedistribution, double &eta);
    VectorQd ContactForceRedistributionTorqueWalking(RobotData &Robot, VectorQd command_torque, double eta = 0.9, double ratio = 1.0, int supportFoot = 0);
    VectorQd ContactForceRedistributionTorque(RobotData &Robot, VectorQd command_torque, double eta = 0.9);

    //////////////////////////JS functions
    Vector3d GetFstarPosJS(LinkData &link_, Eigen::Vector3d Desired_pos, Eigen::Vector3d Current_pos, Eigen::Vector3d Desired_vel, Eigen::Vector3d Current_vel);
    Vector3d GetFstarRotJS(LinkData &link_);

    VectorQd TaskControlTorqueMotor(RobotData &rd_, VectorXd f_star, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv);
    VectorQd TaskControlTorqueExtra(RobotData &rd_, VectorXd f_star, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv);
    VectorXd Forcecompute(RobotData &rd_, VectorXd torque, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv);
    VectorQd Vgravitytoruqe(RobotData &rd_);
    VectorQd newtasktorque(RobotData &rd_, VectorXd f_star, Eigen::MatrixVVd B, Eigen::MatrixVVd B_inv, VectorXd Fc);
    VectorQd ContactForceRedistributionTorqueJS(RobotData &Robot, VectorQd command_torque, double ratio, int supportFoot);
    Vector3d GetZMPpos_fromFT(RobotData &Robot);
    Vector3d GetZMPpos_from_ContactForce(RobotData &Robot, VectorXd ContactForce);
    VectorXd getContactForce(RobotData &Robot, VectorQd command_torque);
    int TaskControlHQP(RobotData &rd_, CQuadraticProgram &qp_h_, const MatrixXd &J_task, const MatrixXd &Jkt_, const VectorXd &fstar_, const MatrixXd lambda_task_, const VectorQd &torque_prev, const MatrixXd &prev_task_null_, VectorXd &fstar_result, VectorXd &contact_result, bool init_trigger_);
    int TaskControlHQP(RobotData &rd_, TaskSpace &ts_, CQuadraticProgram &qp_h_, const VectorQd &torque_prev, const MatrixXd &prev_task_null_, bool init_trigger);

    int CalcContactRedistributeHQP(RobotData &rd_, CQuadraticProgram &qp_contact_, VectorXd torque_input, VectorXd &torque_output, bool init);

    void CheckTorqueLimit(RobotData &rd_, VectorQd command_torque);

    // MatrixXd CalcTaskNull(RobotData &rd_, const MatrixXd &Jkt_, const MatrixXd &Jtask, const MatrixXd )

    void CalcJKT(RobotData &rd_, TaskSpace &ts_);
    void CalcTaskNull(RobotData &rd_, TaskSpace &ts_);
    
    // VectorQd GetGravityOnContactLocal(RobotData Robot, bool left_foot, bool right_foot);


}

#endif