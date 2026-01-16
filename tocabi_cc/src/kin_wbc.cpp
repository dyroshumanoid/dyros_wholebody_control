#include "kin_wbc.h"

using namespace TOCABI;

KinWBC::KinWBC(RobotData& rd) : rd_(rd) 
{

}

void KinWBC::setTaskHierarchy(const TaskMotionType& motion_mode_)
{
    motion_mode = motion_mode_;

    if(motion_mode == TaskMotionType::Walking)
    {
        task_hierarchy = {
            {{COM_id, TaskType::Position}, {Pelvis, TaskType::Orientation}},
            {{Left_Foot, TaskType::Position}, {Left_Foot, TaskType::Orientation}, {Right_Foot, TaskType::Position}, {Right_Foot, TaskType::Orientation}}
        };
    }
    else
    {
        task_hierarchy = {
            {{COM_id, TaskType::Position}, {Pelvis, TaskType::Orientation}},
            {{Left_Foot, TaskType::Position}, {Left_Foot, TaskType::Orientation}, {Right_Foot, TaskType::Position}, {Right_Foot, TaskType::Orientation}},
            {{Upper_Body, TaskType::Orientation}},
            {{Head, TaskType::Orientation}},
            {{Left_Hand, TaskType::Position}, {Left_Hand, TaskType::Orientation}, {Right_Hand, TaskType::Position}, {Right_Hand, TaskType::Orientation}}};
    }
}

void KinWBC::computeTaskSpaceKinematicWBC()
{
    //--- Initialization
    qdot_des.setZero();
    Eigen::MatrixVVd Ni = Eigen::MatrixVVd::Identity();

    //--- Nullspace-based Prioritized Task Execution
    for (const auto& task_group : task_hierarchy)
    {
        int m = 3 * task_group.size();
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(m, MODEL_DOF_VIRTUAL);
        Eigen::VectorXd de = Eigen::VectorXd::Zero(m);
     
        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [idx, type] = task_group[i];

            if (type == TaskType::Position)
            {
                J.block(3 * i, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[idx].local_Jac_v;
                Eigen::Vector3d pos_err = rd_.link_[idx].x_traj - rd_.link_[idx].local_xpos;

                de.segment<3>(3 * i) = pos_err;
            }
            else if (type == TaskType::Orientation)
            {
                J.block(3 * i, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[idx].local_Jac_w;
                Eigen::Vector3d ori_err = -DyrosMath::getPhi(rd_.link_[idx].local_rotm, rd_.link_[idx].r_traj);

                de.segment<3>(3 * i) = ori_err;
            }
            else
            {
                ROS_ERROR("Unknown TaskType");
                assert(type == TaskType::Position || type == TaskType::Orientation);
            }
        }

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);

        qdot_des += J_pinv * (de - J * qdot_des);
        Ni *= (Eigen::MatrixVVd::Identity() - J_pinv * J_pre);
    }

    if(motion_mode == TaskMotionType::Walking)
    {
        const int UPPERBODY_DOF = MODEL_DOF - 12;
        Eigen::MatrixXd J; J.setZero(UPPERBODY_DOF, MODEL_DOF_VIRTUAL);
        J.rightCols(UPPERBODY_DOF).setIdentity();

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);

        qdot_des += J_pinv * ((q_init_des.tail(UPPERBODY_DOF) - rd_.q_.tail(UPPERBODY_DOF)) - J * qdot_des);
    }

    rd_.q_desired_virtual = integrate(rd_.local_q_virtual_, qdot_des);
    rd_.q_desired = rd_.q_desired_virtual.tail(MODEL_DOF);

    rd_.q_ddot_desired_virtual = computeDesiredJointAcceleration(rd_.local_q_virtual_, rd_.local_q_dot_virtual_, rd_.q_desired_virtual, rd_.Kp_virtual_diag, rd_.Kd_virtual_diag);
}

Eigen::VectorQVQd KinWBC::integrate(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &q_delta)
{
    Eigen::VectorQVQd q_integrated_; q_integrated_.setZero();

    //--- Floating-base Integration (Base position)
    q_integrated_.segment(0, 3) = q_current.segment(0, 3) + q_delta.segment(0,3); 

    //--- Floating-base Integration (Base orientation)
    Eigen::Quaterniond current_quat(q_current(39), q_current(3), q_current(4), q_current(5));
    Eigen::Quaterniond desired_quat;
    desired_quat = integrateQuatBodyExp(current_quat, q_delta.segment(3, 3), 1.0);
    q_integrated_(3)  = desired_quat.x();   
    q_integrated_(4)  = desired_quat.y();   
    q_integrated_(5)  = desired_quat.z();   
    q_integrated_(39) = desired_quat.w();   

    //--- Floating-base Integration (Base actuated joints)
    q_integrated_.segment(6, MODEL_DOF) = q_current.segment(6, MODEL_DOF) + q_delta.segment(6, MODEL_DOF);

    return(q_integrated_);
}

Eigen::VectorVQd KinWBC::computeDesiredJointAcceleration(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &qdot_current, const Eigen::VectorQVQd &q_desired, const Eigen::MatrixVVd &Kp, const Eigen::MatrixVVd &Kd)
{
    Eigen::VectorVQd qddot_integrated_; qddot_integrated_.setZero();

    //--- Joint Acceleration Command (Base position)
    qddot_integrated_.segment(0, 3) = Kp.block(0, 0, 3, 3) * (q_desired.segment(0, 3) - q_current.segment(0, 3)) 
                                    + Kd.block(0, 0, 3, 3) * (Eigen::Vector3d::Zero() - qdot_current.segment(0, 3));

    //--- Joint Acceleration Command (Base orientation)
    Eigen::Matrix3d current_rotation = Eigen::Quaterniond(q_current(39), q_current(3), q_current(4), q_current(5)).toRotationMatrix();
    Eigen::Matrix3d desired_rotation = Eigen::Quaterniond(q_desired(39), q_desired(3), q_desired(4), q_desired(5)).toRotationMatrix();
    qddot_integrated_.segment(3, 3) = Kp.block(3, 3, 3, 3) * (-DyrosMath::getPhi(current_rotation, desired_rotation)) 
                                    + Kd.block(3, 3, 3, 3) * (Eigen::Vector3d::Zero() - qdot_current.segment(3, 3));

    //--- Joint Acceleration Command (Base actuated joints)
    qddot_integrated_.segment(6, MODEL_DOF) = Kp.block(6, 6, MODEL_DOF, MODEL_DOF) * (q_desired.segment(6, MODEL_DOF) - q_current.segment(6, MODEL_DOF)) 
                                            + Kd.block(6, 6, MODEL_DOF, MODEL_DOF) * (Eigen::VectorQd::Zero() - qdot_current.segment(6, MODEL_DOF));

    return (qddot_integrated_);
}

void KinWBC::setInitialConfiguration(const Eigen::VectorQd &q_init_des_)
{
    q_init_des.setZero();
    q_init_des = q_init_des_;
}