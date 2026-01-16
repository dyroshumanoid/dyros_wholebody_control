#pragma once

#include <eigen3/Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include "wholebody_functions.h"
#include "utils.h"

class KinWBC {
public:
    KinWBC(RobotData& rd);

    void computeTaskSpaceKinematicWBC();
    void setTaskHierarchy(const TaskMotionType& motion_mode_);
    void setInitialConfiguration(const Eigen::VectorQd &q_init_des_);
    
private:
    RobotData &rd_;
    std::vector<std::vector<TaskInfo>> task_hierarchy;
    TaskMotionType motion_mode;

    Eigen::VectorVQd qdot_des;

    Eigen::VectorQVQd integrate(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &q_delta_);
    Eigen::VectorVQd computeDesiredJointAcceleration(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &qdot_current, const Eigen::VectorQVQd &q_desired, const Eigen::MatrixVVd &Kp, const Eigen::MatrixVVd &Kd);

    Eigen::VectorQd q_init_des;

};