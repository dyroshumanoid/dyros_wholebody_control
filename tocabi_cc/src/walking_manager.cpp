#include "walking_manager.h"

ofstream dataWM1("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM1.txt");
ofstream dataWM2("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM2.txt");
ofstream dataWM3("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM3.txt");
ofstream dataWM4("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM4.txt");
ofstream dataWM5("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM5.txt");
ofstream dataWM6("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM6.txt");
ofstream dataWM7("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM7.txt");
ofstream dataWM8("/home/kwan/catkin_ws/src/tocabi_cc/data/dataWM8.txt");

WalkingManager::WalkingManager(RobotData &rd) : rd_(rd)
{
}

void WalkingManager::computeWalkingMotion()
{
    updateSupportInitialState();

    step_command.setZero();
    step_command << step_length, step_width;
    step_command = rotZaxis2d(foot_yaw_angle) * step_command;

    step_length = step_command(0);
    step_width = step_command(1);

    calcFootstepQueue();
    getZmpTrajectory();
    getComTrajectory();
    contactWrenchCalculator();
    getFootTrajectory();
    if(is_ft_sensor_available == true){
        updateFootPoseFromContactWrench();
    }

    mapSupportToBase();

    dataWM5 << rd_.link_[COM_id].x_traj(0) << "," << rd_.link_[COM_id].x_traj(1) << "," << rd_.link_[COM_id].x_traj(2) << ","
            << rd_.link_[COM_id].local_xpos(0) << "," << rd_.link_[COM_id].local_xpos(1) << "," << rd_.link_[COM_id].local_xpos(2)
            << std::endl;
    dataWM6 << rd_.link_[Left_Foot].x_traj(0) << "," << rd_.link_[Left_Foot].x_traj(1) << "," << rd_.link_[Left_Foot].x_traj(2) << ","
            << rd_.link_[Left_Foot].local_xpos(0) << "," << rd_.link_[Left_Foot].local_xpos(1) << "," << rd_.link_[Left_Foot].local_xpos(2)
            << std::endl;

    updateStepTick();
}

void WalkingManager::calcFootstepQueue()
{
    int foot_contact_idx = local_LF_contact ? -1 : +1;
    static bool is_footstep_queue_init = true;

    if (is_footstep_queue_init == true)
    {
        step_queue.clear();

        Eigen::Vector2d step_command_local;
        step_command_local.setZero();
        for (int i = 0; i < preview_idx; i++)
        {
            double step_length_local = (i == 0) ? (step_length * 0.5) : step_length;

            step_command_local += Eigen::Vector2d(step_length_local, foot_contact_idx * step_width);
            step_queue.push_back(step_command_local);

            foot_contact_idx *= -1;
        }

        is_footstep_queue_init = false;
    }
    else
    {
        if (is_footstep_update == true)
        {
            int foot_sign = (preview_idx % 2 == 0) ? -1 : ((preview_idx < 0) ? 1 : 0);

            Eigen::Vector2d step_command_first = step_queue.front();

            for (auto iter = step_queue.begin(); iter != step_queue.end(); iter++)
            {
                *iter -= step_command_first;
            }

            step_queue.pop_front();

            Eigen::Vector2d step_command_back = step_queue.back();
            Eigen::Vector2d step_command_new = step_command_back + Eigen::Vector2d(step_length, foot_sign * foot_contact_idx * step_width);

            step_queue.push_back(step_command_new);

            is_footstep_update = false;
        }
    }
}

void WalkingManager::getZmpTrajectory()
{
    int total_ticks = static_cast<int>(step_duration* hz_) + static_cast<int>(dsp_duration * hz_);
    int foot_contact_idx = local_LF_contact ? -1 : +1;

    double zmp_offset_y = 0.0;

    if (step_cnt == 0) // DSP
    {
        if (is_zmp_update == true)
        {
            const double transfer_ticks = transfer_duration * hz_;
            const double weight_shift_ticks = 0.5 * hz_;

            zmp_x_traj.setZero(static_cast<int>(transfer_ticks + preview_idx * total_ticks));
            zmp_y_traj.setZero(static_cast<int>(transfer_ticks + preview_idx * total_ticks));

            //--- Stand still
            Eigen::Vector2d p0, p1; 
            p0 = rd_.link_[COM_id].support_xpos_init.head(2);

            zmp_x_traj.segment(0, static_cast<int>(transfer_ticks - weight_shift_ticks)).setConstant(p0(0));
            zmp_y_traj.segment(0, static_cast<int>(transfer_ticks - weight_shift_ticks)).setConstant(p0(1));

            //--- Transfer to first footstep
            p0 = rd_.link_[COM_id].support_xpos_init.head(2);
            p1.setZero();
            p1(1) += foot_contact_idx * zmp_offset_y;

            Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(weight_shift_ticks, 0.0, 1.0);
            Eigen::VectorXd zmp_x_traj_onestep = (1.0 - s.array()) * p0(0) + s.array() * p1(0);
            Eigen::VectorXd zmp_y_traj_onestep = (1.0 - s.array()) * p0(1) + s.array() * p1(1);
            zmp_x_traj.segment(static_cast<int>(transfer_ticks - weight_shift_ticks), static_cast<int>(weight_shift_ticks)) = zmp_x_traj_onestep;
            zmp_y_traj.segment(static_cast<int>(transfer_ticks - weight_shift_ticks), static_cast<int>(weight_shift_ticks)) = zmp_y_traj_onestep;

            //--- Footstep
            for (int i = 0; i < preview_idx; i++)
            {
                if (i == 0)
                {
                    p0.setZero();
                }
                else
                {
                    p0 = step_queue[i - 1];
                }

                p0(1) += foot_contact_idx * zmp_offset_y;

                p1 = step_queue[i];

                // SSP
                zmp_x_traj.segment(transfer_ticks + i * total_ticks, static_cast<int>(step_duration * hz_)).setConstant(p0(0));
                zmp_y_traj.segment(transfer_ticks + i * total_ticks, static_cast<int>(step_duration * hz_)).setConstant(p0(1));

                // DSP
                s.setZero(static_cast<int>(dsp_duration * hz_));
                zmp_x_traj_onestep.setZero(static_cast<int>(dsp_duration * hz_));
                zmp_y_traj_onestep.setZero(static_cast<int>(dsp_duration * hz_));
                s = Eigen::VectorXd::LinSpaced(static_cast<int>(dsp_duration * hz_), 0.0, 1.0);
                zmp_x_traj_onestep = (1.0 - s.array()) * p0(0) + s.array() * p1(0);
                zmp_y_traj_onestep = (1.0 - s.array()) * p0(1) + s.array() * p1(1);

                zmp_x_traj.segment(transfer_ticks + i * total_ticks + static_cast<int>(step_duration * hz_), static_cast<int>(dsp_duration * hz_)) = zmp_x_traj_onestep;
                zmp_y_traj.segment(transfer_ticks + i * total_ticks + static_cast<int>(step_duration * hz_), static_cast<int>(dsp_duration * hz_)) = zmp_y_traj_onestep;

                foot_contact_idx *= -1;
            }

            is_zmp_update = false;

            // dataWM7 << std::setprecision(5) << zmp_x_traj.transpose() << std::endl;
            // dataWM8 << std::setprecision(5) << zmp_y_traj.transpose() << std::endl;
        }
    }
    else
    {
        if (is_zmp_update == true)
        {
            zmp_x_traj.setZero(static_cast<int>(preview_idx * total_ticks));
            zmp_y_traj.setZero(static_cast<int>(preview_idx * total_ticks));

            Eigen::Vector2d p0, p1;
            p0.setZero();
            p1.setZero();

            for (int i = 0; i < preview_idx; i++)
            {
                if (i == 0)
                {
                    p0.setZero();
                }
                else
                {
                    p0 = step_queue[i - 1];
                }

                p1 = step_queue[i];

                // SSP
                zmp_x_traj.segment(i * total_ticks, static_cast<int>(step_duration * hz_)).setConstant(p0(0));
                zmp_y_traj.segment(i * total_ticks, static_cast<int>(step_duration * hz_)).setConstant(p0(1));

                // DSP
                Eigen::VectorXd s = Eigen::VectorXd::LinSpaced(static_cast<int>(dsp_duration * hz_), 0.0, 1.0);
                Eigen::VectorXd zmp_x_traj_onestep = (1.0 - s.array()) * p0(0) + s.array() * p1(0);
                Eigen::VectorXd zmp_y_traj_onestep = (1.0 - s.array()) * p0(1) + s.array() * p1(1);

                zmp_x_traj.segment(i * total_ticks + static_cast<int>(step_duration * hz_), static_cast<int>(dsp_duration * hz_)) = zmp_x_traj_onestep;
                zmp_y_traj.segment(i * total_ticks + static_cast<int>(step_duration * hz_), static_cast<int>(dsp_duration * hz_)) = zmp_y_traj_onestep;
             
                foot_contact_idx *= -1;
            }
            
            is_zmp_update = false;

            dataWM7 << std::setprecision(5) << zmp_x_traj.transpose() << std::endl;
            dataWM8 << std::setprecision(5) << zmp_y_traj.transpose() << std::endl;
        }
    }
}

void WalkingManager::getFootTrajectory()
{
    int support_foot_link_idx, swing_foot_link_idx;

    support_foot_link_idx = (support_phase_indicator_ == ContactIndicator::LeftSingleSupport) ? Left_Foot : Right_Foot;
    swing_foot_link_idx = (support_phase_indicator_ == ContactIndicator::LeftSingleSupport) ? Right_Foot : Left_Foot;

    //--- Swing & Support Feet Trajectory
    footstep_des.setZero();
    footstep_des = step_queue.front();

    if (step_cnt == 0)
    {
        if (local_LF_contact && local_RF_contact) // DSP
        {
            rd_.link_[Left_Foot].x_traj  = rd_.link_[Left_Foot].support_xpos_init;
            rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].support_xpos_init;

            rd_.link_[Left_Foot].r_traj.setIdentity();
            rd_.link_[Right_Foot].r_traj.setIdentity();
        }
    }
    else
    {
        if (!(local_LF_contact && local_RF_contact)) // SSP
        {
            rd_.link_[support_foot_link_idx].x_traj = rd_.link_[support_foot_link_idx].support_xpos_init;

            rd_.link_[swing_foot_link_idx].x_traj(0) = DyrosMath::QuinticSpline(step_time, 0.0, step_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(0), 0.0, 0.0, footstep_des(0), 0.0, 0.0)(0);
            rd_.link_[swing_foot_link_idx].x_traj(1) = DyrosMath::QuinticSpline(step_time, 0.0, step_duration, rd_.link_[swing_foot_link_idx].support_xpos_init(1), 0.0, 0.0, footstep_des(1), 0.0, 0.0)(0);

            double start_time = 0.0;
            double end_time = 0.0;
            if (step_time <= step_duration / 2.0)
            {
                start_time = 0.0;
                end_time = step_duration / 2.0;

                rd_.link_[swing_foot_link_idx].x_traj(2) = DyrosMath::QuinticSpline(step_time, start_time, end_time, rd_.link_[swing_foot_link_idx].support_xpos_init(2), 0.0, 0.0, foot_height, 0.0, 0.0)(0);
            }
            else
            {
                start_time = step_duration / 2.0;
                end_time = step_duration;

                rd_.link_[swing_foot_link_idx].x_traj(2) = DyrosMath::QuinticSpline(step_time, start_time, end_time, foot_height, 0.0, 0.0, 0.0, 0.0, 0.0)(0);
            }

            rd_.link_[support_foot_link_idx].r_traj.setIdentity();
            rd_.link_[swing_foot_link_idx].r_traj.setIdentity();
        }
        else // DSP
        {
            rd_.link_[support_foot_link_idx].x_traj = rd_.link_[support_foot_link_idx].support_xpos_init;
            rd_.link_[swing_foot_link_idx].x_traj(0) = footstep_des(0);
            rd_.link_[swing_foot_link_idx].x_traj(1) = footstep_des(1);
            rd_.link_[swing_foot_link_idx].x_traj(2) = 0.0;

            rd_.link_[support_foot_link_idx].r_traj.setIdentity();
            rd_.link_[swing_foot_link_idx].r_traj.setIdentity();
        }
    }

    dataWM1 << std::setprecision(3)
            << rd_.link_[Left_Foot].x_traj(0) << "," << rd_.link_[Left_Foot].x_traj(1) << "," << rd_.link_[Left_Foot].x_traj(2) << ","
            << rd_.link_[Left_Foot].support_xpos(0) << "," << rd_.link_[Left_Foot].support_xpos(1) << "," << rd_.link_[Left_Foot].support_xpos(2)
            << std::endl;

    dataWM2 << std::setprecision(3)
            << rd_.link_[Right_Foot].x_traj(0) << "," << rd_.link_[Right_Foot].x_traj(1) << "," << rd_.link_[Right_Foot].x_traj(2) << ","
            << rd_.link_[Right_Foot].support_xpos(0) << "," << rd_.link_[Right_Foot].support_xpos(1) << "," << rd_.link_[Right_Foot].support_xpos(2)
            << std::endl;
}

void WalkingManager::getComTrajectory()
{
    static bool first_com = true;
    if (first_com == true)
    {
        com_x_dx_ddx << rd_.link_[COM_id].support_xpos(0),
                        rd_.link_[COM_id].support_v(0),
                        0.0;
        com_y_dy_ddy << rd_.link_[COM_id].support_xpos(1),
                        rd_.link_[COM_id].support_v(1),
                        0.0;

        first_com = false;
    }

    if (is_preview_transition == true)
    {
        com_x_dx_ddx(0) = rd_.link_[COM_id].support_xpos(0);
        com_y_dy_ddy(0) = rd_.link_[COM_id].support_xpos(1);

        is_preview_transition = false;
    }
    int NL = static_cast<int>(1.6 * hz_);
    Eigen::VectorXd zmp_x_window;
    zmp_x_window.setZero(NL);
    Eigen::VectorXd zmp_y_window;
    zmp_y_window.setZero(NL);
    
    zmp_x_window = zmp_x_traj.segment(step_tick, NL);
    zmp_y_window = zmp_y_traj.segment(step_tick, NL);

    zmp_x_ref = zmp_x_traj(step_tick);
    zmp_y_ref = zmp_y_traj(step_tick);


    static Eigen::MatrixXd p_err_sum_x_;
    static Eigen::MatrixXd p_err_sum_y_;
    static bool first_integral = true;
    if (first_integral == true)
    {
        p_err_sum_x_.setZero(1, 1);
        p_err_sum_y_.setZero(1, 1);
        first_integral = false;
    }

    Eigen::MatrixXd ux;
    ux.setZero(1, 1);
    Eigen::MatrixXd uy;
    uy.setZero(1, 1);
    ux = -Gi * p_err_sum_x_ - Gx * com_x_dx_ddx - Gd.transpose() * zmp_x_window;
    uy = -Gi * p_err_sum_y_ - Gx * com_y_dy_ddy - Gd.transpose() * zmp_y_window;

    Eigen::Vector3d com_x_dx_ddx_next;
    com_x_dx_ddx_next.setZero();
    Eigen::Vector3d com_y_dy_ddy_next;
    com_y_dy_ddy_next.setZero();

    com_x_dx_ddx_next = A * com_x_dx_ddx + B * ux;
    com_y_dy_ddy_next = A * com_y_dy_ddy + B * uy;

    com_x_dx_ddx = com_x_dx_ddx_next;
    com_y_dy_ddy = com_y_dy_ddy_next;

    rd_.link_[COM_id].x_traj(0) = com_x_dx_ddx(0);
    rd_.link_[COM_id].x_traj(1) = com_y_dy_ddy(0);
    rd_.link_[COM_id].x_traj(2) = com_height;

    p_err_sum_x_(0) += ((C * com_x_dx_ddx_next)(0) - zmp_x_traj(step_tick + 1));
    p_err_sum_y_(0) += ((C * com_y_dy_ddy_next)(0) - zmp_y_traj(step_tick + 1));

    rd_.link_[Pelvis].r_traj.setIdentity();
    dataWM3 << std::setprecision(3)
            << rd_.link_[COM_id].x_traj(0) << "," << rd_.link_[COM_id].x_traj(1) << "," << rd_.link_[COM_id].x_traj(2) << ","
            << rd_.link_[COM_id].support_xpos(0) << "," << rd_.link_[COM_id].support_xpos(1) << "," << rd_.link_[COM_id].support_xpos(2)
            << std::endl;

    // Calc Capture Point
    cp_desired_(0) = com_x_dx_ddx(0) + com_x_dx_ddx(1) / wn;
    cp_desired_(1) = com_y_dy_ddy(0) + com_y_dy_ddy(1) / wn;

    dataWM4 << std::setprecision(3)
            << zmp_x_ref << "," << zmp_y_ref << ","
            << cp_desired_(0) << "," << cp_desired_(1) << ","
            << cp_measured_(0) << "," << cp_measured_(1)
            << std::endl;
}

void WalkingManager::contactWrenchCalculator()
{
    cp_measured_ = (rd_.link_[COM_id].support_xpos + rd_.link_[COM_id].support_v / wn).head(2);

    ////// CONTACT WRENCH CALCULATION //////
    lfoot_contact_wrench.setZero(6);
    rfoot_contact_wrench.setZero(6);
    Eigen::Vector2d del_zmp = 2.0 * (cp_measured_ - cp_desired_);
    double alpha = 0.0;
    double F_R = 0.0, F_L = 0.0;
    double Tau_all_y = 0.0, Tau_R_y = 0.0, Tau_L_y = 0.0;
    double Tau_all_x = 0.0, Tau_R_x = 0.0, Tau_L_x = 0.0;

    Eigen::Vector2d pL_sharp; pL_sharp.setZero(); pL_sharp = rd_.link_[Left_Foot].support_xpos.segment(0, 2); // pL_sharp(1) -= 0.09;
    Eigen::Vector2d pR_sharp; pR_sharp.setZero(); pR_sharp = rd_.link_[Right_Foot].support_xpos.segment(0, 2); // pR_sharp(1) += 0.09;

    alpha = (zmp_y_ref + del_zmp(1) - pR_sharp(1)) / (pL_sharp(1) - pR_sharp(1));
    alpha = DyrosMath::minmax_cut(alpha, 0.0, 1.0);

    F_R = -(1 - alpha) * (rd_.link_[COM_id].mass) * GRAVITY;
    F_L =     - alpha  * (rd_.link_[COM_id].mass) * GRAVITY;

    //////////// TORQUE ////////////
    Eigen::Vector2d pL; pL.setZero(); pL = rd_.link_[Left_Foot].support_xpos.segment(0, 2);
    Eigen::Vector2d pR; pR.setZero(); pR = rd_.link_[Right_Foot].support_xpos.segment(0, 2);

    Tau_all_x = -((pR(1) - (zmp_y_ref + del_zmp(1))) * F_R + (pL(1) - (zmp_y_ref + del_zmp(1))) * F_L);
    Tau_all_y = +((pR(0) - (zmp_x_ref + del_zmp(0))) * F_R + (pL(0) - (zmp_x_ref + del_zmp(0))) * F_L);

    Tau_R_x =(1 - alpha) * Tau_all_x;
    Tau_R_y =(1 - alpha) * Tau_all_y;
    Tau_L_x =     alpha  * Tau_all_x;
    Tau_L_y =     alpha  * Tau_all_y;

    lfoot_contact_wrench << 0.0, 0.0, F_L, Tau_L_x, Tau_L_y, 0.0;
    rfoot_contact_wrench << 0.0, 0.0, F_R, Tau_R_x, Tau_R_y, 0.0;

    lfoot_contact_wrench *= (-1.0);
    rfoot_contact_wrench *= (-1.0);

    rd_.LF_FT_DES = lfoot_contact_wrench;
    rd_.RF_FT_DES = rfoot_contact_wrench;
}

void WalkingManager::footstepOptimizer()
{   
    double w1_step = 1.0, w2_step = 1.0, w3_step = 1.0, w4_step = 5.0, w5_step = 5.0;

    double L_nom = step_queue[0](0);     
    double L_min = L_nom - 0.2;
    double L_max = L_nom + 0.2;

    double W_nom = step_queue[0](1);   
    double W_min = W_nom - 0.1; 
    double W_max = W_nom + 0.1; 
    
    double T_nom = step_duration;
    double T_min = T_nom - 0.2;  
    double T_max = T_nom + 0.2;
    double tau_nom = exp(wn*T_nom); 
    
    double b_nom_x = L_nom / (exp(wn*T_nom) - 1.0); 
    double b_nom_y = W_nom / (exp(wn*T_nom) - 1.0); 

    double u0_x = 0.0;
    double u0_y = 0.0;

    Eigen::MatrixXd H_step;
    Eigen::VectorXd g_step; 
    
    H_step.setZero(5,5);
    H_step(0,0) = w1_step; // step position in x-direction
    H_step(1,1) = w2_step; // step position in y-direction
    H_step(2,2) = w3_step; // step timing
    H_step(3,3) = w4_step; // DCM offset in x
    H_step(4,4) = w5_step; // DCM offset in y
    
    g_step.setZero(5);
    g_step(0) = -w1_step * L_nom;
    g_step(1) = -w2_step * W_nom; 
    g_step(2) = -w3_step * tau_nom;
    g_step(3) = -w4_step * b_nom_x;  
    g_step(4) = -w5_step * b_nom_y;  

    Eigen::VectorXd lb_step;
    Eigen::VectorXd ub_step;
    Eigen::MatrixXd A_step;         

    A_step.setZero(5,5);
    A_step << 1,    0,  -(cp_measured_(0) - u0_x) * exp(-wn * (step_tick/ hz_) ),   1,  0,
              0,    1,  -(cp_measured_(1) - u0_y) * exp(-wn * (step_tick/ hz_) ),   0,  1,
              1,    0,    0,                                                        0,  0,
              0,    1,    0,                                                        0,  0,
              0,    0,    1,                                                        0,  0;

    lb_step.setZero(5);
    ub_step.setZero(5);

    lb_step(0) = u0_x;
    lb_step(1) = u0_y;
    lb_step(2) = L_min;
    lb_step(3) = W_min;
    lb_step(4) = exp(wn*T_min);
    
    ub_step(0) = u0_x;
    ub_step(1) = u0_y;
    ub_step(2) = L_max;
    ub_step(3) = W_max;
    ub_step(4) = exp(wn*T_max);
    
    Eigen::VectorXd stepping_qp; stepping_qp.setZero(5);
    Eigen::VectorXd stepping_input; stepping_input.setZero(5);
    if (local_LF_contact == true || local_RF_contact == true)
    {   
        QP_stepping.InitializeProblemSize(5, 5);
        QP_stepping.EnableEqualityCondition(1e-8);
        QP_stepping.UpdateMinProblem(H_step, g_step);
        QP_stepping.DeleteSubjectToAx();      
        QP_stepping.UpdateSubjectToAx(A_step, lb_step, ub_step);
    
        if(QP_stepping.SolveQPoases(200, stepping_qp))
        {   
            stepping_input = stepping_qp.segment(0, 5);
        }
        else
        {
        }
    }

    std::cout << "stepping_input: " << stepping_input.transpose() << std::endl;
}

void WalkingManager::updateFootPoseFromContactWrench()
{
    constexpr double LPF_CUTOFF = 50.0;
    constexpr double FORCE_KP   = 1e-4;
    constexpr double FORCE_KD   = 1e-8;
    constexpr double FORCE_DAMPING   = 3.0;
    constexpr double MOMENT_KP  = -0.040;
    constexpr double MOMENT_KD  = -0.0005;
    constexpr double MOMENT_DAMPING  = 10.0;

    Eigen::Vector6d lfoot_ft = (-1.0) * rd_.LF_FT;
    Eigen::Vector6d rfoot_ft = (-1.0) * rd_.RF_FT;

    static Eigen::Vector6d lfoot_ft_lpf = lfoot_ft;
    static Eigen::Vector6d rfoot_ft_lpf = rfoot_ft;

    for (int i = 0; i < 6; i++)
    {
        lfoot_ft_lpf(i) = DyrosMath::lpf(lfoot_ft(i), lfoot_ft_lpf(i), hz_, LPF_CUTOFF);
        rfoot_ft_lpf(i) = DyrosMath::lpf(rfoot_ft(i), rfoot_ft_lpf(i), hz_, LPF_CUTOFF);
    }

    //--- Contact Force
    updateMomentControl(cf_error, 
                        cf_error_pre, 
                        cf_input,
                        lfoot_ft_lpf(2) - rfoot_ft_lpf(2),
                        lfoot_contact_wrench(2) - rfoot_contact_wrench(2),
                        FORCE_KP,
                        FORCE_KD,
                        FORCE_DAMPING,
                        hz_);

    rd_.link_[Left_Foot].x_traj(2)  -= cf_input * 0.5;
    rd_.link_[Right_Foot].x_traj(2) += cf_input * 0.5;

    //--- Contact Moment
    updateMomentControl(cm_lfoot_roll_error,
                        cm_lfoot_roll_error_pre,
                        cm_lfoot_roll_input,
                        lfoot_ft_lpf(3),
                        lfoot_contact_wrench(3),
                        MOMENT_KP,
                        MOMENT_KD,
                        MOMENT_DAMPING,
                        hz_);

    updateMomentControl(cm_lfoot_pitch_error,
                        cm_lfoot_pitch_error_pre,
                        cm_lfoot_pitch_input,
                        lfoot_ft_lpf(4),
                        lfoot_contact_wrench(4),
                        MOMENT_KP,
                        MOMENT_KD,
                        MOMENT_DAMPING,
                        hz_);

    updateMomentControl(cm_rfoot_roll_error,
                        cm_rfoot_roll_error_pre,
                        cm_rfoot_roll_input,
                        rfoot_ft_lpf(3),
                        rfoot_contact_wrench(3),
                        MOMENT_KP,
                        MOMENT_KD,
                        MOMENT_DAMPING,
                        hz_);

    updateMomentControl(cm_rfoot_pitch_error,
                        cm_rfoot_pitch_error_pre,
                        cm_rfoot_pitch_input,
                        rfoot_ft_lpf(4),
                        rfoot_contact_wrench(4),
                        MOMENT_KP,
                        MOMENT_KD,
                        MOMENT_DAMPING,
                        hz_);

    rd_.link_[Left_Foot].r_traj = DyrosMath::rotateWithZ(0.0) 
                                * DyrosMath::rotateWithY(cm_lfoot_pitch_input) 
                                * DyrosMath::rotateWithX(cm_lfoot_roll_input);
    rd_.link_[Right_Foot].r_traj = DyrosMath::rotateWithZ(0.0) 
                                * DyrosMath::rotateWithY(cm_rfoot_pitch_input) 
                                * DyrosMath::rotateWithX(cm_rfoot_roll_input);
}

void WalkingManager::mapSupportToBase()
{
    rd_.link_[Left_Foot].x_traj -= rd_.link_[Pelvis].support_xpos;
    rd_.link_[Right_Foot].x_traj -= rd_.link_[Pelvis].support_xpos;
    rd_.link_[COM_id].x_traj -= rd_.link_[Pelvis].support_xpos;
}

//--- State Initialization
void WalkingManager::updateSupportPhaseIndicator()
{
    /*
    This variable stores the initial contact (support) state of the robot 
    at the moment when a new walking phase (support transition) begins.      
    */
    if (is_phase_indicator_transition == true)
    {
        if(local_LF_contact == true && local_RF_contact == true)
        {
            support_phase_indicator_ = ContactIndicator::DoubleSupport;
        }
        if(local_LF_contact == true && local_RF_contact == false)
        {
            support_phase_indicator_ = ContactIndicator::LeftSingleSupport;
        }
        else if (local_LF_contact == false && local_RF_contact == true)
        {
            support_phase_indicator_ = ContactIndicator::RightSingleSupport;
        }

        is_phase_indicator_transition = false;
    }
}

void WalkingManager::updateSupportInitialState()
{
    if (is_support_transition == true)
    {
        for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
        {
            //--- Support frame
            rd_.link_[idx].support_xpos_init = rd_.link_[idx].support_xpos;
            rd_.link_[idx].support_rotm_init = rd_.link_[idx].support_rotm;
            rd_.link_[idx].support_v_init = rd_.link_[idx].support_v;
            rd_.link_[idx].support_w_init = rd_.link_[idx].support_w;
        }

        is_support_transition = false;
    }
    else
    {
        // Do nothing
    }

    
}

//--- Tick Update
void WalkingManager::updateStepTick()
{
    step_tick++;

    static bool is_transfer_phase = true;
    if (is_transfer_phase == true)
    {
        if (step_tick >= static_cast<int>(transfer_duration * hz_))
        {
            if (support_phase_indicator_ == ContactIndicator::DoubleSupport)
            {
                rd_.is_left_contact_transition = true;
                is_zmp_update = true;
                is_phase_indicator_transition = true;
                is_support_transition = true;
            }
            else
            {
                ROS_ERROR("Contact Indicator are assigned with something wrong value.");
                assert(local_LF_contact == true && local_RF_contact == true);
            }

            step_tick = 0;
            step_cnt++;

            is_transfer_phase = false;
        }
    }
    else
    {
        if(!(local_LF_contact && local_RF_contact)) // SSP
        {
            if (step_tick >= static_cast<int>(step_duration * hz_))
            {
                rd_.is_double_contact_transition = true;
                std::cout << "step_tick: " << step_tick << std::endl;
            }
        }
        else    // DSP
        {
            if (step_tick >= static_cast<int>(step_duration * hz_) + static_cast<int>(dsp_duration * hz_))
            {
                if (support_phase_indicator_ == ContactIndicator::RightSingleSupport)
                {
                    rd_.is_left_contact_transition = true;
                    std::cout << "step_cnt: " << step_cnt << std::endl;
                    std::cout << "step_tick: " << step_tick << std::endl;

                    is_phase_indicator_transition = true;
                    is_support_transition = true;
                    is_footstep_update = true;
                    is_zmp_update = true;
                    is_preview_transition = true;
                }
                else if (support_phase_indicator_ == ContactIndicator::LeftSingleSupport)
                {
                    rd_.is_right_contact_transition = true;
                    std::cout << "step_cnt: " << step_cnt << std::endl;
                    std::cout << "step_tick: " << step_tick << std::endl;

                    is_phase_indicator_transition = true;
                    is_support_transition = true;
                    is_footstep_update = true;
                    is_zmp_update = true;
                    is_preview_transition = true;
                }
                else
                {
                    ROS_ERROR("Contact Indicator are assigned with something wrong value.");
                }

                step_tick = 0;
                step_cnt++;
            }
        }
    }
}

//--- Class Setter
void WalkingManager::setControlFrequency(const double &hz)
{
    hz_ = hz;
}

void WalkingManager::setCenterOfMassHeight(const double &com_height_)
{
    com_height = com_height_;
    wn = sqrt(GRAVITY / com_height);
}

void WalkingManager::setTransferDuration(const double &transfer_duration_)
{
    transfer_duration = transfer_duration_;
}

void WalkingManager::updateContactState(const bool &local_LF_contact_, const bool &local_RF_contact_)
{
    local_LF_contact = local_LF_contact_;
    local_RF_contact = local_RF_contact_;
}

void WalkingManager::setWalkingParameter(const double &step_length_, const double &foot_yaw_angle_, const double &foot_height_)
{
    step_length = step_length_;
    step_width = 0.22;
    foot_yaw_angle = foot_yaw_angle_;
    foot_height = foot_height_;
}

void WalkingManager::setStepDuration(const double &step_duration_)
{
    step_duration = step_duration_;

    step_time = static_cast<double>(step_tick) / hz_;
}

void WalkingManager::setDspDuration(const double &dsp_duration_)
{
    dsp_duration = dsp_duration_;
}

void WalkingManager::isForceTorqueSensorAvailable(const bool &is_ft_sensor_available_)
{
    is_ft_sensor_available = is_ft_sensor_available_;
}

double WalkingManager::getPreviewStepNumber()
{
    return static_cast<double>(preview_idx);
}

ContactIndicator WalkingManager::getSupportPhaseIndicator()
{
    return (support_phase_indicator_);
}

void WalkingManager::findPreviewParameter(double dt, int NL)
{
    A.resize(3, 3);
    A(0, 0) = 1.0;
    A(0, 1) = dt;
    A(0, 2) = dt * dt * 0.5;
    A(1, 0) = 0;
    A(1, 1) = 1.0;
    A(1, 2) = dt;
    A(2, 0) = 0;
    A(2, 1) = 0;
    A(2, 2) = 1;

    B.resize(3);
    B(0) = dt * dt * dt / 6;
    B(1) = dt * dt / 2;
    B(2) = dt;

    C.resize(1, 3);
    C(0, 0) = 1;
    C(0, 1) = 0;
    C(0, 2) = -0.71 / 9.81;

    Eigen::MatrixXd A_bar;
    Eigen::VectorXd B_bar;

    B_bar.resize(4);
    B_bar.segment(0, 1) = C * B;
    B_bar.segment(1, 3) = B;

    Eigen::Matrix1x4d B_bar_tran;
    B_bar_tran = B_bar.transpose();

    Eigen::MatrixXd I_bar;
    Eigen::MatrixXd F_bar;
    A_bar.resize(4, 4);
    I_bar.resize(4, 1);
    F_bar.resize(4, 3);
    F_bar.setZero();

    F_bar.block<1, 3>(0, 0) = C * A;
    F_bar.block<3, 3>(1, 0) = A;

    I_bar.setZero();
    I_bar(0, 0) = 1.0;

    A_bar.block<4, 1>(0, 0) = I_bar;
    A_bar.block<4, 3>(0, 1) = F_bar;

    Eigen::MatrixXd Qe;
    Qe.resize(1, 1);
    Qe(0, 0) = 1.0;

    Eigen::MatrixXd R;
    R.resize(1, 1);
    R(0, 0) = 0.000001;

    Eigen::MatrixXd Qx;
    Qx.resize(3, 3);
    Qx.setZero();

    Eigen::MatrixXd Q_bar;
    Q_bar.resize(3, 3);
    Q_bar.setZero();
    Q_bar(0, 0) = Qe(0, 0);

    Eigen::Matrix4d K;

    K(0, 0) = 1083.572780788710;
    K(0, 1) = 586523.188429418020;
    K(0, 2) = 157943.283121116518;
    K(0, 3) = 41.206077691894;
    K(1, 0) = 586523.188429418020;
    K(1, 1) = 319653984.254277825356;
    K(1, 2) = 86082274.531361579895;
    K(1, 3) = 23397.754069026785;
    K(2, 0) = 157943.283121116518;
    K(2, 1) = 86082274.531361579895;
    K(2, 2) = 23181823.112113621086;
    K(2, 3) = 6304.466397614751;
    K(3, 0) = 41.206077691894;
    K(3, 1) = 23397.754069026785;
    K(3, 2) = 6304.466397614751;
    K(3, 3) = 2.659250532188;

    Eigen::MatrixXd Temp_mat;
    Eigen::MatrixXd Temp_mat_inv;
    Eigen::MatrixXd Ac_bar;
    Temp_mat.resize(1, 1);
    Temp_mat.setZero();
    Temp_mat_inv.resize(1, 1);
    Temp_mat_inv.setZero();
    Ac_bar.setZero();
    Ac_bar.resize(4, 4);

    Temp_mat = R + B_bar_tran * K * B_bar;
    Temp_mat_inv = Temp_mat.inverse();

    Ac_bar = A_bar - B_bar * Temp_mat_inv * B_bar_tran * K * A_bar;

    Eigen::MatrixXd Ac_bar_tran(4, 4);
    Ac_bar_tran = Ac_bar.transpose();

    Gi.resize(1, 1);
    Gx.resize(1, 3);
    Gi(0, 0) = 872.3477; 
    Gx(0, 0) = 945252.1760702;
    Gx(0, 1) = 256298.6905049;
    Gx(0, 2) = 542.0544196;
    Eigen::MatrixXd X_bar;
    Eigen::Vector4d X_bar_col;
    X_bar.resize(4, NL);
    X_bar.setZero();
    X_bar_col.setZero();
    X_bar_col = -Ac_bar_tran * K * I_bar;

    for (int i = 0; i < NL; i++)
    {
        X_bar.block<4, 1>(0, i) = X_bar_col;
        X_bar_col = Ac_bar_tran * X_bar_col;
    }

    Gd.resize(NL);
    Eigen::VectorXd Gd_col(1);
    Gd_col(0) = -Gi(0, 0);

    for (int i = 0; i < NL; i++)
    {
        Gd.segment(i, 1) = Gd_col;
        Gd_col = Temp_mat_inv * B_bar_tran * X_bar.col(i);
    }
}