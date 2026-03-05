#include "mpc_solver.h"

WBMPC::WBMPC(RobotData &rd) : rd_(rd)
{
    //---(1) Load Pinocchio Model
    std::string urdf_path;
    ros::param::get("/tocabi_controller/urdf_path", urdf_path);

    pinocchio::JointModelFreeFlyer floating_base;
    pinocchio::urdf::buildModel(urdf_path, floating_base, model_);
    data_ = pinocchio::Data(model_);

    std::cout << "============ REDUCED MODEL ============" << std::endl;
    const std::vector<std::string> reduce_joint_names = {"Waist1_Joint", "Waist2_Joint", "Upperbody_Joint", 
                                                         "Neck_Joint", "Head_Joint",
                                                         "L_Shoulder1_Joint", "L_Shoulder2_Joint", "L_Shoulder3_Joint", "L_Armlink_Joint", "L_Elbow_Joint", "L_Forearm_Joint", "L_Wrist1_Joint", "L_Wrist2_Joint",
                                                         "R_Shoulder1_Joint", "R_Shoulder2_Joint", "R_Shoulder3_Joint", "R_Armlink_Joint", "R_Elbow_Joint", "R_Forearm_Joint", "R_Wrist1_Joint", "R_Wrist2_Joint"};
    std::vector<pinocchio::JointIndex> reduce_joint_idx; 

    for (const auto& name : reduce_joint_names)
        reduce_joint_idx.push_back(model_.getJointId(name));

    Eigen::VectorXd q_neutral; q_neutral.setZero(model_.nq);

    model_ = pinocchio::buildReducedModel(model_, reduce_joint_idx, q_neutral);
    data_ = pinocchio::Data(model_);

    std::cout << "================================" << std::endl;
    std::cout << "===== Pinocchio Model Info =====" << std::endl;
    std::cout << "model.nq      : "      << model_.nq << std::endl;
    std::cout << "model.nv      : "      << model_.nv << std::endl;
    std::cout << "model.njoints : " << model_.njoints << std::endl;
    std::cout << "model.nbodies : " << model_.nbodies << std::endl;
    std::cout << "model.nframes : " << model_.nframes << std::endl;

    //---(2) Casadi Solver
    solver = casadi::external("solver_function", lib_path);

    std::cout << "=======================================" << std::endl;
    std::cout << "===== Casadi Solver Function Info =====" << std::endl;
    std::cout << "n_in  = " << solver.n_in() << std::endl;
    std::cout << "n_out = " << solver.n_out() << std::endl;

    std::cout << " " << std::endl;

    for (int i = 0; i < solver.n_in(); i++)
    {
        std::cout << "Input[" << i << "] "
                  << solver.name_in(i)
                  << " : "
                  << solver.size1_in(i) << " x "
                  << solver.size2_in(i)
                  << std::endl;
    }

    std::cout << " " << std::endl;

    for (int i = 0; i < solver.n_out(); i++)
    {
        std::cout << "Output[" << i << "] "
                  << solver.name_out(i)
                  << " : "
                  << solver.size1_out(i) << " x "
                  << solver.size2_out(i)
                  << std::endl;
    }
    std::cout << "=======================================" << std::endl;

    //---(3) Initialize
    output_mpc.setZero(solver.size1_out(0));
    output_mpc_prev.setZero(solver.size1_out(0));

    swing_period = gait_period * 0.5;

    dt_vec.resize(mpc_nodes);
    double ratio = dt_max / dt_min;
    double gamma = std::pow(ratio, 1.0 / (mpc_nodes - 1));
    for(int i = 0; i < mpc_nodes; i++)
    {
        dt_vec[i] = dt_min * std::pow(gamma, static_cast<int>(i)); 
        // printf("dt_vec[%d]: %f\n", i, dt_vec[i]);
    }
}

void WBMPC::solve()
{
    output_mpc_prev = output_mpc;

    std::vector<casadi::DM> out = solver(input_mpc);
    casadi::DM sol = out[0];
    output_mpc = CasadiDMToEigenVector(sol);

    retractStackedSolution(output_mpc);
}

void WBMPC::updateMPCSolverInput(const Eigen::VectorQVQd &q, const Eigen::VectorVQd &v, const int& current_tick, int &current_step_num)
{
    //--- Robot State Initialization
    x_init.setZero(model_.nq + model_.nv); 

    Eigen::VectorXd q_reduced;
    q_reduced.setZero(model_.nq);
    q_reduced.head(7) = q.head(7);

    Eigen::VectorXd v_reduced;
    v_reduced.setZero(model_.nv);
    v_reduced.head(6) = v.head(6);

    for (int i = 0; i < (model_.nq - 7); ++i)
    {
        int jid = keep_joint_ids[i];

        q_reduced(7 + i) = q(7 + jid);
        v_reduced(6 + i) = v(6 + jid);
    }

    x_init.head(model_.nq) = q_reduced;
    x_init.tail(model_.nv) = v_reduced;

    //--- Contact Schedule
    current_time = current_tick * (1.0 / mpc_hz_);
    const int ticks_per_swing = std::max(1, static_cast<int>(std::round(swing_period * mpc_hz_)));
    current_step_num = current_tick / ticks_per_swing;

    // --- disturbance spec ---
    const int disturb_start_step = 3;           
    const int disturb_duration_ticks = std::max(1, static_cast<int>(std::round(disturbance_duration * mpc_hz_)));

    const int disturb_start_tick = disturb_start_step * ticks_per_swing;
    const int disturb_end_tick   = disturb_start_tick + disturb_duration_ticks;

    disturb_on = (current_tick >= disturb_start_tick) && (current_tick <  disturb_end_tick);

    contact_schedule.setOnes(2, mpc_nodes);
    swing_schedule.setZero(2, mpc_nodes);

    for(int i = 0; i < mpc_nodes; i++){
        if (i > 0){
            current_time += dt_vec[i-1];
        }

        double gait_phase = fmod(current_time, gait_period) / gait_period;
        double swing_phase = fmod(current_time, swing_period) / swing_period;

        if (gait_phase < 0.5)
        {
            contact_schedule(1, i) = 0;
            swing_schedule(1, i) = swing_phase;
        }
        else
        {
            contact_schedule(0, i) = 0;
            swing_schedule(0, i) = swing_phase;
        }
    }

    n_contacts = 1;

    // Control Target
    base_vel_des.setZero(6);
    base_vel_des(0) = 0.05; // desired forward velocity

    //--- Casadi Vector Conversion
    input_mpc.resize(solver.n_in());

    input_mpc[0]  = EigenToCasadiDM(x_init);                // i0
    input_mpc[1]  = casadi::DM(dt_min);                     // i1
    input_mpc[2]  = casadi::DM(dt_max);                     // i2
    input_mpc[3]  = EigenToCasadiDM(contact_schedule);      // i3
    input_mpc[4]  = EigenToCasadiDM(swing_schedule);        // i4
    input_mpc[5]  = casadi::DM(n_contacts);                 // i5
    input_mpc[6]  = casadi::DM(swing_period);               // i6
    input_mpc[7]  = casadi::DM(swing_height);               // i7
    input_mpc[8]  = EigenToCasadiDM(swing_vel_limits);      // i8
    input_mpc[9]  = casadi::DM(foot_length);                // i9
    input_mpc[10] = casadi::DM(foot_width);                 // i10
    input_mpc[11] = casadi::DM(mu);                         // i11
    input_mpc[12] = EigenToCasadiDM(Q_diag);                // i12
    input_mpc[13] = EigenToCasadiDM(R_diag);                // i13
    input_mpc[14] = EigenToCasadiDM(base_vel_des);          // i14
    input_mpc[15] = EigenToCasadiDM(output_mpc_prev);       // i15
}

void WBMPC::retractStackedSolution(const Eigen::VectorXd& sol_x) 
{
    const int ndx_opt = model_.nv + model_.nv;

    q_sol.clear();
    v_sol.clear();
    a_sol.clear();
    force_sol.clear();
    torque_sol.clear();

    DX_prev.clear();
    U_prev.clear();

    int idx = 0;

    for (int i = 0; i < mpc_nodes; ++i)
    {
        int nx_opt = ndx_opt + nu_opt[i];

        Eigen::VectorXd sol = sol_x.segment(idx, nx_opt);
        idx += nx_opt;

        Eigen::VectorXd dx_sol = sol.head(ndx_opt);
        Eigen::VectorXd x_sol = stateIntegrate(x_init, dx_sol);

            DX_prev.push_back(dx_sol);
            q_sol.push_back(x_sol.head(model_.nq));
            v_sol.push_back(x_sol.tail(model_.nv));

        Eigen::VectorXd u_sol  = sol.tail(nu_opt[i]);
            
            U_prev.push_back(u_sol);
            a_sol.push_back(u_sol.head(model_.nv));
            force_sol.push_back(u_sol.segment(model_.nv, nf));
            torque_sol.push_back(u_sol.tail(nu_opt[i] - model_.nv - nf));
    }

    // ---- terminal dx (dx_last) ----
    Eigen::VectorXd dx_last = sol_x.tail(ndx_opt);
    Eigen::VectorXd x_last = stateIntegrate(x_init, dx_last);

        DX_prev.push_back(dx_last);
        q_sol.push_back(x_last.head(model_.nq));
        v_sol.push_back(x_last.tail(model_.nv));    
}

Eigen::VectorXd WBMPC::stateIntegrate(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)
{
    // x  = [q; v]
    // dx = [dq; dv]
    Eigen::VectorXd x_next(x.size());

    Eigen::VectorXd q  = x.head(model_.nq);
    Eigen::VectorXd v  = x.tail(model_.nv);

    Eigen::VectorXd dq = dx.head(model_.nv);
    Eigen::VectorXd dv = dx.tail(model_.nv);

    // --- q_next = integrate(model, q, dq)
    Eigen::VectorXd q_next; q_next.setZero(model_.nq);
    pinocchio::integrate(model_, q, dq, q_next);

    // --- v_next = v + dv
    Eigen::VectorXd v_next = v + dv;

    // --- x_next = [q_next; v_next]
    x_next.head(model_.nq) = q_next;
    x_next.tail(model_.nv) = v_next;

    return x_next;
}


//--- Getters and Setters
void WBMPC::setWeights(const Eigen::VectorXd &Q, const Eigen::VectorXd &R)
{
    Q_diag = Q;
    R_diag = R;
}

void WBMPC::setDisturbanceSpec(const double &force, const double &theta, const double &duration)
{
    disturbance_force = force;
    disturbance_theta = theta;
    disturbance_duration = duration;
}

void WBMPC::setReferenceNominalPose(const Eigen::VectorQd q_nom_)
{
    q_nom = q_nom_;
}

Eigen::VectorQd WBMPC::getWBMPCJointPositionSolution()
{
    Eigen::VectorQd q_next; q_next = q_nom;

    for(int i = 0; i < (model_.nq - 7); i++)
    {
        int jid = keep_joint_ids[i];

        q_next(jid) = q_sol[1](i + 7);
    }

    return q_next;
}

Eigen::VectorQd WBMPC::getWBMPCJointVelocitySolution()
{
    Eigen::VectorQd v_next; v_next.setZero();

    for(int i = 0; i < (model_.nv - 6); i++)
    {
        int jid = keep_joint_ids[i];

        v_next(jid) = v_sol[1](i + 6);
    }

    return v_next;
}

Eigen::VectorQd WBMPC::getWBMPCJointAccelerationSolution()
{
    Eigen::VectorQd a_next; a_next.setZero();

    for(int i = 0; i < (model_.nv - 6); i++)
    {
        int jid = keep_joint_ids[i];

        a_next(jid) = a_sol[1](i);
    }

    return a_next;
}

Eigen::VectorQd WBMPC::getWBMPCJointTorqueSolution()
{
    Eigen::VectorQd torque_next; torque_next.setZero();

    for(int i = 0; i < (model_.nv - 6); i++)
    {
        int jid = keep_joint_ids[i];

        torque_next(jid) = torque_sol[1](i);
    }

    return torque_next;
}
