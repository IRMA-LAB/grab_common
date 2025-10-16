/**
 * @file kinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 07 Feb 2020
 * @brief File containing definitions of functions declared in kinematics.h.
 */

#include "kinematics.h"

namespace grabcdpr {

    void updatePlatformPose(const Vector3d& position,
        const Vector3d& orientation,
        const Vector3d& pos_PG_loc, PlatformVars& platform)
    {
        // Update platform pose.
        platform.updatePose(position, orientation);
        // Calculate platform baricenter positions expressed in global frame.
        platform.pos_PG_glob = platform.rot_mat * pos_PG_loc;
        platform.pos_OG_glob = platform.position + platform.pos_PG_glob;
    }

    void updatePlatformPose(const Vector3d& position,
        const grabgeom::Quaternion& orientation,
        const Vector3d& pos_PG_loc, PlatformVarsQuat& platform)
    {
        // Update platform pose.
        platform.updatePose(position, orientation);
        // Calculate platform baricenter positions expressed in global frame.
        platform.pos_PG_glob = platform.rot_mat * pos_PG_loc;
        platform.pos_OG_glob = platform.position + platform.pos_PG_glob;
    }

    void updatePlatformPose(const Vector3d& position,
        const Vector3d& orientation,
        const PlatformParams& params, PlatformVars& platform)
    {
        updatePlatformPose(position, orientation, params.pos_PG_loc, platform);
    }

    void updatePlatformPose(const Vector3d& position,
        const grabgeom::Quaternion& orientation,
        const PlatformParams& params, PlatformVarsQuat& platform)
    {
        updatePlatformPose(position, orientation, params.pos_PG_loc, platform);
    }

    void calcPosD(const Vector3d& pos_PD_loc, const Vector3d& pos_OA_glob,
        const Vector3d& position, const Matrix3d rot_mat, CableVarsBase& cable)
    {
        cable.pos_PD_glob = rot_mat * pos_PD_loc;
        cable.pos_OD_glob = position + cable.pos_PD_glob;
        cable.pos_DA_glob = pos_OA_glob - cable.pos_OD_glob;
    }

    void updatePosD(const ActuatorParams& params, const PlatformVarsBase& platform,
        CableVarsBase& cable)
    {
        calcPosD(params.winch.pos_PD_loc, params.pulley.pos_OA_glob, platform.position, platform.rot_mat, cable);
    }

    void calcPulleyVersors(const Vector3d& vers_i_loc, const Vector3d& vers_j_loc,
        CableVarsBase& cable)
    {
        double cos_sigma = cos(cable.swivel_ang);
        double sin_sigma = sin(cable.swivel_ang);
        cable.vers_u = vers_i_loc * cos_sigma + vers_j_loc * sin_sigma;
        cable.vers_w = -vers_i_loc * sin_sigma + vers_j_loc * cos_sigma;
    }

    void updatePulleyVersors(const PulleyParams& params, CableVarsBase& cable)
    {
        calcPulleyVersors(params.vers_i_loc, params.vers_j_loc, cable);
    }

    double calcSwivelAngle(const Vector3d& vers_i_loc, const Vector3d& vers_j_loc,
        const Vector3d& pos_DA_glob, const Matrix3d& rot_mat)
    {
        return atan2(Dot(rot_mat * vers_j_loc, pos_DA_glob),
            Dot(rot_mat * vers_i_loc, pos_DA_glob));
    }

    void updateSwivelAngle(const PulleyParams& params, const PlatformVarsBase& platform,
        CableVarsBase& cable)
    {
        cable.swivel_ang = calcSwivelAngle(params.vers_i_loc, params.vers_j_loc,
            cable.pos_DA_glob, platform.rot_mat);
    }

    double calcTangentAngle(const PulleyParams& params, const Vector3d& vers_u,
        const Vector3d& pos_DA_glob, const Matrix3d& rot_mat)
    {
        double s = Dot(rot_mat * vers_u, pos_DA_glob);
        double app_var = Dot(rot_mat * params.vers_k_loc, pos_DA_glob) / s;
        double psi = 2. * atan(app_var + sqrt(1. - 2. * params.radius / s + SQUARE(app_var)));
        return psi;
    }

    void updateTangentAngle(const PulleyParams& params, const PlatformVarsBase& platform,
        CableVarsBase& cable)
    {
        cable.tan_ang = calcTangentAngle(params, cable.vers_u, cable.pos_DA_glob, platform.rot_mat);
    }

    void calcCableVectors(const double& radius, const Vector3d& vers_k_loc,
        const Matrix3d& rot_mat, CableVarsBase& cable)
    {
        // Versors describing cable exit direction from swivel pulley.
        double cos_psi = cos(cable.tan_ang);
        double sin_psi = sin(cable.tan_ang);
        cable.vers_n = cable.vers_u * cos_psi + vers_k_loc * sin_psi;
        cable.vers_t = cable.vers_u * sin_psi - vers_k_loc * cos_psi;
        // Vector from swivel pulley exit point to platform attaching point.
        cable.pos_BA_glob = cable.pos_DA_glob - radius * rot_mat * (cable.vers_u + cable.vers_n);
    }

    void updateCableVectors(const PulleyParams& params, const PlatformVarsBase& platform,
        CableVarsBase& cable)
    {
        calcCableVectors(params.radius, params.vers_k_loc, platform.rot_mat, cable);
    }

    double calcCableLen(const double& pulley_radius, const Vector3d& pos_BA_glob,
        const double& tan_ang)
    {
        return Norm(pos_BA_glob) + pulley_radius * (M_PI - tan_ang);
    }

    void updateCableLen(const PulleyParams& params, CableVarsBase& cable)
    {
        cable.length = calcCableLen(params.radius, cable.pos_BA_glob, cable.tan_ang);
    }

    double calcMotorCounts(const double tau, const double cable_len,
        const double pulley_radius, const double tan_ang)
    {
        return (cable_len + pulley_radius * (M_PI - tan_ang)) / tau;
    }

    double calcMotorCounts(const ActuatorParams& params, const CableVarsBase& cable)
    {
        return calcMotorCounts(params.winch.transmission_ratio, cable.length,
            params.pulley.radius, cable.tan_ang);
    }

    void updateJacobiansRow_l(const ActuatorParams& params,
        const PlatformVars& platform, CableVars& cable)
    {
        Vector3d temp = -Skew(cable.pos_PD_glob + params.pulley.radius * platform.rot_mat *
            (cable.vers_u + cable.vers_n)) * platform.rot_mat * cable.vers_t;
        cable.geom_jacob_row_l.SetBlock<1, 3>(1, 1, -cable.vers_t.Transpose() * platform.rot_mat.Transpose());
        cable.geom_jacob_row_l.SetBlock<1, 3>(1, 4, temp.Transpose());

        cable.anal_jacob_row_l = cable.geom_jacob_row_l;
        cable.anal_jacob_row_l.SetBlock<1, 3>(1, 4,
            cable.anal_jacob_row_l.GetBlock<1, 3>(1, 4) * platform.h_mat);
    }

    void updateJacobiansRow_l(const ActuatorParams& params,
        const PlatformVarsQuat& platform, CableVarsQuat& cable)
    {
        Vector3d temp = -Skew(cable.pos_PD_glob + params.pulley.radius * platform.rot_mat *
            (cable.vers_u + cable.vers_n)) * platform.rot_mat * cable.vers_t;
        cable.geom_jacob_row_l.SetBlock<1, 3>(1, 1, -cable.vers_t.Transpose() * platform.rot_mat.Transpose());
        cable.geom_jacob_row_l.SetBlock<1, 3>(1, 4, temp.Transpose());

        cable.anal_jacob_row_l.SetBlock<1, 3>(1, 1, cable.geom_jacob_row_l.GetBlock<1, 3>(1, 1));
        cable.anal_jacob_row_l.SetBlock<1, 4>(1, 4,
            cable.geom_jacob_row_l.GetBlock<1, 3>(1, 4) * platform.h_mat);
    }

    void updateJacobiansRow_s(const ActuatorParams& params,
        const PlatformVars& platform, CableVars& cable)
    {
        Vector3d temp = -Skew(cable.pos_PD_glob + cable.pos_DA_glob) * platform.rot_mat * cable.vers_w;
        cable.geom_jacob_row_s.SetBlock<1, 3>(1, 1, -cable.vers_w.Transpose() * platform.rot_mat.Transpose());
        cable.geom_jacob_row_s.SetBlock<1, 3>(1, 4, temp.Transpose());
        cable.geom_jacob_row_s = cable.geom_jacob_row_s / Dot(platform.rot_mat * cable.vers_u, cable.pos_DA_glob);

        cable.anal_jacob_row_s = cable.geom_jacob_row_s;
        cable.anal_jacob_row_s.SetBlock<1, 3>(1, 4,
            cable.anal_jacob_row_s.GetBlock<1, 3>(1, 4) * platform.h_mat);
    }

    void updateCableZeroOrd(const ActuatorParams& params, const PlatformVars& platform,
        CableVars& cable)
    {
        updatePosD(params, platform, cable);     // update segments ending with point A_i.
        updateSwivelAngle(params.pulley, platform, cable); // from 1st kinematic constraint.
        updatePulleyVersors(params.pulley, cable);
        updateTangentAngle(params.pulley, platform, cable); // from 2nd kinematic constraint.
        updateCableVectors(params.pulley, platform, cable); // from 1st kinematic constraint.
        updateCableLen(params.pulley, cable);     // from 3rd kinematic constraint.
        updateJacobiansRow_l(params, platform, cable);
        updateJacobiansRow_s(params, platform, cable);
    }

    void updateCableZeroOrd(const ActuatorParams& params, const PlatformVarsQuat& platform,
        CableVarsQuat& cable)
    {
        updatePosD(params, platform, cable);     // update segments ending with point A_i.
        updateSwivelAngle(params.pulley, platform, cable); // from 1st kinematic constraint.
        updatePulleyVersors(params.pulley, cable);
        updateTangentAngle(params.pulley, platform, cable); // from 2nd kinematic constraint.
        updateCableVectors(params.pulley, platform, cable); // from 1st kinematic constraint.
        updateCableLen(params.pulley, cable);     // from 3rd kinematic constraint.
        updateJacobiansRow_l(params, platform, cable);
    }

    void updateIK0(const Vector3d& position, const Vector3d& orientation,
        const RobotParams& params, RobotVars& vars)
    {
        updatePlatformPose(position, orientation, params.platform, vars.platform);
        std::vector<unsigned int> active_actuators_id = params.activeActuatorsId();

        for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
            updateCableZeroOrd(params.actuators[active_actuators_id[i]], vars.platform,
                vars.cables[i]);
        vars.updateJacobians();
    }
    //TODO control code from here!
    void updateIK0(const Vector6d& pose, const RobotParams& params, RobotVars& vars)
    {
        updateIK0(pose.HeadRows<3>(), pose.TailRows<3>(), params, vars);
    }

    void updateIK0(const arma::vec6& _pose, const RobotParams& params, RobotVars& vars)
    {
        Vector6d pose(_pose.begin(), _pose.end());
        updateIK0(pose, params, vars);
    }

    void updateIK0(const Vector3d& position, const grabgeom::Quaternion& orientation,
        const RobotParams& params, RobotVarsQuat& vars)
    {
        updatePlatformPose(position, orientation, params.platform, vars.platform);
        std::vector<unsigned int> active_actuators_id = params.activeActuatorsId();
        for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
            updateCableZeroOrd(params.actuators[active_actuators_id[i]], vars.platform,
                vars.cables[i]);
        vars.updateJacobians();
    }

    arma::mat calcJacobianL(const RobotVars& vars) { return vars.anal_jacobian; }

    arma::mat calcJacobianSw(const RobotVars& vars)
    {
        arma::mat jacobian_sw(arma::size(vars.anal_jacobian), arma::fill::none);
        for (unsigned int i = 0; i < jacobian_sw.n_rows; ++i)
        {
            arma::rowvec temp =
                arma::join_horiz(-toArmaVec(vars.platform.rot_mat * vars.cables[i].vers_w).t(),
                    toArmaVec(-(vars.platform.h_mat.Transpose() * Skew(vars.cables[i].pos_PD_glob + vars.cables[i].pos_DA_glob) * vars.platform.rot_mat * vars.cables[i].vers_w).Transpose()));
            jacobian_sw.row(i) =
                temp / Dot(vars.platform.rot_mat * vars.cables[i].vers_u, vars.cables[i].pos_DA_glob);
        }
        return jacobian_sw;
    }

    void optFunDK0(const RobotParams& params, const arma::vec& cables_length,
        const arma::vec& swivel_angles, const arma::vec6& pose,
        arma::mat& fun_jacobian, arma::vec& fun_val)
    {
        const size_t kNumCables = params.activeActuatorsNum();
        RobotVars vars(kNumCables, params.platform.rot_parametrization);
        updateIK0(pose, params, vars);

        arma::vec l_constraints(kNumCables, arma::fill::none);
        arma::vec sw_constraints(kNumCables, arma::fill::none);
        for (unsigned int i = 0; i < kNumCables; ++i)
        {
            l_constraints(i) = vars.cables[i].length - cables_length[i];
            sw_constraints(i) = vars.cables[i].swivel_ang - swivel_angles[i];
        }

        arma::mat l_jacobian = calcJacobianL(vars);
        arma::mat sw_jacobian = calcJacobianSw(vars);

        fun_val = arma::join_vert(l_constraints, sw_constraints);
        fun_jacobian = arma::join_vert(l_jacobian, sw_jacobian);
    }

    bool solveDK0(const std::vector<double>& cables_length,
        const std::vector<double>& swivel_angles,
        const VectorXd<POSE_DIM>& init_guess_pose,
        const RobotParams& params, VectorXd<POSE_DIM>& platform_pose,
        const uint8_t nmax /*= 100*/, uint8_t* iter_out /*= nullptr*/)
    {
        static const double kFtol = 1e-6;
        static const double kXtol = 1e-6;

        // First round to init function value and jacobian
        arma::vec func_val;
        arma::mat func_jacob;
        arma::vec6 pose = toArmaVec(init_guess_pose);
        optFunDK0(params, cables_length, swivel_angles, pose, func_jacob, func_val);

        // Init iteration variables
        arma::vec s;
        uint8_t iter = 0;
        double err = 1.0;
        double cond = 0.0;
        // Start iterative process
        while (arma::norm(func_val) > kFtol && err > cond)
        {
            if (iter >= nmax)
                return false; // did not converge
            iter++;
            s = arma::solve(func_jacob, func_val);
            pose -= s;
            optFunDK0(params, cables_length, swivel_angles, pose, func_jacob, func_val);
            err = arma::norm(s);
            cond = kXtol * (1 + arma::norm(pose));
        }

        if (iter_out != nullptr)
            *iter_out = iter;

        platform_pose.Fill(pose.begin(), pose.end());

        return true;
    }

    bool updateDK0(const RobotParams& params, RobotVars& vars)
    {
        // Extract starting conditions from latest robot configuration
        // Cable's variables are expected to be up-to-date
        std::vector<double> cables_length(vars.cables.size(), 0);
        std::vector<double> swivel_angles(vars.cables.size(), 0);
        for (unsigned int i = 0; i < vars.cables.size(); ++i)
        {
            cables_length[i] = vars.cables[i].length;
            swivel_angles[i] = vars.cables[i].swivel_ang;
        }
        // While platform pose is expected to be the latest known/computed value, so not updated
        VectorXd<POSE_DIM> init_guess_pose = vars.platform.pose;

        // Solve direct kinematics
        VectorXd<POSE_DIM> new_pose;
        if (solveDK0(cables_length, swivel_angles, init_guess_pose, params, new_pose))
        {
            // Update inverse kinematics
            updateIK0(new_pose, params, vars);
            return true;
        }
        // Could not solve optimization (failed direct kinematics)
        return false;
    }

    void costFunDkLengthSwivelAHRS(const RobotParams& params, const Measures& state_est_meas,
        RobotVars& vars, const Vector6d pose,
        VectorXd<19>& F, MatrixXd<19, 6>& J) {

        // sensor errors for normalization
        VectorXd<8> length_noise;
        VectorXd<8> swivel_noise;
        Vector3d AHRS_noise;
        for (unsigned int i = 0; i < vars.cables.size(); i++) {
            length_noise(i + 1) = 1 / 0.004;                    // 5 mm of std
            swivel_noise(i + 1) = 1 / (0.8 * M_PI / 180);       // 0.5 deg of std
        }
        for (unsigned int i = 1; i <= 3; i++)
            AHRS_noise(i) = 1 / (1 * M_PI / 180);               // 0.8 deg of std
        VectorXd<19> weights;
        weights.SetBlock<8, 1>(1, 1, length_noise);
        weights.SetBlock<8, 1>(9, 1, swivel_noise);
        weights.SetBlock<3, 1>(17, 1, AHRS_noise);

        // inverse kinematics update
        updateIK0(pose, params, vars);
        VectorXd<8> cable_lengths;
        VectorXd<8> swivel_angles;
        for (unsigned int i = 0; i < vars.cables.size(); i++) {
            cable_lengths(i + 1) = vars.cables[i].length;
            swivel_angles(i + 1) = vars.cables[i].swivel_ang;
        }

        // residual vector and jacobian computation
        F.SetBlock<8, 1>(1, 1, cable_lengths - state_est_meas.lengths);
        F.SetBlock<8, 1>(9, 1, swivel_angles - state_est_meas.swivels);
        F.SetBlock<3, 1>(17, 1, pose.GetBlock<3, 1>(4, 1) - state_est_meas.epsilon);
        F = Diag(weights) * F;

        Matrix3d my_eye(0);
        my_eye.SetBlock<1, 1>(1, 1, 1);
        my_eye.SetBlock<1, 1>(2, 2, 1);
        my_eye.SetBlock<1, 1>(3, 3, 1);
        J.SetBlock<8, 6>(1, 1, vars.anal_jacobian_l);
        J.SetBlock<8, 6>(9, 1, vars.anal_jacobian_s);
        J.SetBlock<3, 6>(17, 1, HorzCat(Matrix3d(0), my_eye));
        J = Diag(weights) * J;
    }

} // end namespace grabcdpr
