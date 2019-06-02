/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

/**
 * @file   ASLAM_gtsam_addFactor.cpp
 * @brief  SLAM in absolute coordinates with GTSAM: addFactor() implementations
 * @author Jose Luis Blanco Claraco
 * @date   Jan 08, 2018
 */

#include <mola-kernel/lock_helper.h>
#include <mola-kernel/variant_helper.h>  // overloaded{}
#include <mola-slam-gtsam/ASLAM_gtsam.h>
#include <mola-slam-gtsam/ConstVelocityFactorSE3.h>
#include <mola-slam-gtsam/gtsam_mola_bridge.h>

using namespace mola;

BackEndBase::AddFactor_Output ASLAM_gtsam::doAddFactor(Factor& newF)
{
    MRPT_START
    ProfilerEntry    tleg(profiler_, "doAddFactor");
    AddFactor_Output o;

    auto lock = lockHelper(isam2_lock_);

    mola::fid_t fid = INVALID_FID;

    std::visit(
        overloaded{
            [&](const FactorRelativePose3& f) { fid = addFactor(f); },
            [&](const FactorDynamicsConstVel& f) { fid = addFactor(f); },
            [&](const FactorStereoProjectionPose& f) { fid = addFactor(f); },
            [&](const SmartFactorStereoProjectionPose& f) {
                fid = addFactor(f);
            },
            [&](const SmartFactorIMU& f) { fid = addFactor(f); },
            [this, newF]([[maybe_unused]] auto f) {
                THROW_EXCEPTION("Unknown factor type!");
            },
        },
        newF);

    o.success       = true;
    o.new_factor_id = fid;

    return o;

    MRPT_END
}

fid_t ASLAM_gtsam::addFactor(const FactorRelativePose3& f)
{
    MRPT_START
    MRPT_LOG_DEBUG("Adding new FactorRelativePose3");

    // Add to the WorldModel:
    worldmodel_->factors_lock_for_write();
    const fid_t new_fid = worldmodel_->factor_push_back(f);
    worldmodel_->factors_unlock_for_write();

    // Relative pose:
    const gtsam::Pose3 measure = toPose3(f.rel_pose_);

    // Initial estimation of the new KF:
    mrpt::math::TPose3D to_pose_est;

    worldmodel_->entities_lock_for_write();

    const auto& kf_from = worldmodel_->entity_by_id(f.from_kf_);
    // const auto& kf_to   = worldmodel_->entity_by_id(f.to_kf_);

    const auto from_pose_est = mola::entity_get_pose(kf_from);

    from_pose_est.composePose(f.rel_pose_, to_pose_est);

    // Store the result just in case we need it as a quick guess in next
    // factors, before running the actual optimizer:
    // Dont update the pose of the global reference, fixed to Identity()
    if (f.to_kf_ != state_.root_kf_id)
        updateEntityPose(
            worldmodel_->entity_by_id(f.to_kf_), toPose3(to_pose_est));

    worldmodel_->entities_unlock_for_write();

    // mapviz:
    {
        auto lock = lockHelper(vizmap_lock_);

        state_.vizmap.nodes[f.to_kf_] = mrpt::poses::CPose3D(to_pose_est);
        state_.vizmap.insertEdgeAtEnd(
            f.from_kf_, f.to_kf_, mrpt::poses::CPose3D(to_pose_est));
    }

    // Measure noise model:
    MRPT_TODO("handle custom noise matrix from input factor");

    const double std_xyz = f.noise_model_diag_xyz_;
    const double std_ang = f.noise_model_diag_rot_;

    auto noise_relpose = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << std_ang, std_ang, std_ang, std_xyz, std_xyz,
         std_xyz)
            .finished());

    MRPT_TODO("robust kernel: make optional");
    auto robust_noise_model = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise_relpose);

    const auto to_pose_key   = state_.mola2gtsam.at(f.to_kf_)[KF_KEY_POSE];
    const auto from_pose_key = state_.mola2gtsam.at(f.from_kf_)[KF_KEY_POSE];
    // (vel keys may be used or not; declare here for convenience anyway)
    const auto to_vel_key = state_.mola2gtsam.at(f.to_kf_)[KF_KEY_VEL];
    // const auto from_vel_key =
    // state_.mola2gtsam.at(f.from_kf_)[KF_KEY_VEL];

    // Add to list of initial guess (if not done already with a former
    // factor):
    if (state_.kf_has_value.count(f.to_kf_) == 0)
    {
        const gtsam::Pose3 p_to_pose_est = toPose3(to_pose_est);
        if (!state_.newvalues.exists(to_pose_key))
            state_.newvalues.insert(to_pose_key, p_to_pose_est);
        else
            state_.newvalues.update(to_pose_key, p_to_pose_est);

        // Init vel (if applicable):
        switch (params_.state_vector)
        {
            case StateVectorType::DynSE3:
            {
                const gtsam::Vector3 vel0 = gtsam::Z_3x1;
                if (!state_.newvalues.exists(to_vel_key))
                    state_.newvalues.insert(to_vel_key, vel0);
                else
                    state_.newvalues.update(to_vel_key, vel0);
            }
            break;
            case StateVectorType::DynSE2:
            {
                THROW_EXCEPTION("TODO");
            }
            break;

            default:
                break;
        };

        state_.kf_has_value.insert(f.to_kf_);
    }

    // Add relative pose factor:
    switch (params_.state_vector)
    {
        case StateVectorType::SE2:
            THROW_EXCEPTION("to do!");
            break;
        case StateVectorType::DynSE2:
            THROW_EXCEPTION("to do!");
            break;

        case StateVectorType::SE3:
        case StateVectorType::DynSE3:
            state_.newfactors
                .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
                    from_pose_key, to_pose_key, measure, robust_noise_model);
            break;
        default:
            THROW_EXCEPTION("Unhandled state vector type");
    };

    return new_fid;

    MRPT_END
}

fid_t ASLAM_gtsam::addFactor(const FactorDynamicsConstVel& f)
{
    MRPT_START
    MRPT_LOG_DEBUG("Adding new FactorDynamicsConstVel");

    // Add to the WorldModel:
    worldmodel_->factors_lock_for_write();
    const fid_t new_fid = worldmodel_->factor_push_back(f);
    worldmodel_->factors_unlock_for_write();

    // Initial estimation of the new KF:
    mrpt::math::TPose3D to_pose_est;

    worldmodel_->entities_lock_for_write();

    const auto& kf_from = worldmodel_->entity_by_id(f.from_kf_);
    const auto& kf_to   = worldmodel_->entity_by_id(f.to_kf_);

    const auto from_tim      = mola::entity_get_timestamp(kf_from);
    const auto from_pose_est = mola::entity_get_pose(kf_from);
    const auto to_tim        = mola::entity_get_timestamp(kf_to);

    MRPT_TODO("Build KF guess from dynamics");
    to_pose_est = from_pose_est;

    // Store the result just in case we need it as a quick guess in next
    // factors, before running the actual optimizer:
    // Dont update the pose of the global reference, fixed to Identity()
    if (f.to_kf_ != state_.root_kf_id)
        updateEntityPose(
            worldmodel_->entity_by_id(f.to_kf_), toPose3(to_pose_est));

    worldmodel_->entities_unlock_for_write();

    const auto to_pose_key   = state_.mola2gtsam.at(f.to_kf_)[KF_KEY_POSE];
    const auto from_pose_key = state_.mola2gtsam.at(f.from_kf_)[KF_KEY_POSE];
    // (vel keys may be used or not; declare here for convenience anyway)
    const auto to_vel_key   = state_.mola2gtsam.at(f.to_kf_)[KF_KEY_VEL];
    const auto from_vel_key = state_.mola2gtsam.at(f.from_kf_)[KF_KEY_VEL];

    // Add to list of initial guess (if not done already with a former
    // factor):
    if (state_.kf_has_value.count(f.to_kf_) == 0)
    {
        const gtsam::Pose3 p_to_pose_est = toPose3(to_pose_est);
        if (!state_.newvalues.exists(to_pose_key))
            state_.newvalues.insert(to_pose_key, p_to_pose_est);
        else
            state_.newvalues.update(to_pose_key, p_to_pose_est);

        // Init vel (if applicable):
        switch (params_.state_vector)
        {
            case StateVectorType::DynSE3:
            {
                const gtsam::Vector3 vel0 = gtsam::Z_3x1;
                if (!state_.newvalues.exists(to_vel_key))
                    state_.newvalues.insert(to_vel_key, vel0);
                else
                    state_.newvalues.update(to_vel_key, vel0);
            }
            break;
            case StateVectorType::DynSE2:
            {
                THROW_EXCEPTION("TODO");
            }
            break;

            default:
                break;
        };

        state_.kf_has_value.insert(f.to_kf_);
    }

    // Add const-vel factor to gtsam itself:
    switch (params_.state_vector)
    {
        case StateVectorType::DynSE3:
        {
            const double dt = mrpt::system::timeDifference(from_tim, to_tim);
            ASSERT_(dt > 0);

            // errors in constant vel:
            const double std_pos = params_.const_vel_model_std_pos;
            const double std_vel = params_.const_vel_model_std_vel;

            const gtsam::Vector6 diag_stds =
                (gtsam::Vector6() << std_pos, std_pos, std_pos, std_vel,
                 std_vel, std_vel)
                    .finished();

            auto noise_velModel =
                gtsam::noiseModel::Diagonal::Sigmas(diag_stds);

            if (dt > 10.0)
            {
                MRPT_LOG_WARN_FMT(
                    "A constant-time velocity factor has been added for "
                    "KFs "
                    "too far-away in time: dT=%.03f s. Adding it, anyway, "
                    "as "
                    "requested.",
                    dt);
            }

            state_.newfactors.emplace_shared<mola::ConstVelocityFactorSE3>(
                from_pose_key, from_vel_key, to_pose_key, to_vel_key, dt,
                noise_velModel);
        }
        break;

        default:
            // Ignore dynamics
            break;
    };

    return new_fid;

    MRPT_END
}

fid_t ASLAM_gtsam::addFactor(const SmartFactorStereoProjectionPose& f)
{
    MRPT_START
    // MRPT_LOG_DEBUG("Adding new SmartFactorStereoProjectionPose");

    // Add to the WorldModel:
    worldmodel_->factors_lock_for_write();
    const fid_t new_fid = worldmodel_->factor_push_back(f);
    worldmodel_->factors_unlock_for_write();

    MRPT_TODO("Take noise params from f");
    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

    gtsam::SmartProjectionParams params(
        gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);

    // params.setRankTolerance(0.1);

    auto factor_ptr = gtsam::SmartStereoProjectionPoseFactor::shared_ptr(
        new gtsam::SmartStereoProjectionPoseFactor(gaussian, params));

    state_.stereo_factors.factors[new_fid]            = factor_ptr;
    state_.newFactor2molaid[state_.newfactors.size()] = new_fid;
    state_.newfactors.push_back(factor_ptr);

    MRPT_LOG_DEBUG_STREAM(
        "SmartFactorStereoProjectionPose: Created empty. fid=" << new_fid);

    return new_fid;

    MRPT_END
}

fid_t ASLAM_gtsam::addFactor(const SmartFactorIMU& f)
{
    MRPT_START
    MRPT_LOG_DEBUG("Adding new SmartFactorIMU");

    // Add to the WorldModel:
    worldmodel_->factors_lock_for_write();
    const fid_t new_fid = worldmodel_->factor_push_back(f);

    {
        Factor&               fa = worldmodel_->factor_by_id(new_fid);
        mola::SmartFactorIMU& sf = std::get<mola::SmartFactorIMU>(fa);
        state_.active_imu_factors.push_back(&sf);
    }

    worldmodel_->factors_unlock_for_write();

    MRPT_TODO("Create the IMUHelper here!");

    return new_fid;

    MRPT_END
}

fid_t ASLAM_gtsam::addFactor(const FactorStereoProjectionPose& f)
{
    MRPT_START
    // MRPT_LOG_DEBUG("Adding new SmartFactorStereoProjectionPose");

    // Add to the WorldModel:
    worldmodel_->factors_lock_for_write();
    const fid_t new_fid = worldmodel_->factor_push_back(f);
    worldmodel_->factors_unlock_for_write();

    MRPT_TODO("Take noise params from f");
    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);

    const auto sp = gtsam::StereoPoint2(
        f.observation_.x_left, f.observation_.x_right, f.observation_.y);

    MRPT_LOG_DEBUG_STREAM(
        "FactorStereoProjectionPose: new_fid="
        << new_fid << " from kf id#" << f.observing_kf_ << " lm #"
        << f.observed_landmark_ << " xl=" << f.observation_.x_left
        << " xr=" << f.observation_.x_right << " y=" << f.observation_.y);

    using namespace gtsam::symbol_shorthand;  // X(), L()

    const gtsam::Key pose_key = X(f.observing_kf_);
    const gtsam::Key lm_key   = L(f.observed_landmark_);

    gtsam::Pose3 cameraPoseOnRobot;

    state_.newfactors.emplace_shared<
        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>>(
        sp, gaussian, pose_key, lm_key, state_.stereo_factors.camera_K, false,
        true, cameraPoseOnRobot);

    return new_fid;

    MRPT_END
}
